#include <RFM69.h>
#include <RFM69_ATC.h>
#include <RFM69_OTA.h>
#include <SPIFlash.h>
#include <SPI.h>
#include <LowPower.h>
#include "EnvironmentData.h"

#define WS_PIN           3 // Wind Speed Pin
#define LED_PIN          9
#define VANE_PIN         A0 // Analog 0
#define DIR_OFFSET       0

#define NODEID      3
#define NETWORKID   100
#define GATEWAYID   1
#define PIGATEWAYID 10
#define FREQUENCY   RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY  "****************" //has to be same 16 characters/bytes on all nodes, not more not less!
//#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
//*********************************************************************************************
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -85
//*********************************************************************************************
#define SEND_LOOPS   5 //send data this many sleep loops (15 loops of 8sec cycles = 120sec ~ 2 minutes)
#define SLEEP_SEC SLEEP_1S
#define SLEEP_LONGEST SLEEP_8S
period_t sleepTime = SLEEP_LONGEST; //period_t is an enum type defined in the LowPower library (LowPower.h)

//****************************************************************************************1*****
#define BATT_MONITOR_EN A3 //enables battery voltage divider to get a reading from a battery, disable it to save power
#define BATT_MONITOR  A7   //through 1Meg+470Kohm and 0.1uF cap from battery VCC - this ratio divides the voltage to bring it below 3.3V where it is scaled to a readable range
#define BATT_FORMULA(reading) reading * 0.00322 * 3.141 //3.125  // >>> fine tune this parameter to match your voltage when fully charged
#define BATT_LOW      3.2  //(volts)
#define BATT_READ_LOOPS  SEND_LOOPS*10  // read and report battery voltage every this many sleep cycles (ex 30cycles * 8sec sleep = 240sec/4min). For 450 cycles you would get ~1 hour intervals between readings
//*****************************************************************************************************************************

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

#define BLINK_EN                 //uncomment to blink LED on every send
//#define SERIAL_EN              //comment out if you don't want any serial output

#ifdef SERIAL_EN
  #define SERIAL_BAUD   115200
  #define DEBUG(input)   {Serial.print(input);}
  #define DEBUGln(input) {Serial.println(input);}
  #define SERIALFLUSH() {Serial.flush();}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
  #define SERIALFLUSH();
#endif
//*****************************************************************************************************************************

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

SPIFlash flash(FLASH_SS, 0xEF30); //WINDBOND 4MBIT flash chip on CS pin D8 (default for Moteino)
WindData windData;
char Wstr[6];
char WTOPstr[6];

char* BATstr="BAT:5.00v"; //longest battery voltage reading message = 9chars
float batteryVolts = 5;
char buffer[50];
float todayTopWS = 0.0;
unsigned long lastResetTime = 0, time = 0, now = 0;

#define RESET_DELAY 600000
#define PGM_TIMEOUT 150000
#define TODAY_WIND_MAX_START 36864
#define YESTERDAY_WIND_MAX_START TODAY_WIND_MAX_START + sizeof(float) + 1

volatile unsigned long Rotations;  // cup rotation counter used in interrupt routine
volatile unsigned long ContactBounceTime;  // Timer to avoid contact bounce in interrupt routine

// This is the function that the interrupt calls to
//increment the rotation count
void RotationIRQ()
{
  // debounce the switch contact.
  if ((millis() - ContactBounceTime) > 15 ) 
  {
    Rotations++;
    ContactBounceTime = millis();
  }
}

void setup()
{
  // Set unused pins to output to save power
  for (uint8_t i=0; i<=A5; i++)
  {
    if (i == RF69_SPI_CS) continue;
    if (i == FLASH_SS) continue;
    if (i == VANE_PIN) continue;
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  // Use internal pullup resistor for wind speed pin
  pinMode(WS_PIN, INPUT_PULLUP);
  pinMode(BATT_MONITOR, INPUT);
  
  delay(10);

#ifdef SERIAL_EN
  Serial.begin(SERIAL_BAUD);
#endif
  pinMode(LED, OUTPUT);
  
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  radio.encrypt(ENCRYPTKEY);

//Auto Transmission Control - dials down transmit power to save battery (-100 is the noise floor, -90 is still pretty good)
//For indoor nodes that are pretty static and at pretty stable temperatures (like a MotionMote) -90dBm is quite safe
//For more variable nodes that can expect to move or experience larger temp drifts a lower margin like -70 to -80 would probably be better
//Always test your ATC mote in the edge cases in your own environment to ensure ATC will perform as you expect
#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
#endif

  radio.writeReg(0x58, 0x2D); // High sensitivity
    
  if (flash.initialize())
  {
    DEBUGln(F("SPI Flash Init OK!"));
  }
  else
  {
    DEBUGln(F("SPI Flash Init FAIL! (is chip present?)"));
  }

  // Read the daily rain amount on start
  ReadBytes(TODAY_WIND_MAX_START, (void*) &todayTopWS, sizeof(todayTopWS));

  // Make sure flash is OK and a valid value is stored.
  // If not, set to zero
  if (isnan(todayTopWS))
    todayTopWS = 0.0;

  flash.sleep();

  windData.hdr.type = WIND;

  sprintf(buffer, "WindMote - transmitting at: %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(buffer);
  
  radio.sendWithRetry(PIGATEWAYID, "START", 6);
}

short sendLoops = 0;
short battReadLoops = 0;
float prevTopWS = 0;

void loop()
{
  now = millis();
    
  if (battReadLoops-- <= 0) //only read battery every BATT_READ_LOOPS cycles
  {
    windData.hdr.batLvl = ReadBatteryVoltage();
    battReadLoops = BATT_READ_LOOPS - 1;
  }

  if (sendLoops-- <= 0)   //send readings every SEND_LOOPS
  {
    sendLoops = SEND_LOOPS - 1;

    windData.windSpeed = CalcWindSpeed();
    sprintf(windData.windDir, "%s", CalcWindDir(DIR_OFFSET));

    if (windData.windSpeed > todayTopWS)
    {
      // Don't want these steps to be interrupted.
      noInterrupts();

      // Keep track of top wind speed for the day
      todayTopWS = windData.windSpeed;
      WriteBytes(TODAY_WIND_MAX_START, (const void*) &todayTopWS, sizeof(todayTopWS));
      
      interrupts();
   }
    
    DEBUG("Wind Speed: ");
    DEBUGln(windData.windSpeed);
    DEBUG("Wind Dir: ");
    DEBUGln(windData.windDir);

    dtostrf(windData.windSpeed, 3, 1, Wstr);

    if (prevTopWS != todayTopWS)
    {
      dtostrf(todayTopWS, 3, 1, WTOPstr);
      sprintf(buffer, "BAT:%sv WS:%s WDIR:%s WST:%s", BATstr, Wstr, windData.windDir, WTOPstr);
      prevTopWS = todayTopWS;
    }
    else
    {
      sprintf(buffer, "BAT:%sv WS:%s WDIR:%s", BATstr, Wstr, windData.windDir);
    }

    char xmitLvl[6];

    sprintf(xmitLvl, " X:%d", radio._transmitLevel);
    strcat(buffer, xmitLvl);
    
    DEBUGln(buffer);
  
    // Send data to pi gateway
    if (radio.sendWithRetry(PIGATEWAYID, buffer, strlen(buffer), 3, 100))
    
    {
      #ifdef BLINK_EN
        Blink(LED, 5);
      #endif
    }

    // Send data to LCD weather monitor
    if (radio.sendWithRetry(GATEWAYID, (const void*)(&windData),
                        sizeof(windData), 3, 100))
    {
      // Did a command to reset the rain amount arrive
      if (radio.DATALEN == 3)   
      {
        if (radio.DATA[0]=='R' && radio.DATA[1]=='S' && radio.DATA[2]=='T')
        {
          // Reset the daily top wind speed to zero
          ResetWind();
        }
        else if (radio.DATA[0]=='P' && radio.DATA[1]=='G' && radio.DATA[2]=='M')
        {
          HandleWirelessUpdate();
        }
      }
    }
  }
  
  SERIALFLUSH();

  time = time + 8000 + millis() - now;

  radio.sleep();
  LowPower.powerDown(sleepTime, ADC_OFF, BOD_OFF);
  DEBUGln("WAKEUP8s");
}

// Perform wireless update
void HandleWirelessUpdate()
{
  unsigned long updateTime = millis();
  
  do
  {
    //When this sketch is on a node where you can afford the power to keep the radio awake all the time
    //   you can make it receive messages and also make it wirelessly programmable
    //   otherwise this section can be removed
    if (radio.receiveDone())
    {
      DEBUG('[');DEBUG(radio.SENDERID);DEBUG("] ");
      for (byte i = 0; i < radio.DATALEN; i++)
        DEBUG((char)radio.DATA[i]);
  
      flash.wakeup();
      // wireless programming token check - this only works when radio is kept awake to listen for WP tokens
      CheckForWirelessHEX(radio, flash, true);
  
      //first send any ACK to request
      DEBUG("   [RX_RSSI:");DEBUG(radio.RSSI);DEBUG("]");
      if (radio.ACKRequested())
      {
        radio.sendACK();
        DEBUG(" - ACK sent.");
      }
      
      DEBUGln();
    }
  } while(TimeSince(updateTime) > PGM_TIMEOUT);
}

void ResetWind()
{
  // Reset only once a day.
  if (TimeSince(lastResetTime) < RESET_DELAY)
  {
    return;
  }
  
  lastResetTime = time;
  
  SendTopWind(todayTopWS);
 
  todayTopWS = 0.0;

  // Reset the daily top wind speed in flash
  WriteBytes(TODAY_WIND_MAX_START, (const void*) &todayTopWS, sizeof(todayTopWS));
}

void SendTopWind(float dailyTopWS)
{
  char ystr[10];

  // Send the data
  dtostrf(dailyTopWS, 3, 1, ystr);
  sprintf(buffer, "WSY:%s", ystr);
  
  DEBUGln(buffer);
  
  // Send the data
  if (radio.sendWithRetry(PIGATEWAYID, buffer, sizeof(buffer), 3, 100))
  {
    #ifdef BLINK_EN
      Blink(LED, 5);
    #endif
  }
}

char* CalcWindDir(int dirOffset)
{
  int vaneValue = analogRead(VANE_PIN);
  
  // translated 0 - 360 direction 
  int windDir = map(vaneValue, 0, 1023, 0, 360);
  
  windDir += dirOffset;

  if (windDir > 360)
    windDir = windDir - 360;

  if (windDir < 0)
    windDir = windDir + 360;

   DEBUG("Direction: ");
   DEBUGln(windDir);

  if (windDir < 22) return "N"; 
  else if (windDir < 67) return "NE"; 
  else if (windDir < 112) return "E"; 
  else if (windDir < 157) return "SE"; 
  else if (windDir < 212) return "S"; 
  else if (windDir < 247) return "SW"; 
  else if (windDir < 292) return "W"; 
  else if (windDir < 337) return "NW"; 
  else return "N"; 
}

float CalcWindSpeed()
{
  float windSpeed = 0.0;

  Rotations = 0;   //Set NbTops to 0 ready for calculations
  attachInterrupt(digitalPinToInterrupt(WS_PIN), RotationIRQ, FALLING);
  delay (3000);  //Wait 3 seconds to average
  detachInterrupt(digitalPinToInterrupt(WS_PIN));

  // convert to mp/h using the formula V=P(2.25/T)
  // V = P(2.25/3) = P * 0.75

  // The RotationIRQ is called once when it
  // is attached.  Need to subtract that out
  if (Rotations > 0) Rotations--;
  
  windSpeed = Rotations * 0.75;

  return windSpeed;
}

float ReadBatteryVoltage()
{
  unsigned int readings = 0;
  
  // Enable battery monitor (via mosfet controlled by A3)
  pinMode(BATT_MONITOR_EN, OUTPUT);
  digitalWrite(BATT_MONITOR_EN, LOW);
  
  delay(3);
  
  for (byte i = 0; i < 5; i++) //take several samples, and average
  {
    readings += analogRead(BATT_MONITOR);
  }
  
  // Disable battery monitor
  pinMode(BATT_MONITOR_EN, INPUT); //highZ mode will allow p-mosfet to be pulled high and disconnect the voltage divider.

  batteryVolts = BATT_FORMULA(readings / 5.0);
  dtostrf(batteryVolts,3,2, BATstr); //update the BATStr which gets sent every BATT_CYCLES
  
  return BATT_FORMULA(readings / 5.0);
}

void EraseBlock(uint32_t addr)
{
  flash.blockErase4K(addr);
}

void ReadBytes(uint32_t addr, void* buf, uint16_t len)
{
  flash.wakeup();
  flash.readBytes(addr, buf, len);
  flash.sleep();
}

void WriteBytes(uint32_t addr, const void* buf, uint16_t len)
{
  flash.wakeup();
  EraseBlock(addr);
  flash.writeBytes(addr, buf, len);
  flash.sleep();
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

unsigned long TimeSince(unsigned long timeStamp)
{
  if (time >= timeStamp)
    return time - timeStamp;
  else
    return ((unsigned long)(-1)) - timeStamp + time;
}
