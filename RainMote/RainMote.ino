#include <RFM69.h>         //get it here: https://github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://github.com/lowpowerlab/rfm69
#include <RFM69_OTA.h>
#include <SPIFlash.h>      //get it here: https://github.com/lowpowerlab/spiflash
#include <SPI.h>           //included in Arduino IDE (www.arduino.cc)
#include <LowPower.h>      //get it here: https://github.com/lowpowerlab/lowpower
                           //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/
#include "EnvironmentData.h"

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE ************
//*********************************************************************************************
#define GATEWAYID       1
#define PIGATEWAYID     10
#define NODEID          20
#define NETWORKID       100
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY       RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY      "****************" //has to be same 16 characters/bytes on all nodes, not more not less!
//#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
//*********************************************************************************************
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -85
//*********************************************************************************************
#define SEND_LOOPS   74 //send data this many sleep loops (15 loops of 8sec cycles = 120sec ~ 2 minutes)
#define SLEEP_FASTEST SLEEP_15MS
#define SLEEP_FAST SLEEP_250MS
#define SLEEP_SEC SLEEP_1S
#define SLEEP_LONG SLEEP_2S
#define SLEEP_LONGER SLEEP_4S
#define SLEEP_LONGEST SLEEP_8S
period_t sleepTime = SLEEP_LONGEST; //period_t is an enum type defined in the LowPower library (LowPower.h)
//*********************************************************************************************
#define BATT_READ_LOOPS  SEND_LOOPS*10  // read and report battery voltage every this many sleep cycles (ex 30cycles * 8sec sleep = 240sec/4min). For 450 cycles you would get ~1 hour intervals between readings
#define BANDGAP_VOLTAGE
//*****************************************************************************************************************************

#define RAIN_PIN         3

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

//#define BLINK_EN                 //uncomment to blink LED on every send
#define SERIAL_EN                //comment out if you don't want any serial output

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
RainData rainData;

char buffer[50];
float rainAmount = 0.0;
// interrupt trigger
volatile bool rainDetected = false;
char Rstr[7];

#define RESET_DELAY 600000
#define PGM_TIMEOUT 150000
#define RN_ARRAY_MAX 30
#define DAILY_RAIN_START 36864
#define RN_ARRAY_START DAILY_RAIN_START + (4 * 1024) + 1
#define RAIN_INCREMENT 0.01  // inches of rain every time bucket tips

char* BATstr = "BAT:3.30v"; //longest battery voltage reading message = 9chars

unsigned long lastResetTime = 0, time = 0, now = 0;

// IRQ handler for rain bucket tip
void RainIRQ()
{
    rainDetected = true;
}

void setup()
{
#ifdef SERIAL_EN
  Serial.begin(SERIAL_BAUD);
#endif
  pinMode(LED, OUTPUT);
  
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
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

  // Set unused pins to output.
  // Helps save power
  for (uint8_t i=0; i<=A5; i++)
  {
    if (i == RF69_SPI_CS) continue;
    if (i == FLASH_SS) continue;
    if (i == RAIN_PIN) continue;
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  if (flash.initialize())
  {
    DEBUGln(F("SPI Flash Init OK!"));
  }
  else
  {
    DEBUGln(F("SPI Flash Init FAIL! (is chip present?)"));
  }

  // Set the RAIN_PIN to input and activate
  // the internal pullup resistor
  pinMode(RAIN_PIN, INPUT_PULLUP);
  
  // Set up the interrupt handler to detect the
  // reed switch when the rain bucket tips
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), RainIRQ, FALLING);

  // Read the daily rain amount on start
  ReadBytes(DAILY_RAIN_START, (void*) &rainAmount, sizeof(rainAmount));

  // Make sure flash is OK and a valid value is stored.
  // If not, set to zero
  if (isnan(rainAmount))
    rainAmount = 0.0;
    
  flash.sleep();

  rainData.hdr.type = RN;
  
  sprintf(buffer, "RainMote - transmitting at: %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(buffer);

  radio.sendWithRetry(PIGATEWAYID, "START", 6);

  #ifdef BLINK_EN
    Blink(LED, 5);Blink(LED, 5);
  #endif
}

byte sendLoops = 0;
short battReadLoops = 0;
float batteryVolts = 3.3;
float prevBatteryVolts = 0.0;

void loop()
{
  now = millis();

  // Store the current rain amount
  if (rainDetected)
  {
    // Don't want these steps to be interrupted.
    noInterrupts();
    rainDetected = false;
    rainAmount += RAIN_INCREMENT;
    WriteBytes(DAILY_RAIN_START, (const void*) &rainAmount, sizeof(rainAmount));
    interrupts();
    
    DEBUG("Rain Fall: ");
    DEBUGln(rainAmount);
    #ifdef BLINK_EN
      Blink(LED, 5);
    #endif
  }
  
  if (battReadLoops-- <= 0) //only read battery every BATT_READ_LOOPS cycles
  {
    readBattery();
    rainData.hdr.batLvl = batteryVolts;
    battReadLoops = BATT_READ_LOOPS - 1;
  }

  if (sendLoops-- <= 0)   //send readings every SEND_LOOPS
  {
    sendLoops = SEND_LOOPS - 1;

    rainData.rainAmount = rainAmount;

    DEBUG("Rain Amt: ");
    DEBUGln(rainData.rainAmount);


    // Send the data
    dtostrf(rainAmount, 1, 3, Rstr);

    if (batteryVolts != prevBatteryVolts)
    {
      sprintf(buffer, "BAT:%sv RN:%s X:%d", BATstr, Rstr, radio._transmitLevel);
      prevBatteryVolts = batteryVolts;
    }
    else
    {
      sprintf(buffer, "RN:%s X:%d", Rstr, radio._transmitLevel);
    }
    
    DEBUGln(buffer);
  
    // Send data to pi gateway
    if (radio.sendWithRetry(PIGATEWAYID, buffer, strlen(buffer), 3, 50))
    {
      #ifdef BLINK_EN
        Blink(LED, 5);
      #endif
    }

    // Send data to LCD weather monitor
    if (radio.sendWithRetry(GATEWAYID, (const void*)(&rainData),
                            sizeof(rainData), 3, 50))
    {     
      // Did a command to reset the rain amount arrive
      if (radio.DATALEN == 3)
      {
        if (radio.DATA[0]=='R' && radio.DATA[1]=='S' && radio.DATA[2]=='T')
        {
          // Reset the rain amount to zero
          ResetRain();
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
  flash.end();
  flash.sleep();
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

// Keep one month of rain data
void ResetRain()
{
  float rnArray[RN_ARRAY_MAX];  // Array to hold 30 days worth rain

  // Reset only once a day.
  if (TimeSince(lastResetTime) < RESET_DELAY)
  {
    return;
  }
  
  lastResetTime = time;
 
  // Load the daily rain amounts into memory
  ReadBytes(RN_ARRAY_START, rnArray, sizeof(rnArray));

  for (byte i = RN_ARRAY_MAX - 1; i > 0; i--)
  {
    // If flash memory gets messed up
    // or its the first time reading
    // write in a zero amount
    if (isnan(rnArray[i-1]))
    {
      rnArray[i] = 0.0;
    }
    else
    {
      rnArray[i] = rnArray[i-1];
    }
  }

  rnArray[0] = rainAmount;
  rainAmount = 0.0;

  // Persist daily rain amounts to flash
  WriteBytes(RN_ARRAY_START, rnArray, sizeof(rnArray));
  WriteBytes(DAILY_RAIN_START, (const void*) &rainAmount, sizeof(rainAmount));
  
  SendRainTotals(rnArray, RN_ARRAY_MAX);
}

void SendRainTotals(float rnArray[], int size)
{
  char ystr[10], wstr[10], mstr[10];
  float yesterdayRA = rnArray[0];
  float weekRA = 0.0, monthRA = 0.0;

  // Compute weekly rainfall amount
  for (byte i = 0; i < 7; i++)
  {
    weekRA += rnArray[i];
  }

  // Compute the monthly rainfall amount
  for (byte i = 0; i < size; i++)
  {
    monthRA += rnArray[i];
  }

  // Send the data
  dtostrf(yesterdayRA, 1, 3, ystr);
  dtostrf(weekRA, 1, 3, wstr);
  dtostrf(monthRA, 1, 3, mstr);
  sprintf(buffer, "RNY:%s RNW:%s RN30:%s", ystr, wstr, mstr);
  
  DEBUGln(buffer);
  
  // Send the data
  if (radio.sendWithRetry(PIGATEWAYID, buffer, sizeof(buffer), 3, 50))
  {
    #ifdef BLINK_EN
      Blink(LED, 5);
    #endif
  }
}

void readBattery()
{
  #ifndef BANDGAP_VOLTAGE
    readBatteryVoltageDivider();
  #else
    readBatteryBandGap();
  #endif
}

#ifndef BANDGAP_VOLTAGE
void readBatteryVoltageDivider()
{
  unsigned int readings=0;
  
  //enable battery monitor on WeatherShield (via mosfet controlled by A3)
  pinMode(BATT_MONITOR_EN, OUTPUT);
  digitalWrite(BATT_MONITOR_EN, LOW);

  for (byte i=0; i<5; i++) //take several samples, and average
    readings+=analogRead(BATT_MONITOR);
  
  //disable battery monitor
  pinMode(BATT_MONITOR_EN, INPUT); //highZ mode will allow p-mosfet to be pulled high and disconnect the voltage divider on the weather shield
    
  batteryVolts = BATT_FORMULA(readings / 5.0);
  dtostrf(batteryVolts,3,2, BATstr); //update the BATStr which gets sent every BATT_CYCLES or along with the MOTION message
  if (batteryVolts <= BATT_LOW) BATstr = "LOW";
}
#endif

void readBatteryBandGap()
{
  batteryVolts = getDoubleMatchVoltage() / 100.0;

  dtostrf(batteryVolts,3,2, BATstr); //update the BATStr which gets sent every BATT_CYCLES or along with the MOTION message
}

uint16_t getRelativeBandgapVoltage()
{ 
  // used for single-shot voltage measurements
  uint16_t rawBandgapMeasurement;
  
  // Read bandgap voltage reference (~1.1V) against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC)); //busy-wait until the ADC is done converting
  rawBandgapMeasurement = ADCL;   //get the low order bits of ADC
  rawBandgapMeasurement |= ADCH<<8; //combine with high order bits of ADC 
  return rawBandgapMeasurement;
}

uint16_t getDoubleMatchVoltage()
{
  // returns the first voltage measurement that matches the immediately successive voltage measurement
  uint16_t rbgv;// relative bandgap voltage
  uint16_t previousVoltage=0;  //set to zero so that first "match" (with only one voltage having been read) will fail.  Need two matching voltages
  uint16_t voltage;
  
  rbgv = getRelativeBandgapVoltage(); // grab the  first voltage measurement
  while (rbgv != previousVoltage)
  {
    //ensure that two successive voltage measurements match
    previousVoltage = rbgv;
    rbgv = getRelativeBandgapVoltage();
  }

  voltage = computedVoltage(rbgv);

  return voltage;
}

uint16_t computedVoltage(uint16_t rbgv)
{
  uint16_t voltage;
  voltage = ((1125300/rbgv)+5)/10;
  return voltage;
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
