#include <RFM69.h>         //get it here: https://github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://github.com/lowpowerlab/rfm69
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LowPower.h>      //get it here: https://github.com/lowpowerlab/lowpower
                           //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE ************
//*********************************************************************************************
#define PIGATEWAYID     10
#define NODEID          16
#define NETWORKID       200
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY       RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY      "****************" //has to be same 16 characters/bytes on all nodes, not more not less!
//#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
//*********************************************************************************************
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -85
//*********************************************************************************************
#define SEND_LOOPS   72 //send data this many sleep loops (15 loops of 8sec cycles = 120sec ~ 2 minutes)
#define SLEEP_FASTEST SLEEP_15MS
#define SLEEP_FAST SLEEP_250MS
#define SLEEP_SEC SLEEP_1S
#define SLEEP_LONG SLEEP_2S
#define SLEEP_LONGER SLEEP_4S
#define SLEEP_LONGEST SLEEP_8S
period_t sleepTime = SLEEP_LONGEST; //period_t is an enum type defined in the LowPower library (LowPower.h)
#define BATT_READ_LOOPS  SEND_LOOPS*10  // read and report battery voltage every this many sleep cycles (ex 30cycles * 8sec sleep = 240sec/4min). For 450 cycles you would get ~1 hour intervals between readings
#define BANDGAP_VOLTAGE
//*********************************************************************************************
#define ONE_WIRE_BUS   4 //Digital pin used for OneWire protocol
#define PWR_PIN        5 // Used to power soil moisture sensor
//*****************************************************************************************************************************


#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

//#define BLINK_EN                 //uncomment to blink LED on every send
//#define SERIAL_EN                //comment out if you don't want any serial output

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

// OneWire instance to communicate with any OneWire devices
OneWire bus(ONE_WIRE_BUS);

// Pass in OneWire refernce to Dallas library
DallasTemperature tempSensor(&bus);

DeviceAddress tempSensorAddr;
char buffer[50];

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
  
  sprintf(buffer, "HottubMote - transmitting at: %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(buffer);

  for (uint8_t i=0; i<=A5; i++)
  {
    if (i == RF69_SPI_CS) continue;
    if (i == ONE_WIRE_BUS) continue;
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  // Initalize library
  tempSensor.begin();

  digitalWrite(PWR_PIN, HIGH);

  // Get address of temperature sensor on OneWire bus
  if (tempSensor.getAddress(tempSensorAddr, 0))
  {
    DEBUG("Found temperature sensor at address: ");
    DEBUGln(tempSensorAddr[0]);
  }

  digitalWrite(PWR_PIN, LOW);
  
  DEBUGln("Done intializing");
  SERIALFLUSH();

  radio.sendWithRetry(PIGATEWAYID, "START", 6);
  
  #ifdef BLINK_EN
    Blink(LED, 100);Blink(LED, 100);
  #endif
}


short sendLoops = 0;
short battReadLoops=0;
char Fstr[10];
float batteryVolts = 5;
float prevBatteryVolts = 0;
char* BATstr="BAT:5.00v"; //longest battery voltage reading message = 9chars


void loop()
{
  if (battReadLoops--<=0) //only read battery every BATT_READ_LOOPS cycles
  {
    // Get battery voltage
    readBattery();
    battReadLoops = BATT_READ_LOOPS-1;
  }
  
  if (sendLoops-- <= 0)   //send readings every SEND_LOOPS
  {
    float temperature = 0;
    byte count = 0;

    sendLoops = SEND_LOOPS - 1;

    // Power up the temperature sensor
    digitalWrite(PWR_PIN, HIGH);
    delay(50);
    
    do
    {
      count++;
      tempSensor.requestTemperaturesByAddress(tempSensorAddr);
      temperature = tempSensor.getTempFByIndex(0);
    } while (temperature >= 185 && count < 21);
    
    DEBUGln(count);
    
    // Power down the temperature sensor
    digitalWrite(PWR_PIN, LOW);

    dtostrf(temperature, 3,2, Fstr);

    if (batteryVolts != prevBatteryVolts)
    {
      sprintf(buffer, "BAT:%s F:%s X:%d", BATstr, Fstr, radio._transmitLevel);
      prevBatteryVolts = batteryVolts;
    }
    else
    {
      sprintf(buffer, "F:%s X:%d", Fstr, radio._transmitLevel);
    }
    DEBUGln(buffer);
    
    // Send the data
    if (radio.sendWithRetry(PIGATEWAYID, buffer, strlen(buffer), 3, 50))
    {
      #ifdef BLINK_EN
        Blink(LED, 10);
      #endif
    }
  }

  SERIALFLUSH();

  radio.sleep();
  LowPower.powerDown(sleepTime, ADC_OFF, BOD_OFF);
  DEBUGln("WAKEUP8s");
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

void Blink(byte PIN, byte DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS/2);
  digitalWrite(PIN,LOW);
  delay(DELAY_MS/2);  
}
