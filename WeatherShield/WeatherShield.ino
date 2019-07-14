// **********************************************************************************************************
// WeatherShield R2 (BME280 sensor) sameple sketch that works with Moteinos equipped with RFM69W/RFM69HW
// It sends periodic weather readings (temp, hum, atm pressure) from WeatherShield to the base node Moteino
// For use with MoteinoMEGA you will have to revisit the pin definitions defined below
// http://www.LowPowerLab.com/WeatherShield
// Example setup (with R1): http://lowpowerlab.com/blog/2015/07/24/attic-fan-cooling-tests/
// **********************************************************************************
// Copyright Felix Rusu 2016, http://www.LowPowerLab.com/contact
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#include <RFM69.h>         //get it here: https://github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://github.com/lowpowerlab/rfm69
//#include <RFM69_OTA.h>     //get it here: https://github.com/lowpowerlab/rfm69
#include <SPIFlash.h>      //get it here: https://github.com/lowpowerlab/spiflash
#include <SPI.h>           //included in Arduino IDE (www.arduino.cc)
#include <Wire.h>          //included in Arduino IDE (www.arduino.cc)
#include <SparkFunBME280.h>//get it here: https://github.com/sparkfun/SparkFun_BME280_Breakout_Board/tree/master/Libraries/Arduino/src
#include <LowPower.h>      //get it here: https://github.com/lowpowerlab/lowpower
                           //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/
#define SEND_TO_WM

#ifdef SEND_TO_WM                           
#include "EnvironmentData.h"
#endif

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE ************
//*********************************************************************************************
#define PIGATEWAYID     10
#define WMID            1
#define NODEID          2
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
#define SEND_LOOPS   72 //send data this many sleep loops (15 loops of 8sec cycles = 120sec ~ 2 minutes)
#define SLEEP_FASTEST SLEEP_15MS
#define SLEEP_FAST SLEEP_250MS
#define SLEEP_SEC SLEEP_1S
#define SLEEP_LONG SLEEP_2S
#define SLEEP_LONGER SLEEP_4S
#define SLEEP_LONGEST SLEEP_8S
period_t sleepTime = SLEEP_LONGEST; //period_t is an enum type defined in the LowPower library (LowPower.h)
//*********************************************************************************************
#define BATT_MONITOR_EN A3 //enables battery voltage divider to get a reading from a battery, disable it to save power
#define BATT_MONITOR  A7   //through 1Meg+470Kohm and 0.1uF cap from battery VCC - this ratio divides the voltage to bring it below 3.3V where it is scaled to a readable range
#define BATT_CYCLES   2    //read and report battery voltage every this many sleep cycles (ex 30cycles * 8sec sleep = 240sec/4min). For 450 cyclesyou would get ~1 hour intervals
#define BATT_FORMULA(reading) reading * 0.00322 * 1.475  // >>> fine tune this parameter to match your voltage when fully charged
#define BATT_LOW      3.6  //(volts)
#define BATT_READ_LOOPS  SEND_LOOPS*10  // read and report battery voltage every this many sleep cycles (ex 30cycles * 8sec sleep = 240sec/4min). For 450 cycles you would get ~1 hour intervals between readings
#define BANDGAP_VOLTAGE
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

SPIFlash flash(FLASH_SS, 0xEF30); //WINDBOND 4MBIT flash chip on CS pin D8 (default for Moteino)

bool flashExists = false;

BME280 bme280;
char Pstr[10];
char Fstr[10];
char Hstr[10];
double F,P,H;
char buffer[50];

#ifdef SEND_TO_WM                           
  THPData thpData;
#endif

void setup(void)
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

  sprintf(buffer, "WeatherMote - transmitting at: %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(buffer);

  Wire.begin();
  Wire.setClock(400000); //Increase to fast I2C speed!

  //initialize weather shield BME280 sensor
  bme280.setI2CAddress(0x77); //0x76,0x77 is valid.
  bme280.beginI2C();
  bme280.setMode(MODE_FORCED); //MODE_SLEEP, MODE_FORCED, MODE_NORMAL is valid. See 3.3
  bme280.setStandbyTime(0); //0 to 7 valid. Time between readings. See table 27.
  bme280.setFilter(0); //0 to 4 is valid. Filter coefficient. See 3.4.4
  bme280.setTempOverSample(1); //0 to 16 are valid. 0 disables temp sensing. See table 24.
  bme280.setPressureOverSample(1); //0 to 16 are valid. 0 disables pressure sensing. See table 23.
  bme280.setHumidityOverSample(1); //0 to 16 are valid. 0 disables humidity sensing. See table 19.
  P = bme280.readFloatPressure() * 0.0002953; //read Pa and convert to inHg
  F = bme280.readTempF();
  H = bme280.readFloatHumidity();
  bme280.setMode(MODE_SLEEP);

  DEBUGln("Initialized BME280");

  if (flash.initialize())
  {
    flashExists = true;
    DEBUGln("Flash found");
    flash.sleep();
  }
  else
  {
    DEBUGln("Flash not found");
  }

  for (uint8_t i=0; i<=A5; i++)
  {
    if (i == RF69_SPI_CS) continue;
    if (i == FLASH_SS) continue;
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

#ifdef SEND_TO_WM                           
  thpData.hdr.type = THP;
#endif
 
  readBattery();

  radio.sendWithRetry(PIGATEWAYID, "START", 6);
  Blink(LED, 10);Blink(LED, 10);

  DEBUGln("Done initializing");

  SERIALFLUSH();
}

char input=0;
byte sendLoops=0;
short battReadLoops=0;
float batteryVolts = 5;
float prevBatteryVolts = 0;
char* BATstr="BAT:5.00v"; //longest battery voltage reading message = 9chars
byte sendLen;

void loop()
{  
  if (battReadLoops--<=0) //only read battery every BATT_READ_LOOPS cycles
  {
    readBattery();
    battReadLoops = BATT_READ_LOOPS-1;
    radio.rcCalibration();
  }
  
  if (sendLoops--<=0)   //send readings every SEND_LOOPS
  {
    sendLoops = SEND_LOOPS-1;
    
    //read BME sensor
    bme280.setMode(MODE_FORCED); //Wake up sensor and take reading
    P = bme280.readFloatPressure() * 0.0002953; //read Pa and convert to inHg
    F = bme280.readTempF();
    H = bme280.readFloatHumidity();
    bme280.setMode(MODE_SLEEP);

    dtostrf(F, 3,2, Fstr);
    dtostrf(H, 3,2, Hstr);
    dtostrf(P, 3,2, Pstr);

    if (batteryVolts != prevBatteryVolts)
    {
      sprintf(buffer, "BAT:%sv F:%s H:%s P:%s X:%d", BATstr, Fstr, Hstr, Pstr, radio._transmitLevel);
      prevBatteryVolts = batteryVolts;
    }
    else
    {
      sprintf(buffer, "F:%s H:%s P:%s X:%d", Fstr, Hstr, Pstr, radio._transmitLevel);
    }

    sendLen = strlen(buffer);
    radio.sendWithRetry(PIGATEWAYID, buffer, sendLen, 3, 50);
    DEBUG(buffer); DEBUG(" (packet length:"); DEBUG(sendLen); DEBUGln(")");

    #ifdef SEND_TO_WM                           
      thpData.temperature = F;
      thpData.humidity = H;
      thpData.pressure = P;
      thpData.hdr.batLvl = batteryVolts;

      radio.sendWithRetry(WMID, (const void*) &thpData, sizeof(thpData), 3, 50);
    #endif

    #ifdef BLINK_EN
      Blink(LED, 5);
    #endif
  }
  
  //When this sketch is on a node where you can afford the power to keep the radio awake all the time
  //   you can make it receive messages and also make it wirelessly programmable
  //   otherwise this section can be removed
  if (radio.receiveDone())
  {
  //  DEBUG('[');DEBUG(radio.SENDERID);DEBUG("] ");
  //  for (byte i = 0; i < radio.DATALEN; i++)
  //    DEBUG((char)radio.DATA[i]);

    //flash.wakeup();
    // wireless programming token check - this only works when radio is kept awake to listen for WP tokens
    //CheckForWirelessHEX(radio, flash, true);

    //first send any ACK to request
    DEBUG("   [RX_RSSI:");DEBUG(radio.RSSI);DEBUG("]");
    if (radio.ACKRequested())
    {
      radio.sendACK();
      DEBUG(" - ACK sent.");
    }
    DEBUGln();
  }
  
  SERIALFLUSH();
  if (flashExists) flash.sleep();
  radio.sleep(); //you can comment out this line if you want this node to listen for wireless programming requests
  LowPower.powerDown(sleepTime, ADC_OFF, BOD_OFF);
  DEBUGln("WAKEUP");
}

void readBattery()
{
  #ifndef BANDGAP_VOLTAGE
    readBatteryVoltageDivider();
  #else
    readBatteryBandGap();
  #endif
}

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
  //if (batteryVolts <= BATT_LOW) BATstr = "LOW";
}

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
