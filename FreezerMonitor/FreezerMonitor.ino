// **********************************************************************************************************
// WeatherShield sketch that works with Moteinos equipped with RFM69W/RFM69HW and WeatherShield
// It sends periodic highly accurate weather readings (temp, hum, atm pressure) from the
//      WeatherShield to the base node/gateway Moteino
// Can be adapted to use Moteinos/Arduinos using RFM12B or other RFM69 variants (RFM69CW, RFM69HCW)
// For use with MoteinoMEGA you will have to revisit the pin definitions defined below
// http://www.LowPowerLab.com/WeatherShield
// Used in this project: http://lowpowerlab.com/blog/2015/07/24/attic-fan-cooling-tests/
// 2015-07-23 (C) Felix Rusu of http://www.LowPowerLab.com/
// **********************************************************************************************************
// License
// **********************************************************************************************************
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
// You should have received a copy of the GNU General    
// Public License along with this program.
// If not, see <http://www.gnu.org/licenses/>.
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// ***************************************************************************************************************************
//#define BMP180
#include <RFM69.h>         //get it here: http://github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>
#include <SPIFlash.h>      //get it here: http://github.com/lowpowerlab/spiflash
#include <SPI.h>           //comes with Arduino
#ifdef BMP180
#include <SFE_BMP180.h>    //get it here: https://github.com/LowPowerLab/SFE_BMP180
#endif
#include <SI7021.h>        //get it here: https://github.com/LowPowerLab/SI7021
#include <Wire.h>          //comes with Arduino
#include <LowPower.h> //get library from: https://github.com/lowpowerlab/lowpower
                      //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/

//*****************************************************************************************************************************
// ADJUST THE SETTINGS BELOW DEPENDING ON YOUR HARDWARE/TRANSCEIVER SETTINGS/REQUIREMENTS
//*****************************************************************************************************************************
#define GATEWAYID   10
#define NODEID      17
#define NETWORKID   100
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY       RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY      "****************" //has to be same 16 characters/bytes on all nodes, not more not less!
//#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI       -75  //target RSSI for RFM69_ATC (recommended > -80)
#define SEND_LOOPS   72 //send data this many sleep loops (15 loops of 8sec cycles = 120sec ~ 2 minutes)
//*********************************************************************************************
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
#define BATT_READ_LOOPS  SEND_LOOPS*5  // read and report battery voltage every this many sleep cycles (ex 30cycles * 8sec sleep = 240sec/4min). For 450 cycles you would get ~1 hour intervals between readings
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

//global program variables
SI7021 weatherShield_SI7021;
#ifdef BMP180
SFE_BMP180 weatherShield_BMP180;
#endif

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

char Pstr[10];
char buffer[50];
SPIFlash flash(8, 0xEF30); //WINDBOND 4MBIT flash chip on CS pin D8 (default for Moteino)
  
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
  
#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
#else
  radio.setPowerLevel(0);
#endif
  
  radio.encrypt(ENCRYPTKEY);
  radio.writeReg(0x58, 0x2D); // High sensitivity

  sprintf(buffer, "WeatherMote - transmitting at: %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(buffer);

  if (flash.initialize())
  {
    DEBUGln(F("SPI Flash Init OK!"));
    flash.sleep();
  }
  else
  {
    DEBUGln(F("SPI Flash Init FAIL! (is chip present?)"));
  }

  //initialize weather shield sensors  
  weatherShield_SI7021.begin();
#ifdef BMP180
  if (weatherShield_BMP180.begin())
  { DEBUGln("BMP180 init success"); }
  else { DEBUGln("BMP180 init fail\n"); }
#endif

#ifdef BLINK_EN
  Blink(LED, 100);Blink(LED, 100);Blink(LED, 100);
#endif

  // Set unused pins to output to save power
  for (uint8_t i=0; i<=A5; i++)
  {
    if (i == RF69_SPI_CS) continue;
    if (i == FLASH_SS) continue;
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }
  
  SERIALFLUSH();
  readBattery();

  radio.sendWithRetry(GATEWAYID, "START", 6);
}

char input=0;
double P;
short sendLoops=0;  //SEND_LOOPS-1;
short battReadLoops=0;
float batteryVolts = 5.0;
float prevBatteryVolts = 0;
char* BATstr="BAT:5.00v"; //longest battery voltage reading message = 9chars
byte sendLen;
bool heater = false;

void loop()
{
  if (battReadLoops--<=0) //only read battery every BATT_READ_LOOPS cycles
  {
    readBattery();
    battReadLoops = BATT_READ_LOOPS-1;
  }
  
  if (sendLoops--<=0)   //send readings every SEND_LOOPS
  {
    sendLoops = SEND_LOOPS-1;
#ifdef BMP180
    P = getPressure();
    P*=0.0295333727; //transform to inHg
    dtostrf(P, 3,2, Pstr);
    sprintf(buffer, "BAT:%sv F:%d H:%d P:%s", BATstr, weatherShield_SI7021.getFahrenheitHundredths(), weatherShield_SI7021.getHumidityPercent(), Pstr);
#else
    if (batteryVolts != prevBatteryVolts)
    {
      sprintf(buffer, "BAT:%sv F:%d H:%d", BATstr, weatherShield_SI7021.getFahrenheitHundredths(), weatherShield_SI7021.getHumidityPercent());
      prevBatteryVolts = batteryVolts;
    }
    else
    {
      sprintf(buffer, "F:%d H:%d", weatherShield_SI7021.getFahrenheitHundredths(), weatherShield_SI7021.getHumidityPercent());
    }
#endif

    sendLen = strlen(buffer);
    DEBUG(buffer); DEBUG(" (packet length:"); DEBUG(sendLen); DEBUGln(")");
    radio.sendWithRetry(GATEWAYID, buffer, sendLen, 3, 75); //retry one time

    #ifdef BLINK_EN
      Blink(LED, 5);
    #endif

    // Turn the heater on if the humidity
    // is 80% or above
    if (weatherShield_SI7021.getHumidityPercent() >= 80) { heater = true; }
    else { heater = false; }
    
    weatherShield_SI7021.setHeater(heater);
  }

  SERIALFLUSH();
  radio.sleep(); //you can comment out this line if you want this node to listen for wireless programming requests
  LowPower.powerDown(sleepTime, ADC_OFF, BOD_OFF);
  DEBUGln("WAKEUP");
}

#ifdef BMP180
double getPressure()
{
  char status;
  double T,P,p0,a;
  // If you want sea-level-compensated pressure, as used in weather reports,
  // you will need to know the altitude at which your measurements are taken.
  // We're using a constant called ALTITUDE in this sketch:
  
  // If you want to measure altitude, and not pressure, you will instead need
  // to provide a known baseline pressure. This is shown at the end of the sketch.
  // You must first get a temperature measurement to perform a pressure reading.
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.
  status = weatherShield_BMP180.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.
    status = weatherShield_BMP180.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.
      status = weatherShield_BMP180.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.
        status = weatherShield_BMP180.getPressure(P,T);
        if (status != 0)
        {
          return P;
        }
      }
    }        
  }
  return 0;
}
#endif

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
