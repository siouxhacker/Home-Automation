#include <RFM69.h>
#include <RFM69_ATC.h>
#include <SPIFlash.h>
#include <SPI.h>
#include <LowPower.h>

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

#define NODEID      21
#define NETWORKID   100
#define PIGATEWAYID 10
#define FREQUENCY   RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY  "" //has to be same 16 characters/bytes on all nodes, not more not less!
//#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
//*********************************************************************************************
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -80
//*********************************************************************************************
#define SEND_LOOPS    450 //send data this many sleep loops (15 loops of 8sec cycles = 120sec ~ 2 minutes)

#define BATT_FORMULA(reading) reading * 0.00322 * 2.95  // >>> fine tune this parameter to match your voltage when fully charged
#define BATT_MONITOR  A7   // through 2Meg+1Meg and 0.1uF cap from battery VCC - this ratio divides the voltage to bring it below 3.3V where it is scaled to a readable range

#define REFERENCE_VOLTAGE 3.3
#define MOISTURE_PIN      A0
#define PWR_PIN           5 // Used to power soil moisture sensor

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

struct VH400
{
  double analogValue;
  double analogValue_sd;
  double voltage;
  double voltage_sd;
  double VWC;
  double VWC_sd;
};

SPIFlash flash(FLASH_SS, 0xEF30); //WINDBOND 4MBIT flash chip on CS pin D8 (default for Moteino)

struct VH400 result;

char buffer[50];

void setup()
{
  // Set unused pins to output to save power
  for (uint8_t i = 0; i <= A5; i++)
  {
    if (i == RF69_SPI_CS) continue;
    if (i == MOISTURE_PIN) continue;
    if (i == BATT_MONITOR) continue;
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  pinMode(MOISTURE_PIN, INPUT);
  pinMode(PWR_PIN, OUTPUT);

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

  sprintf(buffer, "SoilMoistureMote - transmitting at: %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(buffer);

  if (flash.initialize())
  {
    flash.sleep();
    DEBUGln(F("SPI Flash Init OK!"));
  }
  else
  {
    DEBUGln(F("SPI Flash Init FAIL! (is chip present?)"));
  }
  
  radio.sendWithRetry(PIGATEWAYID, "START", 6);

  // Allow cap to charge
  delay(3000);
}

short sendLoops = 0;
float batteryVolts = 5;
char* BATstr="BAT:5.00v"; //longest battery voltage reading message = 9chars
char VWCstr[10];

void loop()
{
  if (sendLoops-- <= 0)   //send readings every SEND_LOOPS
  {
    sendLoops = SEND_LOOPS - 1;

    // Power up the moisture sensor
    digitalWrite(PWR_PIN, HIGH);
    delay(5000);
    float vwc = ReadVH400(MOISTURE_PIN);
    digitalWrite(PWR_PIN, LOW);
    
    dtostrf(vwc, 3, 1, VWCstr);
    readBattery();
    
    sprintf(buffer, "BAT:%sv VWC:%s", BATstr, VWCstr);
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
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  DEBUGln("WAKEUP8s");
}


// MIT License (MIT)
// 
// Copyright (c) 2015. Michael Ewald, GeomaticsResearch LLC.
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// Author: Michael Ewald (mewald@geomaticsresearch.com)
// Web: https://GeomaticsResearch.com
// Last-updated: 2015-07-04
float ReadVH400(int analogPin)
{
  // This function returns Volumetric Water Content by converting the analogPin value to voltage
  // and then converting voltage to VWC using the piecewise regressions provided by the manufacturer
  // at http://www.vegetronix.com/Products/VH400/VH400-Piecewise-Curve.phtml
  
  // NOTE: You need to set analogPin to input in your setup block
  //   ex. pinMode(<analogPin>, INPUT);
  //   replace <analogPin> with the number of the pin you're going to read from.
  
  // Read value and convert to voltage  
  int sensor1DN = analogRead(analogPin);
  float sensorVoltage = sensor1DN * (REFERENCE_VOLTAGE / 1023.0);
  float VWC;
  
  // Calculate VWC
  if (sensorVoltage <= 1.1)
  {
    VWC = 10 * sensorVoltage - 1;
  }
  else if(sensorVoltage > 1.1 && sensorVoltage <= 1.3)
  {
    VWC = 25 * sensorVoltage - 17.5;
  }
  else if(sensorVoltage > 1.3 && sensorVoltage <= 1.82)
  {
    VWC = 48.08 * sensorVoltage - 47.5;
  }
  else if(sensorVoltage > 1.82)
  {
    VWC = 26.32 * sensorVoltage - 7.89;
  }
  return(VWC);
}


struct VH400 ReadVH400_wStats(int analogPin,
                              int nMeasurements = 100,
                              int delayBetweenMeasurements = 50)
{
  // This variant calculates the mean and standard deviation of 100 measurements over 5 seconds.
  // It reports mean and standard deviation for the analog value, voltage, and WVC.
  
  // This function returns Volumetric Water Content by converting the analogPin value to voltage
  // and then converting voltage to VWC using the piecewise regressions provided by the manufacturer
  // at http://www.vegetronix.com/Products/VH400/VH400-Piecewise-Curve.phtml
  
  // NOTE: You need to set analogPin to input in your setup block
  //   ex. pinMode(<analogPin>, INPUT);
  //   replace <analogPin> with the number of the pin you're going to read from.

  
  // Sums for calculating statistics
  int sensorDNsum = 0;
  double sensorVoltageSum = 0.0;
  double sensorVWCSum = 0.0;
  double sqDevSum_DN = 0.0;
  double sqDevSum_volts = 0.0;
  double sqDevSum_VWC = 0.0;

  // Arrays to hold multiple measurements
  int sensorDNs[nMeasurements];
  double sensorVoltages[nMeasurements];
  double sensorVWCs[nMeasurements];

  // Make measurements and add to arrays
  for (int i = 0; i < nMeasurements; i++)
  {
    // Read value and convert to voltage 
    int sensorDN = analogRead(analogPin);
    double sensorVoltage = sensorDN * (REFERENCE_VOLTAGE / 1023.0);
        
    // Calculate VWC
    float VWC;
    if(sensorVoltage <= 1.1) 
    {
      VWC = 10 * sensorVoltage - 1;
    }
    else if(sensorVoltage > 1.1 && sensorVoltage <= 1.3)
    {
      VWC = 25 * sensorVoltage - 17.5;
    }
    else if(sensorVoltage > 1.3 && sensorVoltage <= 1.82)
    {
      VWC = 48.08 * sensorVoltage - 47.5;
    }
    else if(sensorVoltage > 1.82)
    {
      VWC = 26.32 * sensorVoltage - 7.89;
    }

    // Add to statistics sums
    sensorDNsum += sensorDN;
    sensorVoltageSum += sensorVoltage;
    sensorVWCSum += VWC;

    // Add to arrays
    sensorDNs[i] = sensorDN;
    sensorVoltages[i] = sensorVoltage;
    sensorVWCs[i] = VWC;

    // Wait for next measurement
    delay(delayBetweenMeasurements);
  }

  // Calculate means
  double DN_mean = double(sensorDNsum) / double(nMeasurements);
  double volts_mean = sensorVoltageSum / double(nMeasurements);
  double VWC_mean = sensorVWCSum / double(nMeasurements);

  // Loop back through to calculate SD
  for (int i = 0; i < nMeasurements; i++)
  { 
    sqDevSum_DN += pow((DN_mean - double(sensorDNs[i])), 2);
    sqDevSum_volts += pow((volts_mean - double(sensorVoltages[i])), 2);
    sqDevSum_VWC += pow((VWC_mean - double(sensorVWCs[i])), 2);
  }
  
  double DN_stDev = sqrt(sqDevSum_DN / double(nMeasurements));
  double volts_stDev = sqrt(sqDevSum_volts / double(nMeasurements));
  double VWC_stDev = sqrt(sqDevSum_VWC / double(nMeasurements));

  // Setup the output struct
  result.analogValue = DN_mean;
  result.analogValue_sd = DN_stDev;
  result.voltage = volts_mean;
  result.voltage_sd = volts_stDev;
  result.VWC = VWC_mean;
  result.VWC_sd = VWC_stDev;

  // Return the result
  return(result);
}

void readBattery()
{
    readBatteryVoltageDivider();
}

void readBatteryVoltageDivider()
{
  unsigned int readings = 0;
  
  for (byte i=0; i<5; i++) //take several samples, and average
    readings += analogRead(BATT_MONITOR);
    
  batteryVolts = BATT_FORMULA(readings / 5.0);
  dtostrf(batteryVolts,3,2, BATstr); //update the BATStr which gets sent every BATT_CYCLES or along with the MOTION message
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}
