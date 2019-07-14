// **********************************************************************************************************
// GarageMote garage door controller sketch that works with Moteinos equipped with RFM69W/RFM69HW
// Can be adapted to use Moteinos/Arduinos using RFM12B or other RFM69 variants (RFM69CW, RFM69HCW)
// http://www.LowPowerLab.com/GarageMote
// 2015-05-05 (C) Felix Rusu of http://www.LowPowerLab.com/
// **********************************************************************************************************
// It uses 2 hall effect sensors (and magnets mounted on the garage belt/chain) to detect the position of the
// door, and a small signal relay to be able to toggle the garage opener.
// Implementation details are posted at the LowPowerLab blog
// Door status is reported via RFM69 to a base Moteino, and visually on the onboard Moteino LED:
//    - solid ON - door is in open position
//    - solid OFF - door is in closed position
//    - blinking - door is not in either open/close position
//    - pulsing - door is in motion
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
#define LIGHTSENSOR            // uncomment if LDR is attached
//#define WEATHERSHIELD            // uncomment if WeatherShield is present to report temp/humidity/pressure periodically
#define WEATHERSENDDELAY  600000 // send WeatherShield data every so often (ms)
#define LIGHTSENSORDELAY  300000 // send light level every so often
// ***************************************************************************************************************************
#include <RFM69.h>         //get it here: http://github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>
#include <RFM69_OTA.h>
#include <SPIFlash.h>      //get it here: http://github.com/lowpowerlab/spiflash
#include <SPI.h>           //comes with Arduino IDE (www.arduino.cc)

#ifdef WEATHERSHIELD
  #include "DHT.h"
#endif
//*****************************************************************************************************************************
// ADJUST THE SETTINGS BELOW DEPENDING ON YOUR HARDWARE/TRANSCEIVER SETTINGS/REQUIREMENTS
//*****************************************************************************************************************************
#define GATEWAYID   10
#define NODEID      11
#define NETWORKID   100
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY       RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY      "****************" //has to be same 16 characters/bytes on all nodes, not more not less!
//#define IS_RFM69HW      //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI            -85  //target RSSI for RFM69_ATC (recommended > -80)

#define BUTTON              3     // Momentary button pin
#define HALLSENSOR1          A0
#define HALLSENSOR1_EN        4
#define HALLSENSOR2          A1
#define HALLSENSOR2_EN        5
#define LIGHTSENSOR1         A3

#define RELAYPIN1             6
#define RELAYPIN2             7
#define RELAY_PULSE_MS      250  //just enough that the opener will pick it up

#define DOOR_MOVEMENT_TIME 14000 // this has to be at least as long as the max between [door opening time, door closing time]
                                 // my door opens and closes in about 12s
#define STATUS_CHANGE_MIN  1500  // this has to be at least as long as the delay 
                                 // between a opener button press and door movement start
                                 // most garage doors will start moving immediately (within half a second)
#define BUTTON_HOLD_TIME    2000 // Button must be hold this many mseconds before the garage door is actioned
//*****************************************************************************************************************************
#define HALLSENSOR_OPENSIDE   0
#define HALLSENSOR_CLOSEDSIDE 1

#define STATUS_CLOSED        0
#define STATUS_CLOSING       1
#define STATUS_OPENING       2
#define STATUS_OPEN          3
#define STATUS_UNKNOWN       4

#define LED                  9   //pin connected to onboard LED
#define LED_PULSE_PERIOD  5000   //5s seems good value for pulsing/blinking (not too fast/slow)
#define SERIAL_BAUD     115200

#define SERIAL_EN                //comment out if you don't want any serial output

#ifdef SERIAL_EN
  #define DEBUG(input)   {Serial.print(input); delay(1);}
  #define DEBUGln(input) {Serial.println(input); delay(1);}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
#endif

#ifdef WEATHERSHIELD
  #define DHTTYPE DHT21   // DHT 21 (AM2301)
  #define DHTPIN 17        // Pin used to read temp and humidity
  DHT dht(DHTPIN, DHTTYPE);
#endif

//function prototypes
void setStatus(byte newSTATUS, boolean reportStatus=true);
void reportStatus();
boolean hallSensorRead(byte which);
void pulseRelay();

//global program variables
byte STATUS;
unsigned long lastStatusTimestamp=0;
unsigned long ledPulseTimestamp=0;
unsigned long lastWeatherSent=0;
unsigned long lastLightSent=0;
int ledPulseValue=0;
boolean ledPulseDirection=false; //false=down, true=up

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

char sendBuf[30];
SPIFlash flash(8, 0xEF30); //WINDBOND 4MBIT flash chip on CS pin D8 (default for Moteino)

int lastValidReading = 1;
unsigned long lastValidReadingTime = 0;
unsigned long NOW=0;

void setup(void)
{
#ifdef SERIAL_EN
  Serial.begin(SERIAL_BAUD);
#endif
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(HALLSENSOR1, INPUT);
  pinMode(HALLSENSOR2, INPUT);
  pinMode(HALLSENSOR1_EN, OUTPUT);
  pinMode(HALLSENSOR2_EN, OUTPUT);
  pinMode(RELAYPIN1, OUTPUT);
  pinMode(RELAYPIN2, OUTPUT);
  pinMode(LED, OUTPUT);
  
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  
#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
#endif
  
  radio.encrypt(ENCRYPTKEY);
  radio.writeReg(0x58, 0x2D); // High sensitivity

  char buff[50];
  sprintf(buff, "GarageMote : %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(buff);

  if (hallSensorRead(HALLSENSOR_OPENSIDE)==true)
    setStatus(STATUS_OPEN);
  if (hallSensorRead(HALLSENSOR_CLOSEDSIDE)==true)
    setStatus(STATUS_CLOSED);
  else setStatus(STATUS_UNKNOWN);

#ifdef WEATHERSHIELD
  //initialize weather shield sensors  
  dht.begin();
#endif
}

unsigned long doorPulseCount = 0;
char input=0;
boolean reportStatusRequest = false;

void loop()
{
#ifdef SERIAL_EN
  if (Serial.available())
    input = Serial.read();
#endif

  if (input=='r')
  {
    DEBUGln("Relay test...");
    pulseRelay();
    input = 0;
  }

  // Check for button press
  int reading = digitalRead(BUTTON);
  NOW = millis();

  if (reading != lastValidReading && NOW - lastValidReadingTime > 200)
  {
    DEBUGln("Button Pressed");
    
    lastValidReading = reading;
    lastValidReadingTime = NOW;
    
    if (reading == LOW)
    {
      DEBUGln("Button Still Pressed");
      //make sure the button is held down for at least 'BUTTON_HOLD_TIME' before taking action (this is to avoid accidental button presses and consequently Pi shutdowns)
      NOW = millis();
      while (millis() - NOW < BUTTON_HOLD_TIME)
      { 
        delay(10); if (digitalRead(BUTTON) != LOW) return; 
      }

      DEBUGln("Action garage door");
      //BUTTON_HOLD_TIME is satisfied, now action the door.
      pulseRelay();
    }
  }
  
  // UNKNOWN => OPEN/CLOSED
  if (STATUS == STATUS_UNKNOWN && millis()-(lastStatusTimestamp)>STATUS_CHANGE_MIN)
  {
    if (hallSensorRead(HALLSENSOR_OPENSIDE)==true)
      setStatus(STATUS_OPEN);
    if (hallSensorRead(HALLSENSOR_CLOSEDSIDE)==true)
      setStatus(STATUS_CLOSED);
  }

  // OPEN => CLOSING
  if (STATUS == STATUS_OPEN && millis()-(lastStatusTimestamp)>STATUS_CHANGE_MIN)
  {
    if (hallSensorRead(HALLSENSOR_OPENSIDE)==false)
      setStatus(STATUS_CLOSING);
  }

  // CLOSED => OPENING  
  if (STATUS == STATUS_CLOSED && millis()-(lastStatusTimestamp)>STATUS_CHANGE_MIN)
  {
    if (hallSensorRead(HALLSENSOR_CLOSEDSIDE)==false)
      setStatus(STATUS_OPENING);
  }

  // OPENING/CLOSING => OPEN (when door returns to open due to obstacle or toggle action)
  //                 => CLOSED (when door closes normally from OPEN)
  //                 => UNKNOWN (when more time passes than normally would for a door up/down movement)
  if ((STATUS == STATUS_OPENING || STATUS == STATUS_CLOSING) && millis()-(lastStatusTimestamp)>STATUS_CHANGE_MIN)
  {
    if (hallSensorRead(HALLSENSOR_OPENSIDE)==true)
      setStatus(STATUS_OPEN);
    else if (hallSensorRead(HALLSENSOR_CLOSEDSIDE)==true)
      setStatus(STATUS_CLOSED);
    else if (millis()-(lastStatusTimestamp)>DOOR_MOVEMENT_TIME)
      setStatus(STATUS_UNKNOWN);
  }

  reportStatusRequest=false;
  
  if (radio.receiveDone())
  {
    byte newStatus=STATUS;
    DEBUG('[');DEBUG(radio.SENDERID);DEBUG("] ");
    for (byte i = 0; i < radio.DATALEN; i++)
      DEBUG((char)radio.DATA[i]);

    if (radio.DATALEN==3)
    {
      //check for an OPEN/CLOSE/STATUS request
      if (radio.DATA[0]=='O' && radio.DATA[1]=='P' && radio.DATA[2]=='N')
      {
        if (millis()-(lastStatusTimestamp) > STATUS_CHANGE_MIN && (STATUS == STATUS_CLOSED || STATUS == STATUS_CLOSING || STATUS == STATUS_UNKNOWN))
          newStatus = STATUS_OPENING;
        //else radio.Send(requester, "INVALID", 7);
      }
      if (radio.DATA[0]=='C' && radio.DATA[1]=='L' && radio.DATA[2]=='S')
      {
        if (millis()-(lastStatusTimestamp) > STATUS_CHANGE_MIN && (STATUS == STATUS_OPEN || STATUS == STATUS_OPENING || STATUS == STATUS_UNKNOWN))
          newStatus = STATUS_CLOSING;
        //else radio.Send(requester, "INVALID", 7);
      }
      if (radio.DATA[0]=='S' && radio.DATA[1]=='T' && radio.DATA[2]=='S')
      {
        reportStatusRequest = true;
      }
    }
    
    // wireless programming token check
    // DO NOT REMOVE, or GarageMote will not be wirelessly programmable any more!
    CheckForWirelessHEX(radio, flash, true);

    //first send any ACK to request
    DEBUG("   [RX_RSSI:");DEBUG(radio.RSSI);DEBUG("]");
    if (radio.ACKRequested())
    {
      radio.sendACK();
      DEBUG(" - ACK sent.");
    }
    
    //now take care of the request, if not invalid
    if (STATUS != newStatus)
    {
      pulseRelay();
      setStatus(newStatus);
    }
    if (reportStatusRequest)
    {
      reportStatus();
    }
      
    DEBUGln();
  }
  
  //use LED to visually indicate STATUS
  if (STATUS == STATUS_OPEN || STATUS == STATUS_CLOSED) //solid ON/OFF
  {
    digitalWrite(LED, STATUS == STATUS_OPEN ? LOW : HIGH);
  }
  if (STATUS == STATUS_OPENING || STATUS == STATUS_CLOSING) //pulse
  {
    if (millis()-(ledPulseTimestamp) > LED_PULSE_PERIOD/256)
    {
      ledPulseValue = ledPulseDirection ? ledPulseValue + LED_PULSE_PERIOD/256 : ledPulseValue - LED_PULSE_PERIOD/256;

      if (ledPulseDirection && ledPulseValue > 255)
      {
        ledPulseDirection=false;
        ledPulseValue = 255;
      }
      else if (!ledPulseDirection && ledPulseValue < 0)
      {
        ledPulseDirection=true;
        ledPulseValue = 0;
      }
      
      analogWrite(LED, ledPulseValue);
      ledPulseTimestamp = millis();
    }
  }
  if (STATUS == STATUS_UNKNOWN) //blink
  {
    if (millis()-(ledPulseTimestamp) > LED_PULSE_PERIOD/20)
    {
      ledPulseDirection = !ledPulseDirection;
      digitalWrite(LED, ledPulseDirection ? HIGH : LOW);
      ledPulseTimestamp = millis();
    }
  }
  
#ifdef WEATHERSHIELD
  if ((millis()-lastWeatherSent > WEATHERSENDDELAY) || reportStatusRequest)
  {
    lastWeatherSent = millis();
    float temp = ((dht.readTemperature() * 1.8) + 32) * 100.0;
    sprintf(sendBuf, "F:%d H:%d X:%d", (int)temp, (int)dht.readHumidity(), radio._transmitLevel);
    DEBUGln(sendBuf);
    radio.sendWithRetry(GATEWAYID, sendBuf, strlen(sendBuf), 3);
  }
#endif

#ifdef LIGHTSENSOR
  if ((millis() - lastLightSent > LIGHTSENSORDELAY) || reportStatusRequest)
  {
    lastLightSent = millis();

    sprintf(sendBuf, "LL:%d X:%d", GetLightLevel(), radio._transmitLevel);
    DEBUGln(sendBuf);
    radio.sendWithRetry(GATEWAYID, sendBuf, strlen(sendBuf), 3);
  }
#endif
}

//returns TRUE if magnet is next to sensor, FALSE if magnet is away
boolean hallSensorRead(byte which)
{
  //while(millis()-lastStatusTimestamp<STATUS_CHANGE_MIN);
  digitalWrite(which ? HALLSENSOR2_EN : HALLSENSOR1_EN, HIGH); //turn sensor ON
  delay(1); //wait a little
  byte reading = digitalRead(which ? HALLSENSOR2 : HALLSENSOR1);
  digitalWrite(which ? HALLSENSOR2_EN : HALLSENSOR1_EN, LOW); //turn sensor OFF
  return reading==0;
}

void setStatus(byte newSTATUS, boolean reportIt)
{
  if (STATUS != newSTATUS) lastStatusTimestamp = millis();
  STATUS = newSTATUS;
  DEBUGln(STATUS==STATUS_CLOSED ? "CLOSED" : STATUS==STATUS_CLOSING ? "CLOSING" : STATUS==STATUS_OPENING ? "OPENING" : STATUS==STATUS_OPEN ? "OPEN" : "UNKNOWN");
  if (reportIt)
    reportStatus();
}

void reportStatus(void)
{
  char buff[10];
  sprintf(buff, STATUS==STATUS_CLOSED ? "CLOSED" : STATUS==STATUS_CLOSING ? "CLOSING" : STATUS==STATUS_OPENING ? "OPENING" : STATUS==STATUS_OPEN ? "OPEN" : "UNKNOWN");
  byte len = strlen(buff);
  radio.sendWithRetry(GATEWAYID, buff, len, 20, 30);
}

void pulseRelay()
{
  digitalWrite(RELAYPIN1, HIGH);
  digitalWrite(RELAYPIN2, HIGH);
  delay(RELAY_PULSE_MS);
  digitalWrite(RELAYPIN1, LOW);
  digitalWrite(RELAYPIN2, LOW);
}

void Blink(byte PIN, byte DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

#ifdef LIGHTSENSOR
 int GetLightLevel()
 {
   float avgLight = 0;
   float totalLight = 0;
   
      
   for (int i = 0; i < 10; i++)
   {
     totalLight += analogRead(LIGHTSENSOR1);
     delay(100);
   }
   
   avgLight = totalLight / 10;
   
   return avgLight;
 }
 #endif
