#define TEMPHUMIDITY            //uncomment if DHT sensor is present to report temp/humidity/
#include <RFM69.h>         //get it here: http://github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>
#include <RFM69_OTA.h>
#include <SPIFlash.h>      //get it here: http://github.com/lowpowerlab/spiflash
#include <SPI.h>           //comes with Arduino IDE (www.arduino.cc)

#ifdef TEMPHUMIDITY
  #include "DHT.h"
#endif

//*****************************************************************************************************************************
// ADJUST THE SETTINGS BELOW DEPENDING ON YOUR HARDWARE/SITUATION!
//*****************************************************************************************************************************
#define GATEWAYID       10
#define NODEID          13
#define NETWORKID       100
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY       RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY      "***************" //has to be same 16 characters/bytes on all nodes, not more not less!
//#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define LED_PIN         9   //pin connected to onboard LED
#define SUMP_PIN        A0
#define HEATER_PIN      A1
#define BUZZER_PIN      4

#define SERIAL_BAUD        115200
#define SERIAL_EN                //comment out if you don't want any serial output

#define TEMPHUMIDITYSENDDELAY   600000 // send Temperature and Humidity data every 10 min
#define SENSOR_READ_DELAY       5000    // 5 seconds
#define BLINK_DELAY             20000   // 20 seconds
#define NOTIFICATION_DELAY      600000  // 10 minutes
#define ATC_RSSI                -85     //target RSSI for RFM69_ATC (recommended > -80)


#ifdef SERIAL_EN
  #define DEBUG(input)   {Serial.print(input); delay(1);}
  #define DEBUGln(input) {Serial.println(input); delay(1);}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
#endif

#ifdef TEMPHUMIDITY
  #define DHTTYPE DHT21   // DHT 21 (AM2301)
  #define DHTPIN 17        // Pin used to read temp and humidity
  DHT dht(DHTPIN, DHTTYPE);
#endif

//function prototypes
unsigned long TimeSince(unsigned long timeStamp);
void Blink(byte pin, int blinkDelay);
void SendNotification(char* currStatus, char* prevStstus, byte gateway);
boolean WaterExists(byte pin);

RFM69_ATC radio;
/////////////////////////////////////////////////////////////////////////////
// flash(SPI_CS, MANUFACTURER_ID)
// SPI_CS          - CS pin attached to SPI flash chip (8 in case of Moteino)
// MANUFACTURER_ID - OPTIONAL, 0xEF30 for windbond 4mbit flash (Moteino OEM)
/////////////////////////////////////////////////////////////////////////////
SPIFlash flash(8, 0xEF30);

unsigned long sensorReadTimestamp = 0;
unsigned long blinkTimestamp = 0;
unsigned long notificationTimestamp = 0;
unsigned long lastTempHumSent = 0;
boolean waterDetected = false;
boolean reportStatusRequest = false;
char currStatus[10];
char prevStatus[10];
char sendBuf[30];

void setup(void)
{
#ifdef SERIAL_EN  
  Serial.begin(SERIAL_BAUD);
#endif

  pinMode(BUZZER_PIN, INPUT);
  
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  radio.encrypt(ENCRYPTKEY);

  radio.enableAutoPower(ATC_RSSI);
  radio.writeReg(0x58, 0x2D); // High sensitivity
  
  char buff[50];
  sprintf(buff, "WaterMote : %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(buff);

  if (flash.initialize())
  {
    DEBUGln("SPI Flash Init OK!");
  }
  else 
  {
    DEBUGln("SPI Flash Init FAIL! (is chip present?)");
  }
  
  // Initialize prevStstus to a value
  strcpy(prevStatus, "START");

#ifdef TEMPHUMIDITY
  //initialize weather shield sensors  
  dht.begin();
#endif
}


void loop()
{
  if (TimeSince(sensorReadTimestamp) > SENSOR_READ_DELAY)
  {
    DEBUGln("Checking for Water");
    waterDetected = false;
    
    if (WaterExists(SUMP_PIN, 10))
    {
      strcpy(currStatus, "SUMP");
      waterDetected = true;
    }

    if (WaterExists(HEATER_PIN, 300))
    {
      strcpy(currStatus, "HEATER");
      waterDetected = true;
    }
    
    // Notify gateway everything is OK.
    // Acts as heartbeat for node.
    if (!waterDetected)
    {
      strcpy(currStatus, "OK");
    }
    
    if (TimeSince(notificationTimestamp) > NOTIFICATION_DELAY ||
        strcmp(currStatus, prevStatus) != 0)
    {
      SendNotification(currStatus, GATEWAYID);
      notificationTimestamp = millis();
      strcpy(prevStatus, currStatus);
    }
    
    sensorReadTimestamp = millis();
  }

  if (TimeSince(blinkTimestamp) > BLINK_DELAY)
  {
    Blink(LED_PIN, 20);
    blinkTimestamp = millis();
  }
  
  if (radio.receiveDone())
  {
    DEBUG('[');DEBUG(radio.SENDERID);DEBUG("] ");
    for (byte i = 0; i < radio.DATALEN; i++)
      DEBUG((char)radio.DATA[i]);

    if (radio.DATALEN==3)
    {
      if (radio.DATA[0]=='S' && radio.DATA[1]=='T' && radio.DATA[2]=='S')
      {
        reportStatusRequest = true;
      }
      
      if (radio.DATA[0]=='T' && radio.DATA[1]=='M' && radio.DATA[2]=='P')
      {
        reportStatusRequest = true;
      }
    }

    // wireless programming token check
    // DO NOT REMOVE, or Leak Mote will not be wirelessly programmable any more!
    CheckForWirelessHEX(radio, flash, true);

    //first send any ACK to request
    DEBUG("   [RX_RSSI:");DEBUG(radio.RSSI);DEBUG("]");
    if (radio.ACKRequested())
    {
      radio.sendACK();
      DEBUG(" - ACK sent.");
    }
    
    DEBUGln();

    if (reportStatusRequest)
    {
      SendNotification(currStatus, GATEWAYID);
    }
}
  
  // Turn buzzer on/off
  digitalWrite(BUZZER_PIN, waterDetected); 
  
#ifdef TEMPHUMIDITY
  if ((TimeSince(lastTempHumSent) > TEMPHUMIDITYSENDDELAY) ||
      reportStatusRequest)
  {
    lastTempHumSent = millis();
    float temp = ((dht.readTemperature() * 1.8) + 32) * 100.0;
    sprintf(sendBuf, "F:%d H:%d X:%d", (int)temp, (int)dht.readHumidity(), radio._transmitLevel);
    DEBUGln(sendBuf);
    byte sendLen = strlen(sendBuf);
    radio.send(GATEWAYID, sendBuf, sendLen);
    reportStatusRequest = false;
  }
#endif
}

boolean WaterExists(byte pin, int threshold)
{
  int value = analogRead(pin);

  DEBUG("Analog value: ");
  DEBUGln(value);
  
  return value > threshold;
}

unsigned long TimeSince(unsigned long timeStamp)
{
  unsigned long now = millis();
  
  
  if (now >= timeStamp)
    return now - timeStamp;
  else
    return ((unsigned long)(-1)) - timeStamp + now;
}

void SendNotification(char* status, byte gateway)
{
  if (radio.sendWithRetry(gateway, status, strlen(currStatus), 3, 75))
  {
    delay(500);
    Blink(LED_PIN, 20);
  }
}

void Blink(byte pin, int blinkDelay)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(blinkDelay);
  digitalWrite(pin, LOW);
}
