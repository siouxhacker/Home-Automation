#include <RFM69.h>
#include <RFM69_ATC.h>
#include <RFM69_OTA.h>
#include <SPI.h>
#include <SPIFlash.h>
#include <DHT.h>
#include <Wire.h>
#include <DS1307RTC.h>
#include <Time.h>        //http://www.arduino.cc/playground/Code/Time
#include <Timezone.h>    //https://github.com/JChristensen/Timezone
#include <LiquidCrystal.h>
#include "EnvironmentData.h"

#define MOTION_PIN         3 //hardware interrupt 1 (D3)
#define DHT_PIN            4 // Temp & Humidity
#define SPI_CLK_PIN        5
#define SPI_DAT_PIN        6
#define SPI_LAT_PIN        7
#define LED_PIN            9
#define BACK_LIGHT_TIME    30000
#define SENSOR_READ_DELAY  300000

#define NODEID        1
#define WIND_NODEID   3
#define RN_NODEID     20
#define PIGATEWAYID   10
#define NETWORKID     100
#define FREQUENCY     RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY    "****************" //has to be same 16 characters/bytes on all nodes, not more not less!

#define SERIAL_BAUD 115200

#define DISPLAY_LINE_MAX 9
#define LCD_ROW_MAX 4
#define LCD_COLUMN_MAX 20

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11 
#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

RFM69_ATC radio;
DHT dht(DHT_PIN, DHTTYPE);

//US Central Time Zone (Chicago, Minnesota)
TimeChangeRule cdt = {"CDT", Second, Sun, Mar, 2, -300};    //Daylight time = UTC - 5 hours
TimeChangeRule cst = {"CST", First, Sun, Nov, 2, -360};     //Standard time = UTC - 6 hours
Timezone timeZone(cdt, cst);
TimeChangeRule *tcr;        //pointer to the time change rule, use to get TZ abbrev

char DisplayStrs[DISPLAY_LINE_MAX+1][LCD_COLUMN_MAX+1];
volatile unsigned long backLigtStartTime = 0;
unsigned long sensorReadStartTime = SENSOR_READ_DELAY * -1;
//WeatherData remoteWeather;
THPData remoteTHP;
WindData remoteWind;
RainData remoteRn;
unsigned long scrollTimeout = 0;
unsigned int displayLine = 0;
float h, t, batLvl;
int rssi = 0;
time_t lastTX;
short resetDisplayCount = 0;
char sendBuf[15];

// Connect via SPI. Data pin, Clock, Latch
LiquidCrystal lcd(SPI_DAT_PIN, SPI_CLK_PIN, SPI_LAT_PIN);

/////////////////////////////////////////////////////////////////////////////
// flash(SPI_CS, MANUFACTURER_ID)
// SPI_CS          - CS pin attached to SPI flash chip (8 in case of Moteino)
// MANUFACTURER_ID - OPTIONAL, 0x1F44 for adesto(ex atmel) 4mbit flash
//                             0xEF30 for windbond 4mbit flash
//                             0xEF40 for windbond 16/64mbit flash
/////////////////////////////////////////////////////////////////////////////
SPIFlash flash(8, 0xEF30); //EF30 for windbond 4mbit flash

void setup()
{
  // Cannot use interrupts w/ LCD display
  //attachInterrupt(MOTION_PIN, ButtonIRQ, RISING);
  pinMode(MOTION_PIN, INPUT);
  
  dht.begin();
  
  setSyncProvider(RTC.get);

  Serial.begin(SERIAL_BAUD);
  delay(50);
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  radio.setHighPower(); //uncomment only for RFM69HW!
  radio.setPowerLevel(30); // Lower power so motion detector is not set off
  radio.encrypt(ENCRYPTKEY);
  radio.writeReg(0x58, 0x2D); // High sensitivity

  char buff[50];
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
  
  if (flash.initialize())
    Serial.println(F("SPI Flash Init OK!"));
  else
    Serial.println(F("SPI Flash Init FAIL! (is chip present?)"));

  // set up the LCD's number of rows and columns: 
  lcd.begin(LCD_COLUMN_MAX, LCD_ROW_MAX);
  lcd.setBacklight(LOW);

  radio.sendWithRetry(PIGATEWAYID, "START", 6);
  
  delay(5000);
}


void loop()
{
  if (TimeSince(sensorReadStartTime) > SENSOR_READ_DELAY)
  {
    //Serial.println(F("Read Sensor"));
    
    if (!ReadTempAndHum(&t, &h))
    {
      lcd.setCursor(0, 0);
      lcd.print(F("Read error DHT"));
    }
    else
    {
      sensorReadStartTime = millis();
      float temp = t * 100.0;
      sprintf(sendBuf, "F:%d H:%d", (int)temp, (int)h);

      // Send the data
      radio.sendWithRetry(PIGATEWAYID, sendBuf, strlen(sendBuf), 3, 50);
    }

    resetDisplayCount++;

    if (resetDisplayCount > 100)
    {
      // set up the LCD's number of rows and columns: 
      lcd.begin(LCD_COLUMN_MAX, LCD_ROW_MAX);
      resetDisplayCount = 0;
    }
  }
  
  GetRemoteWeather();

  if (TimeSince(scrollTimeout) > 5000)
  {
    FormatTempAndHum(t, h, "In", DisplayStrs[0]);
    FormatTempAndHum(remoteTHP.temperature, remoteTHP.humidity, "Out", DisplayStrs[1]);
    FormatWindSpeedAndDirection(remoteWind.windSpeed, remoteWind.windDir, DisplayStrs[2]);
    
    if (remoteTHP.temperature> 75)
    {
      double hi = ComputeHeatIndex(remoteTHP.temperature, remoteTHP.humidity);
      ChillAndPressure(hi, remoteTHP.pressure, "HI", DisplayStrs[3]);
    }
    else
    {
      double wc = ComputeWindChill(remoteWind.windSpeed, remoteTHP.temperature);
      ChillAndPressure(wc, remoteTHP.pressure, "WC", DisplayStrs[3]);
    }
    
    GetRemoteWeather();
    
    FormatDewPoint(ComputeDewPoint(remoteTHP.temperature, remoteTHP.humidity),
                   DisplayStrs[4]);
    FormatRainfall(remoteRn.rainAmount, DisplayStrs[5]);
    FormatDateTime(timeZone.toLocal(now(), &tcr), DisplayStrs[6]);
    FormatBatteryLvl(batLvl, DisplayStrs[7]);
    FormatRSSI(rssi, DisplayStrs[8]);
    FormatLastTX(lastTX, DisplayStrs[9]);
    
    GetRemoteWeather();

    DisplayWeatherData(displayLine++);

    GetRemoteWeather();

    if (displayLine > DISPLAY_LINE_MAX)
      displayLine = 0;
    
    scrollTimeout = millis();
  }
  
  if (TimeSince(backLigtStartTime) > BACK_LIGHT_TIME)
    lcd.setBacklight(LOW);

  ReadMotionPin(MOTION_PIN);
 
  GetRemoteWeather();
  
  // Check for existing RF data, potentially for a new sketch wireless upload
  // For this to work this check has to be done often enough to be
  // picked up when a GATEWAY is trying hard to reach this node for a new sketch wireless upload
  if (radio.receiveDone())
  {
    CheckForWirelessHEX(radio, flash, true);
  }
}

void DisplayWeatherData(unsigned int startPos)
{
  for (int i = 0; i < LCD_ROW_MAX; i++)
  {
    ClearLcd(i);
    PrintLcd(DisplayStrs[startPos++], i);
    
    if (startPos > DISPLAY_LINE_MAX)
      startPos = 0;
      
    GetRemoteWeather();
  }
}

boolean GetRemoteWeather()
{
  boolean gotData = false;

  if (radio.receiveDone())
  {
    noInterrupts();  // this is necessary to prevent loss of data while copying
    rssi = radio.RSSI;

    if (radio.DATALEN >= sizeof(CommonHdr))
    {
      batLvl = ((CommonHdr*)radio.DATA)->batLvl;
    }
  
    if (radio.DATALEN == sizeof(remoteTHP) &&
        ((CommonHdr*)radio.DATA)->type == THP)
    {
      memcpy(&remoteTHP, (THPData*) radio.DATA, min(sizeof(remoteTHP), radio.DATALEN));
      gotData = true;     
    }
    else if (radio.DATALEN == sizeof(remoteWind) &&
             ((CommonHdr*)radio.DATA)->type == WIND)
    {
      memcpy(&remoteWind, (WindData*) radio.DATA, min(sizeof(remoteWind), radio.DATALEN));
      gotData = true;
    }
    else if (radio.DATALEN == sizeof(remoteRn) &&
             ((CommonHdr*)radio.DATA)->type == RN)
    {
      memcpy(&remoteRn, (RainData*) radio.DATA, min(sizeof(remoteRn), radio.DATALEN));
      gotData = true;
    }

    interrupts();
    
    if (radio.ACKRequested())
    {
      if (hour(lastTX) == 0 && minute(lastTX) > 0 && minute(lastTX) < 20 &&
          (radio.SENDERID == RN_NODEID || radio.SENDERID == WIND_NODEID))
      {
        radio.sendACK("RST", 3);
      }
      else
      {
        radio.sendACK();
      }
    }

    if (gotData)
    {
      lastTX = timeZone.toLocal(now(), &tcr);
      //Serial.print(" nodeId=");
      //Serial.print(remoteWeather.nodeId);
      //Serial.print(" temp=");
      //Serial.print(remoteWeather.temp);
      //Serial.print(" hum=");
      //Serial.print(remoteWeather.hum);
      //Serial.print(" press=");
      //Serial.print(remoteWeather.pressure);
      //Serial.print(" ws=");
      //Serial.print(remoteWeather.windSpeed);
      //Serial.print(" wd=");
      //Serial.print(remoteWeather.windDir);
      //Serial.print(" rain=");
      //Serial.print(remoteWeather.rainAmount);
      //Serial.print(" batLvl=");
      //Serial.print(remoteWeather.batLvl);
      //Serial.print(" RemoteRSSI=");
      //Serial.print(remoteWeather.rssi);
      //Serial.print(" RSSI=");
      //Serial.print(rssi);
    }

    //Serial.println();

    Blink(LED_PIN, 20);
  }
  
  return gotData;
}

void PrintLcd(char* str, int row)
{
  lcd.setCursor(0, row);
  lcd.print(str);
}

void ClearLcd(int row)
{
  lcd.setCursor(0, row);
  lcd.print(F("                    "));
}

void FormatTempAndHum(float t, float h, const char* src, char* str)
{
  char tempBuf[10];
  char humBuf[10];

  dtostrf(t, 4, 1, tempBuf);
  dtostrf(h, 4, 1, humBuf);
  
  sprintf(str, "%s: %s*F %s%%", src, tempBuf, humBuf);
}

void FormatWindSpeedAndDirection(float ws, char* wd, char* str)
{
  char wsBuf[10];
 
  dtostrf(ws, 4, 1, wsBuf);
  sprintf(str, "WS: %s D: %s", wsBuf, wd);
}

void ChillAndPressure(double c, float pressure, const char* src, char* str)
{
  char cBuf[10];
  char pressBuf[10];
  
  dtostrf(c, 3, 0, cBuf);
  dtostrf(pressure, 4, 1, pressBuf);
  sprintf(str, "%s: %s P: %s", src, cBuf, pressBuf);
}

void FormatDewPoint(float dewPoint, char* str)
{
  char dpBuf[10];
  //char rfBuf[10];
  
  dtostrf(dewPoint, 3, 0, dpBuf);
  //dtostrf(rainAmount, 2, 2, rfBuf);
  sprintf(str, "Dew Point: %s", dpBuf);
}

void FormatRainfall(float rainAmount, char* str)
{
  char rfBuf[10];
 
  dtostrf(rainAmount, 2, 2, rfBuf);
  sprintf(str, "Rainfall: %s", rfBuf);
}

void FormatDateTime(time_t t, char* str)
{
  char dateBuf[11];
  char timeBuf[9];

  FormatDate(t, dateBuf);
  FormatTime(t, timeBuf);
  
  sprintf(str, "%s %s", dateBuf, timeBuf);
}

void FormatLastTX(time_t t, char* str)
{
  char timeBuf[9];
  
  FormatTime(t, timeBuf);

  sprintf(str, "Last TX: %s", timeBuf);
}

void FormatDate(time_t t, char* str)
{
  char dayBuf[3];
  char monthBuf[3];
  
  if (day(t) < 10)
    sprintf(dayBuf, "0%d", day(t));
  else
    sprintf(dayBuf, "%d", day(t));
    
  if (month(t) < 10)
    sprintf(monthBuf, "0%d", month(t));
  else
    sprintf(monthBuf, "%d", month(t));
    
  sprintf(str, "%s/%s/%d", monthBuf, dayBuf, year(t));
}

void FormatTime(time_t t, char* str)
{
  char hrBuf[3];
  char minBuf[3];
  const char* ampmBuf;
    
  if (hourFormat12(t) < 10)
    sprintf(hrBuf, "0%d", hourFormat12(t));
  else
    sprintf(hrBuf, "%d", hourFormat12(t));
    
  if (minute(t) < 10)
    sprintf(minBuf, "0%d", minute(t));
  else
    sprintf(minBuf, "%d", minute(t));
    
  if (isAM(t))
    ampmBuf = "am";
  else
    ampmBuf = "pm";
    
  sprintf(str, "%s:%s %s", hrBuf, minBuf, ampmBuf);
}

void FormatBatteryLvl(float battLvl, char* str)
{
  char btBuf[10];
  
  dtostrf(battLvl, 2, 2, btBuf);
  sprintf(str, "Battery Lvl: %s", btBuf);
}

void FormatRSSI(int rssi, char* str)
{
  sprintf(str, "RSSI: %i", rssi);
}

boolean ReadTempAndHum(float *t, float *h)
{
  boolean gotData = true;

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  *h = dht.readHumidity() ;
  *t = dht.readTemperature();
  
  // check if returns are valid, if they are NaN (not a number)
  // then something went wrong!
  if (isnan(*t) || isnan(*h))
  {
    gotData = false;
  }
  else
  {
    *t = (1.8 * *t) + 32.00;
  }
  
  return gotData;
}

void ReadMotionPin(int pin)
{
  if (digitalRead(pin) == HIGH)
  {
    backLigtStartTime = millis();
    lcd.setBacklight(HIGH);
  }
}

double ComputeWindChill(float ws, float temp)
{
  double wc = 35.74 + (0.6215 * temp) - (35.75 * pow( ws, 0.16)) +
              (0.4275 * temp * pow(ws, 0.16));
  
  if (wc > temp) wc = temp;
  
  return wc;
}

double ComputeHeatIndex(float t, float h)
{
  return -42.379 + (2.04901523 * t) + (10.14333127 * h) - (0.22475541 * t * h) - (0.00683783 * t * t)
         - (0.05481717 * h * h) + (0.00122874 * t * t * h) + (0.00085282 * t * h * h)
         - (0.00000199 * t * t * h * h);
}

float ComputeDewPoint(float temp, float hum)
{
  return temp - (9 * (100.0 - hum) / 25);
}

void Blink(int pin, int blinkDelay)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(blinkDelay);
  digitalWrite(pin, LOW);
}

unsigned long TimeSince(unsigned long timeStamp)
{
  unsigned long now = millis();
    
  if (now >= timeStamp)
    return now - timeStamp;
  else
    return ((unsigned long)(-1)) - timeStamp + now;
}
