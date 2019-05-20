#include <RFM69.h>         //get it here: https://github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://github.com/lowpowerlab/rfm69
#include <SPIFlash.h>      //get it here: https://github.com/lowpowerlab/spiflash
#include <SPI.h>           //included in Arduino IDE (www.arduino.cc)
#include <Wire.h>          //included in Arduino IDE (www.arduino.cc)
#include <LowPower.h>      //get it here: https://github.com/lowpowerlab/lowpower
                           //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/
#include <I2C.h>// The AS3935 communicates via SPI or I2C.
#include <PWFusion_AS3935_I2C.h> // include Playing With Fusion AXS3935 libraries

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE ************
//*********************************************************************************************
#define PIGATEWAYID     10
#define NODEID          22
#define NETWORKID       100
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY       RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY      "" //has to be same 16 characters/bytes on all nodes, not more not less!
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
#define BATT_MONITOR  A7   //through 2Meg+1Meg and 0.1uF cap from battery VCC - this ratio divides the voltage to bring it below 3.3V where it is scaled to a readable range
#define BATT_FORMULA(reading) reading * 0.00322 * 2.95  // >>> fine tune this parameter to match your voltage when fully charged
#define BATT_READ_LOOPS  SEND_LOOPS*2  // read and report battery voltage every this many sleep cycles (ex 30cycles * 8sec sleep = 240sec/4min). For 450 cycles you would get ~1 hour intervals between readings
//*****************************************************************************************************************************

#define SI_PIN               7
#define IRQ_PIN              3        // digital pins 2 and 3 are available for interrupt capability
#define AS3935_ADD           0x03     // x03 - standard PWF SEN-39001-R01 config
#define AS3935_CAPACITANCE   112      // <-- SET THIS VALUE TO THE NUMBER LISTED ON YOUR BOARD 

// defines for general chip settings
#define AS3935_INDOORS       0
#define AS3935_OUTDOORS      1
#define AS3935_DIST_DIS      0
#define AS3935_DIST_EN       1

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

#define BLINK_EN                 //uncomment to blink LED on every send
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

// prototypes
void AS3935_ISR();

PWF_AS3935_I2C  lightning0((uint8_t)IRQ_PIN, (uint8_t)SI_PIN, (uint8_t)AS3935_ADD);
SPIFlash flash(FLASH_SS, 0xEF30); //WINDBOND 4MBIT flash chip on CS pin D8 (default for Moteino)

char buffer[50];
// interrupt trigger global var        
volatile bool AS3935_ISR_Trig = false;

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

  sprintf(buffer, "LightningMote - transmitting at: %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(buffer);

  if (flash.initialize()) flash.sleep();

  for (uint8_t i=0; i<=A5; i++)
  {
    if (i == RF69_SPI_CS) continue;
    if (i == FLASH_SS) continue;
    if (i == BATT_MONITOR) continue;
    if (i == IRQ_PIN) continue;
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  // setup for the the I2C library: (enable pullups, set speed to 400kHz)
  I2c.begin();
  I2c.pullup(true);
  I2c.setSpeed(1); 
  delay(2);

  pinMode(SI_PIN, OUTPUT);
  digitalWrite(SI_PIN, HIGH);
  
  lightning0.AS3935_DefInit();   // set registers to default  
  // now update sensor cal for your application and power up chip
  lightning0.AS3935_ManualCal(AS3935_CAPACITANCE, AS3935_OUTDOORS, AS3935_DIST_EN);
                                 // AS3935_ManualCal Parameters:
                                 //   --> capacitance, in pF (marked on package)
                                 //   --> indoors/outdoors (AS3935_INDOORS:0 / AS3935_OUTDOORS:1)
                                 //   --> disturbers (AS3935_DIST_EN:1 / AS3935_DIST_DIS:2)
                                 // function also powers up the chip
                  
  // enable interrupt (hook IRQ pin to Arduino Uno/Mega interrupt input: 0 -> pin 2, 1 -> pin 3 )
  attachInterrupt(1, AS3935_ISR, RISING);
  //lightning0.AS3935_PrintAllRegs();
  AS3935_ISR_Trig = 0;           // clear trigger

  radio.sendWithRetry(PIGATEWAYID, "START", 6);
  Blink(LED, 100);Blink(LED, 100);Blink(LED, 100);

  SERIALFLUSH();

  // Allow cap to charge
  delay(3000);
}

char input=0;
byte sendLoops=0;
byte battReadLoops=0;
float batteryVolts = 5;
char* BATstr="BAT:5.00v"; //longest battery voltage reading message = 9chars

void loop()
{
  if (battReadLoops--<=0) //only read battery every BATT_READ_LOOPS cycles
  {
    readBattery();
    battReadLoops = BATT_READ_LOOPS-1;
  }

  //send readings every SEND_LOOPS
  if (sendLoops-- <= 0 || AS3935_ISR_Trig)
  {
    sendLoops = SEND_LOOPS - 1;

    if (AS3935_ISR_Trig)
    {
      // reset interrupt flag
      AS3935_ISR_Trig = false;

      // Retrieve the interrupt source
      uint8_t int_src = lightning0.AS3935_GetInterruptSrc();
      
      if (1 == int_src)
      {
        uint8_t lightning_dist = lightning0.AS3935_GetLightningDistKm();        
        sprintf(buffer, "BAT:%sv LTDIST:%i", BATstr, lightning_dist);
      }
    }
    else
    {
        sprintf(buffer, "BAT:%sv", BATstr);
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

// this is irq handler for AS3935 interrupts, has to return
// void and take no arguments
void AS3935_ISR()
{
  AS3935_ISR_Trig = 1;
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
