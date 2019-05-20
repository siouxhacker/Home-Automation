/*
 Created 03/05/2012
 By Steve Small
 
 Blinks LEDs on the light tree for lightDelay then activates the solenoid
 
 The circuit:
    An LED is connected to each output pin from 3 to 9
    A solenoid is connected to pin 10.  
 
 */
int preStageLightPin = 3;
int yellowLight1Pin = 4;
int yellowLight2Pin = 5;
int yellowLight3Pin = 6;
int greenLightPin = 7;
int redLightPin = 8;
int solenoidPin = 9;
int startSignalPin = 11;
int signalState;
int lightDelay = 500;      // Sets the default delay time for the led lights
int redLightDelay = 2000;
int solenoidDelay = 1250;   // Sets the default delay for the solenoid
int endOfRaceTimeout = 10000;

// The setup() method runs once, when the sketch starts
void setup()
{                
  // initialize the digital pins as outputs:
  pinMode(preStageLightPin, OUTPUT);     
  pinMode(yellowLight1Pin, OUTPUT);     
  pinMode(yellowLight2Pin, OUTPUT);     
  pinMode(yellowLight3Pin, OUTPUT);     
  pinMode(greenLightPin, OUTPUT);     
  pinMode(redLightPin, OUTPUT);     
  pinMode(solenoidPin, OUTPUT);   
  pinMode(startSignalPin, INPUT);
}

// the loop() method repeats indefinitely until you turn off the power
void loop()                     
{
  signalState = digitalRead(startSignalPin);
  
  if (signalState == HIGH)
  {
    digitalWrite(preStageLightPin, HIGH);  // turn on LED on pin 3
    delay(250);                            // wait 
  
    digitalWrite(yellowLight1Pin, HIGH);   // turn on LED on pin 4
    delay(lightDelay);                     // wait 
    digitalWrite(yellowLight1Pin, LOW);    // turn it off
    
    digitalWrite(yellowLight2Pin, HIGH);   // turn on LED on pin 5
    delay(lightDelay);                     // wait 
    digitalWrite(yellowLight2Pin, LOW);    // turn it off
    
    digitalWrite(yellowLight3Pin, HIGH);   // turn on LED on pin 6
    delay(lightDelay);                     // wait 
    digitalWrite(yellowLight3Pin, LOW);    // turn it off
    
    digitalWrite(greenLightPin, HIGH);     // turn on LED on pin 7
    
    // The solenoid will be turned off in
    // the do-while loop, in case there
    // is a really fast car.  There is
    // a chance the end of race signal
    // could be missed if delay is called.
    digitalWrite(solenoidPin, HIGH);       // turn on Solenoid on pin 9
    
    // Get start time    
    unsigned long startTime = millis();

    do
    {
      if (millis() - startTime > solenoidDelay)
        digitalWrite(solenoidPin, LOW);
        
      signalState = digitalRead(startSignalPin);
    } while((millis() - startTime < endOfRaceTimeout) && signalState == LOW);

    digitalWrite(greenLightPin, LOW);      // turn it off

    // just in case the race ended
    // before the solenoidDelay
    digitalWrite(solenoidPin, LOW);  

    digitalWrite(redLightPin, HIGH);       // turn on LED on pin 8
    delay(redLightDelay);                  // wait 
    digitalWrite(redLightPin, LOW);        // turn it off
    digitalWrite(preStageLightPin, LOW);   // turn off preStageLight
  }
}
