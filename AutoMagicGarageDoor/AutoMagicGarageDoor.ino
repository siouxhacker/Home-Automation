int lightReadPin = 0;
int mainDoorReadPin = 1;
int secondaryDoorReadPin = 2;
int mainDoorOutPin = 2;
int secondaryDoorOutPin = 3;
int blinkPin = 13;

int minDarkLevel = 350;
int minDistance = 60;
int relayClosedDelay = 250;

unsigned long  mainLoopDelay = 30000; //5000
unsigned long  doorOpenTimeout = 600000; //60000

unsigned long mainDoorOpenStartTime;
unsigned long secondaryDoorOpenStartTime;
boolean mainDoorAlreadyOpen = false;
boolean secondaryDoorAlreadyOpen = false;


void setup()
{
  //pinMode(lightReadPin, INPUT);
  //pinMode(mainDoorReadPin, INPUT);
  //pinMode(secondaryDoorReadPin, INPUT);
  pinMode(mainDoorOutPin, OUTPUT);
  pinMode(secondaryDoorOutPin, OUTPUT);
  pinMode(blinkPin, OUTPUT);
  
  Serial.begin(115200);
}


void loop()
{
  if (IsDark())
  {
    // Check Main door if it is open
    if (IsDoorOpen(mainDoorReadPin))
    {
      if (mainDoorAlreadyOpen)
      {
        if (TimeSince(mainDoorOpenStartTime) > doorOpenTimeout)
        {
          Serial.print(F("Closing main door.  Time: "));
          Serial.println(millis());
          CloseDoor(mainDoorOutPin);
          mainDoorAlreadyOpen = false;
        }
        
        BlinkLight(1);
      }
      else
      {
        mainDoorAlreadyOpen = true;
        mainDoorOpenStartTime = millis();
        BlinkLight(2);
      }
    }
    else
      mainDoorAlreadyOpen = false;    
    
    // Check Secondary door if it is open
    if (IsDoorOpen(secondaryDoorReadPin))
    {
      if (secondaryDoorAlreadyOpen)
      {
        if (TimeSince(secondaryDoorOpenStartTime) > doorOpenTimeout)
        {
          Serial.print(F("Closing secondary door.  Time: "));
          Serial.println(millis());
          CloseDoor(secondaryDoorOutPin);
          secondaryDoorAlreadyOpen = false;
        }
        
        BlinkLight(3);
      }
      else
      {
        secondaryDoorAlreadyOpen = true;
        secondaryDoorOpenStartTime = millis();
        BlinkLight(4);
      }
    }
    else
      secondaryDoorAlreadyOpen = false;
  }
  else
  {
    // Reset the door open times
    mainDoorOpenStartTime = secondaryDoorOpenStartTime = millis();
    mainDoorAlreadyOpen = false;
    secondaryDoorAlreadyOpen = false;
  }
   
  //Serial.print(F("Light level: "));
  //Serial.println(GetLightLevel(lightReadPin));
  //Serial.print(F("Distance: "));
  //Serial.println(GetDistance(mainDoorReadPin));
  delay(mainLoopDelay);
}


// Determine if it is dark out
boolean IsDark()
{
  boolean isDark = false;
   
   
  if (GetLightLevel(lightReadPin) <= minDarkLevel)
    isDark = true;
     
  return isDark;
}


 // Determine if the door is open
boolean IsDoorOpen(int analogReadPin)
{
  boolean isDoorOpen = false;
   
   
  if (GetDistance(analogReadPin) <= minDistance)
    isDoorOpen = true;
   
  return isDoorOpen;
}
 

// Gets the average light level every 100 miliseconds
int GetLightLevel(int analogReadPin)
{
  float avgLight = 0;
  float totalLight = 0;
   
  for (int i = 0; i < 10; i++)
  {
    totalLight += analogRead(analogReadPin);
    delay(100);
  }
   
  avgLight = totalLight / 10;
  
  Serial.print(F("Light level: "));
  Serial.println(avgLight);
  
  return avgLight;
}
 
 
float GetDistance(int analogReadPin)
{
  float avgDist = 0;
  float totalDist = 0;
   
  for (int i = 0; i < 10; i++)
  {
    totalDist += (6762 / (analogRead(analogReadPin) - 9)) - 4;
    delay(100);
  }
   
  avgDist = totalDist / 10;
   
  // Eliminate negative values.
  // They occur at long distances
  if (avgDist <= 0 || avgDist < 10)
    avgDist = 200;
  
  Serial.print(F("Distance: "));
  Serial.println(avgDist);

  return avgDist;
}


void CloseDoor(int digitalOutPin)
{
  digitalWrite(digitalOutPin, HIGH);
  delay(relayClosedDelay);
  digitalWrite(digitalOutPin, LOW);
}


void BlinkLight(short numBlinks)
{
  for (int i = 0; i < numBlinks; i++)
  {
    digitalWrite(blinkPin, HIGH);
    delay(150);
    digitalWrite(blinkPin, LOW);
    delay(150);
  }
}


unsigned long TimeSince(unsigned long timeStamp)
{
  unsigned long now = millis();
  
  
  if (now >= timeStamp)
    return now - timeStamp;
  else
    return ((unsigned long)(-1)) - timeStamp + now;
}
