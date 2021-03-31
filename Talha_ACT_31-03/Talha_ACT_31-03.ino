#define EN                  23
#define dirPin              25
#define StepPin             27

#define Tur_pin_Top         19           // RPM Sensor Top Roll
#define Tur_pin_Bottom      18        // RPM Sensor Bottom Roll
#define Right_pin           21               // Proximity Switch for Right and Park/Load position
#define Left_pin            3                // Proximity Switch for Left Return position
#define Load_pin            2                // PushButton to move the carriage to Park/Load position

#define PotTurPin           A1
#define PotDelayPin         A3

#define Distance            1.75  //Filament Diameter
#define interruptors        2     // number of interruptors on opto wheel
#define Pitch               4     // pitch of leadscrew
#define Resolution          200   // step motor resolution for 1.8deg step angle


#define Microsteps          8     //settings on Step driver

#define EndStopOffset       3
#define spoolWidth          10

long int AutonomousSteps   = Microsteps / Pitch  * Resolution * spoolWidth;// steps in the loop for movement
int endStopOffset = Microsteps / Pitch  * Resolution * EndStopOffset;

unsigned long startTime;
unsigned long endTime;
boolean right = LOW, left = HIGH, hallTop = LOW, hallBottom = LOW, load = LOW;
unsigned long duration = 0;
unsigned long Speed = 0;
unsigned long dirChangeDelay = 0;
volatile float delayAdjust = 1;
volatile float speedAdjust = 1;
int c = 0;
volatile float setupNumber = Distance / (Pitch) * Resolution;

//----------Smoothing--------
const int numReadings = 8;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average
//---------SETUP------------------
void setup()
{

  Serial.begin(115200);
  delay(200);
  Serial.println("System Initialized....");
  initPins();
}

//----------LOOP-----------------
void loop()
{ delayAdjust = analogRead (PotDelayPin) / 1023.0;
  speedAdjust = analogRead (PotTurPin) / 1023.0 + 0.5;
  ///Serial.print("SpeedAdjust: "); Serial.print(speedAdjust);Serial.print("delay: "); Serial.println(delayAdjust);
  boolean rightPinState = digitalRead (Right_pin);
  if (load) //load procedure and rightpin not lit)
  {
    if (!left) //  while going left
    {
      digitalWrite(dirPin, LOW);// go to Right
      mov();
    }
    else if (left) // if load is pressed while going right
    { Serial.println("System should Wait unless Swatch is detected ...");
      load = LOW;
      hallTop = LOW;
      hallBottom = LOW;
      left = HIGH;
      right = LOW;
    }
  }
  else
  mov();
  /*
  //in all other cases
  {
    if        ( left && !right && (hallTop || hallBottom) ) //When either Hall triggered go to the LEFT endstop Position
    {
      digitalWrite(dirPin, HIGH);
      mov();
    }
    else if   (!left && right && (hallTop || hallBottom) && rightPinState ) //When either Hall triggered go to the RIGHT endstop Position
    {
      digitalWrite(dirPin, LOW);
      mov();
    }
  }*/
}

//-----------INIT PINS----------------
void initPins()
{
  pinMode(EN, OUTPUT);    // ENABLE AS OUTPUT
  pinMode(StepPin, OUTPUT);  // STEP AS OUTPUT
  pinMode(dirPin, OUTPUT); // DIRECTION AS OUTPUT
  pinMode(PotTurPin, INPUT); // Potentiometer for adjusting Speed
  pinMode(PotDelayPin, INPUT); //Potentiometer for adjusting Delay between direction changes
  digitalWrite(EN, HIGH);  // SET ENABLE TO HIGH FOR EXTERNAL DRIVER, LOW FOR STEPSTICKS

  pinMode(Tur_pin_Top, INPUT_PULLUP);
  pinMode(Tur_pin_Bottom, INPUT_PULLUP);
  pinMode(Right_pin, INPUT_PULLUP);
  pinMode(Left_pin, INPUT_PULLUP);
  pinMode(Load_pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(Tur_pin_Top), hallButTop, FALLING);
  attachInterrupt(digitalPinToInterrupt(Tur_pin_Bottom), hallButBottom, FALLING);
  attachInterrupt(digitalPinToInterrupt(Right_pin), rightBut, FALLING);
  attachInterrupt(digitalPinToInterrupt(Left_pin), leftBut, FALLING);
  attachInterrupt(digitalPinToInterrupt(Load_pin), loadBut, RISING);

  delay(1000);
  if (digitalRead(Left_pin) == LOW)
  {
    Serial.println("System is on left side initally ");
    left = LOW;
    right = HIGH;
    hallTop = LOW;
    hallBottom = LOW;
    delay(100);
    Serial.println("Move  Right on swatch...");
  }
}
//-------HALL TOP--------------------
void hallButTop()
{
  load = LOW;
  hallTop = HIGH;
  hallBottom = LOW;
  //Serial.println("Swatch Change Top");
  count();
}
//-------HALL BOTTOM-----------------
void hallButBottom()
{
  load = LOW;
  hallTop = LOW;
  hallBottom = HIGH;
  //Serial.println("Swatch Change Bottom");
  count();
}
//---------------------------

//-------------------Main Calculation----------------
void count()
{
  c++;
  if (c == 1)
  {
    startTime = millis();
  }
  else if ( c == 2)
  {
    endTime = millis();
    duration = endTime - startTime;
    Speed = ((1000000.0 / (Microsteps * (1000.0 / (duration * interruptors)) * 200)) ) * speedAdjust;; //FORMULA
    dirChangeDelay = duration * interruptors * 1000.0/ 4.0 * delayAdjust;
    //Speed = (1000000.0 / (Microsteps * (1000.0 / (duration * interruptors)) * setupNumber)) / 2/speedAdjust; //FORMULA
    //Speed = 350/speedAdjust; //FORMULA
    //Speed = (1000000.0 / (1000.0 / (duration * interruptors) * setupNumber)) / 2 / speedAdjust; //FORMULA
    //Speed = (1000000.0 / (Microsteps * (1000.0 / (duration * interruptors)) * setupNumber))/2;//speedAdjust; //FORMULA
    //Speed = 300* delayAdjust;
    //Serial.print("SpeedAdjust: "); Serial.println(speedAdjust);
    //Serial.print("DelayAdjust: "); Serial.println(delayAdjust);
    //Serial.print("dCD: "); Serial.println(dirChangeDelay);
 //   Serial.print("duration: "); Serial.println(duration*4);
    //float RPS = 1000.0 / (duration* interruptors);
//   float RPS = 1000000.0/ (Speed *2.0 *(Microsteps* Resolution));
//    Serial.print("Speed "); Serial.println(Speed);
//    Serial.print("Speed RPS: "); Serial.println(RPS);
    c = 0;
  }
   // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = duration*4;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
    Serial.println(average);
  }

  // calculate the average:
  average = total / numReadings;

}
//----------RIGHT-----------------
void rightBut()
{
  //detachInterrupt(digitalPinToInterrupt(Tur_pin_Top));
  //detachInterrupt(digitalPinToInterrupt(Tur_pin_Bottom));
  digitalWrite(dirPin,HIGH);
  Serial.println("Right EndStop Detected...");
  left = HIGH;
  right = LOW;
  hallTop = LOW;
  hallBottom = LOW;
  delay(dirChangeDelay);
  //attachInterrupt(digitalPinToInterrupt(Tur_pin_Top), hallButTop, FALLING);
  //attachInterrupt(digitalPinToInterrupt(Tur_pin_Bottom), hallButBottom, FALLING);
  //Serial.println("Moving LEFT...");
}

//-----------LEFT----------------
void leftBut()
{digitalWrite (dirPin,HIGH);
  //detachInterrupt(digitalPinToInterrupt(Tur_pin_Top));
  //detachInterrupt(digitalPinToInterrupt(Tur_pin_Bottom));
   Serial.println("Left EndStop Detected...");
  left = LOW;
  right = HIGH;
  hallTop = LOW;
  hallBottom = LOW;
  delay(dirChangeDelay);
  //attachInterrupt(digitalPinToInterrupt(Tur_pin_Top), hallButTop, FALLING);
  //attachInterrupt(digitalPinToInterrupt(Tur_pin_Bottom), hallButBottom, FALLING);
  //Serial.println("Moving RIGHT...");
}
//---------LOAD------------------
void loadBut()
{ delayMicroseconds(1000000);
  digitalRead (loadBut);
  byte loadState = digitalRead (loadBut);
  detachInterrupt(digitalPinToInterrupt(Tur_pin_Top));
  detachInterrupt(digitalPinToInterrupt(Tur_pin_Bottom));
  Speed = (1000000.0 / (Microsteps * (1000.0 / (180)) * setupNumber)) / 2; //***Speed for Going to Park Position**********
  Serial.println("Load Button Pressed..");
  //Serial.println("Move to Right-EndStop and Wait");
  digitalWrite(dirPin, HIGH);
  for (int i = 0; i < EndStopOffset; i++)//ACT
  {
    digitalWrite(StepPin, HIGH);
    delayMicroseconds(Speed);
    digitalWrite(StepPin, LOW);
    delayMicroseconds(Speed);
  }
  left = LOW;
  right = HIGH;
  hallTop = LOW;
  hallBottom = LOW;
  load = HIGH;
  delay(100);

}

//---------MOV------------------
void mov()
{
  for (int i = 0; i < AutonomousSteps; i++)//ACT
  {
    digitalWrite(StepPin, HIGH);
    delayMicroseconds(Speed);
    digitalWrite(StepPin, LOW);
    delayMicroseconds(Speed);
  }
    digitalWrite(dirPin, !digitalRead(dirPin));
}
