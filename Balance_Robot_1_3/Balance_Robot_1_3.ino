#define directionPinA    8
#define directionPinB    9
#define pwmPin           10
#define encoderPinA      2
#define encoderPinB      3
#define NUMREADINGS      10
#define LOOPTIME         10

//second encoder&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
#define directionPinA2    13
#define directionPinB2    12
#define pwmPin2           11
#define encoderPinA2      4
#define encoderPinB2      7
//second encoder&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&


#include <Wire.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);
 
double pitch; 
int readings[NUMREADINGS];
int pwmValue = 0;
volatile long count = 0;
unsigned long lastMilli = 0;
int desiredSpeed = 0;
int actualSpeed = 0;
double Kp = 25.0;//22.000;
double Kd = 10.000;
double K = 150;
double lastError = 0;
double equilibrium = -2;
float error = 0;
float pidTerm = 0;
/////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);

  /* Initialise the sensor */
 if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);  
  bno.setExtCrystalUse(true);

//Second Encoder Start&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
  pinMode(directionPinA2, OUTPUT);
  pinMode(directionPinB2, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  pinMode(encoderPinA2, INPUT);
  pinMode(encoderPinB2, INPUT);
  digitalWrite(encoderPinA2, HIGH); // internal pullup for encoders
  digitalWrite(encoderPinB2, HIGH);
//Second Encoder End&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

  pinMode(directionPinA, OUTPUT);
  pinMode(directionPinB, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  digitalWrite(encoderPinA, HIGH); // internal pullup for encoders
  digitalWrite(encoderPinB, HIGH);
  attachInterrupt(1, rencoder, FALLING); //set pin no 2 as interrupt pin
  for (int i = 0; i < NUMREADINGS; i++) readings [i] = 0;
  analogWrite(pwmPin, pwmValue);
  //setting motor direction as FORWARD
  digitalWrite(directionPinA, HIGH);
  digitalWrite(directionPinB, LOW);


//Second Encoder Start&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
  analogWrite(pwmPin2, pwmValue);
  //setting motor direction as FORWARD
  digitalWrite(directionPinA2, HIGH);
  digitalWrite(directionPinB2, LOW);
//Second Encoder End&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&



}

void loop() {

  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  pitch = event.orientation.y ;
    /* Display the floating point data */
  
  //userInput();
  if ((millis() - lastMilli) >= LOOPTIME)
  {
    lastMilli = millis();
    //calculateActualSpeed();
    pwmValue = updatePID(equilibrium, pitch);
    analogWrite(pwmPin, pwmValue);
    analogWrite(pwmPin2, pwmValue); //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    //function for PWM to correct orientation from pitch error
    pitchControl();

  
    //printData();
  }

  

}


//This functions reads encoder data//////////////////////////////////
void rencoder() {
  if (digitalRead(encoderPinB) == HIGH)
    count--;
  else
    count++;
}

////This function caclulates motor speed using enccoder count//////////
void calculateActualSpeed() {
  static long countAnt = 0;

  actualSpeed = ((count - countAnt) * (60 * (1000 / LOOPTIME))) / (30);
  countAnt = count;
}

//This functions updates pwm via PID//////////////////////////////////
int updatePID(float targetValue, float currentValue) {

  error = targetValue - currentValue;
//  printData();
  pidTerm = (Kp * error) + (Kd * (error - lastError));
  lastError = error;
  
//  printData()
  return constrain (int(abs(pidTerm)), 0, 255);
}

//This function prints varoius data on serial monitor
void printData() {
  Serial.print("Pitch: "); Serial.print(pitch);  
  Serial.print("  error: "); Serial.print(error);
  Serial.print("  lastError: "); Serial.print(lastError);
  Serial.print("  pidTerm: "); Serial.println(pidTerm);



}

//This function reads user input commands
int userInput() {
  char input;
  if (Serial.available());
  input = Serial.read();
  switch (input) {
    case 'f':
      desiredSpeed += 10;
      if (desiredSpeed > 5000) desiredSpeed = 5000;
      break;

    case 's':
      desiredSpeed -= 10;
      if (desiredSpeed < 0) desiredSpeed = 0;
      break;

    case 'r':
      digitalWrite(directionPinA, LOW);
      digitalWrite(directionPinB, HIGH);
      break;

    case 'e':
      digitalWrite(directionPinA, HIGH);
      digitalWrite(directionPinB, LOW);
      break;

    case 'x':
      digitalWrite(directionPinA, LOW);
      digitalWrite(directionPinB, LOW);
      desiredSpeed = 0;
      pwmValue = 0;
      break;

    case 'p':
      Kp += 0.001;
      break;

    case ';':
      Kp -= 0.001;
      break;

    case 'o':
      Kd += 0.001;
      break;

    case 'l':
      Kd -= 0.001;
      break;

  }
}

void pitchControl(){
  if (pitch > equilibrium){
      digitalWrite(directionPinA, LOW);
      digitalWrite(directionPinB, HIGH);
      digitalWrite(directionPinA2, LOW);
      digitalWrite(directionPinB2, HIGH);
  }
  else {
      digitalWrite(directionPinA, HIGH);
      digitalWrite(directionPinB, LOW);
      digitalWrite(directionPinA2, HIGH);
      digitalWrite(directionPinB2, LOW);
  }
  //desiredSpeed = K * (pitch);
}











