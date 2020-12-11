#include <Arduino.h>
// DIR-Pins
const int driverDIR = 22;
const int driverDIR2 = 24;
const int driverDIR3 = 26;
// reference direction
bool setdir = LOW;

// Things sent by the Pi
int fromPi = 0;

// Array for the step-counters
int counterArr[] = {0,0,0};
// Acceleration in an Array so I dont have to recalculate everytime and thereby slow everything down
int counterAccel[] = {0,0,0};

// For the STEP-Pins
const int mtrPinArr[] = {23,25,27};

// Only change steps, and the velocity of the main joint
// The velocitys of the other joints are set to failsafe my calculations later on
//int mtrSteps[] = {6400, 12800};
float mtrSteps[] = {6400, 10000};
float mtrVel[] = {500,1000};

// multiplier to calculate velocity's of every joint
float multipl = mtrSteps[1]/mtrSteps[0];

// To use the Timer
unsigned long prevTimeArr[] = {0,0,0};

// Hall-sensors used as homing-sensors are attached to be used as interrupts
#define Hall_1 2
//volatile boolean sens1 = false;

// Sensors
volatile boolean sens[] = {false,false,false};

// Size of the acceleration, here it takes 3000 steps to go from 1500 to 100
unsigned int accelArray[3000];

// Array for interruot of homing-sensor 1
void nullAchse1(){
//  setdir = !setdir;
  Serial.println("stop");
  sens[0] = true;
}

// State-machine, to insure the completion of the movement before continuing with the main-loop
bool velocityHold(int mtr, unsigned long currentTime, int steps, unsigned long vel){
  // when all the steps are done, true gets returned
  // or when the homing-sensor of the motor registers as true
  if(counterArr[mtr] >= steps || sens[mtr]){
    return true;
  }
  // otherwise steps are done
  else{
//    if(currentTime - prevTimeArr[mtr] >= vel){
    // steps via timer
    if(currentTime - prevTimeArr[mtr] >= accelArray[counterAccel[mtr]]){
      digitalWrite(mtrPinArr[mtr], HIGH);
      digitalWrite(mtrPinArr[mtr], LOW);
      prevTimeArr[mtr] = currentTime;
      counterArr[mtr] += 1;
//      if(counterAccel[mtr] <= 2998){
//        counterAccel[mtr] += 1;
//      }
      // acceleration and afterwards keeping the set speed
      if(accelArray[counterAccel[mtr]] >= vel){
        counterAccel[mtr] += 1;
      }
    }
    return false;
  }
}

void setup() {
  pinMode(driverDIR,OUTPUT);
  pinMode(driverDIR2,OUTPUT);
  pinMode(driverDIR3,OUTPUT);

  for(int i=0; i<3; i++){
    pinMode(mtrPinArr[i], OUTPUT);
  }

  pinMode(Hall_1,INPUT);

  attachInterrupt(digitalPinToInterrupt(Hall_1), nullAchse1, FALLING);

  Serial.begin(9600);
//  Serial.println(multipl);

  // Acceleration via formula from maths-website
  for(int i = 0; i < 3000; i++){
    accelArray[i] = 0.00000000000000000009*pow(i-5000,6)+100;
  }

  // the speed of every joint is adapted to the first joint
  if(multipl>1){
    mtrVel[1] = mtrVel[0] / multipl - 50;
  }
  else if(multipl<1){
    mtrVel[1] = mtrVel[0] / multipl + 50;
  }
  else{
    mtrVel[1] = mtrVel[0] / multipl;
  }
  // currently accelerating to only 100, so the delay cant be lower than that
  if(mtrVel[1]<100){
    mtrVel[1] = 100;
    mtrVel[0] = mtrVel[1] * multipl;
  }
}

void loop() {
  // directions in the way the motors spin to find their homing-sensor
  digitalWrite(driverDIR,setdir);
  digitalWrite(driverDIR2,setdir);
  digitalWrite(driverDIR3,!setdir);

  // get current time for later usage
  unsigned long currentTime = micros();

  // state-machine is getting called
  bool mtr1done = velocityHold(0, currentTime, mtrSteps[0], mtrVel[0]);
  bool mtr2done = velocityHold(1, currentTime, mtrSteps[1], mtrVel[1]);
  bool mtr3done = velocityHold(2, currentTime, mtrSteps[1], mtrVel[1]);

  // after the movement every counter has to be reset and I added an extra delay of 2 seconds here
  if(mtr1done && mtr2done && mtr3done){
    counterArr[0] = 0;
    counterArr[1] = 0;
    counterArr[2] = 0;
    counterAccel[0] = 0;
    counterAccel[1] = 0;
    counterAccel[2] = 0;
    if(Serial.available() > 0){
      fromPi = Serial.read();
      Serial.println("received: ");
      Serial.println(fromPi, DEC);
    }
    delay(2000);
  }
}