#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SoftwareSerial.h>
SoftwareSerial BTserial(6, 7); // SRX | STX

Adafruit_BNO055 bno = Adafruit_BNO055(55); //Create bno object

const float PPMM = 45*48/(65*PI); //Divides pulses/rev by mm/rev to get pulses per millimeter
//Initialize global variables
int outputs[2];
float deltapos[2];
float velocity[2];
volatile int pos[2] = {0,0};
float errorPV = 0;
float errorIV = 0;
float errorPT = 0;
float errorIT = 0;

//PID parameters
const float PY = 1;
const float PV = .05;
const float IV = .03;
const float DV = .04;
const float PT = 60;
const float IT = 5;
const float DT = 1;
/*const float PY = 1;
const float PV = 4000;
const float IV = 100000;
const float DV = 40;
const float PT = 200;
const float IT = 1000;
const float DT = 20;*/


//Declare pins
int encPinsA[2] = {2,3};
int encPinsB[2] = {4,5};
int motorPinsA[2] = {10,12};
int motorPinsB[2] = {11,13};
int pwmPins[2] = {9,8};

void setup() {
  Serial.begin(9600);  //Turn on the Serial Monitor
  BTserial.begin(9600);
  
  Serial.println("Orientation Sensor Test");  //Initialise the sensor
  if(!bno.begin())
  {
    //There was a problem detecting the BNO055 ... check your connections
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  delay(1000);  //Pause
  bno.setExtCrystalUse(true);
  }
  else{
    Serial.println("Test Passed! Waiting for Bluetooth Signal...");
  }
  
  for (int k = 0; k < 2; k++){
    pinMode(encPinsB[k], INPUT);
    pinMode(encPinsA[k], INPUT);
    pinMode(motorPinsA[k], OUTPUT); 
    pinMode(motorPinsB[k], OUTPUT); 
    pinMode(pwmPins[k], OUTPUT); 
  }

  attachInterrupt(digitalPinToInterrupt(encPinsA[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(encPinsA[1]), readEncoder<1>, RISING);
}

long int time1 = millis();
long int timeA1 = millis();
float prevpos[2] = {0,0};

void loop() {
  //Serial.print("Position1: "); Serial.print(pos[0]); Serial.print("     Position2: "); Serial.println(pos[1]);
  float setvelocity = readBluetooth()/10;
  float setyaw = 0;
  float yawOutput = yawControl(setyaw);
  sensors_event_t event; 
  bno.getEvent(&event);
  float theta = event.orientation.y+3;
  if (theta>180){
    theta -= 360;
   }
  theta = theta*-1;
  //Serial.print("theta:"); Serial.println(theta);
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_LINEARACCEL);

  delay(10);
  float settheta = velocityPID(setvelocity);
  float linOutput = thetaPID(settheta, theta);
  outputs[0] = linOutput + yawOutput;
  outputs[1] = linOutput - yawOutput;
  driveMotors();
  
  Serial.print("  Z-Orientation: "); Serial.print(theta);
  Serial.print("  Set Velocity: "); Serial.print(setvelocity);
  Serial.print("  Set Angle: "); Serial.print(settheta);
  Serial.print("  Motor Output: "); Serial.println(linOutput);
}

int readBluetooth(){
  while (BTserial.available()!=1){
    BTserial.read();
  }
  return (BTserial.read() - 128);
}

float velocityPID(float setvelocity){
  long int time2 = millis();
  float timeinterval = (time2-time1)/1000.0;
  time1 = time2;
  for(int j=0; j<2; j++){
    deltapos[j] = pos[j] - prevpos[j];
    prevpos[j] = pos[j];
    velocity[j] = (deltapos[j])/PPMM*timeinterval;
  }
  float linvelocity = (velocity[0]+velocity[1])/2;
  //Serial.print("  Encoder Velocity: "); Serial.println(linvelocity); 
  //linvelocity += (acceleration * timeinterval);
  float preverrorPV = errorPV;
  errorPV = (setvelocity - linvelocity);
  errorIV += errorPV*timeinterval;
  //Serial.println(errorIV); 
  float errorDV = (errorPV - preverrorPV)/timeinterval;
  return (errorPV*PV+errorIV*IV+errorDV*DV);
}

float thetaPID(float settheta, float theta){
  long int time2 = millis();
  float timeinterval = (time2-timeA1)/1000.0;
  timeA1 = time2;
  float preverrorPT = errorPT;
  errorPT = (settheta - theta);
  errorIT += errorPT*timeinterval;
  float errorDT = (errorPT - preverrorPT)/timeinterval; 
  //Serial.print("Previous Error: "); Serial.print(preverrorPT); Serial.print("Current Error: "); Serial.print(errorPT); Serial.print("Timeinterval: "); Serial.println(timeinterval);
  //Serial.print("Proportional: ");Serial.print(errorPT*PT);Serial.print("  Integral: ");Serial.print(errorIT*IT);Serial.print("  Derivative: ");Serial.println(errorDT*DT);
  //Serial.println(errorPT*PT+errorIT*IT+errorDT*DT);
  return (errorPT*PT+errorIT*IT+errorDT*DT);
}

float yawControl(float setyaw){
  float yawvelocity = velocity[0]-velocity[1];
  float error = setyaw - yawvelocity;
  return(error*PY) ;
}

void driveMotors(){
  for (int x=0;x<2;x++){
     outputs[x] = (int)outputs[x];
     if (outputs[x]>255){
     outputs[x] = 255;
     }
     else if (outputs[x]<-255){
      outputs[x] = -255;
     }
     analogWrite(pwmPins[x], abs(outputs[x]));
     if (outputs[x]<0){
      digitalWrite(motorPinsA[x], HIGH);
      digitalWrite(motorPinsB[x], LOW);
     }
     else if (outputs[x]>0){
      digitalWrite(motorPinsA[x], LOW);
      digitalWrite(motorPinsB[x], HIGH);
     }
     else{
      digitalWrite(motorPinsA[x], LOW);
      digitalWrite(motorPinsB[x], LOW);
     }
  }
}

template <int k>
void readEncoder(){
  int b = digitalRead(encPinsB[k]);
  if(b>0){
    pos[k]++;
  }
  else{
    pos[k]--;
  }
}
