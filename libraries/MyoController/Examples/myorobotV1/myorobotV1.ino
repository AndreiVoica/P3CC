#include <MyoController.h>

#include <Dynamixel_Serial.h>
//#include "Dynamixel_Serial.h"
#include <SoftwareSerial.h>

#define FIST_PIN 3
#define WAVEIN_PIN 4
#define WAVEOUT_PIN 5
#define FINGERSSPREAD_PIN 6
#define DOUBLETAP_PIN 7

//test = Pose();
MyoController myo = MyoController();

signed short minutes, seconds;
  char timeline[16];   

#define SERVO_ControlPin 0x03     // Control pin of buffer chip, NOTE: this does not matter becasue we are not using a half to full contorl buffer.
#define SERVO_SET_Baudrate 57600    // Baud rate speed which the Dynamixel will be set too (57600)
#define LED13 0x0D
#define CW_LIMIT_ANGLE 0x001        // lowest clockwise angle is 1, as when set to 0 it set servo to wheel mode
#define CCW_LIMIT_ANGLE 0xFFF       // Highest anit-clockwise angle is 0XFFF, as when set to 0 it set servo to wheel mode

SoftwareSerial mySerial(10, 11);    // RX, TX

//Servo test;

//int *data;
//#define POSE_NONE 0
//int pose = WAVEIN_PIN;

void setup(){
  pinMode(FIST_PIN, INPUT);
  pinMode(WAVEIN_PIN, INPUT);
  pinMode(WAVEOUT_PIN, INPUT);
  pinMode(FINGERSSPREAD_PIN, INPUT);
  pinMode(DOUBLETAP_PIN, INPUT);
  //test.attach(WAVEIN_PIN);
  myo.initMyo();
  //Serial.flush();                                       // Clear the serial buffer of garbage data before running the code.
  mySerial.begin(SERVO_SET_Baudrate);                   // We now need to set Ardiuno to the new Baudrate speed 115200
  Serial.begin(57600);                                  // Start serial communication on baudrate 57600
  Dynamixel.begin(mySerial);                            // Calling mySerial function which sets 10 pin as the 2nd RX serial pin, and sets pin 11 as the 2nd TX serial pin
  Dynamixel.setDirectionPin(SERVO_ControlPin);          // Optional. Set direction control pin which control if the program writes or reads to and from the robot

  // Turn on hold on the servos:
  Dynamixel.setHoldingTorque(0x01, true);               //Turn on hold torque on servo 1
  Dynamixel.setHoldingTorque(0x02, true);               //Turn on hold torque on servo 2
  Dynamixel.setHoldingTorque(0x03, true);               //Turn on hold torque on servo 3
  Dynamixel.setHoldingTorque(0x04, true);               //Turn on hold torque on servo 4
  Dynamixel.setHoldingTorque(0x05, true);               //Turn on hold torque on servo 5

  // Set the Profile acceleration.
  Dynamixel.setProfileAcceleration(0x01, 10);  //Set the Profile Acceleration for each servo. (max. is 32767)
  Dynamixel.setProfileAcceleration(0x02, 10);  //Set the Profile Acceleration for each servo. (max. is 32767)
  Dynamixel.setProfileAcceleration(0x03, 10);  //Set the Profile Acceleration for each servo. (max. is 32767)
  Dynamixel.setProfileAcceleration(0x04, 300);  //Set the Profile Acceleration for each servo. (max. is 32767)
  Dynamixel.setProfileAcceleration(0x05, 300);  //Set the Profile Acceleration for each servo. (max. is 32767)
  

  // Set the Profile velocity.
  Dynamixel.setProfileVelocity(0x01, 20);  //Set the Profile Velocity for each servo. (max. is 1023)
  Dynamixel.setProfileVelocity(0x02, 20);  //Set the Profile Velocity for each servo. (max. is 1023)
  Dynamixel.setProfileVelocity(0x03, 20);  //Set the Profile Velocity for each servo. (max. is 1023) 
  Dynamixel.setProfileVelocity(0x04, 200);  //Set the Profile Velocity for each servo. (max. is 1023)
  Dynamixel.setProfileVelocity(0x05, 200);  //Set the Profile Velocity for each servo. (max. is 1023)

  Serial.flush();
}

void(* resetFunc) (void) = 0;

void loop(){
   myo.updatePose();
   switch ( myo.getCurrentPose() ){
    case rest:
      digitalWrite(FIST_PIN,LOW); 
      digitalWrite(WAVEIN_PIN,LOW);
      digitalWrite(WAVEOUT_PIN,LOW);
      digitalWrite(FINGERSSPREAD_PIN,LOW);
      digitalWrite(DOUBLETAP_PIN,LOW);
      Dynamixel.setNGoalPositions(2048, 2048, 2048, 1094, 2922);
      delay(1000);
      Serial.flush();
      break;
    case fist:
      digitalWrite(FIST_PIN,HIGH);
      Dynamixel.setNGoalPositions(2048, 2048, 2048, 1094, 2922);
      delay(1000);
      Serial.flush();
      break;
    case waveIn:
      digitalWrite(WAVEIN_PIN, HIGH);
      Dynamixel.setNGoalPositions(2769, 800, 2300, 1950, 2125);
      delay(1000);
      Serial.flush();
      break;
    case waveOut:
      digitalWrite(WAVEOUT_PIN,HIGH);
      Dynamixel.setNGoalPositions(2048, 2048, 2048, 1094, 2922);
      delay(1000);
      Serial.flush();
      break;
    case fingersSpread:
      digitalWrite(FINGERSSPREAD_PIN,HIGH);
      Dynamixel.setNGoalPositions(2048, 2048, 2048, 1094, 2922);
      delay(1000);  
      Serial.flush();
      break;
    case doubleTap:
      digitalWrite(DOUBLETAP_PIN,HIGH);
      Dynamixel.setNGoalPositions(2048, 2048, 2048, 1094, 2922);
      delay(1000);
      Serial.flush();
      break;
   }
   delay(100);
}
/*int id1, id2, id3, id4, id5;

void kill()
{
  myo.updatePose();
  myo.getCurrentPose();
  if(waveIn){
    id1 = 2769;
    id2 = 800;
    id3 = 2300;
    id4 = 1950;
    id5 = 2125;
    digitalWrite(WAVEIN_PIN, HIGH);
    Dynamixel.setNGoalPositions(id1, id2, id3, id4, id5); //jos deschide
    delay(1000);
    Serial.flush();
  }else{
    //resetFunc();
    id1 = 2048;
    id2 = 2048;
    id3 = 2048;
    id4 = 1094;
    id5 = 2922;
    digitalWrite(WAVEIN_PIN, LOW);
    Dynamixel.setNGoalPositions(id1, id2, id3, id4, id5);
    }
} */
/*
void TestNothing()
{
  digitalWrite(WAVEIN_PIN, HIGH);
  Dynamixel.setNGoalPositions(2048, 2048, 2048, 1094, 2922);
  Serial.flush();
}

void TestServopos()
{
  digitalWrite(WAVEIN_PIN, HIGH);
  Dynamixel.setNGoalPositions(2769, 800, 2300, 1950, 2125); //jos deschide
  Serial.flush();
}
*/
 /*digitalWrite(WAVEIN_PIN, LOW);
    myo.updatePose();
    myo.getCurrentPose();
 
    if(myo.getCurrentPose() == waveIn){
    digitalWrite(WAVEIN_PIN, HIGH);
    Dynamixel.setNGoalPositions(2048, 2048, 2048, 1094, 2922);
   // Serial.flush();
    delay(5000);
    }else{
      Dynamixel.setNGoalPositions(2769, 800, 1900, 1094, 2922);
     // Serial.flush();
      delay(5000);
      */
/*void killme(){
    myo.updatePose();
    myo.getCurrentPose();
    if(myo.getCurrentPose() == waveIn){
    digitalWrite(WAVEIN_PIN, HIGH);
    Dynamixel.setNGoalPositions(2048, 2048, 2048, 1094, 2922);
    Serial.flush();
    delay(1000);
    }else{
      Dynamixel.setNGoalPositions(2769, 800, 1900, 1094, 2922);
      Serial.flush();
      delay(1000);
}*/
 
/*void go(){
  myo.updatePose();
  myo.getCurrentPose();
  if(myo.getCurrentPose()==WAVEIN_PIN){
    digitalWrite(WAVEIN_PIN, HIGH);
    Dynamixel.setNGoalPositions(2048, 2048, 2048, 1094, 2922);
    Serial.flush();
    delay(10000);
 }else{
    Dynamixel.setNGoalPositions(2769, 800, 1900, 1094, 2922); //open
    Serial.flush();
    delay(10000);
   // resetFunc();
    digitalWrite(WAVEIN_PIN, LOW);
    }
  }
  
*/
