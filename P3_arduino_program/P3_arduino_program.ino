#include <SoftwareSerial.h>
#include <MyoController.h>
#include <Dynamixel_Serial.h>

#define SERVO_ControlPin 0x04       // Control pin of buffer chip, NOTE: this does not matter becasue we are not using a half to full contorl buffer.
#define SERVO_SET_Baudrate 57600    // Baud rate speed which the Dynamixel will be set too (57600)
#define LED13 0x0D
//#define CW_LIMIT_ANGLE 0//0x001        // lowest clockwise angle is 1, as when set to 0 it set servo to wheel mode
//#define CCW_LIMIT_ANGLE 0xFFF     // Highest anit-clockwise angle is 0XFFF, as when set to 0 it set servo to wheel mode

MyoController myo = MyoController();
int a1 = 1;

void setup() {
  myo.initMyo();
  Serial1.flush();                                      // Clear the serial buffer of garbage data before running the code.
  Serial1.begin(SERVO_SET_Baudrate);                   // We now need to set Ardiuno to the new Baudrate speed 115200
  Dynamixel.begin(57600);                            // Calling mySerial function which sets 10 pin as the 2nd RX serial pin, and sets pin 11 as the 2nd TX serial pin
  Dynamixel.setDirectionPin(SERVO_ControlPin);          // Optional. Set direction control pin which control if the program writes or reads to and from the robot
  
  // Turn on hold on the servos:
  Dynamixel.setHoldingTorque(0x01, true);               //Turn on hold torque on servo 1
  Dynamixel.setHoldingTorque(0x02, true);               //Turn on hold torque on servo 2
  Dynamixel.setHoldingTorque(0x03, true);               //Turn on hold torque on servo 3
  Dynamixel.setHoldingTorque(0x04, true);               //Turn on hold torque on servo 4
  Dynamixel.setHoldingTorque(0x05, true);               //Turn on hold torque on servo 5

  // Set the Profile acceleration.
  Dynamixel.setProfileAcceleration(0x01, 70);  //Set the Profile Acceleration for each servo. (max. is 32767)
  Dynamixel.setProfileAcceleration(0x02, 70);  //Set the Profile Acceleration for each servo. (max. is 32767)
  Dynamixel.setProfileAcceleration(0x03, 70);  //Set the Profile Acceleration for each servo. (max. is 32767)
  Dynamixel.setProfileAcceleration(0x04, 70);  //Set the Profile Acceleration for each servo. (max. is 32767)
  Dynamixel.setProfileAcceleration(0x05, 70);  //Set the Profile Acceleration for each servo. (max. is 32767)
  

  // Set the Profile velocity.
  //Dynamixel.setProfileVelocity(0x01, 100);  //Set the Profile Velocity for each servo. (max. is 1023)
  //Dynamixel.setProfileVelocity(0x02, 100);  //Set the Profile Velocity for each servo. (max. is 1023)
  //Dynamixel.setProfileVelocity(0x03, 100);  //Set the Profile Velocity for each servo. (max. is 1023)
  //Dynamixel.setProfileVelocity(0x04, 20);  //Set the Profile Velocity for each servo. (max. is 1023)
  //Dynamixel.setProfileVelocity(0x05, 20);  //Set the Profile Velocity for each servo. (max. is 1023)
}


void m1() {
  int id1 = Dynamixel.getPosition(0x01);
  int id4 = Dynamixel.getPosition(0x04);
  int id5 = Dynamixel.getPosition(0x05);
  myo.updatePose();
  myo.getCurrentPose();
  char a = myo.getCurrentPose();
  while (a == waveIn) {
    Dynamixel.getPosition(0x01);
    Dynamixel.setGoalVelocity(0x01, 10);
    myo.updatePose();
    myo.getCurrentPose();
    if (myo.getCurrentPose() != waveIn) {
      a = 0;
      Dynamixel.setGoalVelocity(0x01, 0);
      myo.endMyo();
    }
    delay(50);
    Serial1.flush();
  }
  char a2 = myo.getCurrentPose();
  while (a2 == waveOut) {
    Dynamixel.getPosition(0x01);
    Dynamixel.setGoalVelocity(0x01, -10);
    myo.updatePose();
    myo.getCurrentPose();
    if (myo.getCurrentPose() != waveOut) {
      a2 = 0;
      Dynamixel.setGoalVelocity(0x01, 0);
      myo.endMyo();
    }
    delay(50);
    Serial1.flush();
  }
  
  char b = myo.getCurrentPose();
  while (b == fist) {
    Dynamixel.getPosition(0x04);
    Dynamixel.getPosition(0x05);
    Dynamixel.setGoalVelocity(0x04, 20);
    Dynamixel.setGoalVelocity(0x05, -20);
    myo.updatePose();
    myo.getCurrentPose();
    if (myo.getCurrentPose() != fist) {
      b = 0;
      Dynamixel.setGoalVelocity(0x04, 0);
      Dynamixel.setGoalVelocity(0x05, 0);
      myo.endMyo();
    }
    delay(50);
    Serial1.flush();
  }
  char c = myo.getCurrentPose();
  while (c == fingersSpread) {
    Dynamixel.getPosition(0x04);
    Dynamixel.getPosition(0x05);
    Dynamixel.setGoalVelocity(0x04, -20);
    Dynamixel.setGoalVelocity(0x05, 20);
    myo.updatePose();
    myo.getCurrentPose();
    if (myo.getCurrentPose() != fingersSpread) {
      c = 0;
      Dynamixel.setGoalVelocity(0x04, 0);
      Dynamixel.setGoalVelocity(0x05, 0);
      myo.endMyo();
    }
    delay(50);
    Serial1.flush();
  }
  delay(100);
  Serial1.flush();
  Serial.flush();
  myo.endMyo();
}

void m2() {
  int id2 = Dynamixel.getPosition(0x02);
  int id3 = Dynamixel.getPosition(0x03);
  myo.updatePose();
  char d = myo.getCurrentPose();
  while (d == waveIn) {
    Dynamixel.getPosition(0x02);
    Dynamixel.setGoalVelocity(0x02, 10);
    myo.updatePose();
    myo.getCurrentPose();
    if (myo.getCurrentPose() != waveIn) {
      d = 0;
      Dynamixel.setGoalVelocity(0x02, 0);
      myo.endMyo();
    }
    delay(50);
    Serial1.flush();
  }
  char e = myo.getCurrentPose();
  while (e == waveOut) {
    Dynamixel.getPosition(0x02);
    Dynamixel.setGoalVelocity(0x02, -10);
    myo.updatePose();
    myo.getCurrentPose();
    if (myo.getCurrentPose() != waveOut) {
      e = 0;
      Dynamixel.setGoalVelocity(0x02, 0);
      myo.endMyo();
    }
    delay(50);
    Serial1.flush();
  }
  char f = myo.getCurrentPose();
  while (f == fist) {
    Dynamixel.getPosition(0x03);
    Dynamixel.setGoalVelocity(0x03, 10);
    myo.updatePose();
    myo.getCurrentPose();
    if (myo.getCurrentPose() != fist) {
      f = 0;
      Dynamixel.setGoalVelocity(0x03, 0);
      myo.endMyo();
    }
    delay(50);
    Serial1.flush();
  }
  char g = myo.getCurrentPose();
  while (g == fingersSpread) {
    Dynamixel.getPosition(0x03);
    Dynamixel.setGoalVelocity(0x03, -10);
    myo.updatePose();
    myo.getCurrentPose();
    if (myo.getCurrentPose() != fingersSpread) {
      g = 0;
      Dynamixel.setGoalVelocity(0x03, 0);
    }
  }
  delay(100);
  Serial1.flush();
  Serial.flush();
  myo.endMyo();
}

void loop() {
  char dt = myo.getCurrentPose();
  if (dt == doubleTap) {
    a1++;
    delay(100);
  }
  if (a1 % 2 == 0) {
    m1();
    Serial1.flush();
    delay(100);
  }
  else {
    m2();
    Serial1.flush();
    delay(100);
  }
  myo.endMyo();
}
