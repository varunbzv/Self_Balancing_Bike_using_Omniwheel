/*
Project: Self-Balancing Bike using Omni-Wheel
Author: Varun Bhatlawande
Date: April 2025

Description:
PID-controlled self-balancing robotic system using
MPU6050 IMU feedback and Arduino Nano.

Hardware:
- Arduino Nano
- MPU6050 IMU
- Motor Driver
- DC Motors
- Omniwheel
*/

#include <SPI.h>
//#include <RF24.h>
#include <Wire.h>
#include <MPU6050_light.h>

//RF24 radio(10, 8);

//const byte add_1 = 5;
//char msg_1;

MPU6050 mpu(Wire);

unsigned long timer = 0;
unsigned long prevTime = 0;

#define motorPin1 A2
#define enablePin1 6
#define motorPin2 A3

#define boMotorPin1 9   // Replace with your actual BO motor control pin
#define boEnablePin 5   // Replace with your actual BO motor enable pin
#define boMotorPin2 4    // Replace with your actual BO motor control pin

///////////////////////////////////////////////////////////// PID TUNING ///////////////////////////////////////////////////////////////////
// PID parameters
double Kp = 50 ;    // Proportional gain max=300 min=50 good=100,150 ------------>50
double Ki = 100;      // Integral gain  //0--------max200       correct - 100--------------->100
double Kd = 5;    // Derivative gain  max=10    correct - 5 -------------->5

// PID parameters for yaw
double Kp_yaw = 15 ;  // Proportional gain for yaw max=60 min=  5 -------------->15
double Ki_yaw = 5;  // Integral gain for yaw----------max100 -------------->5
double Kd_yaw = 5; // Derivative gain for yaw // max=50 ------------------->5
///////////////////////////////////////////////////////// VARIABLE INITIALIZATION /////////////////////////////////////////////////////////////////////////
double targetroll = 0;
double targetYaw = 0; 

double previousError = 0;
double previousYawError = 0;
double integral = 0;
double integralYaw = 0;

// Motor control variables
int motorSpeed = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin(); // Initialize the Wire library
  mpu.begin();  // Initialize the MPU6050
//  radio.begin();
  
  mpu.calcOffsets(); // gyro and accelerometer calibration

  // Initialize motor control pins for the main motor
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin1, OUTPUT);

  // Initialize motor control pins for the BO motor (if used)
  pinMode(boMotorPin1, OUTPUT);
  pinMode(boMotorPin2, OUTPUT);
  pinMode(boEnablePin, OUTPUT);
  digitalWrite(boMotorPin1, LOW);
  digitalWrite(boMotorPin2, LOW);
  
//  radio.openReadingPipe(1, add_1);
//// Set data rate to RF24_250KBPS, RF24_1MBPS, or RF24_2MBPS
//  radio.setDataRate(RF24_2MBPS); // Adjust the data rate as needed
//  radio.setPALevel(RF24_PA_MIN);
//  radio.startListening();
}

void executeCommand(char command) {
  switch (command) {
    // 1 Forward command
    case 'F':
      
      digitalWrite(boMotorPin1, HIGH);
      digitalWrite(boMotorPin2, LOW);
      
//      Serial.println(msg_1);
      break;
    case 'B':
      // Backward command
      
      digitalWrite(boMotorPin1, LOW);
      digitalWrite(boMotorPin2, HIGH);
      
//      Serial.println(msg_1);
      break;
//    case 'L':
//      targetYaw += 2 ; 
//      delay(5);
//      break;
//    case 'R':
//      targetYaw -= 2; 
//      delay(5);
//      break;
    case 'S':
      digitalWrite(boMotorPin1, LOW);
      digitalWrite(boMotorPin2, LOW);

//      Serial.println(msg_1);
      break;
  }
}
void pid(){
  unsigned long currentTime = millis();
  double dt = (currentTime - prevTime) / 1000.0; // Calculate time step in seconds
  prevTime = currentTime;

  mpu.update();
  double roll = mpu.getAngleX();
  double yaw = mpu.getAngleZ(); // Assuming Z axis corresponds to Yaw
   
  // Calculate Roll PID error
  double rollError = targetroll - roll;

  // Calculate Yaw PID error
  double yawError = targetYaw - yaw;

  // Update integral terms for Roll and Yaw (multiply by dt)
  integral += rollError * dt;
//  integral = constrain(integral, -100, 100); // Prevent integral windup

  integralYaw += yawError * dt;
//  integralYaw = constrain(integralYaw, -100, 100); // Prevent integral windup

  // Calculate derivative terms for Roll and Yaw (divide by dt)
  double derivative = (rollError - previousError) / dt;
  double derivativeYaw = (yawError - previousYawError) / dt;

  // Calculate Roll PID output
  double output = Kp * rollError + Ki * integral + Kd * derivative;

  // Calculate Yaw PID output
  double yawOutput = Kp_yaw * yawError + Ki_yaw * integralYaw + Kd_yaw * derivativeYaw;

  // Combine Roll and Yaw PID outputs (adjust weights as needed)
  double combinedOutput = output + yawOutput;

  // Map combined PID output to motor speed
  motorSpeed = map(combinedOutput, -90, 90, -255, 255);
  // Constrain motor speed
  motorSpeed = constrain(motorSpeed, -255, 255);
//  Serial.println(motorSpeed);

  // Motor control based on combined PID output for the main motor
  if (motorSpeed > 0) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  } else {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  }

  // Set motor speed using PWM for the main motor
  analogWrite(enablePin1, 250);
//  Serial.println(combinedOutput);
  // Update previous errors for Roll and Yaw
  previousError = rollError;
  previousYawError = yawError;
}
void loop() {
     //Call PID control function at 50 ms intervals
  if (millis() - timer >= 30) {  //40, -------------->30
    timer = millis();
    pid();
  }
  
//  if (radio.available()) {
//    radio.read(&msg_1, sizeof(msg_1));
//    Serial.println(msg_1);
//    analogWrite(boEnablePin, 200);
//    executeCommand(msg_1);
//  }

}
