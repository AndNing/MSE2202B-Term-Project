
/*
 * MSE 2202 Term Project Code
 * 
 * Authors: Andrew Ning, Mohammed Asim Iqbal Zafar, Matthew Bertuzzi, Mohammed Ali Sarfraz
 * Date: 17/03/21
 * Language: Arduino
 */


#include <Servo.h>
#include <uSTimer2.h>
#include <Wire.h>
#include <I2CEncoder.h>

#include "locatewall.h"
#include "retrievecube.h"
#include "locatepyramid.h"
#include "retrievepyramid.h"
#include "converttesseract.h"

//Servo Motors
Servo servo_RightTrackMotor;
Servo servo_LeftTrackMotor;
Servo servo_LiftMotor;
Servo servo_ArmMotor;
Servo servo_MagnetMotor;

//Encoders
I2CEncoder encoder_RightTrackMotor;
I2CEncoder encoder_LeftTrackMotor;

//Ports
const int ci_Right_Track_Motor;
const int ci_Left_Track_Motor;
const int ci_Lift_Motor;
const int ci_Arm_Motor;
const int ci_Magnet_Motor;
const int ci_Cube_Force_Sensor;
const int ci_Pyramid_Force_Sensor;
const int ci_IR_Sensor;
const int ci_Start_Button;
const int ci_Enable_Switch;
const int ci_Ultrasonic_Ping;
const int ci_Ultrasonic_Data;
const int ci_I2C_SDA;
const int ci_I2C_SCL;

//Constants
const int ci_Right_Track_Stop;
const int ci_Left_Track_Stop;
const int ci_Lift_Motor_Raise;
const int ci_Lift_Motor_Lower;
const int ci_Magnet_Motor_Retract;
const int ci_Magnet_Motor_Extend;
const int ci_Cube_Present_Force;
const int ci_Pyramid_Force_Present;
const int ci_Wall_Distance;

//variables
unsigned long ul_Echo_Time;
unsigned int ui_Track_Speed;
unsigned int ui_Left_Track;
unsigned int ui_Right_Track;
long l_Left_Track_Position;
long l_Right_Track_Position;

unsigned int ui_Current_Task = 1;
unsigned int ui_Turn_Difference;

bool bt_Enabled;


void setup() {
  Wire.begin();
  Serial.begin(9600);

  pinMode(ci_Enable_Switch, OUTPUT);
  pinMode(ci_Right_Track_Motor, OUTPUT);
  pinMode(ci_Left_Track_Motor, OUTPUT);
  servo_RightTrackMotor.attach(ci_Right_Track_Motor);
  servo_LeftTrackMotor.attach(ci_Left_Track_Motor);

  pinMode(ci_Lift_Motor, OUTPUT);
  pinMode(ci_Arm_Motor, OUTPUT);
  pinMode(ci_Magnet_Motor, OUTPUT);
  servo_LiftMotor.attach(ci_Lift_Motor);
  servo_ArmMotor.attach(ci_Arm_Motor);
  servo_MagnetMotor.attach(ci_Magnet_Motor);

  encoder_LeftTrackMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftTrackMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightTrackMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);  
  encoder_RightTrackMotor.setReversed(true);  // adjust for positive count when moving forward

  pinMode(ci_Start_Button, INPUT);
  pinMode(ci_IR_Sensor, INPUT);
  pinMode(ci_Pyramid_Force_Sensor, INPUT);
  pinMode(ci_Cube_Force_Sensor, INPUT);
  pinMode(ci_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_Ultrasonic_Data, INPUT);
}

void loop() {
  bt_Enabled = digitalRead(ci_Enable_Switch);

  if (bt_Enabled) {
    if (ui_Current_Task == 1) {
      locatewall();
    }
    else if (ui_Current_Task == 2) {
      retrievecube();
    }
    else if (ui_Current_Task == 3) {
      locatepyramid();
    }
    else if (ui_Current_Task == 4) {
      retrievepyramid();
    }
    else if (ui_Current_Task == 5) {
      converttesseract();
    }
  }
}






