
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
const int ci_Right_Track_Motor = 8;
const int ci_Left_Track_Motor = 9;
const int ci_Shovel_Motor = 12;
const int ci_Arm_Motor = 10;
const int ci_Magnet_Motor = 11;

//const int ci_Cube_Force_Sensor;
//const int ci_Pyramid_Force_Sensor;
const int ci_IR_Sensor = 5;
const int ci_IR_Sensor2 = 6;

const int ci_Start_Button;
const int ci_Enable_Switch;

const int ci_Ultrasonic_Ping_Top = 2;
const int ci_Ultrasonic_Data_Top = 3;
const int ci_Ultrasonic_Ping_Side_Back = 4;
const int ci_Ultrasonic_Data_Side_Back = 5;
const int ci_Ultrasonic_Ping_Side_Front = 6;
const int ci_Ultrasonic_Data_Side_Front = 7;

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

int inByte = 0;
int val = 0;
int count = 0;
int check = 0;

unsigned int ui_Current_Task = 1;
unsigned int ui_Turn_Difference;

bool bt_Enabled;


void setup() {
  Wire.begin();
  Serial.begin(9600);

  while (!Serial) {
    //wait for serial port to conect. Needed for native USB port only
  }

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
  pinMode(ci_Ultrasonic_Ping_Top, OUTPUT);
  pinMode(ci_Ultrasonic_Data_Top, INPUT);
  pinMode(ci_Ultrasonic_Ping_Side_1, OUTPUT);
  pinMode(ci_Ultrasonic_Data_Side_1, INPUT);
  pinMode(ci_Ultrasonic_Ping_Side_2, OUTPUT);
  pinMode(ci_Ultrasonic_Data_Side_2, INPUT);
  pinMode(ci_Ultrasonic_Ping_Back, OUTPUT);
  pinMode(ci_Ultrasonic_Data_Back, INPUT);

  My_Receiver.enableIRIn(); //Start receiver

  pinMode (A0, INPUT);
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


void PingTop()
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_Ping_Top, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Ping_Top, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW 
  ul_Echo_Time_Top = pulseIn(ci_Ultrasonic_Data_Top, HIGH, 10000);

}

void PingSideFront()
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_Ping_Side_Front, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Ping_Side_Front, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW 
  ul_Echo_Time_Side_Front = pulseIn(ci_Ultrasonic_Data_Side_Front, HIGH, 10000);

}

void PingSideBack()
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_Ping_Side_Back, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Ping_Side_Back, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW 
  ul_Echo_Time_Side_Back = pulseIn(ci_Ultrasonic_Data_Side_Back, HIGH, 10000);

}  

void receiveEvent(int howMany){
    int c = (int(Wire.read()))*5;
    Serial.println(c);
}

void DriveToGetCube(){

  GetUltrasonicValues();
  
  if((ul_Echo_Time_Side_Front_Val>=(frontUltraVal-20))&&(ul_Echo_Time_Side_Front_Val<=(frontUltraVal+20))&&(ul_Echo_Time_Side_Back_Val>=(backUltraVal-20))&&(ul_Echo_Time_Side_Back_Val<=(backUltraVal+20))){
      servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed_Cube);
      servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed_Cube);
      Serial.println("STRAIGHT");
  }

  else if((ul_Echo_Time_Side_Back_Val <(backUltraVal-20))&&(ul_Echo_Time_Side_Front_Val>=(frontUltraVal-20))){
      servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed_Cube);
      servo_RightMotor.writeMicroseconds(1380);
      Serial.println("TURN RIGHT");
  }

  else if((ul_Echo_Time_Side_Front_Val<(frontUltraVal-20))&&(ul_Echo_Time_Side_Back_Val>=(backUltraVal-20))){
      servo_LeftMotor.writeMicroseconds(1380);
      servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed_Cube);
      Serial.println("TURN LEFT");
  }
  
  else if(((ul_Echo_Time_Side_Front_Val)<(frontUltraVal-20))&&((ul_Echo_Time_Side_Back_Val)<(backUltraVal-20))){
      servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed_Cube);
      servo_RightMotor.writeMicroseconds(1380);
      Serial.println("TURN RIGHT");
  }

  else if((ul_Echo_Time_Side_Front_Val>(frontUltraVal+20))&&(ul_Echo_Time_Side_Back_Val>(backUltraVal+20))){
      servo_LeftMotor.writeMicroseconds(1380);
      servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed_Cube);
      Serial.println("TURN LEFT");
  }
}

void CallibrateUltrasonics(){
  for(int i = 0; i<41;i++){
    PingSideFront();
    PingSideBack();
    
    if(i>0){
      backUltraVal = backUltraVal + ul_Echo_Time_Side_Back;
      frontUltraVal = frontUltraVal + ul_Echo_Time_Side_Front;
    }
    
    delay(50);
    
  }

  backUltraVal = backUltraVal/40;
  frontUltraVal = frontUltraVal/40;
  
}

void GetUltrasonicValues(){

  ul_Echo_Time_Side_Back_Val = 0;
  ul_Echo_Time_Side_Front_Val = 0;
  
  for (int i = 0; i<41;i++){
    PingSideFront();
    PingSideBack();
    delay(2);
    if(i>0){
      ul_Echo_Time_Side_Back_Val = ul_Echo_Time_Side_Back_Val + ul_Echo_Time_Side_Back;
      ul_Echo_Time_Side_Front_Val = ul_Echo_Time_Side_Front_Val + ul_Echo_Time_Side_Front;
    }
  }

  ul_Echo_Time_Side_Back_Val = (ul_Echo_Time_Side_Back_Val)/40;
  ul_Echo_Time_Side_Front_Val = (ul_Echo_Time_Side_Front_Val)/40;
}

void FindPyramid(){

  val = 0;

  if (count == 0){

    while(check != 0){
      if(Serial.read() == -1){
        check = 0;
      }
    }

    if (Serial.available()>0){
      val = Serial.read();
    }

    Serial.println(val);

    servo_LeftMotor.writeMicroseconds(1650);
    servo_RightMotor.writeMicroseconds(1380);

    if ((val == 69) || (val == 65)){

      servo_LeftMotor.writeMicroseconds(1500);
      servo_RightMotor.writeMicroseconds(1500);

      count = 1;

      refTime = millis();
  
    }
  
  }

  if (count == 1){

    while((millis()-refTime)<2000){
      servo_LeftMotor.writeMicroseconds(1680);
      servo_RightMotor.writeMicroseconds(1680);

      PingTop();

      if(ul_Echo_Time_Top>2000){
        countTwo = 1;
        break;
      }
      
    }

    count++;

    refTime = millis();

  }

  if (countTwo == 1){

    while(1 == 1){
      servo_LeftMotor.writeMicroseconds(1500);
      servo_RightMotor.writeMicroseconds(1500);
    }
  
  }

  if (count == 2){

  //Serial.println("Turn 2");
  
    while((millis()-refTime)<1000){
      servo_LeftMotor.writeMicroseconds(1350);
      servo_RightMotor.writeMicroseconds(1680);

      PingTop();

      if(ul_Echo_Time_Top>2000){
        countTwo = 1;
        break;
      }
    
    }

    count = 0;

    Serial.write(inByte);

    check = 1;
  }

  if (countTwo == 1){

    while(1 == 1){
      servo_LeftMotor.writeMicroseconds(1500);
      servo_RightMotor.writeMicroseconds(1500);
    }
  
  }

}





