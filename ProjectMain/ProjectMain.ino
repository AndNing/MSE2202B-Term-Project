
/*
 * MSE 2202 Term Project Code
 * 
 * Authors: Andrew Ning, Mohammed Asim Iqbal Zafar, Matthew Bertuzzi, Mohammed Ali Sarfraz
 * Date: 17/03/21
 * Language: Arduino
 */


#include <Wire.h>
#include <Servo.h>

Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_Platform;
Servo servo_Magnet;
Servo servo_Shovel;
Servo servo_Stopper;

const int ci_Ultrasonic_Ping_Top = 2;
const int ci_Ultrasonic_Data_Top = 3;
const int ci_Ultrasonic_Ping_Side_Back = 4;
const int ci_Ultrasonic_Data_Side_Back = 5;
const int ci_Ultrasonic_Ping_Side_Front = 6;
const int ci_Ultrasonic_Data_Side_Front = 7;
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int ci_Platform = 10;
const int ci_Magnet = 11;
const int ci_Shovel = 12;
const int ci_Stopper = 13;//75 = up, 


unsigned int ui_Left_Motor_Speed = 1585;
unsigned int ui_Right_Motor_Speed = 1585;
unsigned int ui_Left_Motor_Speed_Cube = 1400;
unsigned int ui_Right_Motor_Speed_Cube = 1400;

unsigned long ul_Echo_Time_Top;
unsigned long ul_Echo_Time_Side_Back = 0;
unsigned long ul_Echo_Time_Side_Front = 0;
unsigned long ul_Echo_Time_Side_Back_Val = 0;
unsigned long ul_Echo_Time_Side_Front_Val = 0;
unsigned long refTime = 0;
unsigned long backUltraVal = 0;
unsigned long frontUltraVal = 0;

int count = 0;
int pingPrev = 0;
int val = 0;
int lastVal=0;
int inByte = -1;
int check = 0;
int countTwo = 0;
int c = 0;
int pos = 0;
int posTwo = 0;
int countOut = 0;
int IRValOne = 0;
int IRValTwo = 0;


void setup() {

  Serial.begin(2400);
  Wire.begin(8);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  //Set-Up Cube Switch
  pinMode(A2, INPUT);

  //Set-Up Pyramid Switch
  pinMode(A1, INPUT);

  //Set-Up Stopper Servo
  pinMode(ci_Stopper, OUTPUT);
  servo_Stopper.attach(ci_Stopper);

  //Set-Up Platform Servo
  pinMode(ci_Platform, OUTPUT);
  servo_Platform.attach(ci_Platform);

  //Set-Up Magnet Servo
  pinMode(ci_Magnet, OUTPUT);
  servo_Magnet.attach(ci_Magnet);

  //Set-Up Shovel Servo
  pinMode(ci_Shovel, OUTPUT);
  servo_Shovel.attach(ci_Shovel);
 
  //Set Up Ultrasonic Sensors
  pinMode(ci_Ultrasonic_Ping_Top, OUTPUT);
  pinMode(ci_Ultrasonic_Data_Top, INPUT);

  pinMode(ci_Ultrasonic_Ping_Side_Back, OUTPUT);
  pinMode(ci_Ultrasonic_Data_Side_Back, INPUT);

  pinMode(ci_Ultrasonic_Ping_Side_Front, OUTPUT);
  pinMode(ci_Ultrasonic_Data_Side_Front, INPUT);
  
  // set up drive motors
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);  

}

void loop() {
  //bt_Enabled = digitalRead(ci_Enable_Switch);

  //if (bt_Enabled) {

  if(countOut == 0){
    CallibrateUltrasonics();
    countOut++;
    servo_Shovel.write(160);
    servo_Stopper.write(75);
    servo_Magnet.write(90);
    servo_Platform.write(105);
    IRValOne = 73;//AE = 65 69, IO = 73 79
    IRValTwo = 79;

  }
  
  switch(countOut){

    case 1:{

      Wire.onReceive(receiveEvent);

      if ((c>240) && (c<300)){

        servo_LeftMotor.writeMicroseconds(1500);
        servo_RightMotor.writeMicroseconds(1500);
        GetCubeInCorner();
      }
      
      DriveToGetCube();


      while(val>1020){
        if((millis()-refTime)>500){
          countOut++;
          break;
        }
        val = analogRead(A2);
      }

      refTime = millis();

      break;
      
    }

    case 2:{

      Wire.endTransmission();

      refTime = millis();

      while((millis()-refTime)<2000){
        servo_LeftMotor.writeMicroseconds(1500);
        servo_RightMotor.writeMicroseconds(1500);
      }

      countOut++;

      break;
      
    }

    case 3:{

      FindPyramid();

      if (countTwo == 1){
        countOut++;
        Serial.println("YES");
        servo_LeftMotor.writeMicroseconds(1500);
        servo_RightMotor.writeMicroseconds(1500);
      }

      break;
    }

    case 4:{
    
      PickUpPyramid();

      if (countTwo == 1){
        countOut++;
        Serial.println("YES");
      }

      break;
      
    }

    case 5:{//Drop off cube and lower pyramid onto it

     refTime = millis();
    
     while((millis()-refTime)<1000){
        servo_LeftMotor.writeMicroseconds(1400);
        servo_RightMotor.writeMicroseconds(1400);
     }
    
    servo_LeftMotor.writeMicroseconds(1500);
    servo_RightMotor.writeMicroseconds(1500);
    
    for (pos = 105; pos >= 10; pos--) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      servo_Platform.write(pos);             // tell servo to go to position in variable 'pos'
      delay(30);
    }

    for (pos = 70; pos >= 20; pos--) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      servo_Magnet.write(pos);             // tell servo to go to position in variable 'pos'
      delay(30);
    }

    refTime = millis();
    
    while((millis()-refTime)<2100){
        servo_LeftMotor.writeMicroseconds(1400);
        servo_RightMotor.writeMicroseconds(1400);
     }

     refTime = millis();

     while((millis()-refTime)<200){
        servo_LeftMotor.writeMicroseconds(1390);
        servo_RightMotor.writeMicroseconds(1650);
     }

     while((millis()-refTime)<280){
        servo_LeftMotor.writeMicroseconds(1650);
        servo_RightMotor.writeMicroseconds(1390);
     }

     servo_LeftMotor.writeMicroseconds(1500);
     servo_RightMotor.writeMicroseconds(1500);

     for (pos = 180; pos >= 38; pos--) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        servo_Shovel.write(pos);             // tell servo to go to position in variable 'pos'
        delay(30);                       // waits 15ms for the servo to reach the position
     }

     servo_Stopper.write(75);

     delay(500);

     refTime = millis();

     while((millis()-refTime)<2000){
        servo_LeftMotor.writeMicroseconds(1350);
        servo_RightMotor.writeMicroseconds(1350);
     }

      countOut++;

      break;
      
    }

    case 6:{

      servo_LeftMotor.writeMicroseconds(1500);
      servo_RightMotor.writeMicroseconds(1500);
      
      break;
      
    }
      

    default:{
      break;
    }


  //}
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
    c = (int(Wire.read()))*5;
    //Serial.println(c);
}

void DriveToGetCube(){

  GetUltrasonicValues();
  
  if((ul_Echo_Time_Side_Front_Val>=(frontUltraVal-20))&&(ul_Echo_Time_Side_Front_Val<=(frontUltraVal+20))&&(ul_Echo_Time_Side_Back_Val>=(backUltraVal-20))&&(ul_Echo_Time_Side_Back_Val<=(backUltraVal+20))){
      servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed_Cube);
      servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed_Cube);
      //Serial.println("STRAIGHT");
  }

  else if((ul_Echo_Time_Side_Back_Val <(backUltraVal-20))&&(ul_Echo_Time_Side_Front_Val>=(frontUltraVal-20))){
      servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed_Cube);
      servo_RightMotor.writeMicroseconds(1320);
      //Serial.println("TURN RIGHT");
  }

  else if((ul_Echo_Time_Side_Front_Val<(frontUltraVal-20))&&(ul_Echo_Time_Side_Back_Val>=(backUltraVal-20))){
      servo_LeftMotor.writeMicroseconds(1320);
      servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed_Cube);
      //Serial.println("TURN LEFT");
  }
  
  else if(((ul_Echo_Time_Side_Front_Val)<(frontUltraVal-20))&&((ul_Echo_Time_Side_Back_Val)<(backUltraVal-20))){
      servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed_Cube);
      servo_RightMotor.writeMicroseconds(1320);
      //Serial.println("TURN RIGHT");
  }

  else if((ul_Echo_Time_Side_Front_Val>(frontUltraVal+20))&&(ul_Echo_Time_Side_Back_Val>(backUltraVal+20))){
      servo_LeftMotor.writeMicroseconds(1320);
      servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed_Cube);
      //Serial.println("TURN LEFT");
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

    servo_LeftMotor.writeMicroseconds(1380);
    servo_RightMotor.writeMicroseconds(1650);

    if ((val == IRValOne) || (val == IRValTwo)){

      servo_LeftMotor.writeMicroseconds(1500);
      servo_RightMotor.writeMicroseconds(1500);

      count = 1;

      refTime = millis();
  
    }
  
  }

  if (count == 1){ //The pyramid has been located, drive toward the pyramid for 2 seconds

      while((millis()-refTime)<300){
        PingTop();
        Serial.println("HHHH");
        Serial.println(ul_Echo_Time_Top);
      }

    
    while((millis()-refTime)<1500){
      servo_LeftMotor.writeMicroseconds(1680);
      servo_RightMotor.writeMicroseconds(1680);

      PingTop();

      Serial.println(ul_Echo_Time_Top);

      if((ul_Echo_Time_Top>1050)||(ul_Echo_Time_Top == 0)){
        countTwo = 1;
        count = -5;
        break;
      }
      
    }

    count++;

    refTime = millis();

  }

  if (count == 2){

  //Serial.println("Turn 2");


    while((millis()-refTime)<300){
        PingTop();
        Serial.println("HHHH");
        Serial.println(ul_Echo_Time_Top);
    }
      
    while((millis()-refTime)<800){
      servo_LeftMotor.writeMicroseconds(1350);
      servo_RightMotor.writeMicroseconds(1680);

      PingTop();

      Serial.println(ul_Echo_Time_Top);

      if((ul_Echo_Time_Top>1100)||(ul_Echo_Time_Top == 0)){
        countTwo = 1;
        count = -5;
        break;
      }
    
    }

    servo_LeftMotor.writeMicroseconds(1500);
    servo_RightMotor.writeMicroseconds(1500);
    refTime = millis();

    while((millis()-refTime)<300){
        PingTop();
        Serial.println("HHHH");
        Serial.println(ul_Echo_Time_Top);
    }
      
    while((millis()-refTime)<1700){
      servo_LeftMotor.writeMicroseconds(1680);
      servo_RightMotor.writeMicroseconds(1350);

      PingTop();

      Serial.println(ul_Echo_Time_Top);

      if((ul_Echo_Time_Top>1100)||(ul_Echo_Time_Top == 0)){
        countTwo = 1;
        count = -5;
        break;
      }
    
    }

    count = 0;

    Serial.write(inByte);

    check = 1;
  }

}

void GetCubeInCorner(){
  
  for (pos = 105, posTwo = 70; pos >= 50; pos--, posTwo++) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo_Platform.write(pos); 
    servo_Magnet.write(posTwo);             // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
    
}

void PickUpPyramid(){
  
  servo_Shovel.write(30);
  delay(1000);

  refTime = millis();
  
  while(analogRead(A1)<100){
    
    if((millis()-refTime)<1000){
      
      servo_LeftMotor.writeMicroseconds(1600);
      servo_RightMotor.writeMicroseconds(1600);

    }

    else if((millis()-refTime)<1400){
      servo_LeftMotor.writeMicroseconds(1350);
      servo_RightMotor.writeMicroseconds(1680);
    }

    else if((millis()-refTime)<1800){
      servo_LeftMotor.writeMicroseconds(1680);
      servo_RightMotor.writeMicroseconds(1350);
    }

    else if((millis()-refTime)>1400){
      refTime = millis();
    }
    
  }

  servo_LeftMotor.writeMicroseconds(1500);
  servo_RightMotor.writeMicroseconds(1500);
  servo_Stopper.write(35);
  delay(1000);
  
  refTime = millis();

  while((millis()-refTime)<1000){
    servo_LeftMotor.writeMicroseconds(1420);
    servo_RightMotor.writeMicroseconds(1420);
  }
  
  servo_LeftMotor.writeMicroseconds(1500);
  servo_RightMotor.writeMicroseconds(1500);
  delay(1000);

   for (pos = 30; pos <= 180; pos++) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo_Shovel.write(pos);             // tell servo to go to position in variable 'pos'
    delay(30);                       // waits 15ms for the servo to reach the position
  }

  servo_LeftMotor.writeMicroseconds(1400);
  servo_RightMotor.writeMicroseconds(1400); 
  
}





