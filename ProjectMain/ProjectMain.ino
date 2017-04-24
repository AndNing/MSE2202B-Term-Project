/*
 * MSE 2202 Term Project Code
 * 
 * Authors: Andrew Ning, Mohammed Asim Iqbal Zafar, Matthew Bertuzzi, Mohammed Ali Sarfraz
 * Date: 17/03/21
 * Language: Arduino
 * The code below has been designed for microcontroller two of the MSEduino
 */

/*The Wire library hass been included to allow for I2C ommunication between the 
two microcontrollers in the MSEduino in a master-slave configuration. The servo library is included
to allow for the operation of servo motors.*/

#include <Wire.h>
#include <Servo.h>

/*Below includes the initialization of all servo objects for the four servos and two motors used in the design of the mechatronic system.
All servos, motors, and sensors have had the constant integer corresponding to the pinout used for the component defined and initialized.
Default motor speed have also been set for the mechatronic system so that the speed can be easily adjusted. Finally, miscellaneous long and integer
values have been defined and initialized for use with various sensors, case tracking, and timing in the design.*/

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
const int ci_Stopper = 13;

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
unsigned long refTimeTwo = 0;

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

/*The serial monitor begins reading at a 2400 baud rate because the default, 9600 baud rate is too fast to receive/pick-up a proper
value for the infrared signal transmitted by the A/E and I/O pyramids. Additionally, the wire connects to port 8 during the
set-up, allowing for the reception of data from microcontroller one. Within the setup portion of the arduino code, all pins in
use are initialized as either inputs and ouputs, while all servos and motors are initalized appropriately.*/

  Serial.begin(2400);
  Wire.begin(8);
  
/*wait for serial port to connect*/
  
  while (!Serial) {
    ;
  }

/*Set-Up Cube Switch*/
  
  pinMode(A2, INPUT);

/*Set-Up Pyramid Switch*/
  
  pinMode(A1, INPUT);

/*Set-Up Stopper Servo*/
  
  pinMode(ci_Stopper, OUTPUT);
  servo_Stopper.attach(ci_Stopper);

/*Set-Up Platform Servo*/
  
  pinMode(ci_Platform, OUTPUT);
  servo_Platform.attach(ci_Platform);

/*Set-Up Magnet Servo*/
  
  pinMode(ci_Magnet, OUTPUT);
  servo_Magnet.attach(ci_Magnet);

/*Set-Up Shovel Servo*/
  
  pinMode(ci_Shovel, OUTPUT);
  servo_Shovel.attach(ci_Shovel);
 
/*Set Up Ultrasonic Sensors*/
  
  pinMode(ci_Ultrasonic_Ping_Top, OUTPUT);
  pinMode(ci_Ultrasonic_Data_Top, INPUT);

  pinMode(ci_Ultrasonic_Ping_Side_Back, OUTPUT);
  pinMode(ci_Ultrasonic_Data_Side_Back, INPUT);

  pinMode(ci_Ultrasonic_Ping_Side_Front, OUTPUT);
  pinMode(ci_Ultrasonic_Data_Side_Front, INPUT);
  
/*Set up drive motors*/
  
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);  

}

void loop() {

/*Once the robot is placed in the initial position in which is to commence operation, the if statement below is executed a single time. The code in the
if statement was not included in the "setup" although it only runs a sigle time because of the potential needed to re-run the statement in an iteration of the code.
Once connected to a power supply (operation did not begin with the push of a button because there were not enough digital I/O ports available on microcontroller two)
the system would callibrate the ultrasonic sensors to approximate the desire distance from the wall, place the servo motors in their starting positions, and initialize
the integer values to read the signals provided by the pyramid in use. The integer values for comparison to the signal read by the IR sensor had to be manually changed,
and then reuploaded depending on which pyramid was used in the arena.*/

  if(countOut == 0){
    
    CallibrateUltrasonics();
    countOut++;
    servo_Shovel.write(160);
    servo_Stopper.write(75);
    servo_Magnet.write(90);
    servo_Platform.write(105);
    IRValOne = 73;                // if A/E pyramid, use 65 and 69, if I/O pyramid, use 73 and 79
    IRValTwo = 79;

  }

/*The main loop code for the mechatronic system operated off of the use of a switch statement with multiple cases, each of which contained code for specific tasks.
A switch statement was used as the underlying structure for the code because separating the functions of the robot into multiple cases allowed for a simple method
to complete specific tasks as soon as other tasks were accomplised. Additionally, based on sensor data obtained or timing, backtracking and following a
set sequence of steps were simplified.*/
  
  switch(countOut){

/*The first case statement involves driving the robot until it has obtained the tessaract cube. In order to obtain the cube, the robot
drives along the wall while keeping the rotating arm at a constant position along the wall. In order to identify if the mechatronic system
has reached the end of the side wall containing the cube, data from the front ultrasonic sensor readings is sent from microcontroller one to
microcontroller two using the "Wire" library on the call of the function, "receiveEvent." If the ultrasonic sensor reading, identified by the
variable "c" falls within the range given in the following if statement, the robot stops, identifying that it has reached the corner, and excutes
a function to obtain the cube in the corner. To drive smoothly along the wall, the mechatronic system excutes the DriveToGetCube() function.
At any time during the traversal of the wall with the cube, if the robot picks up the cube, a lever switch is activated. When activated, the lever
switch reads a value of approximately 1023 due to the 10 bit analog to digital converter. if the value is read consistently for 0.5 seconds, the
next case is entred.*/

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

/*Within the second case statement, the objective is to set-up the robot to begin scanning for the pyramid. The case
provides the opportunity for the robot to turn counter-clockwise to a position perpendicular to the cube wall through
the first while loop. The turn is executed over a 1.1 second period of time. After this, the robot then drives forward
to obtain a more central position in the arena over a 3 second time period.*/

    case 2:{

      Wire.endTransmission();

      refTime = millis();

      while((millis()-refTime)<1100){
        servo_LeftMotor.writeMicroseconds(1380);
        servo_RightMotor.writeMicroseconds(1650);
      }

      while((millis()-refTime)<4100){
        servo_LeftMotor.writeMicroseconds(1680);
        servo_RightMotor.writeMicroseconds(1680);
      }

      countOut++;

      refTimeTwo = millis();

      break;
      
    }

/*In the third case, the robot begins to scan the arena for the infrared signal emitted by the pyramid through the "FindPyramid" function.
Once the pyramid is in an obtainable position in front of the robot, it executes the first if statement and moves to the next case. If
the robot is unable to locate the pyramid after 15 seconds, it reposition itself by moving back to the second case statement.*/

    case 3:{

      FindPyramid();

      if (countTwo == 1){
        countOut++;
        servo_LeftMotor.writeMicroseconds(1500);
        servo_RightMotor.writeMicroseconds(1500);
      }

      if((millis() - refTimeTwo)>15000){
        countOut = 2;
      }

      break;
    }

/*The fourth case statement involves the mechatronic system performin the necessary tasks to pick up the pyramid using the
"PickUpPyramid" function. Once the pyramid hasbeen lifted, the fifth case statement begins.*/

    case 4:{
    
      PickUpPyramid();

      if (countTwo == 1){
        countOut++;
      }

      break;
      
    }

/*The fifth and final case statement was used to place the hoisted pyramid ontop of the tessaract cube. The tasks
executed in this statment were not included in a separate function because the case was often modified in attempt
to opimize the performance. The individual components of the case are described within the case statement.*/

    case 5:{

     refTime = millis();

/*Once the mechatronic system has the pyramid, it reverses for one second in order to move away from the wall, and then stops.*/
    
     while((millis()-refTime)<1000){
        servo_LeftMotor.writeMicroseconds(1400);
        servo_RightMotor.writeMicroseconds(1400);
     }
    
    servo_LeftMotor.writeMicroseconds(1500);
    servo_RightMotor.writeMicroseconds(1500);

/*The for loops below allow the robot to rotate the arm holding the cube until the arm is positioned behind the robot and then
pull back the magnet using a rack and pinion system in order to drop off the tessaract cube*/
    
    for (pos = 105; pos >= 10; pos--) {
      servo_Platform.write(pos);             
      delay(30);
    }

    for (pos = 70; pos >= 20; pos--) {
      servo_Magnet.write(pos);             
      delay(30);
    }

    refTime = millis();

/*When the cube has been dropped, the robot begins to reverse over the cube for 2.3 seconds. The time frame allows the
robot to move back until the front edge of the pyramid catches and holds the tessaract.*/
    
    while((millis()-refTime)<2300){
        servo_LeftMotor.writeMicroseconds(1400);
        servo_RightMotor.writeMicroseconds(1400);
     }

     refTime = millis();

/*When the tessaract cube is under the front endge of the pyramid, the robot quickly drives forward and then shakes side-to-side
to centre the cube beneath the hole in the pyramid before stopping.*/

     while((millis()-refTime)<100){
        servo_LeftMotor.writeMicroseconds(1650);
        servo_RightMotor.writeMicroseconds(1650);
     }
     
     while((millis()-refTime)<350){
        servo_LeftMotor.writeMicroseconds(1390);
        servo_RightMotor.writeMicroseconds(1650);
     }

     while((millis()-refTime)<430){
        servo_LeftMotor.writeMicroseconds(1650);
        servo_RightMotor.writeMicroseconds(1390);
     }

     servo_LeftMotor.writeMicroseconds(1500);
     servo_RightMotor.writeMicroseconds(1500);

/*Once the tessaract cube is in the desired position, the shovel used to hoist the pyramid is lowered
to the ground and the stopper arm used to hold the pyramid in place is released.*/

     for (pos = 180; pos >= 38; pos--) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        servo_Shovel.write(pos);             // tell servo to go to position in variable 'pos'
        delay(30);                       // waits 15ms for the servo to reach the position
     }

     servo_Stopper.write(75);

     delay(500);

     refTime = millis();

/*After a 0.5 second delay, the mechatronic system reverses at a high speed to remove the pyramid from the shovel.*/

     while((millis()-refTime)<2000){
        servo_LeftMotor.writeMicroseconds(1350);
        servo_RightMotor.writeMicroseconds(1350);
     }

      countOut++;

      break;
      
    }

/*Upon the completion of the fifth case statement, the objective is assumed complete. The sixth case statement
is used to stop the robot.*/

    case 6:{

      servo_LeftMotor.writeMicroseconds(1500);
      servo_RightMotor.writeMicroseconds(1500);
      
      break;
      
    }

/*Default statement used for compiling issues.*/

    default:{
      break;
    }

  }
  
}

/*The three ping functions below correspond to ultrasonic sensors on the top of the robot and the side of it.
All ping functions follow the same format, but use different variables corresponding to the specific sensor.
The code for the ping functions was extracted from the code used for the line following robots. By sending a pulse
over a 10 microsecond period and taking the time for the input to go from high to low, distances can be converted to a value.*/

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

/*When receiving data from microcontroller one, the value read from the first microcontroller is
a byte value. As a result, it is casted as an integer, then multiplied by 20 because the data
sent from the first microcontroller is an ultrasonic sensor value which is typically greater then 256.
To simplify the sending of such data, the value is divided by 20, sent, then multiplied by 20 to
bring an approximated value back. Since the reading does not have to be too accurate, this method
is simpler then breaking up the read integer value into multiple bytes.*/

void receiveEvent(int howMany){
    c = (int(Wire.read()))*20;
}

/*The "DriveToGetCube" function is used to allow the robot to drive straight allong the wall that
the cube is placed on. The function makes use of two ultrasonic sensors placed on the side of the robot,
and the function operates based on the use of callibrated values taken before the operation of the robot begins.
When the function is executed, the robot is travelling backwards, thus the "back" ultrasonic sensor is the
front sensor, while the "front" ultrasonic sensor is the back sensor.*/

void DriveToGetCube(){

/*"GetUltrasonicValues" is called to take an averaged input from the two ultrasonic sensors on the side of the robot.*/

  GetUltrasonicValues();

/*Using a series of in and else-if statements, the mechatronic system is able to identify whether it should drive straight or turn.
In the first if statement, if both side ultrasonic sensors read a value equal to the callibrated values, plus or minus 20, then
the robot travels straight.*/
  
  if((ul_Echo_Time_Side_Front_Val>=(frontUltraVal-20))&&(ul_Echo_Time_Side_Front_Val<=(frontUltraVal+20))&&(ul_Echo_Time_Side_Back_Val>=(backUltraVal-20))&&(ul_Echo_Time_Side_Back_Val<=(backUltraVal+20))){
      servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed_Cube);
      servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed_Cube);
      //Serial.println("STRAIGHT");
  }

/*In the else-if statement below, if the back side ultrasonic sensor (front side) is too close to the wall, while the front
ultrasonic sensor (back side) is greater than the minimum bound of the desired distance, the robot turns right to centre
itself.*/

  else if((ul_Echo_Time_Side_Back_Val <(backUltraVal-20))&&(ul_Echo_Time_Side_Front_Val>=(frontUltraVal-20))){
      servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed_Cube);
      servo_RightMotor.writeMicroseconds(1320);
      //Serial.println("TURN RIGHT");
  }

/*In the else-if statement below, if the front side ultrasonic sensor (back side) is too close to the wall, while the back
ultrasonic sensor (front side) is greater than the minimum bound of the desired distance, the robot turns left to centre
itself.*/

  else if((ul_Echo_Time_Side_Front_Val<(frontUltraVal-20))&&(ul_Echo_Time_Side_Back_Val>=(backUltraVal-20))){
      servo_LeftMotor.writeMicroseconds(1320);
      servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed_Cube);
      //Serial.println("TURN LEFT");
  }

/*In the else-if statement below, if the back side ultrasonic sensor (front side) and
front ultrasonic sensor (back side) are too close to the wall, the robot turns right to centre
itself.*/
  
  else if(((ul_Echo_Time_Side_Front_Val)<(frontUltraVal-20))&&((ul_Echo_Time_Side_Back_Val)<(backUltraVal-20))){
      servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed_Cube);
      servo_RightMotor.writeMicroseconds(1320);
      //Serial.println("TURN RIGHT");
  }

/*In the else-if statement below, if the back side ultrasonic sensor (front side) and
front ultrasonic sensor (back side) are too far from the wall, the robot turns left to centre
itself.*/

  else if((ul_Echo_Time_Side_Front_Val>(frontUltraVal+20))&&(ul_Echo_Time_Side_Back_Val>(backUltraVal+20))){
      servo_LeftMotor.writeMicroseconds(1320);
      servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed_Cube);
      //Serial.println("TURN LEFT");
  }
}

/*The "CallibrateUltrasonics" function is used to to take comparison values for the "DriveToGetCube" function
above. Using a for loop, 40 readings are taken from both side ultrasonic sensors, and then averaged in order to get
a more accurate and less volatile value for comparison. The 50 second delay is used in order to allow for less error.*/

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

/*"GetUltrasonicValues" is functionally similar to "CallibrateUltrasonics," however "GetUltrasonicValues"
is use to average 40 readings from both side ultrasonic sensors while driving, There was a need for averaging
40 readings instead of using the unaveraged values because the sensor readings were somewhat volatile. By averaging
the readings taken from the sensors, although there was a time dealy between adjustments, the robot travelled
more smoothly.*/

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

/*"FindPyramid" is used to locate the position of the pyramid using the IR sensor in combination with the ultrasonic
sensor that faces downward on the top of the robot. During this point in the operation of the robot, it is travelling
in the forward direction, but does not take any inputs to identify whether it has collided with a wall.*/

void FindPyramid(){

/*The variable "count" is used below to navigate through the if statements of the function. Within the first
if statement the while loop is used on iterations of the function after the robot has located the infrared
signal emitted by the pyramid for the first time. Once the signal has been found once, a value of -1 is written
to the serial monitor. Through doing so, the infrared signal will not be read until the serial monitor is flused of
the multiple possible readings taken from the first scan of the pyramid. while taking readings, the robot rotates counterclockwise
until it identifies a signal from the pyramid. If the required signal is found, it stops and moves to the next if
statement.*/

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

/*In the second if statement monitored by "count," the robot drives forward for 2 seconds while searching
for the pyramid.*/

  if (count == 1){

/*The while loop below is used to flush out random readings from the top ultrasonic sensor that faces downward. When
the ping function is called for the sensor, it often reads random values over a short period before stabilizing. The
0.3 second period of readings is used to prevent the garbage values from being taken as data.*/

      while((millis()-refTime)<300){
        PingTop();
        Serial.println("HHHH");
        Serial.println(ul_Echo_Time_Top);
      }

/*Once the ultrasonic sensor data has been cleared, the mechatronic system drives forward (in the direction
of the pyramid) for two seconds. While driving forward, readings are taken from the top ultrasonic sensor.
If a zero reading is taken, or a reading greater then 1600 is taken, then the pyramid is infront of the robot
and the next if statement begins. Due to the sloped surface of the pyramid, it tended to provide garbage values from readings taken, which was
used as a state. If the pyramid is infront of the robot, countTwo is increased to one in order to allow movement to the
next case.*/
    
    while((millis()-refTime)<2300){
      servo_LeftMotor.writeMicroseconds(1680);
      servo_RightMotor.writeMicroseconds(1680);

      PingTop();

      Serial.println(ul_Echo_Time_Top);

      if((ul_Echo_Time_Top>1600)||(ul_Echo_Time_Top == 0)){
        countTwo = 1;
        count = -5;
        break;
      }
      
    }

    count++;

    refTime = millis();

  }

/*The third if statement is used to rotate the robot left and right in order to reposition it for
a new scan, or find the pyramid in front of it.*/

  if (count == 2){

/*The while loop below is used to flush out random readings from the top ultrasonic sensor that faces downward. When
the ping function is called for the sensor, it often reads random values over a short period before stabilizing. The
0.3 second period of readings is used to prevent the garbage values from being taken as data.*/

    while((millis()-refTime)<300){
        PingTop();
        Serial.println("HHHH");
        Serial.println(ul_Echo_Time_Top);
    }

/*Once the ultrasonic sensor data has been cleared, the mechatronic system rotates counterclockwise for 0.5 seconds.
While rotating, readings are taken from the top ultrasonic sensor. If a zero reading, or a reading greater
then 1600 is taken, then the pyramid is infront of the robot. If the pyramid is infront of the robot, countTwo is
increased to one in order to allow movement to the next case.*/
      
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

/*The while loop below is used to flush out random readings from the top ultrasonic sensor that faces downward. When
the ping function is called for the sensor, it often reads random values over a short period before stabilizing. The
0.3 second period of readings is used to prevent the garbage values from being taken as data.*/

    while((millis()-refTime)<300){
        PingTop();
        Serial.println("HHHH");
        Serial.println(ul_Echo_Time_Top);
    }
    
/*Once the ultrasonic sensor data has been cleared, the mechatronic system rotates clockwise for 1.4 seconds.
While rotating, readings are taken from the top ultrasonic sensor. If a zero reading, or a reading greater
then 1600 is taken, then the pyramid is infront of the robot. If the pyramid is infront of the robot, countTwo
is increased to one in order to allow movement to the next case. If the pyramid is not found, them count is reset
to 0 to restart the scanning sequence. Additionally, "inByte" contains a value of -1, which is written to the serial
monitor so that it can be identified when to begin the re-scanning for the pyrmaid infrared signal.*/
      
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

/*"GetCubeInCorner" is used to rotate the arm used to pick up the tessaract while extending the rack which the
permanent magnet is attached to in order to reach out and grab the cube if it is located in the corner. Once the
front ultrasonic sensor reads that the robot has reached the end wall, the for loop allows the arm to slowly swing
out and reach for the tessaract.*/

void GetCubeInCorner(){
  
  for (pos = 105, posTwo = 70; pos >= 50; pos--, posTwo++) {
    servo_Platform.write(pos); 
    servo_Magnet.write(posTwo);     
    delay(15);                    
  }
    
}

/*"PickUpPyramid" once the pyramid has been located in front of the robot using the ultrasonics sensors, the robot
completes a series of tasks in order to pick it up.*/

void PickUpPyramid(){

/*Initially, the shovel of the robot is lowered so that it is level with the ground, so that it
can scoop up the pyramid.*/
  
  servo_Shovel.write(30);
  delay(1000);

  refTime = millis();

/*Once the shovel is lowered, the robot executes the while loop below until the pyramid has been scooped.
For the first two seconds, the robot travels straight. If the pyramid does not make contact with the lever
switch on the back of the shovel, the robot quickly swivels left and right to reposition the pyramid in the second
and third if statements. The final statement is use to reset the timer and complete another attempt. When competing
the function, the aim is to move the pyramid against a wall or conduit until it makes contact with the lever switch.
Once contact is made, the pyramid is on the shovel and the while loop is exited.*/
  
  while(analogRead(A1)<100){
    
    if((millis()-refTime)<2000){
      
      servo_LeftMotor.writeMicroseconds(1600);
      servo_RightMotor.writeMicroseconds(1600);

    }

    else if((millis()-refTime)<2300){
      servo_LeftMotor.writeMicroseconds(1350);
      servo_RightMotor.writeMicroseconds(1680);
    }

    else if((millis()-refTime)<2700){
      servo_LeftMotor.writeMicroseconds(1680);
      servo_RightMotor.writeMicroseconds(1350);
    }

    else if((millis()-refTime)>2700){
      refTime = millis();
    }
    
  }

/*The robot stops and the stopper arm lowers to hold the pyramid in place.*/

  servo_LeftMotor.writeMicroseconds(1500);
  servo_RightMotor.writeMicroseconds(1500);
  servo_Stopper.write(35);
  delay(1000);
  
  refTime = millis();

/*The robot reverses for one second in order to move the pyramid away from the wall.*/

  while((millis()-refTime)<1000){
    servo_LeftMotor.writeMicroseconds(1420);
    servo_RightMotor.writeMicroseconds(1420);
  }
  
  servo_LeftMotor.writeMicroseconds(1500);
  servo_RightMotor.writeMicroseconds(1500);
  delay(1000);

/*The robot raises the shovel to a position in which it could driver over the tessaract cube
with the pyramid and begins to reverse before moving to the next case.*/

   for (pos = 30; pos <= 165; pos++) {
    servo_Shovel.write(pos);
    delay(30);
  }

  servo_LeftMotor.writeMicroseconds(1400);
  servo_RightMotor.writeMicroseconds(1400); 
  
}

