//IMU LIBRARIES
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define BNO055_SAMPLERATE_DELAY_MS (100)

//IMU Initialization
Adafruit_BNO055 bno = Adafruit_BNO055(55);
int azm=0;//0 up, 1 right, 2 down, 3 right
float up;
float down;
float Lleft;
float Rright;

//IR Sensor Pins
#define FRONT_RIGHT A1
#define FRONT_LEFT A2
//#define FRONT_CENTER A2
#define BACK_RIGHT A0
#define BACK_LEFT A3


//TODO: fine tune threshold
const int numAvg = 10;

//threshold for detecting objects head on
const int front_thresh = 450;

//threshold for orthogonal sensor detection (state 2)
const int back_thresh = 300;

//for navigating square blocks
const int unit_distance = 66 / 3;

//control of speed, INVERSE
int speed_control = 0;

//timing required to make sure wheels clear obstacle
const int wheel_clear = 5;

// sensor initialization
int front_right = 0;
int front_left = 0;
int front_center = 0;
int back_right = 0;
int back_left = 0;

int state = 0;
int turn_dir = -1; //right = 0, left = 1
bool blocked = false;

//MOTOR PIN SETUP
int left=7;
int l1=8;
int l2=9;
int right=4;
int r1=5;
int r2=6;
volatile int ldist=0;
volatile int rdist=0;

void setup() {
  
  Serial.println("Setting up...");
  
  Serial.begin(9600);


  pinMode(left, OUTPUT);//DC motor on/off
  pinMode(l1, OUTPUT);//DC direction 1
  pinMode(l2, OUTPUT);//DC direction 2
  pinMode(right, OUTPUT);//DC motor on/off
  pinMode(r1, OUTPUT);//DC direction 1
  pinMode(r2, OUTPUT);//DC direction 2 
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  
  pinMode(13, OUTPUT); //LED for IMU calibration 

  //Encoder setup
  attachInterrupt(digitalPinToInterrupt(2), RightDistance, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), LeftDistance, FALLING);

  calibrateIMU();
}

void loop() {

  //sensorTest();

  //move into initial position
  int back_left = sample(BACK_LEFT, numAvg);
  while(back_left < back_thresh) {
    Serial.println(back_left);
    moveForward(1);
    back_left = sample(BACK_LEFT, numAvg);
  }
  obstacle_avoidance(unit_distance * 0.75);
  

  
  int test = 0;
  boolean temp_blocked = false;
  
  //try path 1
  test = obstacle_avoidance(unit_distance * 3);
  
  //if blocked, bo back to start
  if(test != 0) {
    temp_blocked = true;
    obstacle_avoidance(test - 5);
  } else {
    turnRight();
    test = obstacle_avoidance(unit_distance * 3);
    
    if(test != 0) {
      temp_blocked = true;
      obstacle_avoidance(test - 5);
      
      turnLeft();
      obstacle_avoidance(unit_distance * 3);
    } else {
      turnLeft();
    }
  }
  
  //try path 2 if blocked
  if(temp_blocked) {
    turnLeft();
    obstacle_avoidance(unit_distance * 3 - 7);
    delay(1000);
    turnLeft();
    obstacle_avoidance(unit_distance * 3);
  }
  

  //move to exit choice
  obstacle_avoidance(unit_distance * 2.5);

  temp_blocked = false;

  turnRight();
  test = obstacle_avoidance(unit_distance * 1.25);
  delay(1000);
  turnLeft();


  if(sample(BACK_LEFT, numAvg) > (250)) { //exit blocked
    turnLeft();
    turnLeft();
    obstacle_avoidance(unit_distance * 2.5);
    delay(1000);
    turnRight();
    moveForward(unit_distance * 2);
    delay(1000);
    turnRight();
    obstacle_avoidance(unit_distance * 2.5);
    delay(1000);
  } else {
    turnLeft();

    moveForward(unit_distance);
  }

  //send signal to crane to begin
  Serial.println('x');

  move_hospital();
  delay(2000000); //TODO: exit program
}


//keep track of total blockage
int num_blocked = 0;
    
int obstacle_avoidance(int dur) {
  state = 0;
  turn_dir = -1;
  int curr_dur = 0;
  blocked = false;
  
  while(curr_dur < dur) {
    //state 0: forward
    changeState(state);
    curr_dur++;
  
    //state 1: determine direction, rotate  degrees
    if(checkFront()) {
      state = 1;
      changeState(state);
    }
  
    
    
    //state 2: move to free position
    if(turn_dir != -1) {
      state = 2;
      changeState(state);

      
      while(blocked) {

        Serial.println(num_blocked);
        //try turning the other way if blocked
        //if has been blocked more than once, path completely blocked
        if(blocked && (num_blocked >= 1)) {

          //reset params
          blocked = false;
          num_blocked = 0;
          
          //turn in direction back towards start
          (turn_dir == 0) ? turnRight() : turnLeft();
          return curr_dur;

          
        }
        
        else if(blocked && (num_blocked < 1)) {
          num_blocked++;
          blocked = false;
          
          //change direction and go back to state 2
          turn_dir = (turn_dir == 0) ? 1 : 0;
          turnRight();
          turnRight();
          changeState(state);
        }
      }

      state = 3;
      changeState(state);
    }
  
    state = 0;
    
  }

  delay(200);
  return 0;
}


int sample(int ir, int num) {
  int val = 0;
  for(int i = 0; i < num; i++) {
    val += analogRead(ir);
  }
  return val / num;
}


void changeState(int state) {

  
  if(state == 0) {
    Serial.println("State 0 - Forward...");
    moveForward(1);
  } else if(state == 1) {
    //TODO: may want to take multiple measurements, look at last few samples?
    Serial.println("State 1 - Turning...");
    back_right = sample(BACK_RIGHT, numAvg);
    back_left = sample(BACK_LEFT, numAvg);

    turn_dir = (back_right < back_left) ? 0 : 1;
    (turn_dir == 0) ? turnRight() : turnLeft();
    
  } else if(state == 2) {
    //Serial.println("State 2 - Moving to free space...");
    int back = (turn_dir == 0) ? sample(BACK_LEFT, numAvg) : sample(BACK_RIGHT, numAvg); //sample of back sensor

    //keep moving while there is something blocking, and check that forward is free
    blocked = false;
    while((back > back_thresh)) {
      moveForward(1);
      blocked = checkFront();
      if(blocked) {
        break;
      }

      //if turned right, check back_left
      back = (turn_dir == 0) ? sample(BACK_LEFT, numAvg) : sample(BACK_RIGHT, numAvg);
    }

    //move foreward a bit to ensure that wheels do not catch
    for(int i = 0; i < wheel_clear; i++) {
      //drive forward
      moveForward(1);
      blocked = checkFront();
      if(blocked) {
        break;
      }
    }
  } else if(state == 3) {
    //Serial.println("State 3 - Turning...");
    (turn_dir == 0) ? turnLeft() : turnRight();

    //reset turn direction parameter
    turn_dir = -1;
  }
}


bool checkFront() {
  front_right = sample(FRONT_RIGHT, numAvg);
  front_left = sample(FRONT_LEFT, numAvg);
  //front_center = sample(FRONT_CENTER, numAvg);

  //return true if obstructed
  return ((front_right > front_thresh) || (front_left > front_thresh) );//||  (front_center> front_thresh));     
}

void turnRight(){
  float currentHeading=0;
  float newHeading;
  if(azm==0){
    newHeading=Rright;
    azm=1;
  }else if(azm==1){
    newHeading=down;
    azm=2;
  }else if(azm==2){
    newHeading=Lleft;
    azm=3;
  }else if(azm==3){
    newHeading=up;
    azm=0;
  }
  ldist=0;
  rdist=0;
  int oldxr=rdist;//intermediate variables for while loops
  int oldxl=ldist;
  digitalWrite(l1,HIGH);//Left wheel revese
  digitalWrite(l2,LOW);
  digitalWrite(r1,LOW);//Right wheel forward
  digitalWrite(r2,HIGH);
  while(abs(newHeading-currentHeading)>4){
    sensors_event_t event;
    bno.getEvent(&event);
    currentHeading=event.orientation.x;
    digitalWrite(right,HIGH);
    while(rdist==oldxr){}//Right wheel move twice as fast as left wheel
    digitalWrite(right,LOW);
    oldxr=rdist;
    digitalWrite(left,HIGH);
    while(ldist==oldxl){}
    digitalWrite(left,LOW);
    oldxl=ldist;
    if(abs(newHeading-currentHeading)<45){
      delay(5);
    }else if(abs(newHeading-currentHeading)<20){
      delay(45);
    }
  }
  return; 
 
}
void turnLeft(){
  float currentHeading=0;
  float newHeading;
  if(azm==0){
    newHeading=Lleft;
    azm=3;
  }else if(azm==1){
    newHeading=up;
    azm=0;
  }else if(azm==2){
    newHeading=Rright;
    azm=1;
  }else if(azm==3){
    newHeading=down;
    azm=2;
  }
  ldist=0;
  rdist=0;
  int oldxr=rdist;//intermediate variables for while loops
  int oldxl=ldist;
  digitalWrite(l1,LOW);//Left wheel revese
  digitalWrite(l2,HIGH);
  digitalWrite(r1,HIGH);//Right wheel forward
  digitalWrite(r2,LOW);
  while(abs(newHeading-currentHeading)>4){
    sensors_event_t event;
    bno.getEvent(&event);
    currentHeading=event.orientation.x;
    digitalWrite(right,HIGH);
    while(rdist==oldxr){}//Right wheel move twice as fast as left wheel
    digitalWrite(right,LOW);
    oldxr=rdist;
    digitalWrite(left,HIGH);
    while(ldist==oldxl){}
    digitalWrite(left,LOW);
    oldxl=ldist;
    if(abs(newHeading-currentHeading)<45){
      delay(5);
    }else if(abs(newHeading-currentHeading)<20){
      delay(45);
    }
  }
  return; 
  
}
void moveForward(int dist){//forward method, takes number of shaft revolutions
  int lcorrect=0;
  int rcorrect=0;
  float currentHeading=0;
  float newHeading;
  if(azm==0){
    newHeading=up;
  }else if(azm==1){
    newHeading=Rright;
  }else if(azm==2){
    newHeading=down;
  }else if(azm==3){
    newHeading=Lleft;
  }
  ldist=0;//rezeros distance counter to avoid issues with int rollover
  rdist=0;
  int oldxr=rdist;//intermediate variables for while loops
  int oldxl=ldist;
  digitalWrite(l1,HIGH);//both wheels forward
  digitalWrite(l2,LOW);
  digitalWrite(r1,HIGH);
  digitalWrite(r2,LOW); 
  while(ldist<25*dist){//run until robot has moved far enough forward
    sensors_event_t event;
    bno.getEvent(&event);
    currentHeading=event.orientation.x;
    if(newHeading-currentHeading>4){
      lcorrect=1;
    }else if(newHeading-currentHeading<-4){
      rcorrect=1;
    }else{
      lcorrect=0;
      rcorrect=0;
    }
    digitalWrite(right,HIGH);//pair of while loops to ensure that one wheel does not get too far
    while(rdist==oldxr+rcorrect){}//ahead of other wheel, uses encoders to monitor
    digitalWrite(right,LOW);
    oldxr=rdist;
    digitalWrite(left,HIGH);
    while(ldist==oldxl+lcorrect){}
    digitalWrite(left,LOW);
    oldxl=ldist;
    delay(speed_control);
  }
  return;
}

void RightDistance(){//handles interrupt from right wheel motor encoder
  rdist++;
}

void LeftDistance(){////handles interrupt from left wheel motor encoder
  ldist++;
}

void sensorTest() {
  front_right = sample(FRONT_RIGHT, numAvg);
  front_left = sample(FRONT_LEFT, numAvg);
  back_right = sample(BACK_RIGHT, numAvg);
  back_left = sample(BACK_LEFT, numAvg);
  Serial.print(back_left);
  Serial.print('\t');
  Serial.print(front_left);
  Serial.print('\t');
  Serial.print(front_right); 
  Serial.print('\t');
  Serial.print(back_right);
  Serial.print('\n');
}

void calibrateIMU() {
  if(!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  digitalWrite(13, HIGH);

  bno.setExtCrystalUse(true);
  delay(200);
  Serial.println("Calibrate sensor");
  delay(10000);
  digitalWrite(13, LOW);
  Serial.println("Stop moving");
  delay(4000);

  sensors_event_t event;
  bno.getEvent(&event);
  up=event.orientation.x;
  if(up>270){
    Lleft=up-90;
    down=up-180;
    Rright=up-270;
  }else if(up>180){
    Lleft=up-90;
    down=up-180;
    Rright=up+90;
  }else if(up>90){
    Lleft=up-90;
    down=up+180;
    Rright=up+90;
  }else{
    Lleft=up+270;
    down=up+180;
    Rright=up+90;
  }

}

void move_{
  
 int incomingByte;
  while (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    incomingByte = Serial.read();
    // if it's a capital H (ASCII 72), turn on the LED:
    if (incomingByte == 'a') {
      moveForward(2*unit_distance);
      break;
    }
  }
}

