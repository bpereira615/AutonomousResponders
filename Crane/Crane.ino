#define CARRIAGE_MOTOR 6
#define CARRIAGE_FORWARD 4
#define CARRIAGE_BACKWARD 7

#define CLAW_SERVO 11

//MOTOR PIN SETUP
#define SPOOL 5
#define SPOOL_UP 9
#define SPOOL_DOWN 8
#define SPOOL_INTER 2

#define BASE 10
#define BASE_CLOCK 12
#define BASE_COUNTER 13
#define BASE_INTER 3

//import Servo library
#include <Servo.h>
Servo claw_servo;

volatile int spool_dist=0;
volatile int base_dist=0;


#include <Wire.h>  
#include <PixyI2C.h>
PixyI2C pixy;
int xco;
int yco;
int block;

int sig=0;

//general speed control for motors
int speed_control = 1;

//control for base counter/clockwise
int base_speed = 255;

//for forward/bachward movement of carraige
int unit_time = 100;

//number number of encoder clicks
int unit_encoder = 5;


//for loop hack, use whiles
int idx = 0;


void setup() {
  Serial.begin(9600);

  //pixy initialization
  pixy.init();

  pinMode(CARRIAGE_MOTOR, OUTPUT);
  digitalWrite(CARRIAGE_MOTOR, LOW);
  pinMode(CARRIAGE_FORWARD, OUTPUT);
  digitalWrite(CARRIAGE_FORWARD, LOW);
  pinMode(CARRIAGE_BACKWARD, OUTPUT);
  digitalWrite(CARRIAGE_BACKWARD, LOW);

  pinMode(SPOOL, OUTPUT);//DC motor on/off
  digitalWrite(SPOOL, LOW);
  pinMode(SPOOL_UP, OUTPUT);//DC direction 1
  digitalWrite(SPOOL_UP, LOW);
  pinMode(SPOOL_DOWN, OUTPUT);//DC direction 2
  digitalWrite(SPOOL_DOWN, LOW);
  
  pinMode(BASE, OUTPUT);//DC motor on/off
  digitalWrite(BASE, LOW);
  pinMode(BASE_CLOCK, OUTPUT);//DC direction 1
  digitalWrite(BASE_CLOCK, LOW);
  pinMode(BASE_COUNTER, OUTPUT);//DC direction 2 
  digitalWrite(BASE_COUNTER, LOW);
  
  pinMode(SPOOL_INTER, INPUT);
  pinMode(BASE_INTER, INPUT);

  pinMode(CLAW_SERVO, OUTPUT);


  //Encoder setup
  attachInterrupt(digitalPinToInterrupt(SPOOL_INTER), SpoolDistance, FALLING);
  attachInterrupt(digitalPinToInterrupt(BASE_INTER), BaseDistance, FALLING);
  
  analogWrite(CARRIAGE_MOTOR, 0);

  claw_servo.attach(CLAW_SERVO);
  close_claw();

  
}

void loop(){
  //initial delay for motor spike control


  delay(3000);
  //wait_ambulance();

  Serial.print("BEGIN");

  //move to 45 degree starting point
  idx = 0;
  while(idx < 10) { //45 degrees
    Serial.println(idx);
    ccw();
    idx++;
  }

  //center on the object, first counter/clockwise, then for/backward
  delay(2000);//lets pixy find object
  findObjectTH(1);
  findObjectR(1);

  //fix claw drop when carriage moves out
  digitalWrite(SPOOL_UP,HIGH);
  digitalWrite(SPOOL_DOWN,LOW);
  digitalWrite(SPOOL, HIGH);
  delay(1000);
  digitalWrite(SPOOL, LOW);


  //drop the claw
  open_claw();
  claw_down(1600*3);
  delay(1500);

  
  //grab victim and pick up
  close_claw();
  delay(1500);
  claw_up(4500);
  

  //turn 90 degrees to allow fror ambulance viewing
  idx = 0;
  while(idx < 20) { //90 degrees
    ccw();
    idx++;
  }
  
  //find the ambulance and center carriage
  refresh(2);
  delay(2000);
  findObjectTH(2);
  findObjectR(2);

  //fix claw drop when carriage moves out  
  digitalWrite(SPOOL_UP,HIGH);
  digitalWrite(SPOOL_DOWN,LOW);
  digitalWrite(SPOOL, HIGH);
  delay(1000);
  digitalWrite(SPOOL, LOW);
  

  //lower victim onto ambulance
  claw_down(4850*3);
  
  //release victivm
  delay(1500);
  open_claw();
  delay(1500);
  claw_up(10000); //temp divide by 2


  //signal_ambulance();

  delay(10000000000);
  
}

void refresh(int code){//updates x and y coordinates of selected color code
  block=pixy.getBlocks();
  if(block){

    idx = 0;
    while(idx < block) {
      if(pixy.blocks[idx].signature==code){
        xco=pixy.blocks[idx].x;
        yco=pixy.blocks[idx].y;
      }
      idx++;
    }
  }
}

void findObjectR(int code){//finds color code in R direction
  boolean found = false;
  while(!found){
    refresh(code);
    if(yco<90){
      forward();
    }else if(yco>110){
      back();
    }else{
      found=true;
    }
  }
  digitalWrite(CARRIAGE_FORWARD, HIGH);
  digitalWrite(CARRIAGE_BACKWARD, LOW);
  digitalWrite(CARRIAGE_MOTOR, HIGH);
  delay(50);
  digitalWrite(CARRIAGE_MOTOR, LOW);
}

void findObjectTH(int code){//finds color code in Theta direction
  boolean found = false;
  while(!found){
    refresh(code);
    if(xco>170){//pixy is installed backwards
      ccw();
    }else if(xco<140){
      cw();
    }else{ 
      found=true;
    }
  }
}

void forward(){
  digitalWrite(CARRIAGE_FORWARD, HIGH);
  digitalWrite(CARRIAGE_BACKWARD, LOW);
  Serial.print("Crane claw forward: ");
  Serial.println(yco);
  digitalWrite(SPOOL_UP,HIGH);
  digitalWrite(SPOOL_DOWN,LOW);
  digitalWrite(SPOOL, HIGH);
  analogWrite(CARRIAGE_MOTOR, 255); //TODO: calibration
  delay(unit_time);
  digitalWrite(CARRIAGE_MOTOR, LOW);
  digitalWrite(SPOOL, LOW);
  return;
}

void back(){
  digitalWrite(CARRIAGE_FORWARD, LOW);
  digitalWrite(CARRIAGE_BACKWARD, HIGH);
  Serial.print("Crane claw backward: ");
  Serial.println(yco);
  analogWrite(CARRIAGE_MOTOR, 255);
  delay(unit_time);
  digitalWrite(CARRIAGE_MOTOR, LOW);
  return;
}

void cw(){
  Serial.println("Crane boom counterclockwise!");
  base_dist=0;//rezeros distance counter to avoid issues with int rollover
  digitalWrite(BASE_CLOCK, LOW);
  digitalWrite(BASE_COUNTER, HIGH);
  while(base_dist< 25){
    digitalWrite(BASE, HIGH);
    delay(15);
    digitalWrite(BASE, LOW);
    delay(100);
  }
  digitalWrite(BASE, LOW);
  return;
  return;
}

void ccw(){
  Serial.println("Crane boom clockwise!");
  base_dist=0;//rezeros distance counter to avoid issues with int rollover
  digitalWrite(BASE_CLOCK, HIGH);
  digitalWrite(BASE_COUNTER, LOW);
  digitalWrite(BASE, HIGH);
  while(base_dist< 25){
    digitalWrite(BASE, HIGH);
    delay(15);
    digitalWrite(BASE, LOW);
    delay(100);
  }
  digitalWrite(BASE, LOW);
  return;
  return;
}


void claw_down(int dist){//forward method, takes number of shaft revolutions
  spool_dist=0;//rezeros distance counter to avoid issues with int rollover
  digitalWrite(SPOOL_UP,LOW);
  digitalWrite(SPOOL_DOWN,HIGH);
  digitalWrite(SPOOL, HIGH);
  while(spool_dist < dist){
    //delay(speed_control);
  }
  digitalWrite(SPOOL_UP,HIGH);
  digitalWrite(SPOOL_DOWN,LOW);
  digitalWrite(SPOOL, HIGH);
  delay(40);
  digitalWrite(SPOOL, LOW);
  return;
}


void claw_up(int dist){//forward method, takes number of shaft revolutions
  spool_dist=0;//rezeros distance counter to avoid issues with int rollover
  digitalWrite(SPOOL_UP,HIGH);
  digitalWrite(SPOOL_DOWN,LOW);
  digitalWrite(SPOOL, HIGH);
  while(spool_dist< dist){
    //delay(speed_control);
  }
  digitalWrite(SPOOL, LOW);
  return;
}

void open_claw() {
  claw_servo.write(0);
}

void close_claw() {
  claw_servo.write(180);
}

void wait_ambulance() {
  int incomingByte;
  while (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    incomingByte = Serial.read();
    if (incomingByte == 'z') {
      break;
    }
  }
}


void signal_ambulance() {
  idx = 0;
  while(idx < 10) {
    Serial.print('y');
    delay(200);
    idx++;
  }
}
void SpoolDistance(){//handles interrupt from right wheel motor encoder
  spool_dist++;
}

void BaseDistance(){////handles interrupt from left wheel motor encoder
  base_dist++;
}
