#define CARRIAGE_MOTOR 6
#define CARRIAGE_FORWARD 4
#define CARRIAGE_BACKWARD 7

#define CLAW_SERVO 11

//MOTOR PIN SETUP
#define SPOOL 5
#define SPOOL_UP 8
#define SPOOL_DOWN 9
#define SPOOL_INTER 2

#define BASE 10
#define BASE_CLOCK 12
#define BASE_COUNTER 13
#define BASE_INTER 3



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

void setup() {
  Serial.begin(9600);

  //pixy initialization
  pixy.init();

  pinMode(CARRIAGE_MOTOR, OUTPUT);
  pinMode(CARRIAGE_FORWARD, OUTPUT);
  pinMode(CARRIAGE_BACKWARD, OUTPUT);

  pinMode(SPOOL, OUTPUT);//DC motor on/off
  pinMode(SPOOL_UP, OUTPUT);//DC direction 1
  pinMode(SPOOL_DOWN, OUTPUT);//DC direction 2
  
  pinMode(BASE, OUTPUT);//DC motor on/off
  pinMode(BASE_CLOCK, OUTPUT);//DC direction 1
  pinMode(BASE_COUNTER, OUTPUT);//DC direction 2 
  
  pinMode(SPOOL_INTER, INPUT);
  pinMode(BASE_INTER, INPUT);

  pinMode(CLAW_SERVO, OUTPUT);


  //Encoder setup
  attachInterrupt(digitalPinToInterrupt(SPOOL_INTER), SpoolDistance, FALLING);
  attachInterrupt(digitalPinToInterrupt(BASE_INTER), BaseDistance, FALLING);
  
  analogWrite(CARRIAGE_MOTOR, 0);

  delay(2000);//lets pixy find object
}

void SpoolDistance(){//handles interrupt from right wheel motor encoder
  spool_dist++;
}

void BaseDistance(){////handles interrupt from left wheel motor encoder
  base_dist++;
}

void loop(){
  /*
  digitalWrite(SPOOL_UP, HIGH);
  digitalWrite(SPOOL_DOWN, LOW);
  digitalWrite(SPOOL, HIGH);
  delay(15);
  digitalWrite(SPOOL, LOW);
  delay(50);
  //delay(1000);
  //digitalWrite(BASE, LOW);
  //delay(3000);
  */
  /*
  // see if there's incoming serial data:
  while (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    incomingByte = Serial.read();
    // if it's a capital H (ASCII 72), turn on the LED:
    if (incomingByte == 'z') {
      break;
    }
  }
  

  forward();
  delay(3000);
  backward();
  delay(3000); 
*/

  
  refresh();
//  Serial.println(xco);
  Serial.println(yco);
//  findObject();
  delay(1000);
  
}

void refresh(){
  block=pixy.getBlocks();
  if(block){
    for(int i;i<block;i++){
      if(pixy.blocks[i].signature==1){
        xco=pixy.blocks[i].x;
        yco=pixy.blocks[i].y;
      }
    }
  }
}

void findObject(){
  boolean found = false;
  while(!found){
    refresh();
    /*
    if(xco>180){
      ccw();
    }else if(xco<140){
      cw();
    }else 
    */
    if(yco<80){
      forward();
    }else if(yco>120){
      back();
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
  analogWrite(CARRIAGE_MOTOR, 255); //TODO: calibration
  delay(unit_time);
  digitalWrite(CARRIAGE_MOTOR, LOW);
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

void ccw(){
  Serial.println("Crane boom counterclockwise!");
  base_counterclockwise(unit_encoder);
  
  return;
}

void cw(){
  Serial.println("Crane boom clockwise!");
  base_clockwise(unit_encoder);
  return;
}


void claw_down(int dist){//forward method, takes number of shaft revolutions
  spool_dist=0;//rezeros distance counter to avoid issues with int rollover
  
  digitalWrite(SPOOL_UP,LOW);
  while(spool_dist< dist){
    digitalWrite(SPOOL_DOWN,HIGH);
    delay(speed_control);
  }
  digitalWrite(SPOOL_DOWN,LOW);
  return;
}


void claw_up(int dist){//forward method, takes number of shaft revolutions
  spool_dist=0;//rezeros distance counter to avoid issues with int rollover
  
  digitalWrite(SPOOL_DOWN,LOW);
  while(spool_dist< dist){
    digitalWrite(SPOOL_UP,HIGH);
    delay(speed_control);
  }
  digitalWrite(SPOOL_UP,LOW);
  return;
}


void base_clockwise(int dist){//forward method, takes number of shaft revolutions
  base_dist=0;//rezeros distance counter to avoid issues with int rollover
  
  digitalWrite(BASE_COUNTER,LOW);
  while(spool_dist< dist){
    analogWrite(BASE_CLOCK, base_speed);
    delay(speed_control);
  }
  analogWrite(BASE_CLOCK, 0);
  return;
}



void base_counterclockwise(int dist){//forward method, takes number of shaft revolutions
  base_dist=0;//rezeros distance counter to avoid issues with int rollover
  
  digitalWrite(BASE_CLOCK,LOW);
  while(spool_dist< dist){
    analogWrite(BASE_COUNTER, base_speed);
    delay(speed_control);
  }
  analogWrite(BASE_COUNTER, 0);
  return;
}
