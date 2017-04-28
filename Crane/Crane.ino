#define CARRIAGE_MOTOR 6
#define CARRIAGE_FORWARD 4
#define CARRIAGE_BACKWARD 3

#include <SPI.h>  
#include <Pixy.h>
#include <SoftwareSerial.h>

SoftwareSerial XBee(7,8);
Pixy pixy;
int xco;
int yco;
int block;

int sig=0;

void setup() {
  XBee.begin(9600);
  Serial.begin(9600);

  //pixy initialization
  pixy.init();

  pinMode(CARRIAGE_MOTOR, OUTPUT);
  pinMode(CARRIAGE_FORWARD, OUTPUT);
  pinMode(CARRIAGE_BACKWARD, OUTPUT);
  
  analogWrite(CARRIAGE_MOTOR, 0);

  delay(2000);//lets pixy find object
}

void loop() {
  /*
  while(sig==0){
    if(XBee.available()){
      sig=XBee.read();
    }
  }
  */
/*
  forward();
  delay(3000);
  backward();
  delay(3000); 
*/
///*

  
  refresh();
//  Serial.println(xco);
//  Serial.println(yco);
  delay(1000);

 // */
  
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
  /*
  if(xco>180){
    ccw();
  }else if(xco<140){
    cw();
  }else 
  */
  if(yco>120){
    forward();
  }else if(yco<80){
    back();
  }else{
    Serial.println("Drop the claw!");
    analogWrite(CARRIAGE_MOTOR, 0);
  }
  
}

void forward(){
  digitalWrite(CARRIAGE_FORWARD, HIGH);
  digitalWrite(CARRIAGE_BACKWARD, LOW);
  Serial.println("Crane claw forward!");
  analogWrite(CARRIAGE_MOTOR, 200);
  delay(200);
  digitalWrite(CARRIAGE_MOTOR, LOW);
  return;
}

void back(){
  digitalWrite(CARRIAGE_FORWARD, LOW);
  digitalWrite(CARRIAGE_BACKWARD, HIGH);
  Serial.println("Crane claw backward!");
  analogWrite(CARRIAGE_MOTOR, 180);
  delay(200);
  digitalWrite(CARRIAGE_MOTOR, LOW);
  return;
}

void ccw(){
  Serial.println("Crane boom counterclockwise!");
  analogWrite(CARRIAGE_MOTOR, 0);
  return;
}

void cw(){
  Serial.println("Crane boom clockwise!");
  analogWrite(CARRIAGE_MOTOR, 0);
  return;
}


