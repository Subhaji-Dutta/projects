#include <Servo.h>          //Servo motor library. This is standard library
#include <NewPing.h>        //Ultrasonic sensor function library. You must install this library

#define a 5
#define b 6
#define c 7
#define d 8
#define e 9
#define f 10

const int LeftMotorForward = 1;
const int LeftMotorBackward = 2;
const int RightMotorForward = 3;
const int RightMotorBackward = 4;

#define trig_pin A1 //analog input 1
#define echo_pin A2 //analog input 2

#define maximum_distance 200
boolean goesForward = false;
int distance = 100;

NewPing sonar(trig_pin, echo_pin, maximum_distance); //sensor function
Servo servo_motor; //our servo name

void setup() {
  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  pinMode(a,INPUT);
  pinMode(b,INPUT);
  pinMode(c,INPUT);
  pinMode(d,INPUT);
  pinMode(e,INPUT);
  pinMode(f,INPUT);
  Serial.begin(9600);
  
  servo_motor.attach(10); //our servo pin

  servo_motor.write(115);
  delay(2000);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
}

int lookRight(){  
  servo_motor.write(50);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
}

int lookLeft(){
  servo_motor.write(170);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
  delay(100);
}

int readPing(){
  delay(70);
  int cm = sonar.ping_cm();
  if (cm==0){
    cm=250;
  }
  return cm;
}

void moveStop(){
  
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
}

void moveForward(){
    
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorForward, HIGH);
  
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorBackward, LOW); 
  
}

void moveBackward(){

  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);
  
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorForward, LOW);
  
}

void turnRight(){

  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);
  
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);
  
  
}

void turnLeft(){

  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);

}
void loop() {
  if(digitalRead(e)==0)
  {
    Serial.println("auto");
    int distanceRight = 0;
  int distanceLeft = 0;
  delay(50);

  if (distance <= 35){
    Serial.println(distance);
    moveStop();
    delay(300);
    moveBackward();
    delay(400);
    moveStop();
    delay(300);
    distanceRight = lookRight();
    delay(300);
    distanceLeft = lookLeft();
    delay(300);

    if (distance >= distanceLeft){
      turnRight();
      delay(500);
      moveForward();
    }
    else{
      turnLeft();
      delay(500);
      moveForward();
    }
  }
  else{
    moveForward(); 
  }
    distance = readPing();

  }
  else if (digitalRead(f)==0)
  {
    Serial.println("manu");
    if((digitalRead(a)==0)&&(digitalRead(c)==0))
    {
     Serial.println("forward"); 
      moveForward();
    }
    if((digitalRead(b)==0)&&(digitalRead(d)==0))
    {
      Serial.println("back");
      moveBackward();
    }
    if((digitalRead(a)==0)&&(digitalRead(d)==0))
    {
      Serial.println("left");
      turnLeft();
    }
    if((digitalRead(b)==0)&&(digitalRead(c)==0))
    {
      Serial.println("right");
      turnRight();
    }
  }
  else
  {
    Serial.println("stop");
  moveStop();  
  }

}
