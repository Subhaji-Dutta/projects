#include <Servo.h>
#include <AFMotor.h>
#define Echo A0
#define Trig A1
#define motor 10
#define spoint 103
char value;
int distance;
int Left;
int Right;
int L = 0;
int R = 0;
Servo servo;
#define M1 9
#define M2 8
#define M3 7
#define M4 6
#define L1 5
#define L2 4
#define V 3
#define O 2


void setup() {
  Serial.begin(9600);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  servo.attach(motor);
  pinMode(M1,OUTPUT);
  pinMode(M2,OUTPUT);
  pinMode(M3,OUTPUT);
  pinMode(M4,OUTPUT);
  pinMode(L2,OUTPUT);
  pinMode(L1,OUTPUT);
  pinMode(V,OUTPUT);
  pinMode(O,OUTPUT);
}
void loop() {
  //Obstacle();
  //voicecontrol();
  if (Serial.available() > 0) {
    value = Serial.read();
    Serial.println(value);
  }
  switch(value){
    case 'F' :
    forward();
    break;
    case 'B' :
    backward();
    break;
    case 'L' :
    left();
    break;
    case 'R' :
    right();
    break;
    case 'S' :
    Stop();
    break;
    case 'X' :
    Obstacle();
    break;
    case 'x' :
    digitalWrite(O,LOW);
    break;
    case 'V' :
    voicecontrol();
    break;
    case 'v' :
    digitalWrite(V,LOW);
    break;
    case 'W' :
    digitalWrite(L1, HIGH);
    break;
    case 'w' :
    digitalWrite(L1, LOW);
    break;
    case 'U' :
    digitalWrite(L2, HIGH);
    break;
    case 'u' :
    digitalWrite(L2, LOW);
    break;
    default :
    Stop();
  }
}
void Obstacle() {
  digitalWrite(O,HIGH);
  distance = ultrasonic();
  if (distance <= 12) {
    Stop();
    backward();
    delay(100);
    Stop();
    L = leftsee();
    servo.write(spoint);
    delay(800);
    R = rightsee();
    servo.write(spoint);
    if (L < R) {
      right();
      delay(500);
      Stop();
      delay(200);
    } else if (L > R) {
      left();
      delay(500);
      Stop();
      delay(200);
    }
  } else {
    forward();
  }
}
void voicecontrol() {
  digitalWrite(V,HIGH);
  L=ultrasonic();
  if(L<=7){
    for (int pos = 0; pos <= 180; pos += 1) { 
    // in steps of 1 degree
    servo.write(pos);              
    delay(15);                       
  }
  for (int pos = 180; pos >= 0; pos -= 1) { 
    servo.write(pos);              
    delay(15);                       
  }
  }
 }
  
// Ultrasonic sensor distance reading function
int ultrasonic() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  long t = pulseIn(Echo, HIGH);
  long cm = t / 29 / 2; //time convert distance
  return cm;
}
void forward() {
  digitalWrite(M1,HIGH);
  digitalWrite(M2,LOW);
  digitalWrite(M3,HIGH);
  digitalWrite(M4,LOW);
}
void backward() {
  digitalWrite(M1,LOW);
  digitalWrite(M2,HIGH);
  digitalWrite(M3,LOW);
  digitalWrite(M4,HIGH);
}
void right() {
  digitalWrite(M1,LOW);
  digitalWrite(M2,HIGH);
  digitalWrite(M3,HIGH);
  digitalWrite(M4,LOW);
}
void left() {
  digitalWrite(M1,HIGH);
  digitalWrite(M2,LOW);
  digitalWrite(M3,LOW);
  digitalWrite(M4,HIGH);
}
void Stop() {
  digitalWrite(M1,LOW);
  digitalWrite(M2,LOW);
  digitalWrite(M3,LOW);
  digitalWrite(M4,LOW);
}
int rightsee() {
  servo.write(20);
  delay(800);
  Left = ultrasonic();
  return Left;
}
int leftsee() {
  servo.write(180);
  delay(800);
  Right = ultrasonic();
  return Right;
}
