#include <NewPing.h>

const int LeftMotorForward = 6;
const int LeftMotorBackward = 5;
const int RightMotorForward = 4;
const int RightMotorBackward = 3;

//sensor pins
#define trig_pin A0 //analog input 1
#define echo_pin A2 //analog input 2


int davomiylik, sm;
#define maximum_distance 200
NewPing sonar(trig_pin, echo_pin, maximum_distance);

void plusUltra() {
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(5);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(15);
  digitalWrite(trig_pin, LOW);
  davomiylik = pulseIn(echo_pin, HIGH);
  sm = davomiylik / 58;
}

void setup() {
  //Serial.begin(9600);
pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  
}

void loop() {
plusUltra();
  //Serial.println(sm);
  if (sm > 1 && sm < 50) { //Agar 50 sm masofada biror raqib ko'rinsa 
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorForward, HIGH);
  
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorBackward, LOW); 
    delay(300);
  } else {                 //Raqib ko'rinmasa 
 digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);
  
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);
    delay(10);
  }
}
