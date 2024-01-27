/*
Lamp 1 connected to PinOut 2 Arduino
Lamp 2 connected to PinOut 3 Arduino
Lamp 3 connected to PinOut 4 Arduino
Lamp 4 connected to PinOut 5 Arduino
Lamp 5 connected to PinOut 6 Arduino
Lamp 6 connected to PinOut 7 Arduino
Lamp 7 connected to PinOut 8 Arduino
Lamp 8 connected to PinOut 9 Arduino
--->you can connected to relay modul 8 channel

Serial data sending from Bluetooth Controll Lamp.apk
data '1'-'8' to on is lamp 1-8
data 'A'-'H' to off is lamp 1-8
data '9' to on ALL is lamp
data 'I' to off ALL is lamp
======================================================================================*/

#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11); //Pin10 RX , Pin 11 TX connected to--> Bluetooth TX,RX

#define Lamp1 2
#define Lamp2 3
#define Lamp3 4
#define Lamp4 5
#define Lamp5 6
#define Lamp6 7
#define Lamp7 8
#define Lamp8 9

//If Out to active Low change ON 0 and OFF 1
//If Out to active High change ON 1 and OFF 0
#define ON 0
#define OFF 1
char val;
String statusLamp1,statusLamp2,statusLamp3,statusLamp4,statusLamp5,statusLamp6,statusLamp7,statusLamp8;

void setup() {
  pinMode(Lamp1,OUTPUT);digitalWrite (Lamp1,OFF);
  pinMode(Lamp2,OUTPUT);digitalWrite (Lamp2,OFF);
  pinMode(Lamp3,OUTPUT);digitalWrite (Lamp3,OFF);
  pinMode(Lamp4,OUTPUT);digitalWrite (Lamp4,OFF);
  pinMode(Lamp5,OUTPUT);digitalWrite (Lamp5,OFF);
  pinMode(Lamp6,OUTPUT);digitalWrite (Lamp6,OFF);
  pinMode(Lamp7,OUTPUT);digitalWrite (Lamp7,OFF);
  pinMode(Lamp8,OUTPUT);digitalWrite (Lamp8,OFF);
  mySerial.begin(9600);
  Serial.begin(9600);
}

void loop() {
//cek data serial from bluetooth android App
if( mySerial.available() >0 ) {
    val = mySerial.read();
    Serial.println(val); 
}
//Lamp is on
  if( val == '1' ) {
    digitalWrite(Lamp1,ON); statusLamp1="1"; }
  else if( val == '2' ) {
    digitalWrite(Lamp2,ON); statusLamp2="2"; }
  else if( val == '3' ) {
    digitalWrite(Lamp3,ON); statusLamp3="3"; }
  else if( val == '4' ) {
    digitalWrite(Lamp4,ON); statusLamp4="4"; }
  else if( val == '5' ) {
    digitalWrite(Lamp5,ON); statusLamp5="5"; }
  else if( val == '6' ) {
    digitalWrite(Lamp6,ON); statusLamp6="6"; }
  else if( val == '7' ) {
    digitalWrite(Lamp7,ON); statusLamp7="7"; }
  else if( val == '8' ) {
    digitalWrite(Lamp8,ON); statusLamp8="8"; }
  else if( val == '9' ) {
    digitalWrite(Lamp1,ON); statusLamp1="1"; 
    digitalWrite(Lamp2,ON); statusLamp2="2"; 
    digitalWrite(Lamp3,ON); statusLamp3="3"; 
    digitalWrite(Lamp4,ON); statusLamp4="4"; 
    digitalWrite(Lamp5,ON); statusLamp5="5"; 
    digitalWrite(Lamp6,ON); statusLamp6="6"; 
    digitalWrite(Lamp7,ON); statusLamp7="7"; 
    digitalWrite(Lamp8,ON); statusLamp8="8"; 
 }
 //Lamp is off
  else if( val == 'A' ) {
    digitalWrite(Lamp1,OFF); statusLamp1="A"; }
  else if( val == 'B' ) {
    digitalWrite(Lamp2,OFF); statusLamp2="B"; }
  else if( val == 'C' ) {
    digitalWrite(Lamp3,OFF); statusLamp3="C"; }
  else if( val == 'D' ) {
    digitalWrite(Lamp4,OFF); statusLamp4="D"; }
  else if( val == 'E' ) {
    digitalWrite(Lamp5,OFF); statusLamp5="E"; }
  else if( val == 'F' ) {
    digitalWrite(Lamp6,OFF); statusLamp6="F"; }
  else if( val == 'G' ) {
    digitalWrite(Lamp7,OFF); statusLamp7="G"; }
  else if( val == 'H' ) {
    digitalWrite(Lamp8,OFF); statusLamp8="H"; }
  else if( val == 'I' ) {
    digitalWrite(Lamp1,OFF); statusLamp1="A";
    digitalWrite(Lamp2,OFF); statusLamp2="B";
    digitalWrite(Lamp3,OFF); statusLamp3="C";
    digitalWrite(Lamp4,OFF); statusLamp4="D";
    digitalWrite(Lamp5,OFF); statusLamp5="E";
    digitalWrite(Lamp6,OFF); statusLamp6="F";
    digitalWrite(Lamp7,OFF); statusLamp7="G";
    digitalWrite(Lamp8,OFF); statusLamp8="H";
 }
  //synchronize Arduino to APK
 else if( val == 'S' ) {
    //send data to android apk
    delay(500);    
    mySerial.println(statusLamp1+statusLamp2+statusLamp3+statusLamp4+statusLamp5+statusLamp6+statusLamp7+statusLamp8+"J"); //delay(500);
    val=' ';
 }
}


//Arduino project created by: pujar
//www.mutekla.com
//Apk Android remote controll suport this project, download on Playstore:
//Bluetooth Controll Lamp.apk
//https://play.google.com/store/apps/details?id=dev.merahkemarun.btcontrolllamp
