#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>

#define auth " UFZzj31e4ZZsXd8-Ez617gEzRijTPcZ3"                 // You should get Auth Token in the Blynk App.  
#define ssid "#VEER"             //Enter Wifi Name
#define pass "12345678"         //Enter wifi Password

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

#define RM1     13    
#define RM2     12    
#define LM1     14   
#define LM2     27

#define relay1  26
#define relay2  25
#define relay3  33
#define relay4  18




WidgetLED l1 (V8);
WidgetLED l2 (V9);
WidgetLED l3 (V10);
WidgetLED l4 (V11);
WidgetLED l5 (V12);


int n = 0;

void setup()
{
    Serial.begin(9600);
    Blynk.begin(auth, ssid, pass);

    pinMode(RM1, OUTPUT);
    pinMode(RM2, OUTPUT);
    pinMode(LM1, OUTPUT);
    pinMode(LM2, OUTPUT);
    
    pinMode(relay1,OUTPUT);
    pinMode(relay2,OUTPUT);
    pinMode(relay3,OUTPUT);
    pinMode(relay4,OUTPUT);
    
    servo1.attach(4);
    servo2.attach(2);
    servo3.attach(5);
    servo4.attach(15);

    Serial.begin(9600);
}
BLYNK_WRITE(V0)
{
  int pinValue = param.asInt(); //SMD LIGHTS
  if (pinValue==1)//ON
  {
    l1.on();
  n=2;
  }
  else if (pinValue==2)//BLINK
  {
      n=1;
   }
  else
  {
    l1.off();
    n=0;
   digitalWrite(relay1, HIGH); 
  }
}

BLYNK_WRITE(V1)
{
  int pinValue = param.asInt(); // BUZZER
  digitalWrite(relay2, pinValue);
  if(pinValue ==0)
  {
  l2.on();
  Serial.println("r2 on");
  // process received value
}
else
{
  l2.off();
  Serial.println("r2 off");
}
}
BLYNK_WRITE(V2)
{
  int pinValue = param.asInt(); // LIGHTS
 digitalWrite(relay3,pinValue);
 if(pinValue ==0)
 {
   l3.on();
  Serial.println("r3 ");
 }
 else
 {
  l3.off();
  Serial.println("r3 ");// process received value
 }
}

BLYNK_WRITE(V3)//X AXIS
{
  servo1.write(param.asInt());
  Serial.print("X=   ");
  Serial.println(param.asInt());
}
BLYNK_WRITE(V4)//Y AXIS
{  
  servo2.write(param.asInt());
  Serial.print("Y=   ");
  Serial.println(param.asInt());
}
BLYNK_WRITE(V5)
{  
  int y = param.asInt();
  if (y==1)//OPEN
  {
  servo3.write(140);
  Serial.println("open");
  l4.on();
  servo4.write(40);
  }
  else//CLOSE
  {
   servo3.write(180);
  servo4.write(0); 
  Serial.println("Close");
  l4.off();
  }
}

BLYNK_WRITE(V7)
{
  int pinValue = param.asInt(); // LIGHTS
 digitalWrite(relay4,pinValue);
 if(pinValue ==0)
 {
   l5.on();
  Serial.println("r4 ");
 }
 else
 {
  l5.off();
  Serial.println("r4 ");// process received value
 }
}



BLYNK_WRITE(V6)//MOVE
{
    int x = param.asInt();
    if(x==1)//FORWARD
    {
      digitalWrite(RM1,HIGH);    
      digitalWrite(RM2,LOW);     
      digitalWrite(LM1,HIGH);   
      digitalWrite(LM2,LOW);
      Serial.println("FORWARD");
    }
    else if (x==2)//RIGHT
    {
      digitalWrite(RM1,LOW);    
      digitalWrite(RM2,LOW);     
      digitalWrite(LM1,HIGH);   
      digitalWrite(LM2,LOW);
      Serial.println("RIGHT");
    }
    else if (x==3)//LEFT
    {
      digitalWrite(RM1,HIGH);    
      digitalWrite(RM2,LOW);     
      digitalWrite(LM1,LOW);   
      digitalWrite(LM2,LOW);
       Serial.println("LEFT");
    }
    else if (x==4)//BACK
    {
      digitalWrite(RM1,LOW);    
      digitalWrite(RM2,HIGH);     
      digitalWrite(LM1,LOW);   
      digitalWrite(LM2,HIGH);
      Serial.println("BACK");
    }
    else//STOP
    {
      digitalWrite(RM1,LOW);    
      digitalWrite(RM2,LOW);     
      digitalWrite(LM1,LOW);   
      digitalWrite(LM2,LOW);
    }
}
void loop()
{
 Blynk.run();
 if (n==1)
 {
   digitalWrite(relay1, LOW);
    Serial.println("light on");
    l1.on();
    delay(2000);
    digitalWrite(relay1, HIGH);
    l1.off();
    Serial.println("light off");
    delay(1000);
 }
 else
 {
  digitalWrite(relay1, HIGH);
 }
 if (n==2)
 {
   digitalWrite(relay1, LOW);
    Serial.println("light on");
    
 }
 else
 {
  digitalWrite(relay1, HIGH);
 }
}
