#define BLYNK_TEMPLATE_ID "TMPLV6FbLxVJ"
#define BLYNK_DEVICE_NAME "AFFRObot"
#define BLYNK_AUTH_TOKEN "PAqjSE0mId0_yL3kdB5NuyDTAUqdTBPa"

#include <WiFi.h>
//#include <ESP8266WiFi.h> 
#include <BlynkSimpleEsp32.h>

char auth[] = BLYNK_AUTH_TOKEN;

char ssid[] = "#VEER";  // type your wifi name
char pass[] = "12345678";  // type your wifi password


#include "DHT.h"
#define DHTPIN 13 
#define DHTTYPE DHT11
#define m 5
#define s 25
#define w 26
DHT dht(DHTPIN, DHTTYPE);
float t, h;
float sm,wat;
int sat=3,sp=0,l;
double la=88.38,lon=22.57;



void setup()
{
  Serial.begin(9600);
  dht.begin();
  Blynk.begin(auth, ssid, pass);
  delay(2000); 

}

void loop() 
{
  Blynk.run();
    h = dht.readHumidity();
  t = dht.readTemperature(); 
  sm = analogRead(s)/2;
  wat = map(analogRead(w),0,1024,0,10);
  l= digitalRead(m) ;
  Blynk.virtualWrite(V0, h);
  Blynk.virtualWrite(V1, t);
  Blynk.virtualWrite(V2, sm);
  Blynk.virtualWrite(V3, wat);
  Blynk.virtualWrite(V4, sp);
  Blynk.virtualWrite(V5, sat);
  Blynk.virtualWrite(V6, lon);
  Blynk.virtualWrite(V7, la);
  Blynk.virtualWrite(V8, l);
  

}
