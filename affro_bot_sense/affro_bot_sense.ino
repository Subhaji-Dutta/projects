#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <DHT.h>

static const int RXPin = 4, TXPin = 5;   // GPIO 4=D2(conneect Tx of GPS) and GPIO 5=D1(Connect Rx of GPS
static const uint32_t GPSBaud = 9600; //if Baud rate 9600 didn't work in your case then use 4800

int fire,rain,motion;
float water,smoke,temperature,humidity,spd,sats;
#define DHTPIN 16
#define DHTTYPE    DHT11 
TinyGPSPlus gps; // The TinyGPS++ object
WidgetMap myMap(V0);  // V0 for virtual pin of Map Widget
DHT dht(DHTPIN, DHTTYPE);
SoftwareSerial ss(RXPin, TXPin);  // The serial connection to the GPS device

BlynkTimer timer;


String bearing;  //Variable to store orientation or direction of GPS

char auth[] = "--------------------";              //Your Project authentication key
char ssid[] = "-------";                                       // Name of your network (HotSpot or Router name)
char pass[] = "-------";                                      // Corresponding Password



//unsigned int move_index;         // moving index, to be used later
unsigned int move_index = 1;       // fixed location for now
  


void setup()
{
  Serial.begin(115200);
  Serial.println();
  ss.begin(GPSBaud);
  Blynk.begin(auth, ssid, pass);
  timer.setInterval(5000L, checkGPS); // every 5s check if GPS is connected, only really needs to be done once
  pinMode(14,INPUT);
  pinMode(12,INPUT);
  pinMode(13,INPUT);
  pinMode(0,INPUT);
  pinMode(2,INPUT);
}
void checkGPS(){
  if (gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
      Blynk.virtualWrite(V4, "GPS ERROR");  // Value Display widget  on V4 if GPS not detected
  }
}

void loop()
{
    
  Blynk.run();
  timer.run();
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  fire = digitalRead(14);
  rain = digitalRead(12);
  motion = digitalRead(13);
  water = analogRead(0);
  smoke = analogRead(2);
  Blynk.virtualWrite(V6, temperature);   
  Blynk.virtualWrite(V7, humidity);
  Blynk.virtualWrite(V8, fire);   
  Blynk.virtualWrite(V9, rain);
  Blynk.virtualWrite(V10, motion);   
  Blynk.virtualWrite(V11, water);
  Blynk.virtualWrite(V12, smoke); 
  
  if (gps.location.isValid() ) 
  {    
    float latitude = (gps.location.lat());     //Storing the Lat. and Lon. 
    float longitude = (gps.location.lng()); 
    
    Serial.print("LAT:  ");
    Serial.println(latitude, 6);  // float to x decimal places
    Serial.print("LONG: ");
    Serial.println(longitude, 6);
    Blynk.virtualWrite(V1, String(latitude, 6));   
    Blynk.virtualWrite(V2, String(longitude, 6));  
    myMap.location(move_index, latitude, longitude, "GPS_Location");
    spd = gps.speed.kmph();               //get speed
       Blynk.virtualWrite(V3, spd);
       
       sats = gps.satellites.value();    //get number of satellites
       Blynk.virtualWrite(V4, sats);

       bearing = TinyGPSPlus::cardinal(gps.course.value()); // get the direction
       Blynk.virtualWrite(V5, bearing);                   
  }  

}
