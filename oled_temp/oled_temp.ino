
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Servo.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Servo servo;
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define DHTPIN 14     // Digital pin connected to the DHT sensor


#define relay1 16

#define Led1 12
#define Led2 0
#define Led3 13

const int analogInPin= A0;
// Uncomment the type of sensor in use:
#define DHTTYPE    DHT11     // DHT 11
//#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);
int sensorValue = 0;



void setup() {
  Serial.begin(115200);

  dht.begin();

   servo.attach(2);
 pinMode (relay1, OUTPUT);

 pinMode (Led1, OUTPUT);
 pinMode (Led2, OUTPUT);
 pinMode (Led3, OUTPUT);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);
  display.clearDisplay();
  display.setTextColor(WHITE);
}

void loop() {
  delay(1000);
  sensorValue = analogRead(analogInPin);
 int temp = map(sensorValue, 0, 1024, 20 , 40);
  int hi = temp+2;
   int lo = temp-2;
  //read temperature and humidity
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  Serial.print(sensorValue);
  Serial.print("   ");
  Serial.print(temp);
  Serial.print("   ");
  Serial.print(h);
  Serial.print("   ");
  Serial.print(t);
  Serial.println(" ");
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
  }
  // clear display
  display.clearDisplay();
  display.setTextSize(1);
  // display temperature
  display.setCursor(0,0);
  display.print("Temp: ");
  display.print(t);
  display.print(" ");
  display.print("C");
  display.setTextSize(1);
  // display humidity
  display.setCursor(0, 20);
  display.print("H: ");
  display.print(h);
  display.print(" %"); 

  display.setTextSize(1);
display.setCursor(0, 40);
  display.print("Req: ");
  display.print(temp);
  display.print(" C "); 
  display.display(); 

  if (t>hi){
    digitalWrite(relay1, HIGH);
    digitalWrite(Led2, HIGH);
    digitalWrite(Led1,LOW);
    servo.write(0);
    digitalWrite(Led3,LOW);
  
  }else if ((t<=hi)&&(t>=lo)){
     
      digitalWrite(Led1,LOW);
      digitalWrite(Led2,LOW);
      servo.write(45);
      digitalWrite(Led3,HIGH);
      
    }
    else
    {
      
       
      digitalWrite(relay1,LOW);
      servo.write(90);
      digitalWrite(Led2,LOW);
      digitalWrite(Led3,LOW);
     digitalWrite(Led1,HIGH);
  }
}
