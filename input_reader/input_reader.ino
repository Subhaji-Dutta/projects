#define a 4
#define b 3
#define c 2
#define d 1
#define e 5
#define f 8
#define g 9
#define h 10
#define i 11
#define j 12
void setup() {
  pinMode (a, INPUT);
  pinMode (b, INPUT);
  pinMode (c, INPUT);
  pinMode (d, INPUT);
  pinMode (e, INPUT);
   pinMode (f, OUTPUT);
    pinMode (g, OUTPUT);
     pinMode (h, OUTPUT);
      pinMode (i, OUTPUT);
       pinMode (j, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (digitalRead(a)==1)
  {
    digitalWrite(f, LOW);
   Serial.println(" 2 on"); 
  }
if (digitalRead(b)==1)
{
  digitalWrite(g, LOW);
 Serial.println("3 on");
}
if (digitalRead(c)==1)
{
  digitalWrite(h, LOW);
 Serial.println("4 on");
}
if (digitalRead(d)==1)
{
  digitalWrite(i, LOW);
 Serial.println("5 on");
}
if (digitalRead(e)==1)
{
  digitalWrite(j, LOW);
 Serial.println("6 on");
}
} 
