#define A_m 12
#define A_m1 11
#define A_m2 10
#define A_m3 9
#define M_c 8
#define M_c1 7
#define M_c2 6
#define M_c3 5
#define O_i 4
#define O_i1 3
#define O_i2 2
#define O_i3 1

void setup() 
{
 pinMode(A_m, INPUT);
 pinMode(A_m1, INPUT);
 pinMode(A_m2, INPUT);
 pinMode(A_m3, INPUT);
 pinMode(M_c, INPUT);
 pinMode(M_c1, INPUT);
 pinMode(M_c2, INPUT);
 pinMode(M_c3, INPUT);
 pinMode(O_i, OUTPUT);
 pinMode(O_i1, OUTPUT);
 pinMode(O_i2, OUTPUT);
 pinMode(O_i3, OUTPUT);
 Serial.begin(9600);
}

void loop()
{
  if(((digitalRead(A_m)== 1) && (digitalRead(A_m2)== 1))||((digitalRead(M_c)== 1) && (digitalRead(M_c2)== 1)))
  {
    Serial.println("FORWARD");
    digitalWrite(O_i, HIGH);
    digitalWrite(O_i1, LOW);
    digitalWrite(O_i2, HIGH);
    digitalWrite(O_i3, LOW);
  }
  if(((digitalRead(A_m)== 1) && (digitalRead(A_m2)== 1))||((digitalRead(M_c)== 1) && (digitalRead(M_c2)== 1)))
  {
    Serial.println("RIGHT");
    digitalWrite(O_i, HIGH);
    digitalWrite(O_i1, LOW);
    digitalWrite(O_i2, LOW);
    digitalWrite(O_i3, HIGH); 
  }
  if(((digitalRead(A_m)== 1) && (digitalRead(A_m2)== 1))||((digitalRead(M_c)== 1) && (digitalRead(M_c2)== 1)))
  {
    Serial.println("LEFT");
    digitalWrite(O_i, LOW);
    digitalWrite(O_i1, HIGH);
    digitalWrite(O_i2, HIGH);
    digitalWrite(O_i3, LOW); 
  }
  if(((digitalRead(A_m)== 1) && (digitalRead(A_m2)== 1))||((digitalRead(M_c)== 1) && (digitalRead(M_c2)== 1)))
  {
    Serial.println("BACK"); 
    digitalWrite(O_i, HIGH);
    digitalWrite(O_i1, HIGH);
    digitalWrite(O_i2, LOW);
    digitalWrite(O_i3, HIGH); 
  }
}
