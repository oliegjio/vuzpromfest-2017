#include <I2CEncoder.h>
#include <Wire.h>


#define M1_PLUS 6
#define M1_MINUS 7

#define M2_PLUS 46
#define M2_MINUS 47

#define RIGHTLINE 12
#define CENTRALLINE 11
#define LEFTLINE  10

I2CEncoder encoder;

int speed = 75;
int sRotaSpeed = 130;
int bRotaSpeed = 100;

void block()
{
  analogWrite(M1_PLUS, HIGH);
  analogWrite(M1_MINUS , HIGH);

  analogWrite(M2_PLUS, HIGH);
  analogWrite(M2_MINUS, HIGH);
}

void stay()
{
  analogWrite(M1_PLUS, LOW);
  analogWrite(M1_MINUS , LOW);

  analogWrite(M2_PLUS, LOW);
  analogWrite(M2_MINUS, LOW);
}

void left()
{
  while(encoder.getPosition() < 1.97 )
  {
    analogWrite(M1_PLUS, 0);
    analogWrite(M1_MINUS , bRotaSpeed);
  
    analogWrite(M2_PLUS, bRotaSpeed);
    analogWrite(M2_MINUS, 0);
  }
  block();
  encoder.zero();
}

void _left()
{
  analogWrite(M1_PLUS, sRotaSpeed);
  analogWrite(M1_MINUS , 0);

  analogWrite(M2_PLUS, 0);
  analogWrite(M2_MINUS, sRotaSpeed);
}

void right()
{
  while(encoder.getPosition() < 1.97 * 3 - 0.4 )
  {  
    analogWrite(M1_PLUS, 0);
    analogWrite(M1_MINUS , bRotaSpeed);
  
    analogWrite(M2_PLUS, bRotaSpeed);
    analogWrite(M2_MINUS, 0);
  }
  block();
  encoder.zero();
}

void _right()
{
  analogWrite(M1_PLUS, sRotaSpeed);
  analogWrite(M1_MINUS , 0);

  analogWrite(M2_PLUS, 0);
  analogWrite(M2_MINUS, sRotaSpeed);
}

void front()
{
  analogWrite(M1_PLUS, speed);
  analogWrite(M1_MINUS , 0);

  analogWrite(M2_PLUS, speed);
  analogWrite(M2_MINUS, 0);
}

void setup()
{
  Wire.begin();
  encoder.init(MOTOR_269_ROTATIONS, MOTOR_269_TIME_DELTA);
  
  pinMode (M1_PLUS, OUTPUT);
  pinMode (M1_MINUS,OUTPUT);
  pinMode (M2_PLUS, OUTPUT);
  pinMode (M2_MINUS,OUTPUT);
  
  pinMode (RIGHTLINE, INPUT);
  pinMode (CENTRALLINE, INPUT);
  pinMode (LEFTLINE, INPUT);
  
  Serial.begin(9600);

  encoder.zero();
}


int cV = 0;
int lV = 0;
int rV = 0;

void loop()
{
  int sRight = analogRead(RIGHTLINE);
  int sCenter = analogRead(CENTRALLINE);
  int sLeft = analogRead(LEFTLINE);

  int fTime1 = 150;
  int fTime2 = 150;

  if(Serial.available() > 0){
    int angle = Serial.parseInt();
    switch (angle){
      case 0:
        front();
        delay(fTime1);
        break;
        
      case 90:
        front();
        delay(fTime1);
        left();
        front();
        delay(fTime2);
        break;
        
      case -90:
        front();
        delay(fTime1);
        right();
        front();
        delay(fTime2);
        break;
        
    }  
  }
  int edge = 700;

  int d1 = 200;
  int d2 = 400;
  int d3 = 150;
  
  float k1 = 1 + (sCenter - cV) / 30;
  float u = k1 * ((sRight - lV) - (sLeft - rV));

  analogWrite(M1_PLUS, u + 50);
  analogWrite(M1_MINUS , 0);

  analogWrite(M2_PLUS, 0);
  analogWrite(M2_MINUS, -u + 50);

  delay(50);
  block();
  delay(150);

  cV = sCenter;
  lV = sLeft;
  rV = sRight;
}
