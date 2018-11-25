#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_ANGLE 122
#define SERVO_MIN 150
#define SERVO_MAX 400

//#define SERVOMIN  map(1100, 0, 2000, 0, 4096)
//#define SERVOMAX  map(1900, 0, 2000, 0, 4096)

#define led 13
int pulselen = 200;



void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  
  pinMode(led, OUTPUT);

  pwm.begin();
  
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  
}

void loop() {
  if (Serial.available() > 0) {
    char a = Serial.read();
    switch (a) {
      case 'r':
        pulselen+=1;
        break;
      case 'R':
        pulselen+=10;
        break;
      case 'l':
        pulselen-=1;
        break;
      case 'L':
        pulselen-=10;
        break;
      }
    pwm.setPWM(0, 0, pulselen);
    Serial.println( pulselen);
  }
}

