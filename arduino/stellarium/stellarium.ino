#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  map(1100, 0, 2000, 0, 4096)
#define SERVOMAX  map(1900, 0, 2000, 0, 4096)
#define led 13
const float pi = 3.14;
int steps = 100;
int pulselen;

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  
  pinMode(led, OUTPUT);

  pwm.begin();
  
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  
}

void loop() {
  digitalWrite(led, HIGH);
  for (float current = 0; current < (2*pi); current+=(pi/steps)) {
    Serial.println(current);
    pulselen = mapfloat(cos(current), -1, 1, 150, 400);
    Serial.println(pulselen);
    Serial.println("###");
    pwm.setPWM(0, 0, pulselen);
    delay(20);
  }
  digitalWrite(led, LOW);
  for (float current = (2*pi); current > 0; current-=(pi/steps)) {
    pulselen = mapfloat(cos(current), -1, 1, 150, 400);
    pwm.setPWM(0, 0, pulselen);
    delay(20);
  }
}

