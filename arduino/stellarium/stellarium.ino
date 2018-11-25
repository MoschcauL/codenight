#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_ANGLE 122
#define SERVO_MIN 150
#define SERVO_MAX 400

//#define SERVOMIN  map(1100, 0, 2000, 0, 4096)
//#define SERVOMAX  map(1900, 0, 2000, 0, 4096)

#define led 13
const float pi = 3.14;
int steps = 100;
int pulselen;

int posA = 0;

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float clip(float value, int min_value, int max_value) {
  float ret = (value<min_value)?min_value:(value>max_value)?max_value:value;
  digitalWrite(led, value!=ret);
  return ret;
}

float absfloat(float number) {
  return (number>0)?number:-number;
}

float directToAngle(float angle, int min_angle, int max_angle, int port, int *pos) {
  angle = clip(angle, min_angle, max_angle);
  for (float current = *pos; absfloat(current-angle)>=0.6; current+=(current-angle<0)?1:-1) {
    Serial.println(current);
    pulselen = mapfloat(current, min_angle, max_angle, 150, 400);
    pwm.setPWM(port, 0, pulselen);
    delay(20);
  }
    pulselen = mapfloat(angle, min_angle, max_angle, 150, 400);
    pwm.setPWM(port, 0, pulselen);
    *pos = angle;  
}

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  
  pinMode(led, OUTPUT);

  pwm.begin();
  
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  
}

void loop() {
  /*for (float current = 0; current < 1; current+=(1.0/steps)) {
    Serial.println(current);
    pulselen = mapfloat(current, 0, 1, 150, 400);
    Serial.println(pulselen);
    Serial.println("###");
    pwm.setPWM(0, 0, pulselen);
    delay(10);
  }
  digitalWrite(led, HIGH);
  delay(3000);
  digitalWrite(led, LOW);
  for (float current = 1; current > 0; current-=(1.0/steps)) {
    Serial.println(current);
    pulselen = mapfloat(current, 0, 1, 150, 400);
    Serial.println(pulselen);
    Serial.println("###");
    pwm.setPWM(0, 0, pulselen);
    delay(10);
  }
  digitalWrite(led, HIGH);
  delay(3000);
  digitalWrite(led, LOW);
  */
  while (Serial.available() > 0) {
    int longitude = Serial.parseInt();
    int latitude = Serial.parseInt();
    if (Serial.read() == '\n') {
      Serial.print(longitude);
      Serial.println(latitude);
    }
    Serial.print(longitude);
    Serial.println(latitude);
    directToAngle(longitude, -30, 30, 0, &posA);
  }
 
  
}

