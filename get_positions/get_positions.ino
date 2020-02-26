#include <Adafruit_PWMServoDriver.h>

#define SERVO_FREQ 50
#define NUM_SERVOS 11

Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver();

void setup() {
  Serial.begin(9600);
  
  driver.begin();
  driver.setOscillatorFrequency(27000000);
  driver.setPWMFreq(SERVO_FREQ);
}

void loop() {
  // put your main code here, to run repeatedly:
  int servo_0 = analogRead(A0);
  int board_0 = driver.getPWM(0);
  int servo_1 = analogRead(A1);
  int board_1 = driver.getPWM(2);
  int servo_2 = analogRead(A2);
  int board_2 = driver.getPWM(4);
  int servo_3 = analogRead(A3);
  int board_3 = driver.getPWM(6);
  Serial.println("Servo 1");
  Serial.println(servo_0);
  Serial.println(board_0);
  Serial.println("Servo 2");
  Serial.println(servo_1);
  Serial.println(board_1);
  Serial.println("Servo 3");
  Serial.println(servo_2);
  Serial.println(board_2);
  Serial.println("Servo 4");
  Serial.println(servo_3);
  Serial.println(board_3);
  Serial.println(" ");
  delay(5000);
}
