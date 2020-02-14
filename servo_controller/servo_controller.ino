#include <ros.h>
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVO_FREQ 50         

ros::NodeHandle nh;

// actuate callback function

void setup() {
  // setup functions
  // largely cribbed from a variety of examples, MAY NEED TO ALTER THINGS A BIT
  nh.initNode();        // initialize node

}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(1);
}
