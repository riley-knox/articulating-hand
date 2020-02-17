#include <ros.h>
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <articulating_hand/ServoDrive.h>

#define SERVO_FREQ 50         

ros::NodeHandle nh;

// actuation callback function
void servo_callback(const articulating_hand::ServoDrive& cmd){
  ;
}

// initialize subscriber
ros::Subscriber<articulating_hand::ServoDrive> servo_sub("servo_cmd", &servo_callback);

void setup() {
  // setup functions
  // largely cribbed from a variety of examples, MAY NEED TO ALTER THINGS A BIT
  nh.initNode();            // initialize node

  nh.subscribe(servo_sub);  // initialize subscriber

}

void loop() {
  nh.spinOnce();
  delay(1);
}
