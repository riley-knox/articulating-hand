#include <ros.h>
#include <Servo.h>            // PROBABLY DON'T NEED, MAYBE DELETE WHEN CLEANING UP ROS NODE
#include <Adafruit_PWMServoDriver.h>
#include <articulating_hand/ServoArray.h>

#define SERVO_FREQ 50
#define SERVO_MIN 10
#define SERVO_MAX 5000
#define NUM_SERVOS 11
#define BAUD 115200

// instantiate node handle; allows publisher/subscriber creation and handles serial communication
ros::NodeHandle nh;

// instantiate servo driver board interfacing
Adafruit_PWMServoDriver srvDrvr = Adafruit_PWMServoDriver();

// actuation callback function
void servo_callback(const articulating_hand::ServoArray& cmd){
  for (int i = 0; i < NUM_SERVOS; i++){
    uint8_t servoNum = cmd.srv_comms[i].servo_num;           // servo number
    uint16_t servoPos = cmd.srv_comms[i].servo_pos;          // servo position

    srvDrvr.setPWM(servoNum, 0, servoPos);
  }
}

// initialize subscriber
ros::Subscriber<articulating_hand::ServoArray> servo_sub("servo_cmd", &servo_callback);

void setup() {
  nh.getHardware()->setBaud(BAUD);      // initialize serial communication; baud = 115200

  nh.initNode();            // initialize node

  nh.subscribe(servo_sub);  // initialize subscriber

  srvDrvr.begin();
  srvDrvr.setOscillatorFrequency(27000000);
  srvDrvr.setPWMFreq(SERVO_FREQ);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
