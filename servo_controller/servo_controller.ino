#include <ros.h>
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <articulating_hand/ServoDrive.h>

#define SERVO_FREQ 50
#define SERVO_MIN 10
#define SERVO_MAX 5000
#define NUM_SERVOS 11

// instantiate node handle; allows publisher/subscriber creation and handles serial communication
ros::NodeHandle nh;

// instantiate servo driver board interfacing
Adafruit_PWMServoDriver srvDrvr = Adafruit_PWMServoDriver();

// actuation callback function
void servo_callback(const articulating_hand::ServoDrive& cmd){
  uint8_t servoNum = cmd.servo_num;           // servo number
  uint16_t servoPos = cmd.servo_pos;          // servo position

  if ((0 <= servoNum) && (servoNum < NUM_SERVOS) &&       // make sure servo number and position are in range
      (SERVO_MIN < servoPos) && (servoPos < SERVO_MAX)){
    digitalWrite(13, HIGH);           // turn on onboard LED

    uint16_t pulseLength = servoPos;

//    for (uint16_t pulseLength = servoPos; pulseLength < SERVO_MAX; pulseLength++){
    srvDrvr.setPWM(servoNum, 0, pulseLength);
//    }

    delay(500);

    digitalWrite(13, LOW);            // turn off onboard LED
  }
  else {                      // if servo number or position is out of range
    digitalWrite(10, HIGH);   // turn on external LED
    delay(1000);              // wait 1s
    digitalWrite(10, LOW);    // turn off external LED
  }

}

// initialize subscriber
ros::Subscriber<articulating_hand::ServoDrive> servo_sub("servo_cmd", &servo_callback);

void setup() {
  nh.initNode();            // initialize node

  nh.subscribe(servo_sub);  // initialize subscriber

  srvDrvr.begin();
  srvDrvr.setOscillatorFrequency(27000000);
  srvDrvr.setPWMFreq(SERVO_FREQ);

  pinMode(10, OUTPUT);      // external LED pin
  pinMode(13, OUTPUT);      // onboard LED pin
}

void loop() {
  nh.spinOnce();
  delay(1);
}
