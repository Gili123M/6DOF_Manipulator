#include "Wire.h"
#include <Adafruit_PWMServoDriver.h>

#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define FREQUENCY             50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#include "Arduino.h"
#include "ArduinoHardware.h"
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/JointState.h>

// Set ROS - handler, subscribe message, publish message (debugging)
ros::NodeHandle  nh;
std_msgs::Int16MultiArray str_msg2;
ros::Publisher chatter("servo", &str_msg2);
int servoDegree[6];

// Define Motor Outputs on PCA9685 board
int motor1 = 0;
int motor2 = 1;
int motor3 = 2;
int motor4 = 3;
int motor5 = 4;
int motor6 = 5;

// Define Motor position variables
double mtr1;
double mtr2;
double mtr3;
double mtr4;
double mtr5;
double mtr6;

// Function move motor to ROS angle
void servo_cb(const sensor_msgs::JointState& cmd_msg)
{
  mtr1 = trimLimits(radiansToDegrees(abs(cmd_msg.position[0])));
  mtr2 = trimLimits(radiansToDegrees(abs(cmd_msg.position[1])));
  mtr3 = trimLimits(radiansToDegrees(abs(cmd_msg.position[2])));
  mtr4 = trimLimits(radiansToDegrees(abs(cmd_msg.position[3])));
  mtr5 = trimLimits(radiansToDegrees(abs(cmd_msg.position[4])));
  mtr6 = trimLimits(radiansToDegrees(abs(cmd_msg.position[5])));

  // Store motor movements for publishing back to ROS (debugging)
  servoDegree[0] = mtr1;
  servoDegree[1] = mtr2;
  servoDegree[2] = mtr3;
  servoDegree[3] = mtr4;
  servoDegree[4] = mtr5;
  servoDegree[5] = mtr6;

  moveMotorDeg(mtr1, motor1);
  moveMotorDeg(mtr2, motor2);
  moveMotorDeg(mtr3, motor3);
  moveMotorDeg(mtr4, motor4);
  moveMotorDeg(mtr5, motor5);
  moveMotorDeg(mtr6, motor6);}  

  ros::Subscriber<sensor_msgs::JointState> sub("joint_states", servo_cb);

  void setup()
  {
    // Setup ROS fir subscribe and publish
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(chatter);

    // Setup PWM Controller object
    pwm.begin();
    pwm.setPWMFreq(FREQUENCY);
  }

  // Function to move motor to specific position
  void moveMotorDeg(int moveDegree, int motorOut)
  {
    int pulse_wide, pulse_width;

    // Convert to pulse width
    pulse_wide = map(moveDegree, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);

    //Control Motor
    pwm.setPWM(motorOut, 0, pulse_width);
  }

  void loop()
  {
    str_msg2.data = servoDegree;
    str_msg2.data_length = 6;
    chatter.publish( &str_msg2 );
    nh.spinOnce();
  }

  // Convert radians to degreees
  double radiansToDegrees(float position_radians)
  {
    position_radians = position_radians * 57.2958;
    return position_radians;
  }

  // Sometimes servo angle goes just above 180 or just below 0 - trim to 0 or 180
  double trimLimits(double mtr_pos)
  {
    if(mtr_pos > 180) {
      mtr_pos = 180;
    }
    if(mtr_pos < 0) {
      mtr_pos = 0;
    }
    return mtr_pos;
  }
