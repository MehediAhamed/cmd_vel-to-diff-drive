
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

// Define physical parameters
const float wheelbase = 200.0; // Distance between front and rear axles (mm)
const float trackWidth = 150.0; // Distance between left and right wheels (mm)
const float wheelRadius = 50.0; // Radius of the wheels (mm)

int mot1_lpwm = 3;
int mot1_rpwm = 5;

int mot2_lpwm = 10;
int mot2_rpwm = 9;

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> twistSub("cmd_vel", &twistCallback);


void twistCallback(const geometry_msgs::Twist& twist_msg) {


  float linearVel = twist_msg.linear.x;
  float angularVel = twist_msg.angular.z;

  // Calculate the steering angles for left and right wheels using Ackermann formula
  float leftWheelAngle = atan(wheelbase / (wheelbase + trackWidth) * tan(angularVel));
  float rightWheelAngle = atan(wheelbase / (wheelbase - trackWidth) * tan(angularVel));

  // Calculate individual motor speeds based on linear velocity
  float motorSpeed = abs(linearVel) * 255; // Adjust this value based on your motor control range

  // Calculate left and right wheel speeds based on linear velocity and angles
  float leftWheelSpeed = motorSpeed / cos(leftWheelAngle);
  float rightWheelSpeed = motorSpeed / cos(rightWheelAngle);


  digitalWrite(mot1_lpwm, leftWheelSpeed >= 0 ? HIGH : LOW);
  analogWrite(mot1_lpwm, abs(leftWheelSpeed));

  digitalWrite(mot2_lpwm, rightWheelSpeed >= 0 ? HIGH : LOW);
  analogWrite(mot2_lpwm, abs(rightWheelSpeed));
//
//  digitalWrite(motorRearLeftDir, leftWheelSpeed >= 0 ? HIGH : LOW);
//  analogWrite(motorRearLeftPWM, abs(leftWheelSpeed));

//  digitalWrite(motorRearRightDir, rightWheelSpeed >= 0 ? HIGH : LOW);
//  analogWrite(motorRearRightPWM, abs(rightWheelSpeed));

  Serial.print(leftWheelSpeed);
  Serial.print("\t");
  Serial.println(rightWheelSpeed);
  
}



void setup() {

  Serial.begin(115200);

  pinMode(mot1_lpwm, OUTPUT);
  pinMode(mot1_rpwm, OUTPUT);
  pinMode(mot2_lpwm, OUTPUT);
  pinMode(mot2_lpwm, OUTPUT);
//  pinMode(motorRearLeftPWM, OUTPUT);
//  pinMode(motorRearLeftDir, OUTPUT);
//  pinMode(motorRearRightPWM, OUTPUT);
//  pinMode(motorRearRightDir, OUTPUT);


  nh.getHardware()->setBaud(115200); // Set the baud rate to match your ROS environment

  nh.initNode();
  nh.subscribe(twistSub);
}

void loop() {
  
  
  nh.spinOnce();  
}
