

#include <ros.h>


#include <geometry_msgs/Twist.h>

#define PWM_MIN 0
#define PWMRANGE 100

//functions
void setupPins();
void Stop();
void onTwist(const geometry_msgs::Twist &msg);
float mapPwm(float x, float out_min, float out_max);

// Pins

int motor1_fpwm=3;
int motor1_rpwm=5;

int motor2_rpwm=9;
int motor2_fpwm=10;


void onTwist(const geometry_msgs::Twist &msg)
{
  
 // Cap values at [-1 .. 1]
 float x = max(min(msg.linear.x, 1.0f), -1.0f);
 float z = max(min(msg.angular.z, 1.0f), -1.0f);

 // Calculate the intensity of left and right wheels.

 float l = (msg.linear.x - msg.angular.z) / 2;
 float r = (msg.linear.x + msg.angular.z) / 2;

 // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.
 
   float lPwm = mapPwm(fabs(l), PWM_MIN, PWMRANGE);
   float rPwm = mapPwm(fabs(r), PWM_MIN, PWMRANGE);

 
      //Set direction pins and PWM
 analogWrite(motor1_fpwm,(l > 0) *lPwm);
 analogWrite(motor1_rpwm, (l < 0)*lPwm);
 analogWrite(motor2_fpwm, (r > 0)*rPwm);
 analogWrite(motor2_rpwm, (r < 0)*rPwm);

    
  if(msg.linear.x >0)digitalWrite(LED_BUILTIN,HIGH);
  else digitalWrite(LED_BUILTIN,LOW);
   

}


ros::NodeHandle node;

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &onTwist);



void setup()
{
  node.initNode();
  node.subscribe(sub);
  setupPins();
}





void loop()
{
  node.spinOnce();

}

void setupPins()
{

  pinMode(motor1_fpwm, OUTPUT);
  pinMode(motor1_rpwm, OUTPUT);
  pinMode(motor2_fpwm, OUTPUT);
  pinMode(motor2_rpwm, OUTPUT);


}

void Stop(){
  digitalWrite(motor1_fpwm,LOW);
  digitalWrite(motor1_rpwm,LOW);
  digitalWrite(motor2_fpwm,LOW);
  digitalWrite(motor2_rpwm,LOW);
 
}

// Map x value from [0 .. 1] to [out_min .. out_max]
float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}
