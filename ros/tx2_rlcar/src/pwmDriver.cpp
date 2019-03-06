#include "ros/ros.h"
#include "tx2_rlcar/THETA.h"
#include "JHPWMPCA9685.h"
#include <time.h>
#include <stdlib.h>
#include <sensor_msgs/Joy.h>

#define PWM_FULL_FORWARD 1241
#define PWM_NEUTRAL 1861
#define PWM_FULL_REVERSE 2482
#define PWM_FREQUENCY 300

#define ESC_CHANNEL 10
#define SERVO_CHANNEL 8

void msgCallback(const sensor_msgs::Joy::ConstPtr& msg);
void initDriver(PCA9685* pca9685);

PCA9685* pca9685 = new PCA9685();

int main(int argc, char **argv){
  initDriver(pca9685);
  ros::init(argc, argv, "pwmDriver");

  ros::NodeHandle nh;
  ros::Subscriber ros_sub = nh.subscribe("joy", 100, msgCallback);
  ros::spin();
  return 0;
}

void msgCallback(const sensor_msgs::Joy::ConstPtr& msg){
  float survo = msg->axes[0];
  ROS_INFO("Servo Passed By Joystick Data is : %f", survo);
  if(survo > 0) {
    pca9685->setPWM(SERVO_CHANNEL, 0, PWM_NEUTRAL + ((PWM_FULL_REVERSE - PWM_NEUTRAL) * survo));
    printf("Value of Servo : %lf\n", PWM_NEUTRAL + ((PWM_FULL_REVERSE - PWM_NEUTRAL) * survo));
  } else {  
    survo = -survo;
    pca9685->setPWM(SERVO_CHANNEL, 0, PWM_NEUTRAL - ((PWM_NEUTRAL - PWM_FULL_FORWARD) * survo));

    printf("Value of Servo : %lf\n", PWM_NEUTRAL - ((PWM_NEUTRAL - PWM_FULL_FORWARD) * survo));
  }
  
  float motor = msg->axes[1];
  ROS_INFO("Motor Passed By JoyStick Data is : %f", motor);
  if(motor > 0) {
    pca9685->setPWM(ESC_CHANNEL, 0, PWM_NEUTRAL + ((PWM_FULL_REVERSE - PWM_NEUTRAL) * motor) * 0.7);
  } else { 
    motor = -motor;
    pca9685->setPWM(ESC_CHANNEL, 0, PWM_NEUTRAL - ((PWM_NEUTRAL - PWM_FULL_FORWARD) * motor) * 0.7);
  }
 
  float bButton = msg->buttons[1];
  if(bButton > 0){
    system("rosnode kill zed/zed_wrapper_node zed/zed_state_publisher");
  }  
} 

void initDriver(PCA9685* pca9685){
  if(pca9685->openPCA9685() < 0) {
    perror("Could not open PCA9685 : " );
  } else {
    printf("PCA9685 Device Address : 0x%02X\n", pca9685->kI2CAddress);
    pca9685->setAllPWM(0,0);
    pca9685->setPWM(SERVO_CHANNEL, 0, PWM_NEUTRAL);
    pca9685->reset();
    pca9685->setPWMFrequency(PWM_FREQUENCY);
    sleep(1);
    
  }
}

