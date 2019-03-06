#include "ros/ros.h"
#include "tx2_rlcar/THETA.h"
#include "JHPWMPCA9685.h"
#include <time.h>
#include <stdlib.h>
#include <sensor_msgs/Joy.h>


void msgCallback(const sensor_msgs::Joy::ConstPtr& msg);

int main(int argc, char **argv){
  ros::init(argc, argv, "capture_right");

  ros::NodeHandle nh;
  ros::Subscriber ros_sub = nh.subscribe("joy", 100, msgCallback);
  ros::spin();
  return 0;
}

void msgCallback(const sensor_msgs::Joy::ConstPtr& msg){
  float aButton = msg->buttons[0];
  if(aButton > 0) {
    system("rosrun image_view image_saver image:=/zed/right/image_rect_color _filename_format:=right_%04d.jpg");
  } 
} 


