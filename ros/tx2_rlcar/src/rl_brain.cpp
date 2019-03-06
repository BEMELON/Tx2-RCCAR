#include "ros/ros.h"
#include "tx2_rlcar/THETA.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "rl_brain");
  ros::NodeHandle nh;

  ros::Publisher ros_pub=nh.advertise<tx2_rlcar::THETA>("theta", 100);
  ros::Rate loop_rate(10);

  tx2_rlcar::THETA msg;

  int count = 0;

  while(ros::ok()) {
	msg.stamp = ros::Time::now();
	msg.data = count;

	ROS_INFO("Send msg = %d", msg.stamp.sec);
	ROS_INFO("Send msg = %d", msg.stamp.nsec);
	ROS_INFO("Send msg = %d", msg.data);

	ros_pub.publish(msg);

	loop_rate.sleep();

	count++;
  }

  return 0;
}
   
