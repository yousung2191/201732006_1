#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"

std_msgs::Bool flag_AEB;

void UltraSonarCallback2(const std_msgs::Bool::ConstPtr& msg)
{
	
	if(msg->data)
	{
		ROS_INFO("AEB_Activation");
	}
	else
	{
		ROS_INFO("AEB_off");
	}
}

int main(int argc, char **argv)
{
	int count = 0;
	
	ros::init(argc, argv,"aeb_controller");
	
	ros::NodeHandle n;
	
	ros::Rate loop_rate(1);
	
	ros::Subscriber sub2 = n.subscribe("bool1", 1000, UltraSonarCallback2);
	
	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
		++count;
	}
	
	return 0;
}
