#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"

std_msgs::Bool flag_AEB1;
std_msgs::Bool flag_AEB2;

void UltraSonarCallback(const sensor_msgs::Range::ConstPtr& msg)
{
	ROS_INFO("Sonar Seq: [%d]", msg->header.seq);
	ROS_INFO("Sonar Range: [%f]", msg->range);
	
	if(msg->range <=1.0)
{
	ROS_INFO("Activation1");
	flag_AEB1.data = true;
}
else
{
	flag_AEB1.data = false;
}
}

void UltraSonarCallback1(const sensor_msgs::Range::ConstPtr& msg1)
{
	ROS_INFO("Sonar Seq: [%d]", msg1->header.seq);
	ROS_INFO("Sonar Range: [%f]", msg1->range);
	
	if(msg1->range <=1.0)
{
	ROS_INFO("Activation2");
	flag_AEB2.data = true;
}
else
{
	flag_AEB2.data = false;
}
}

int main(int argc, char**argv) {
	int count = 0;
	
	ros::init(argc, argv, "aeb_controller");
	
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("range", 1000, UltraSonarCallback);
	ros::Subscriber sub1 = n.subscribe("range1", 1000, UltraSonarCallback1);
	
	ros::Rate loop_rate(30);
	
	
	while(ros::ok()) {
		loop_rate.sleep();
		ros::spinOnce();
		++count;
	}
	return 0;
}
