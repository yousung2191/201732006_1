#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"

std_msgs::Bool flag_AEB;

void UltraSonarCallback(const sensor_msgs::Range::ConstPtr& msg)
{
	ROS_INFO("Sonar Seq: [%d]", msg->header.seq);
	ROS_INFO("Sonar Range: [%f]", msg->range);
	
	if(msg -> range <= 1.0)
	{
		ROS_INFO("AEB_Activation1");
		flag_AEB.data = true;
	}
	else
	{
		flag_AEB.data = false;
	}
}

void UltraSonarCallback2(const sensor_msgs::Range::ConstPtr& msg)
{
	ROS_INFO("Sonar2 Seq: [%d]", msg->header.seq);
	ROS_INFO("Sonar2 Range: [%f]", msg->range);
	
	if(msg -> range <= 1.0)
	{
		ROS_INFO("AEB_Activation2");
		flag_AEB.data = true;
	}
	else
	{
		flag_AEB.data = false;
	}
}

int main(int argc, char **argv)
{
	int count = 0;
	
	ros::init(argc, argv,"aeb_controller");
	
	ros::NodeHandle n;
	
	ros::Rate loop_rate(10);
	
	ros::Subscriber sub = n.subscribe("range", 1000, UltraSonarCallback);
	ros::Subscriber sub2 = n.subscribe("/RangeSonar1", 1000, UltraSonarCallback2); //package: ultra_sonar_test -> ultra_sonar_test_node 
	
	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
		++count;
	}
	
	return 0;
}
