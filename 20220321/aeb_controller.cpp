#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"

std_msgs::Bool flag_AEB;
std_msgs::Float32 delta_range;
std_msgs::Float32 old_sonar_range;
geometry_msgs::Twist cmd_vel_msg;


void UltraSonarCallback(const sensor_msgs::Range::ConstPtr& msg)
{
	ROS_INFO("Sonar Seq: [%d]", msg->header.seq);
	ROS_INFO("Sonar Range: [%f]", msg->range);
	
	if(msg->range <=1.8)
	{
		ROS_INFO("AEB_Activation");
		flag_AEB.data=true;
	}
	else
	{
		flag_AEB.data=false;
	}
	delta_range.data = old_sonar_range.data-msg->range;
    ROS_INFO("delta_range : [%f]", delta_range.data);
    old_sonar_range.data = msg->range;
}

void CarControlCallback(const geometry_msgs::Twist& msg)
{
	ROS_INFO("Cmd_vel : linear_x [%f]", msg.linear.x);
	
	cmd_vel_msg = msg;
	//ROS_INFO("Cmd_vel : linear_x [%f]", cmd_vel_msg.linear.x);
}

void UltraSonarCallback2(const sensor_msgs::Range::ConstPtr& msg)
{
	ROS_INFO("Sonar2 Seq: [%d]", msg->header.seq);
	ROS_INFO("Sonar2 Range: [%f]", msg->range);
}

int main(int argc, char **argv)
{
	int count = 0;
	
	ros::init(argc, argv,"aeb_controller");
	
	ros::NodeHandle n;
	
	ros::Rate loop_rate(10);
	
	ros::Subscriber sub = n.subscribe("/range", 1000, UltraSonarCallback);
	ros::Subscriber sonar_sub = n.subscribe("/RangeSonar1", 1000, UltraSonarCallback2);
	ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 10, &CarControlCallback);
	
	ros::Publisher pub_aeb_activation_flag = n.advertise<std_msgs::Bool>("/aeb_activation_flag", 1);
	ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 10);
	
	while(ros::ok())
	{
		if((count%10)==0)
		{
			pub_aeb_activation_flag.publish(flag_AEB);
		}
		
		if(flag_AEB.data == true)
		{
			cmd_vel_msg.linear.x = 0;
			pub_cmd_vel.publish(cmd_vel_msg);
		}
		
		else
		{
			pub_cmd_vel.publish(cmd_vel_msg);
		}
		loop_rate.sleep();
		ros::spinOnce();
		++count;
	}
	
	return 0;
}
