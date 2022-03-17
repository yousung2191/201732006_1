#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"

std_msgs::Bool flag_AEB;

void UltraCallBack(const sensor_msgs::Range::ConstPtr& msg)
{
  if(msg->range <= 1.0)
  {
		flag_AEB.data = true;
  }
  else
  {
		flag_AEB.data = false;
  }
}

int main(int argc, char**argv) {
	int count = 0;
	
	ros::init(argc, argv, "aeb_publisher");
	
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("range", 1000, UltraCallBack);
	
	ros::Publisher pub1 = n.advertise<std_msgs::Bool>("bool1", 1000);
	
	ros::Rate loop_rate(1);
	
	
	while(ros::ok()) {
		pub1.publish(flag_AEB);   
		loop_rate.sleep();
		ros::spinOnce();
		++count;
	}
	return 0;
}
