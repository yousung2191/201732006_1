#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char**argv) {
	int count = 0;
	
	ros::init(argc, argv, "aeb_publisher");
	
	ros::NodeHandle n;
	
	ros::Publisher num_pub1 = n.advertise<sensor_msgs::Range>("range1", 1000);
	
	ros::Rate loop_rate(10);
	
	
	while(ros::ok()) {
		sensor_msgs::Range msg1;
		num_pub1.publish(msg1);
		loop_rate.sleep();
		ros::spinOnce();
		++count;
	}
	return 0;
}
