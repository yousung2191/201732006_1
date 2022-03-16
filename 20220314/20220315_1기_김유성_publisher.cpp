#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <stdio.h>

float a, b, result = 0;

void chatter1(const std_msgs::Float32::ConstPtr& n1)
{
   ROS_INFO("a = [%f]", n1->data);
   a = n1->data;
}

void chatter2(const std_msgs::Float32::ConstPtr& n2)
{
   ROS_INFO("b = [%f]", n2->data);
   b = n2->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "std_msgs_listener");


  ros::NodeHandle n;
  ros::Rate loop_rate(0.5);
  
  ros::Subscriber sub1 = n.subscribe("num1", 1000, chatter1);
  ros::Subscriber sub2 = n.subscribe("num2", 1000, chatter2);
  
  
  int count = 0;
  while(ros::ok()) {
	  
  result = a+b;
  ROS_INFO("a+b = [%f]", result);
  
  result = a-b;
  ROS_INFO("a-b = [%f]", result);
  
  result = a*b;
  ROS_INFO("a*b = [%f]", result);
  
  result = a/b;
  ROS_INFO("a/b = [%f]", result);
  
  ros::spinOnce();
  
  loop_rate.sleep();
  ++count;
 }
  return 0;
}
