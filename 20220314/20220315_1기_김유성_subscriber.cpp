#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <std_msgs/Float32.h>

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "std_msgs_talker");
  
  ros::NodeHandle n;
  
  ros::Publisher num_pub1 = n.advertise<std_msgs::Float32>("num1", 1000);
  ros::Publisher num_pub2 = n.advertise<std_msgs::Float32>("num2", 1000);
  
  ros::Rate loop_rate(10);
  
  int count = 0;
  while (ros::ok())
  {
	  
    std_msgs::Float32 r_num1, r_num2;
    
    printf("input num: ");
    scanf("%f %f", &r_num1.data, &r_num2.data);
    
    num_pub1.publish(r_num1);
    num_pub2.publish(r_num2);

    ros::spinOnce();
    
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
