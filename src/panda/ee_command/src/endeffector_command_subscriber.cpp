#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

static std_msgs::Float64MultiArray ee_command;

void EndeffectorCallback(const std_msgs::Float64MultiArrayPtr &command)
{
  //ee_command.data.resize(3);
  ee_command = *command;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "endeffector_command_subscriber");


  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  while (ros::ok())
	{
                //static std_msgs::Float64MultiArray ee_command;
                ros::Subscriber sub = n.subscribe("/panda/endeffector_command", 1, EndeffectorCallback);
                ee_command.data.resize(3);
                std_msgs::Float64MultiArray a;
                a.data.resize(3);
                a.data[1]=2;

                //std::vector<double> b(3);

                //a = ee_command;
                //b = a.data;
                //printf("I heard: [%f]", ee_command.data[1]);
                //printf("\n");

		ros::spin();
		loop_rate.sleep();
	}
  
  

         return 0;
}



