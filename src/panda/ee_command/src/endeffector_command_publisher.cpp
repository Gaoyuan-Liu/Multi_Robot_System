#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <iostream>
int main(int argc, char **argv)
{
	ros::init(argc, argv,"endeffector_command_publisher");
	ros::NodeHandle root_nh;

	ros::Publisher pose_publisher = root_nh.advertise<std_msgs::Float64MultiArray>("/panda/endeffector_command", 100);
        
	ros::Rate loop_rate(1000);
	ros::Time startTime = ros::Time::now();
	std_msgs::Float64MultiArray position_command;
        position_command.data.resize(3);
	while (ros::ok())
	{
            if ((ros::Time::now()-startTime).toSec() <= 10)
            {    
		position_command.data[0] = 0.28;
		position_command.data[1] = 0.45;
		position_command.data[2] = 0.71;
            }
            else if ((ros::Time::now()-startTime).toSec() > 10)
            {
                position_command.data[0] = 0.15;
		position_command.data[1] = 0.15;
		position_command.data[2] = 0.55;
            }

		pose_publisher.publish(position_command);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}


