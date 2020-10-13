
//#include "std_msgs/Float64MultiArray.h"
// Include from track_ik
#include "panda_utils/panda_trac_ik.h"
#include "panda_interface/panda_pose_controller.h"
#include "std_msgs/Float64MultiArray.h"
#include <std_msgs/Bool.h>
#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <stabilizing_control/stabilizing_control.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"panda_trac_ik_taskspace"); // initialize ROS, specify ROS node, and initialize this ROS node
    StabilizingControl stabcontrol("panda_trac_ik_taskspace");
    ros::NodeHandle root_nh; // to create a publisher
    ros::NodeHandle root_nh_2;
    hiro_panda::PandaTracIK _panda_ik_service;

    geometry_msgs::Pose _target_pose;
    KDL::JntArray _joints_result;

    ros::Publisher pose_publisher = root_nh.advertise<std_msgs::Float64MultiArray>("/panda/endeffector_state", 100);
    ros::Publisher is_feasible_publisher = root_nh.advertise<std_msgs::Bool>("/panda/is_feasible", 100);

    ros::Publisher q_r_publisher = root_nh.advertise<std_msgs::Float64MultiArray>("/panda/joint_references", 100);


    ros::Rate loop_rate(1000); 
    ros::Time startTime = ros::Time::now();
    _panda_ik_service = hiro_panda::PandaTracIK();

    StabilizingControl::PositionVector ee_last;
    // Initial psoition
    /*stabcontrol.ee_command[0] = 0.25;
    stabcontrol.ee_command[1] = 0.35;
    stabcontrol.ee_command[2] = 0.65;*/
    /*ee_last[0] = 0.25;
    ee_last[1] = 0.35;
    ee_last[2] = 0.65;*/
    _target_pose.orientation.w = 1;
    _target_pose.orientation.x = 0;
    _target_pose.orientation.y = 0;
    _target_pose.orientation.z = 0;
    _target_pose.position.x = 0.15;
    _target_pose.position.y = 0.05;
    _target_pose.position.z = 0.75;
    
    // This is where ik really happens
    KDL::JntArray ik_result = _panda_ik_service.perform_ik(_target_pose);// Send the is_feasible flag

    _joints_result = (_panda_ik_service.is_valid) ? ik_result : _joints_result;


    
    // The IK calculation shoud be out of the loop, because every time the calculation may give different results, which make the arm vibrate.
    

    while(ros::ok())
    {

        // Read command from /panda/endeffector_command
        stabcontrol.geteecommand(); //Get stablecontrol.ee_command form eecommand topic
        /*std::cout << "x_command is " << stabcontrol.ee_command[0] << std::endl;
        std::cout << "y_command is " << stabcontrol.ee_command[1] << std::endl;
        std::cout << "z_command is " << stabcontrol.ee_command[2] << std::endl;*/
        if (ee_last != stabcontrol.ee_command && stabcontrol.ee_command[2] != 0)
        {
            
            _target_pose.position.x = stabcontrol.ee_command[0];
            _target_pose.position.y = stabcontrol.ee_command[1];
            _target_pose.position.z = stabcontrol.ee_command[2];
            KDL::JntArray ik_result = _panda_ik_service.perform_ik(_target_pose);// Send the is_feasible flag
            _joints_result = (_panda_ik_service.is_valid) ? ik_result : _joints_result;
        }
        
        ee_last = stabcontrol.ee_command;
        // Publish the is_feasible flag
        std_msgs::Bool feasible_msg;
        feasible_msg.data = _panda_ik_service.is_feasible;
        is_feasible_publisher.publish(feasible_msg);



            // Implement the joint angles
            if (_joints_result.rows() != 7)
            {   
                ROS_ERROR("Panda Pose Controller: Wrong Amount of Rows Received From TRACIK");
            }
            for (int i=0; i<7; i++)
            {
            stabcontrol.q_r[i] = _joints_result(i);
            //std::cout << " q_r" << i << " = " << stabcontrol.q_r[i] << std::endl;
            }

            // Send q_r to erg
            std_msgs::Float64MultiArray q_r_msg;
            q_r_msg.data.resize(7);
            for (int i=0; i<7; i++)
            {
            q_r_msg.data[i] = stabcontrol.q_r[i];
            }
            q_r_publisher.publish(q_r_msg);
            stabcontrol.PIDcontrol(); // Effort control here

            // Publish the end-effector current position



            stabcontrol.forwardKinematics();
            std_msgs::Float64MultiArray ee_state_msg;
            ee_state_msg.data.resize(3);
            ee_state_msg.data[0] = stabcontrol.ee_state[0];
            ee_state_msg.data[1] = stabcontrol.ee_state[1];
            ee_state_msg.data[2] = stabcontrol.ee_state[2];
            pose_publisher.publish(ee_state_msg);

            ros::spinOnce();
            loop_rate.sleep();

    }
    return 0;
}
