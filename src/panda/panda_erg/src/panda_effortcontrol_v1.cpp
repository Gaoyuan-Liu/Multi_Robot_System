#include <cmath>
#include <iostream>

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

std::vector<double> q_lowerlimit = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}; // [rad]
std::vector<double> q_upperlimit = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973}; // [rad]
std::vector<double> q(7);   // current joint angles [rad]
std::vector<double> q_r(7);  // reference joint angles [rad]

std::vector<double> dotq(7); // current joint velocities [rad/s]

std::vector<double> tau_cmd(7); // torque to be applied to robot [Nm]

std::vector<double> Kp = {12.0, 16.0, 18.0, 15.0, 8.0, 5.0, 2.0}; 
std::vector<double> Ki = {20.0, 20.0, 20.0, 20.0, 0.0, 0.0, 0.0}; 
std::vector<double> Kd = {8.0, 4.5, 1.0, 3.0, 2.0, 1.0, 0.5}; 

std::vector<double> i_error = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
std::vector<double> i_clamp = {2.5, 4.0, 2.5, 2.5, 1.0, 1.0, 1.0}; 

double samplingFreq = 1000.0; //[Hz]

bool get_q; // get current joint angles
bool get_qr; // get reference joint angles

// publishers and subscribers
ros::Subscriber sub_state;
ros::Publisher pub_effort_cmd[7];


void jointStatesCallback(const sensor_msgs::JointState& jointstatesmsg)
{
    if( (get_q == false) || (get_q == true && get_qr==true) )
    {
        // ROS_INFO("in jointStatesCallback");
        for (int i=0; i<jointstatesmsg.position.size(); i++)
        {
            if(i==0 || i==1)
            {
                // i=0: panda_finger_joint1, i=1: panda_finger_joint_2
            }
            else
            {
                q[i-2] = jointstatesmsg.position[i]; // arm joints
                dotq[i-2] = jointstatesmsg.velocity[i];
            }
        } 
        get_q = true;
    }   

    if(get_q == true && get_qr==false)
    {
        for (int i=0; i<q.size(); i++)
        {
            std::cout << "panda_joint_q" + std::to_string(i+1) + " : " + std::to_string(q_lowerlimit[i]) + " < " + std::to_string(q[i]) + " < " + std::to_string(q_upperlimit[i])  << std::endl;
        }
        std::cout << std::endl;
    }
}

int main(int argc, char **argv)
{
    get_q = false;
    get_qr = false;

    // initialize ROS, specify ROS node, and initialize this ROS node
    ros::init(argc, argv, "panda_effortcontrol_v1"); 
    ros::NodeHandle n;
    ros::Rate loop_rate(samplingFreq); 

    // get the current joint angles
    sub_state = n.subscribe("/panda/joint_states",1,jointStatesCallback);

    // send effort commands to robot
    for(int i=0; i<q.size(); i++)
    {
        pub_effort_cmd[i] = n.advertise<std_msgs::Float64>("/panda/joint" + std::to_string(i+1) + "_effort_controller/command",100);
    }
    while(ros::ok())
    {
        if(get_q==true && get_qr==false)
        {
            // std::cout << "Give 7 desired joint angles: ";
            // for (int i=0; i<q_r.size();i++)
            // {
            //     double input;
            //     std::cin >> input;
            //     q_r[i] = input;
            // }
            for (int i=0; i<q_r.size();i++)
            {
                q_r[i] = q[i];
            }
            q_r[0] = 1.0;
            q_r[1] = 1.0;
            q_r[2] = 1.0; 
            q_r[3] = -1.0; 
            q_r[4] = 1.0; 
            q_r[5] = 1.0; 
            q_r[6] = 1.0; 

            
            get_qr=true;
        }

        else if(get_q==true && get_qr==true)
        {
            sub_state = n.subscribe("/panda/joint_states",1,jointStatesCallback);
            for (int i=0; i<q_r.size();i++)
            {
                i_error[i] = i_error[i] + (q_r[i] - q[i]) *(1.0/samplingFreq);//(ros::Time::now() - start).toSec();
                if (i_error[i] < -i_clamp[i])
                {
                    i_error[i] = -i_clamp[i];
                }
                else if (i_error[i] > i_clamp[i])
                {
                    i_error[i] = i_clamp[i];
                }
                tau_cmd[i] = Kp[i]*(q_r[i]-q[i]) - Kd[i]*dotq[i] + Ki[i]*i_error[i];
            }
            for (int i=0; i<q_r.size();i++)
            {   
                std_msgs::Float64 tau_cmd_msg;
                tau_cmd_msg.data = tau_cmd[i];  
                pub_effort_cmd[i].publish(tau_cmd_msg);             
            }
        }

        ros::spinOnce();
        loop_rate.sleep();    
    }
    return 0;
}