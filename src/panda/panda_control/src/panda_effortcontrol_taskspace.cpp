#include <stabilizing_control/stabilizing_control.h>
#include "std_msgs/Float64MultiArray.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv,"panda_effortcontrol_taskspace"); // initialize ROS, specify ROS node, and initialize this ROS node

    ros::NodeHandle root_nh; // to create a publisher
    

    StabilizingControl stabcontrol("panda_effortcontrol_taskspace");
    ros::Publisher pose_publisher = root_nh.advertise<std_msgs::Float64MultiArray>("/panda/endeffector_state", stabcontrol.samplingFreq);
    ros::Rate loop_rate(stabcontrol.samplingFreq); 
    ros::Time startTime = ros::Time::now();

    while(ros::ok())
    {
        if ((ros::Time::now()-startTime).toSec() <= 10)
        {
            // all reference joint angles = current joint angles
            for (int i=0; i<stabcontrol.q_r.size();i++)
            {
                stabcontrol.q_r[i] = stabcontrol.q[i];
            }
            stabcontrol.PIDcontrol();

            // reference finger displacement = current finger displacement
            for (int i=0; i<stabcontrol.finger_r.size();i++)
            {
                stabcontrol.finger_r[i] = 0.0; //stabcontrol.finger[i];
            }
            stabcontrol.handControl();
            
        }

        else if ((ros::Time::now()-startTime).toSec() > 10)
        {
            // reference end-effector in task space
            stabcontrol.pos_r[0] = 0.2858; // x position
            stabcontrol.pos_r[1] = 0.4451; // y position
            stabcontrol.pos_r[2] = 0.6106; // z position
            //stabcontrol.pos_r[2] = 0.6106;

            stabcontrol.orient_r[0] = 0.6077; // x quaternion
            stabcontrol.orient_r[1] = 0.7544; // y quaternion
            stabcontrol.orient_r[2] = -0.2471; // z quaternion
            stabcontrol.orient_r[3] = 0.0266; // w quaternion
 
            /*stabcontrol.orient_r[0] = 0.0; // x quaternion
            stabcontrol.orient_r[1] = 0.0; // y quaternion
            stabcontrol.orient_r[2] = 0.0; // z quaternion
            stabcontrol.orient_r[3] = 1; // w quaternion*/


            // inverse kinematics: output = q_r
            stabcontrol.inverseKinematics(); 

            // reference finger displacement
            stabcontrol.finger_r[0] = 0.04;
            stabcontrol.finger_r[1] = 0.04;

            // control
            stabcontrol.PIDcontrol();
            stabcontrol.handControl();
            stabcontrol.forwardKinematics();
            std_msgs::Float64MultiArray ee_state_msg;
            ee_state_msg.data.resize(3);
            ee_state_msg.data[0] = stabcontrol.ee_state[0];
            ee_state_msg.data[1] = stabcontrol.ee_state[1];
            ee_state_msg.data[2] = stabcontrol.ee_state[2];
            pose_publisher.publish(ee_state_msg);
        }

        ros::spinOnce();
        loop_rate.sleep(); 
        
    }
    return 0;
}
