#include <stabilizing_control/stabilizing_control.h>


int main(int argc, char **argv)
{
    float pi = 3.1415;
    ros::init(argc,argv,"panda_effortcontrol_jointspace"); // initialize ROS, specify ROS node, and initialize this ROS node
    StabilizingControl stabcontrol("panda_effortcontrol_jointspace");
    ros::Rate loop_rate(stabcontrol.samplingFreq); 
    ros::Time startTime = ros::Time::now();
                                                                                                                                                                                                                                         
    while(ros::ok())
    {
        if ((ros::Time::now()-startTime).toSec() <= 5)
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

        else if ((ros::Time::now()-startTime).toSec() > 5)
        {
            // reference joint angles
            /*stabcontrol.q_r[0] = 1.0; //3.0
            stabcontrol.q_r[1] = 0.5; //1.0;
            stabcontrol.q_r[2] = 0.0; //1.0; 
            stabcontrol.q_r[3] = -0.5; //-1.0; 
            stabcontrol.q_r[4] = 0.0; //1.0; 
            stabcontrol.q_r[5] = 0.5; //1.0; 
            stabcontrol.q_r[6] = 0.0; //1.0;*/ 

            /*stabcontrol.q_r[0] = 57.294 * (pi/180); //3.0
            stabcontrol.q_r[1] = 90 * (pi/180); //1.0;
            stabcontrol.q_r[2] = 90 * (pi/180); //1.0; 
            stabcontrol.q_r[3] = 17.7813 * (pi/180); //-1.0; 
            stabcontrol.q_r[4] = 94.5292 * (pi/180); //1.0; 
            stabcontrol.q_r[5] = 62.1296 * (pi/180); //1.0; 
            stabcontrol.q_r[6] = 90 * (pi/180); //1.0;*/

            stabcontrol.q_r[0] = 57.294 * (pi/180);
            stabcontrol.q_r[1] = 45 * (pi/180); 
            stabcontrol.q_r[2] = -90 * (pi/180); 
            stabcontrol.q_r[3] = -90 * (pi/180);  
            stabcontrol.q_r[4] = 0 * (pi/180);  
            stabcontrol.q_r[5] = 0 * (pi/180);  
            stabcontrol.q_r[6] = 0 * (pi/180); 


            // reference finger displacement
            stabcontrol.finger_r[0] = 0.04;
            stabcontrol.finger_r[1] = 0.04;

            // control
            stabcontrol.PIDcontrol();
            stabcontrol.handControl();
            stabcontrol.forwardKinematics();
        }

        ros::spinOnce();
        loop_rate.sleep(); 
        
    }
    return 0;
}
