#include <testlib/testlib.h>

int main(int argc, char **argv)
{
    // initialize ROS, specify ROS node, and initialize this ROS node
    ros::init(argc, argv, "panda_effortcontrol_v2"); 
    TestLibrary testlibobj("panda_effortcontrol_v2");
    ros::Rate loop_rate(testlibobj.samplingFreq); 
    ros::Time startTime = ros::Time::now();

    while(ros::ok())
    {
        if ((ros::Time::now()-startTime).toSec() <= 10)
        {
            // all reference joint angles = current joint angles
            for (int i=0; i<testlibobj.q_r.size();i++)
            {
                testlibobj.q_r[i] = testlibobj.q[i];
                testlibobj.q_v[i] = testlibobj.q[i];
            }
            // stay where you are
            testlibobj.PIDcontrol();
            testlibobj.forwardKinematics();
        }
        else if ((ros::Time::now()-startTime).toSec() > 10)
        {
            testlibobj.q_r[0] = 3.0; //1.5; //1.0; //3.0
            testlibobj.q_r[1] = 0.5; //1.0;
            testlibobj.q_r[2] = 0.0; //1.0; 
            testlibobj.q_r[3] = -0.5; //-1.0; 
            testlibobj.q_r[4] = 0.0; //1.0; 
            testlibobj.q_r[5] = 0.5; //1.0; 
            testlibobj.q_r[6] = 0.0; //1.0; 

            testlibobj.attractionField();
            testlibobj.qRepulsionField();
            testlibobj.obstSphereRepulsionField();
            testlibobj.navigationField();
            testlibobj.simDSM();
            testlibobj.qvUpdate();
            testlibobj.PIDcontrol();
        }

        ros::spinOnce();
        loop_rate.sleep();    
    }
    return 0;
}