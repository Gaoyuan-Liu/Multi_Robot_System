#include <explicit_reference_governor/explicit_reference_governor.h>


std::chrono::time_point<std::chrono::system_clock> start, end, end_aftersleep; 

int main(int argc, char **argv)
{
    ros::init(argc,argv,"panda_trajbasedERG_potentialfields"); // initialize ROS, specify ROS node, and initialize this ROS node
    ExplicitReferenceGovernor erg("panda_trajbasedERG_potentialfields");
    ros::Rate loop_rate(erg.samplingFreq); 
    // ros::Time startTime = ros::Time::now();
    erg.startTime = ros::Time::now();
    erg.reference_starttime = 5.0; 

    arma::vec q_start; 
    // q_start = {0.0000, -0.7854, 0.0000, -0.5, 0.0000, 1.5708, 0.7854}; 
    // q_start = {-1.86, 1.46, 1.65, -1.75, -1.44, 1.51, 0.69}; // case A
    // q_start = {-0.48, 0.38, 2.38, -1.48, -0.22, 1.52, 0.02}; // case B
    // q_start = {0.50, -0.67, 1.36, -1.56, 0.66, 1.44, -0.45}; // case C            
    q_start = {0.0000, -0.7854, 0.0000, -2.3562, 0.0000, 1.5708, 0.7854}; // case D
    // q_start = {0.0000, 0.2146, 0.0000, -2.3562, 0.0000, 1.5708, 0.7854}; // case D
    // q_start = {1.0000, -0.7854, 0.0000, -2.3562, 0.0000, 1.5708, 0.7854}; // case D


    /*** ERG ***/
    std_msgs::Float64MultiArray user_ref_msg;
    user_ref_msg.data.resize(7);
    erg.q_r = q_start;
    while(ros::ok())
    {
        start = std::chrono::system_clock::now(); 
        if ((ros::Time::now()-erg.startTime).toSec() <= erg.reference_starttime) // reference_starttime=5.0s
        {
            //erg.q_r = q_start;
            //erg.q_r = erg.q_r_receive;
            erg.q_v = erg.q; // initialization

            
            for (int i = 0; i < 7; i++)
            {
                user_ref_msg.data[i]=erg.q_r(i);
            } 
            erg.pub_user_ref.publish(user_ref_msg);

            // KDL::Frame startframe = erg.forwardKinematics(erg.q_r);
            // std::cout << "start config: \n" << erg.q << std::endl;
            // std::cout << "reference config: \n" << erg.q_r << std::endl;
            // std::cout << "rotation matrix reference frame: \n" << startframe.M  << std::endl;
            // std::cout << "end-effector position reference frame: \n" << startframe.p << std::endl;
            // std::cout << std::endl;

            erg.kdl_q = erg.armaToKdlVector(erg.q);
            // erg.kdl_dotq = erg.armaToKdlVector(erg.dotq);
            erg.dyn_param->JntToGravity(erg.kdl_q, erg.kdl_gravity_effort); 
            // erg.dyn_param->JntToCoriolis(erg.kdl_q, erg.kdl_dotq, erg.kdl_coriolis_effort);
            erg.gravity_effort = erg.kdlToArmaVector(erg.kdl_gravity_effort);
            // erg.coriolis_effort = erg.kdlToArmaVector(erg.kdl_coriolis_effort);

            // erg.tau_cmd = erg.PDgcorControl(erg.q_r,erg.q,erg.dotq,erg.Kp,erg.Kd,erg.gravity_effort,erg.coriolis_effort);
            erg.tau_cmd = erg.PIDcontrol(erg.q_r,erg.q,erg.dotq,erg.i_error,erg.i_clamp,erg.Kp,erg.Ki,erg.Kd,erg.samplingFreq,erg.gravity_effort);
            erg.sendTorqueCommands(erg.tau_cmd);

            erg.sendHandPositionCommands(erg.finger_r);
        }

        else if ((ros::Time::now()-erg.startTime).toSec() > erg.reference_starttime)
        {
            if(erg.printpred == 0 && (ros::Time::now()-erg.printPredTime).toSec()>0.5)
            {
                erg.printpred = 1;
            }

            double amplitude = 1.0; 
            double period = 100.0; 
            double omega = 2.0 * 3.1415 / period; 
            //for (int i=0; i<7; i++)
            //{
            //    erg.q_r(i) = q_start(i) +  amplitude*sin(omega*((ros::Time::now().toSec() - erg.reference_starttime)));
            //}

            erg.q_r = erg.q_r_receive;
        
          
            erg.trajBasedDSM(erg.q_r, erg.q, erg.dotq,
                             erg.i_error, erg.i_clamp,
                             erg.Kp, erg.Ki, erg.Kd,
                             erg.samplingFreq_pred, erg.pred_horizon, erg.startTime); // output = DSM
            // end = std::chrono::system_clock::now(); 
            // std::chrono::duration<double> elapsed_seconds = end - start;   
            // std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n"; 
            
            // erg.q_v = erg.qvUpdate(erg.q_v,
            //                        erg.rho, erg.DSM,
            //                        erg.samplingFreq);

            for (int i=0; i<7; i++){
                user_ref_msg.data[i]=erg.q_r(i);
            }
            erg.pub_user_ref.publish(user_ref_msg);

            erg.kdl_q = erg.armaToKdlVector(erg.q);
            // erg.kdl_dotq = erg.armaToKdlVector(erg.dotq);
            erg.dyn_param->JntToGravity(erg.kdl_q, erg.kdl_gravity_effort); 
            // erg.dyn_param->JntToCoriolis(erg.kdl_q, erg.kdl_dotq, erg.kdl_coriolis_effort);
            erg.gravity_effort = erg.kdlToArmaVector(erg.kdl_gravity_effort);
            // erg.coriolis_effort = erg.kdlToArmaVector(erg.kdl_coriolis_effort);
            // erg.tau_cmd = erg.PDgcorControl(erg.q_r,erg.q,erg.dotq,erg.Kp,erg.Kd,erg.gravity_effort,erg.coriolis_effort);
            // for (int i=0; i<7;i++)
            // {
            //     erg.tau_cmd(i) = erg.Kp(i)*(erg.q_r(i)-erg.q(i)) - erg.Kd(i)*erg.dotq(i) + erg.gravity_effort(i) + erg.coriolis_effort(i);
            // }
            
            erg.tau_cmd = erg.PIDcontrol(erg.q_r,erg.q,erg.dotq,erg.i_error,erg.i_clamp,erg.Kp,erg.Ki,erg.Kd,erg.samplingFreq,erg.gravity_effort);
            // erg.tau_cmd = {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000}; //
            // erg.tau_cmd = erg.PIDcontrol(erg.q_r,erg.q,erg.dotq,erg.i_error,erg.i_clamp,erg.Kp,erg.Ki,erg.Kd,erg.samplingFreq,erg.gravity_effort);
            erg.sendTorqueCommands(erg.tau_cmd);

            erg.sendHandPositionCommands(erg.finger_r);
        }
        end = std::chrono::system_clock::now(); 
        std::chrono::duration<double> elapsed_seconds = end - start;   
        // std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n"; 

        ros::spinOnce();
        loop_rate.sleep(); 

        end_aftersleep = std::chrono::system_clock::now(); 
        std::chrono::duration<double> elapsed_seconds_aftersleep = end_aftersleep - start;   
        // std::cout << "elapsed time: " << elapsed_seconds_aftersleep.count() << "s\n"; 
        // std::cout << std::endl;
        std_msgs::Float64 chrono_1loop_msg;
        chrono_1loop_msg.data = elapsed_seconds_aftersleep.count();
        erg.pub_chrono.publish(chrono_1loop_msg);
        
    }
    return 0;
}
