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

    /*** step reference ***/
    // while(ros::ok())
    // {
    //     start = std::chrono::system_clock::now(); 
    //     if ((ros::Time::now()-erg.startTime).toSec() <= erg.reference_starttime)
    //     {
    //         erg.q_r = q_start;
    //         erg.q_v = erg.q; // initialization

    //         erg.kdl_q = erg.armaToKdlVector(erg.q);
    //         // erg.kdl_dotq = erg.armaToKdlVector(erg.dotq);
    //         erg.dyn_param->JntToGravity(erg.kdl_q, erg.kdl_gravity_effort); 
    //         // erg.dyn_param->JntToCoriolis(erg.kdl_q, erg.kdl_dotq, erg.kdl_coriolis_effort);
    //         erg.gravity_effort = erg.kdlToArmaVector(erg.kdl_gravity_effort);
    //         // erg.coriolis_effort = erg.kdlToArmaVector(erg.kdl_coriolis_effort);

    //         erg.tau_cmd = erg.PIDcontrol(erg.q_r,erg.q,erg.dotq,erg.i_error,erg.i_clamp,erg.Kp,erg.Ki,erg.Kd,erg.samplingFreq,erg.gravity_effort);
    //         erg.sendTorqueCommands(erg.tau_cmd);

    //         erg.sendHandPositionCommands(erg.finger_r);
    //     }

    //     else if ((ros::Time::now()-erg.startTime).toSec() > erg.reference_starttime)
    //     {
    //         if ((ros::Time::now()-erg.startTime).toSec() <= erg.reference_starttime+5.0)
    //         {
    //             for (int i=0; i<7; i++)
    //             {
    //                 erg.q_r(i) = q_start(i) + 0.2; 
    //                 // if (i==0)
    //                 // {
    //                 //    erg.q_r(i) = q_start(i) + 0.2;  
    //                 // }
    //                 // else
    //                 // {
    //                 //     erg.q_r(i) = q_start(i);
    //                 // }
                    
    //                 // if ((q_start(i) + 1.0) > (erg.q_upperlimit(i)-0.1))
    //                 // {
    //                 //     erg.q_r(i) = q_start(i) - 1.0;
    //                 // }
    //                 // else {
    //                 //     erg.q_r(i) = q_start(i) + 1.0; 
    //                 // }
    //             }
    //         }
    //         else {
    //             for (int i=0; i<7; i++){
    //                 erg.q_r(i) = q_start(i); 
    //             }
    //         }
        
    //         erg.kdl_q = erg.armaToKdlVector(erg.q);
    //         // erg.kdl_dotq = erg.armaToKdlVector(erg.dotq);
    //         erg.dyn_param->JntToGravity(erg.kdl_q, erg.kdl_gravity_effort); 
    //         // erg.dyn_param->JntToCoriolis(erg.kdl_q, erg.kdl_dotq, erg.kdl_coriolis_effort);
    //         erg.gravity_effort = erg.kdlToArmaVector(erg.kdl_gravity_effort);
    //         // erg.coriolis_effort = erg.kdlToArmaVector(erg.kdl_coriolis_effort);
            
    //         // erg.tau_cmd = erg.PDgcorControl(erg.q_r,erg.q,erg.dotq,erg.Kp,erg.Kd,erg.gravity_effort,erg.coriolis_effort);
    //         // erg.tau_cmd = {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000}; //
    //         erg.tau_cmd = erg.PIDcontrol(erg.q_r,erg.q,erg.dotq,erg.i_error,erg.i_clamp,erg.Kp,erg.Ki,erg.Kd,erg.samplingFreq,erg.gravity_effort);
    //         erg.sendTorqueCommands(erg.tau_cmd);

    //         erg.sendHandPositionCommands(erg.finger_r);
    //     }
    //     end = std::chrono::system_clock::now(); 
    //     std::chrono::duration<double> elapsed_seconds = end - start;   
    //     // std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n"; 

    //     ros::spinOnce();
    //     loop_rate.sleep(); 

    //     end_aftersleep = std::chrono::system_clock::now(); 
    //     std::chrono::duration<double> elapsed_seconds_aftersleep = end_aftersleep - start;   
    //     // std::cout << "elapsed time: " << elapsed_seconds_aftersleep.count() << "s\n"; 
    //     // std::cout << std::endl;
    //     std_msgs::Float64 chrono_1loop_msg;
    //     chrono_1loop_msg.data = elapsed_seconds_aftersleep.count();
    //     erg.pub_chrono.publish(chrono_1loop_msg);
    // }



    /*** sine reference ***/
    // while(ros::ok())
    // {
    //     start = std::chrono::system_clock::now(); 
    //     if ((ros::Time::now()-erg.startTime).toSec() <= erg.reference_starttime)
    //     {
    //         erg.q_r = q_start;
    //         erg.q_v = erg.q; // initialization

    //         erg.kdl_q = erg.armaToKdlVector(erg.q);
    //         // erg.kdl_dotq = erg.armaToKdlVector(erg.dotq);
    //         erg.dyn_param->JntToGravity(erg.kdl_q, erg.kdl_gravity_effort); 
    //         // erg.dyn_param->JntToCoriolis(erg.kdl_q, erg.kdl_dotq, erg.kdl_coriolis_effort);
    //         erg.gravity_effort = erg.kdlToArmaVector(erg.kdl_gravity_effort);
    //         // erg.coriolis_effort = erg.kdlToArmaVector(erg.kdl_coriolis_effort);

    //         erg.tau_cmd = erg.PIDcontrol(erg.q_r,erg.q,erg.dotq,erg.i_error,erg.i_clamp,erg.Kp,erg.Ki,erg.Kd,erg.samplingFreq,erg.gravity_effort);
    //         erg.sendTorqueCommands(erg.tau_cmd);

    //         erg.sendHandPositionCommands(erg.finger_r);
    //     }

    //     else if ((ros::Time::now()-erg.startTime).toSec() > erg.reference_starttime)
    //     {
    //         double amplitude = 0.5; 
    //         double period = 20.0; 
    //         double omega = 2.0 * 3.1415 / period; 
    //         for (int i=0; i<7; i++)
    //         {
    //             erg.q_r(i) = q_start(i) +  amplitude*sin(omega*((ros::Time::now().toSec() - erg.reference_starttime)));
    //         }

    //         erg.kdl_q = erg.armaToKdlVector(erg.q);
    //         // erg.kdl_dotq = erg.armaToKdlVector(erg.dotq);
    //         erg.dyn_param->JntToGravity(erg.kdl_q, erg.kdl_gravity_effort); 
    //         // erg.dyn_param->JntToCoriolis(erg.kdl_q, erg.kdl_dotq, erg.kdl_coriolis_effort);
    //         erg.gravity_effort = erg.kdlToArmaVector(erg.kdl_gravity_effort);
    //         // erg.coriolis_effort = erg.kdlToArmaVector(erg.kdl_coriolis_effort);
            
    //         // erg.tau_cmd = erg.PDgcorControl(erg.q_r,erg.q,erg.dotq,erg.Kp,erg.Kd,erg.gravity_effort,erg.coriolis_effort);
    //         // erg.tau_cmd = {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000}; //
    //         erg.tau_cmd = erg.PIDcontrol(erg.q_r,erg.q,erg.dotq,erg.i_error,erg.i_clamp,erg.Kp,erg.Ki,erg.Kd,erg.samplingFreq,erg.gravity_effort);
    //         erg.sendTorqueCommands(erg.tau_cmd);

    //         erg.sendHandPositionCommands(erg.finger_r);
    //     }
    //     end = std::chrono::system_clock::now(); 
    //     std::chrono::duration<double> elapsed_seconds = end - start;   
    //     // std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n"; 

    //     ros::spinOnce();
    //     loop_rate.sleep(); 

    //     end_aftersleep = std::chrono::system_clock::now(); 
    //     std::chrono::duration<double> elapsed_seconds_aftersleep = end_aftersleep - start;   
    //     // std::cout << "elapsed time: " << elapsed_seconds_aftersleep.count() << "s\n"; 
    //     // std::cout << std::endl;
    //     std_msgs::Float64 chrono_1loop_msg;
    //     chrono_1loop_msg.data = elapsed_seconds_aftersleep.count();
    //     erg.pub_chrono.publish(chrono_1loop_msg);
    // }


    /*** ERG ***/
    std_msgs::Float64MultiArray user_ref_msg;
    user_ref_msg.data.resize(7);
    while(ros::ok())
    {
        start = std::chrono::system_clock::now(); 
        if ((ros::Time::now()-erg.startTime).toSec() <= erg.reference_starttime)
        {
            erg.q_r = q_start;
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
            for (int i=0; i<7; i++)
            {
                erg.q_r(i) = q_start(i) +  amplitude*sin(omega*((ros::Time::now().toSec() - erg.reference_starttime)));
            }

            
            // if ((ros::Time::now()-erg.startTime).toSec() <= erg.reference_starttime+5.0)
            // {
            //     // std::cout << (ros::Time::now()-erg.startTime).toSec() << std::endl;
            //     for (int i=0; i<7; i++)
            //     {
            //         erg.q_r(i) = q_start(i) + 0.2; 
            //         // if (i==1)
            //         // {
            //         //    erg.q_r(i) = q_start(i) + 0.2;  
            //         // }
            //         // else
            //         // {
            //         //     erg.q_r(i) = q_start(i);
            //         // }
            //         // std::cout << erg.q_r << std::endl;
                    
            //         // if ((q_start(i) + 1.0) > (erg.q_upperlimit(i)-0.1))
            //         // {
            //         //     erg.q_r(i) = q_start(i) - 1.0;
            //         // }
            //         // else {
            //         //     erg.q_r(i) = q_start(i) + 1.0; 
            //         // }
            //     }
            //     // KDL::Frame referenceframe = erg.forwardKinematics(erg.q_r);
            //     // std::cout << "start config: \n" << erg.q << std::endl;
            //     // std::cout << "reference config: \n" << erg.q_r << std::endl;
            //     // std::cout << "rotation matrix reference frame: \n" << referenceframe.M  << std::endl;
            //     // std::cout << "end-effector position reference frame: \n" << referenceframe.p << std::endl;
            //     // std::cout << std::endl;
            // }
            // else if((ros::Time::now()-erg.startTime).toSec() > erg.reference_starttime+5.0 && (ros::Time::now()-erg.startTime).toSec() <= erg.reference_starttime+10.0)
            // {
            //      // std::cout << (ros::Time::now()-erg.startTime).toSec() << std::endl;
            //     for (int i=0; i<7; i++)
            //     {
            //         erg.q_r(i) = q_start(i) + 0.4; 
            //         // if (i==1)
            //         // {
            //         //    erg.q_r(i) = q_start(i) + 0.4;  
            //         // }
            //         // else
            //         // {
            //         //     erg.q_r(i) = q_start(i);
            //         // }
            //     }
            // }
            // else if((ros::Time::now()-erg.startTime).toSec() > erg.reference_starttime+10.0 && (ros::Time::now()-erg.startTime).toSec() <= erg.reference_starttime+15.0)
            // {
            //      // std::cout << (ros::Time::now()-erg.startTime).toSec() << std::endl;
            //     for (int i=0; i<7; i++)
            //     {
            //         erg.q_r(i) = q_start(i) + 0.6; 
            //         // if (i==1)
            //         // {
            //         //    erg.q_r(i) = q_start(i) + 0.6;  
            //         // }
            //         // else
            //         // {
            //         //     erg.q_r(i) = q_start(i);
            //         // }  
            //     }
            // }
            // else if((ros::Time::now()-erg.startTime).toSec() > erg.reference_starttime+15.0 && (ros::Time::now()-erg.startTime).toSec() <= erg.reference_starttime+20.0)
            // {
            //      // std::cout << (ros::Time::now()-erg.startTime).toSec() << std::endl;
            //     for (int i=0; i<7; i++)
            //     {
            //         erg.q_r(i) = q_start(i) + 0.8; 
            //         // if (i==1)
            //         // {
            //         //    erg.q_r(i) = q_start(i) + 0.8;  
            //         // }
            //         // else
            //         // {
            //         //     erg.q_r(i) = q_start(i);
            //         // }
            //     }
            // }
            // else {
            //     // std::cout << (ros::Time::now()-erg.startTime).toSec() << std::endl;
            //     for (int i=0; i<7; i++)
            //     {
            //         erg.q_r(i) = q_start(i) + 1.0; 
            //         // if (i==1)
            //         // {
            //         //    erg.q_r(i) = q_start(i) + 1.0;  
            //         // }
            //         // else
            //         // {
            //         //     erg.q_r(i) = q_start(i);
            //         // }
            //     }
            // }
        //     erg.q_r = erg.inverseKinematics(erg.q_init, erg.pos_r, erg.orient_r); 

            // // TEST 1 
            // erg.q_r(0) = -0.5; 
            // erg.q_r(1) = 1.0;
            // erg.q_r(2) = 0.5;
            // erg.q_r(3) = -0.5; 
            // erg.q_r(4) = 0.5;
            // erg.q_r(5) = 0.5;
            // erg.q_r(6) = 0.5;

            // // TEST 2
            // erg.q_r(0) = 3.5; //1.0; //3.0
            // erg.q_r(1) = 2.0;//0.5; //1.0;
            // erg.q_r(2) = 3.0;//0.0; //1.0; 
            // erg.q_r(3) = -3.5;//-0.5; //-1.0; 
            // erg.q_r(4) = -3.0;//0.0; //1.0; 
            // erg.q_r(5) = -0.5;//0.5; //1.0; 
            // erg.q_r(6) = 3.0;//0.0; //1.0; 

            // erg.kdl_x = erg.forwardKinematics(erg.q_r);

            // erg.rho_att = erg.attractionField(erg.q_r,erg.q_v,erg.eta);
            // erg.rho_rep_q = erg.qRepulsionField(erg.q_v, 
            //                                     erg.q_lowerlimit, erg.q_upperlimit, 
            //                                     erg.zeta_q, erg.delta_q);
            
            // erg.panda_jointpositions = erg.frameToJointPositionsMatrix(erg.q_v, erg.panda_frame_ids); 

            // erg.lambda = erg.computeLambda(erg.panda_jointpositions, erg.obst_spheres);
            // erg.Pij = erg.computePij(erg.panda_jointpositions, erg.lambda);
            // erg.obstSphereRep = erg.computeSphericalRepulsion(erg.Pij, erg.obst_spheres, erg.zeta_obst_sphere, erg.delta_obst_sphere);
            // erg.rho_rep_obst_sphere = erg.obstSpereRepulsionField(erg.q_v, erg.obstSphereRep, erg.lambda, erg.panda_frame_ids);

            // erg.munu = erg.computeMuNu(erg.panda_jointpositions, erg.obst_cylinders);
            // erg.SijTij = erg.computeSijTij(erg.panda_jointpositions, erg.obst_cylinders, erg.munu);
            // erg.obstCylinderRep = erg.computeCylindricalRepulsionField(erg.SijTij, erg.obst_cylinders, erg.zeta_obst_cylinder, erg.delta_obst_cylinder);
            // erg.rho_rep_obst_cylinder = erg.obstCylinderRepulsionField(erg.q_v, erg.obstCylinderRep, erg.munu, erg.panda_frame_ids);
                                                
            // erg.rho = erg.navigationField(erg.rho_att, erg.rho_rep_q, erg.rho_rep_obst_sphere, erg.rho_rep_obst_cylinder);

            // // start = std::chrono::system_clock::now(); 
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