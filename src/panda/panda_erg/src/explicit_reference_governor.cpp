#include <explicit_reference_governor/explicit_reference_governor.h>
#include <ros/package.h>

#include <string>
#include <cstdlib>
#include <algorithm>

using namespace std;
using namespace KDL;
using namespace Eigen; 

ExplicitReferenceGovernor::ExplicitReferenceGovernor(string name) : n(name)
{
    is_q_printed = false;
    printpred = true;

    samplingFreq = 1000.0; 

    getFingerLimitsRefs(); //finger_lowerlimit finger_upperlimit finger_r
    finger = arma::zeros<arma::vec>(2); 
    
    getQLimits(); //q_lowerlimit, q_upperlimit

    getTauLimits();  //tau_lowerlimit, tau_upperlimit

    q_init = {0.0, 0.0, 0.0, -0.5, 0.0, 0.5, 0.0};
    q = arma::zeros<arma::vec>(7);  
    getQRefs(); // q_r   
    q_v = arma::zeros<arma::vec>(7); 
    dotq = arma::zeros<arma::vec>(7);
    q_r_receive = arma::zeros<arma::vec>(7);;
    getPoseRefs(); // pos x, pos y, pos z, quat x, quat y, quat z, quat w

    getControlParameters(); //Kp, Ki, Kd, i_clamp
    i_error = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    tau_cmd = arma::zeros<arma::vec>(7);
    gravity_effort = arma::zeros<arma::vec>(7);

    panda_frame_ids = {1,2,3,4,5,6,7,8}; // frame 0 = base
    panda_frames = arma::zeros<arma::cube>(4,4,panda_frame_ids.size()); // 4 x 4 x #joints+end-effector
    panda_jointpositions = arma::zeros<arma::mat>(3,panda_frame_ids.size()); // 3 x #joints+end-effector
    panda_jacobian = arma::zeros<arma::mat>(3,size(panda_jointpositions,1)-1); // 3 x #links

    getObstacleSphereParameters(); //obst_spheres
    lambda = arma::zeros<arma::mat>(size(panda_jointpositions,1)-1,size(obst_spheres,1)); // #links x #obstacles
    Pij = arma::zeros<arma::mat>(3*size(lambda,0),size(lambda,1)); // 3*#links x #obstacles
    obstSphereRep = arma::zeros<arma::mat>(size(Pij,0),size(Pij,1)); // 3*#links x #obstacles

    getObstacleCylinderParameters(); // obst_cylinders
    
    getErgNavParameters(); // eta, zeta_q, delta_q, zeta_obst_sphere, delta_obst_sphere, zeta_obst_cyl, delta_obst_cyl
    rho_att = arma::zeros<arma::vec>(7);
    rho_rep_q = arma::zeros<arma::vec>(7);
    rho_rep_obst_sphere = arma::zeros<arma::vec>(7);
    rho = arma::zeros<arma::vec>(7);

    getErgDsmParameters(); // pred_horizon, samplingFreq_pred, kappa_tau_cmd, kappa_q, kappa_obst_sphere 
    DSM = 0.0;
    q_pred = arma::zeros<arma::vec>(7);
    dotq_pred = arma::zeros<arma::vec>(7);
    ddotq_pred = arma::zeros<arma::vec>(7);
    i_error_pred = arma::zeros<arma::vec>(7);
    mass_matrix_pred = arma::zeros<arma::mat>(7,7);
    coriolis_effort_pred = arma::zeros<arma::vec>(7);
    gravity_effort_pred = arma::zeros<arma::vec>(7);
    tau_pred = arma::zeros<arma::vec>(7);

    // ROS subscribers and publishers
    // Liu add
    sub_reference = n.subscribe("/panda/joint_references", 1,&ExplicitReferenceGovernor::jointReferencesCallback, this);
    sub_state = n.subscribe("/panda/joint_states",1,&ExplicitReferenceGovernor::jointStatesCallback, this);
    for(int i=0; i<q.size(); i++)
    {
        pub_joint_cmd[i] = n.advertise<std_msgs::Float64>("/panda/joint" + std::to_string(i+1) + "_position_controller/command",100);
        pub_joint_effort_cmd[i] = n.advertise<std_msgs::Float64>("/panda/joint" + std::to_string(i+1) + "_effort_controller/command",100);
    }
    for(int i=0; i<finger.size(); i++)
    {
        if (i==0)
        {
            pub_finger_cmd[i] = n.advertise<std_msgs::Float64>("/panda/joint_leftfinger_position_controller/command",100);
        }
        else if(i==1)
        {
            pub_finger_cmd[i] = n.advertise<std_msgs::Float64>("/panda/joint_rightfinger_position_controller/command",100);
        }   
    }
    for(int i=0; i<q.size(); i++)
    {
        pub_user_ref = n.advertise<std_msgs::Float64MultiArray>("/ERG/NF/user_reference",100);
        pub_applied_ref = n.advertise<std_msgs::Float64MultiArray>("/ERG/NF/applied_reference",100);
    }
    for(int i=0; i<8; i++)
    {
        pub_prediction = n.advertise<std_msgs::Float64MultiArray>("/ERG/DSM/prediction",100);
    }
    for(int i=0; i<7; i++)
    {
        pub_gravity_effort = n.advertise<std_msgs::Float64MultiArray>("/StabControl/gravity_effort",100);
        pub_coriolis_effort = n.advertise<std_msgs::Float64MultiArray>("/StabControl/coriolis_effort",100);
    }
    pub_DSM = n.advertise<std_msgs::Float64>("/ERG/DSM/overall",100);
    pub_DSM_tau = n.advertise<std_msgs::Float64>("/ERG/DSM/tau",100);
    pub_DSM_q = n.advertise<std_msgs::Float64>("/ERG/DSM/q",100);
    pub_DSM_obst_sphere = n.advertise<std_msgs::Float64>("/ERG/DSM/obst_sphere",100);
    pub_DSM_obst_cylinder = n.advertise<std_msgs::Float64>("/ERG/DSM/obst_cylinder",100);
    pub_chrono = n.advertise<std_msgs::Float64>("/chrono_1loop",100);


    // create KDL chain
    // if panda_arm_hand.urdf.xacro is adapted, then don't forget to run in a terminal with sourced workspace
    // rosrun xacro xacro --inorder -o model.urdf model.urdf.xacro
    // https://p4sser8y-words.readthedocs.io/ROS/orocos_kdl.html#usage
    // kdl_parser::treeFromFile("/home/kmerckae/src/constrained_control_robotarm/ros_ws/src/panda_description/urdf/panda_arm_hand.urdf", kdl_tree);
    string path = ros::package::getPath("panda_description");
    if (!kdl_parser::treeFromFile(path + "/urdf/panda_arm_hand.urdf", kdl_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
    }
    if (!kdl_tree.getChain("panda_link0", "panda_hand", kdl_chain))
    {
        ROS_ERROR("Failed to get kdl chain");
    }
    ROS_INFO("Successfully get KDL Chain with %d joints", kdl_chain.getNrOfJoints());
    std::cout  << std::endl;

    joint_num = kdl_chain.getNrOfJoints();
    kdl_q_lowerlimit.resize(joint_num);
    kdl_q_upperlimit.resize(joint_num);
    for (int i = 0; i < joint_num; i++)
    {
        kdl_q_lowerlimit(i) = q_lowerlimit[i]; 
        kdl_q_upperlimit(i) = q_upperlimit[i];         
    }
    kdl_q_init.resize(joint_num);
    kdl_q.resize(joint_num);
    kdl_q_r.resize(joint_num);
    kdl_dotq.resize(joint_num);
    
    jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(kdl_chain));
    kdl_jac.resize(joint_num);

    kdl_gravity_effort.resize(joint_num);
    gravity = Vector {0.0, 0.0, -9.81};
    dyn_param = make_shared<ChainDynParam>(kdl_chain, gravity);
    kdl_mass_matrix.resize(joint_num);
    kdl_coriolis_effort.resize(joint_num);

    kdl_q_pred.resize(joint_num);
    kdl_dotq_pred.resize(joint_num);
    kdl_gravity_effort_pred.resize(joint_num);
    kdl_coriolis_effort_pred.resize(joint_num);
    kdl_mass_matrix_pred.resize(joint_num);    
}

ExplicitReferenceGovernor::~ExplicitReferenceGovernor()
{
}

void ExplicitReferenceGovernor::getControlParameters()
{
    double Kp_txt;
    double Ki_txt;
    double Kd_txt;
    double iclamp_txt;
    int counter = 0 ;

    string path = ros::package::getPath("panda_erg");
    ifstream controlParameterFile(path + "/launch/control_parameters.txt");
    if(controlParameterFile.is_open())
    {
        string dummyLine;
        getline(controlParameterFile,dummyLine);
        while (controlParameterFile  >> Kp_txt >> Ki_txt >> Kd_txt >> iclamp_txt)
        {
            counter++;
        }
        controlParameterFile.close();
    }
    else
    {
        std::cout << "unable to open file" << std::endl;
    }
    Kp = arma::zeros<arma::vec>(counter); 
    Ki = arma::zeros<arma::vec>(counter); 
    Kd = arma::zeros<arma::vec>(counter); 
    i_clamp = arma::zeros<arma::vec>(counter); 
    counter = 0;
    controlParameterFile.open(path + "/launch/control_parameters.txt");
    if(controlParameterFile.is_open())
    {
        string dummyLine;
        getline(controlParameterFile,dummyLine);
        while (controlParameterFile  >> Kp_txt >> Ki_txt >> Kd_txt >> iclamp_txt)
        {
            Kp[counter] = Kp_txt;
            Ki[counter] = Ki_txt;
            Kd[counter] = Kd_txt;
            i_clamp[counter] = iclamp_txt;
            counter++;
        }
        controlParameterFile.close();
    }
    else
    {
        std::cout << "unable to open file" << std::endl;
    }        
}

void ExplicitReferenceGovernor::getQLimits()
{
    double q_lowerlimit_txt; 
    double q_upperlimit_txt;
    int counter = 0 ;

    string path = ros::package::getPath("panda_erg");
    ifstream qLimitsFile(path + "/launch/q_limits.txt");
    if(qLimitsFile.is_open())
    {
        string dummyLine;
        getline(qLimitsFile,dummyLine);
        while (qLimitsFile  >> q_lowerlimit_txt >> q_upperlimit_txt)
        {
            counter++;
        }
        qLimitsFile.close();
    }
    else
    {
        std::cout << "unable to open file" << std::endl;
    }
    q_lowerlimit = arma::zeros<arma::vec>(counter); 
    q_upperlimit = arma::zeros<arma::vec>(counter); 
    counter = 0;
    qLimitsFile.open(path + "/launch/q_limits.txt");
    if(qLimitsFile.is_open())
    {
        string dummyLine;
        getline(qLimitsFile,dummyLine);
        while (qLimitsFile  >> q_lowerlimit_txt >> q_upperlimit_txt)
        {
            q_lowerlimit[counter] = q_lowerlimit_txt;
            q_upperlimit[counter] = q_upperlimit_txt;
            counter++;
        }
        qLimitsFile.close();
    }
    else
    {
        std::cout << "unable to open file" << std::endl;
    }        
}

void ExplicitReferenceGovernor::getTauLimits()
{
    double tau_lowerlimit_txt; 
    double tau_upperlimit_txt;
    int counter = 0 ;

    string path = ros::package::getPath("panda_erg");
    ifstream tauLimitsFile(path + "/launch/tau_limits.txt");
    if(tauLimitsFile.is_open())
    {
        string dummyLine;
        getline(tauLimitsFile,dummyLine);
        while (tauLimitsFile  >> tau_lowerlimit_txt >> tau_upperlimit_txt)
        {
            counter++;
        }
        tauLimitsFile.close();
    }
    else
    {
        std::cout << "unable to open file" << std::endl;
    }
    tau_lowerlimit = arma::zeros<arma::vec>(counter); 
    tau_upperlimit = arma::zeros<arma::vec>(counter); 
    counter = 0;
    tauLimitsFile.open(path + "/launch/tau_limits.txt");
    if(tauLimitsFile.is_open())
    {
        string dummyLine;
        getline(tauLimitsFile,dummyLine);
        while (tauLimitsFile  >> tau_lowerlimit_txt >> tau_upperlimit_txt)
        {
            tau_lowerlimit[counter] = tau_lowerlimit_txt;
            tau_upperlimit[counter] = tau_upperlimit_txt;
            counter++;
        }
        tauLimitsFile.close();
    }
    else
    {
        std::cout << "unable to open file" << std::endl;
    }        
}

void ExplicitReferenceGovernor::getFingerLimitsRefs()
{
    double finger_lowerlimit_txt; 
    double finger_upperlimit_txt;
    double finger_r_txt;
    int counter = 0 ;

    string path = ros::package::getPath("panda_erg");
    ifstream fingerLimitsRefsFile(path + "/launch/finger_limitsrefs.txt");
    if(fingerLimitsRefsFile.is_open())
    {
        string dummyLine;
        getline(fingerLimitsRefsFile,dummyLine);
        while (fingerLimitsRefsFile  >> finger_lowerlimit_txt >> finger_upperlimit_txt >> finger_r_txt)
        {
            counter++;
        }
        fingerLimitsRefsFile.close();
    }
    else
    {
        std::cout << "unable to open file" << std::endl;
    }
    finger_lowerlimit = arma::zeros<arma::vec>(counter); 
    finger_upperlimit = arma::zeros<arma::vec>(counter); 
    finger_r = arma::zeros<arma::vec>(counter);
    counter = 0;
    fingerLimitsRefsFile.open(path + "/launch/finger_limitsrefs.txt");
    if(fingerLimitsRefsFile.is_open())
    {
        string dummyLine;
        getline(fingerLimitsRefsFile,dummyLine);
        while (fingerLimitsRefsFile  >> finger_lowerlimit_txt >> finger_upperlimit_txt >> finger_r_txt)
        {
            finger_lowerlimit[counter] = finger_lowerlimit_txt;
            finger_upperlimit[counter] = finger_upperlimit_txt;
            finger_r[counter] = finger_r_txt;
            counter++;
        }
        fingerLimitsRefsFile.close();
    }
    else
    {
        std::cout << "unable to open file" << std::endl;
    }       
}

void ExplicitReferenceGovernor::getQRefs()
{
    double q_r_txt; 
    int counter = 0 ;

    string path = ros::package::getPath("panda_erg");
    ifstream qRefsFile(path + "/launch/q_refs.txt");
    if(qRefsFile.is_open())
    {
        string dummyLine;
        getline(qRefsFile,dummyLine);
        while (qRefsFile  >> q_r_txt)
        {
            counter++;
        }
        qRefsFile.close();
    }
    else
    {
        std::cout << "unable to open file" << std::endl;
    }
    q_r = arma::zeros<arma::vec>(counter); 
    counter = 0;
    qRefsFile.open(path + "/launch/q_refs.txt");
    if(qRefsFile.is_open())
    {
        string dummyLine;
        getline(qRefsFile,dummyLine);
        while (qRefsFile >> q_r_txt)
        {
            q_r[counter] = q_r_txt;
            counter++;
        }
        qRefsFile.close();
    }
    else
    {
        std::cout << "unable to open file" << std::endl;
    }  
}

void ExplicitReferenceGovernor::getPoseRefs()
{
    double posx_txt;
    double posy_txt;
    double posz_txt;
    double quatx_txt;
    double quaty_txt;
    double quatz_txt;
    double quatw_txt; 

    pos_r = arma::zeros<arma::vec>(3);
    orient_r = arma::zeros<arma::vec>(4);

    string path = ros::package::getPath("panda_erg");
    ifstream poseRefsFile(path + "/launch/pose_refs.txt");
    if(poseRefsFile.is_open())
    {
        string dummyLine;
        getline(poseRefsFile,dummyLine);
        while (poseRefsFile  >> posx_txt >> posy_txt >> posz_txt >> quatx_txt >> quaty_txt >> quatz_txt >> quatw_txt)
        {
            pos_r(0) = posx_txt; 
            pos_r(1) = posy_txt;
            pos_r(2) = posz_txt;
            orient_r(0) = quatx_txt;
            orient_r(1) = quaty_txt;
            orient_r(2) = quatz_txt;
            orient_r(3) = quatw_txt;
        }
        poseRefsFile.close();
    }
    else
    {
        std::cout << "unable to open file" << std::endl;
    }     
}
void ExplicitReferenceGovernor::getObstacleSphereParameters()
{
    double centerx_txt;
    double centery_txt;
    double centerz_txt;
    double radius_txt;
    int counter = 0 ; // = number of obstacles 

    string path = ros::package::getPath("panda_erg");
    ifstream obstSpheresFile(path + "/launch/obst_spheres.txt");
    if(obstSpheresFile.is_open())
    {
        string dummyLine;
        getline(obstSpheresFile,dummyLine);
        while (obstSpheresFile  >> centerx_txt >> centery_txt >> centerz_txt >> radius_txt)
        {
            counter++;
        }
        obstSpheresFile.close();
    }
    else
    {
        std::cout << "unable to open file" << std::endl;
    }
    obst_spheres = arma::zeros<arma::mat>(4,counter); 
    counter = 0;
    obstSpheresFile.open(path + "/launch/obst_spheres.txt");
    if(obstSpheresFile.is_open())
    {
        string dummyLine;
        getline(obstSpheresFile,dummyLine);
        while (obstSpheresFile  >> centerx_txt >> centery_txt >> centerz_txt >> radius_txt)
        {
            obst_spheres(0,counter) = centerx_txt;
            obst_spheres(1,counter) = centery_txt;
            obst_spheres(2,counter) = centerz_txt;
            obst_spheres(3,counter) = radius_txt;
            counter++;
        }
        obstSpheresFile.close();
    }
    else
    {
        std::cout << "unable to open file" << std::endl;
    }       
}
void ExplicitReferenceGovernor::getObstacleCylinderParameters()
{
    double centerx_txt;
    double centery_txt;
    double centerz_txt;
    double radius_txt;
    double length_txt;
    int counter = 0 ; // = number of obstacles 

    string path = ros::package::getPath("panda_erg");
    ifstream obstCylindersFile(path + "/launch/obst_cylinders.txt");
    if(obstCylindersFile.is_open())
    {
        string dummyLine;
        getline(obstCylindersFile,dummyLine);
        while (obstCylindersFile  >> centerx_txt >> centery_txt >> centerz_txt >> radius_txt >> length_txt)
        {
            counter++;
        }
        obstCylindersFile.close();
    }
    else
    {
        std::cout << "unable to open file" << std::endl;
    }
    obst_cylinders = arma::zeros<arma::mat>(5,counter); 
    counter = 0;
    obstCylindersFile.open(path + "/launch/obst_cylinders.txt");
    if(obstCylindersFile.is_open())
    {
        string dummyLine;
        getline(obstCylindersFile,dummyLine);
        while (obstCylindersFile  >> centerx_txt >> centery_txt >> centerz_txt >> radius_txt >> length_txt)
        {
            obst_cylinders(0,counter) = centerx_txt;
            obst_cylinders(1,counter) = centery_txt;
            obst_cylinders(2,counter) = centerz_txt;
            obst_cylinders(3,counter) = radius_txt;
            obst_cylinders(4,counter) = length_txt;
            counter++;
        }
        obstCylindersFile.close();
    }
    else
    {
        std::cout << "unable to open file" << std::endl;
    }       
}
void ExplicitReferenceGovernor::getErgNavParameters()
{
    double eta_txt; 
    double zeta_q_txt;
    double delta_q_txt;
    double zeta_obst_sphere_txt;
    double delta_obst_sphere_txt;
    double zeta_obst_cylinder_txt;
    double delta_obst_cylinder_txt;

    string path = ros::package::getPath("panda_erg");
    ifstream ergNavParametersFile(path + "/launch/erg_nav_parameters.txt");
    if(ergNavParametersFile.is_open())
    {
        string dummyLine;
        getline(ergNavParametersFile,dummyLine);
        while (ergNavParametersFile  >>  eta_txt >> zeta_q_txt >> delta_q_txt >> zeta_obst_sphere_txt >> delta_obst_sphere_txt >> zeta_obst_cylinder_txt >> delta_obst_cylinder_txt)
        {
            eta = eta_txt;
            zeta_q = zeta_q_txt;
            delta_q = delta_q_txt;
            zeta_obst_sphere = zeta_obst_sphere_txt;
            delta_obst_sphere = delta_obst_sphere_txt;
            zeta_obst_cylinder = zeta_obst_cylinder_txt;
            delta_obst_cylinder = delta_obst_cylinder_txt;
        }
        ergNavParametersFile.close();
    }
    else
    {
        std::cout << "unable to open file" << std::endl;
    }    
}

void ExplicitReferenceGovernor::getErgDsmParameters()
{
    unsigned int pred_horizon_txt; 
    double samplingFreq_pred_txt;
    double kappa_tau_txt;
    double kappa_q_txt;
    double kappa_obst_sphere_txt;
    double kappa_obst_cylinder_txt;

    string path = ros::package::getPath("panda_erg");
    ifstream ergDsmParametersFile(path + "/launch/erg_dsm_parameters.txt");
    if(ergDsmParametersFile.is_open())
    {
        string dummyLine;
        getline(ergDsmParametersFile,dummyLine);
        while (ergDsmParametersFile  >>  pred_horizon_txt >> samplingFreq_pred_txt >> kappa_tau_txt >> kappa_q_txt >> kappa_obst_sphere_txt >> kappa_obst_cylinder_txt)
        {
            pred_horizon = pred_horizon_txt;
            samplingFreq_pred = samplingFreq_pred_txt;
            kappa_tau_cmd = kappa_tau_txt;
            kappa_q = kappa_q_txt;
            kappa_obst_sphere = kappa_obst_sphere_txt;
            kappa_obst_cylinder = kappa_obst_cylinder_txt;
        }
        ergDsmParametersFile.close();
    }
    else
    {
        std::cout << "unable to open file" << std::endl;
    }     
}


/***************************************************************************/
void ExplicitReferenceGovernor::jointReferencesCallback(const std_msgs::Float64MultiArray& jointreferencesmsg)
{
    for (int i=0; i<7; i++)
    {
        q_r_receive(i) = jointreferencesmsg.data[i];
    }    
}
/***************************************************************************/



void ExplicitReferenceGovernor::jointStatesCallback(const sensor_msgs::JointState& jointstatesmsg)
{
    // ROS_INFO("in jointStatesCallback");
    for (int i=0; i<jointstatesmsg.position.size(); i++)
    {
        if(i==0 || i==1)
        {
            finger[i] = jointstatesmsg.position[i]; // finger displacement [m]
        }
        else
        {
            q[i-2] = jointstatesmsg.position[i]; // arm joint positions [rad]
            dotq[i-2] = jointstatesmsg.velocity[i]; // arm joint velocities [rad/s]
        }
    }    

    if(is_q_printed == false)
    {
        for (int i=0; i<q.size(); i++)
        {
            // q_r[i]=q[i]; // initialization of q_r
            std::cout << "panda_joint_q" + std::to_string(i+1) + " : " + std::to_string(q_lowerlimit[i]) + " < " + std::to_string(q[i]) + " < " + std::to_string(q_upperlimit[i])  << std::endl;
        }
        for (int i=0; i<finger.size(); i++)
        {
            // finger_r[i] = finger[i]; // initialization of finger_r
            std::cout << "panda_finger" + std::to_string(i+1) + " : " + std::to_string(finger_lowerlimit[i]) + " < " + std::to_string(finger[i]) + " < " + std::to_string(finger_upperlimit[i])  << std::endl;
        }
        std::cout << std::endl;
        is_q_printed = true;
    }
}

arma::vec ExplicitReferenceGovernor::PIDcontrol(arma::vec& q_setpoint_arg, arma::vec& q_current_arg, arma::vec& dotq_current_arg, 
                                                                arma::vec& i_error_arg, arma::vec& i_clamp_arg,
                                                                arma::vec& Kp_arg, arma::vec& Ki_arg, arma::vec& Kd_arg,
                                                                double samplingFreq_arg, arma::vec& gravity_effort_arg)
{
    arma::vec result; 
    result  = arma::zeros<arma::vec>(joint_num);
    for (int i=0; i<joint_num;i++)
    {
        i_error_arg(i) = i_error_arg(i) + (q_setpoint_arg(i) - q_current_arg(i)) *(1.0/samplingFreq_arg);
        if (i_error_arg(i) < -i_clamp_arg(i))
        {
            i_error_arg(i) = -i_clamp_arg(i);
        }
        else if (i_error_arg(i) > i_clamp_arg(i))
        {
            i_error_arg(i) = i_clamp_arg(i);
        }
        result(i) = Kp_arg(i)*(q_setpoint_arg(i)-q_current_arg(i)) - Kd_arg(i)*dotq_current_arg(i) + Ki_arg(i)*i_error_arg(i) + gravity_effort_arg(i);
    }
    return(result);
}

arma::vec ExplicitReferenceGovernor::PDgcorControl(arma::vec& q_setpoint_arg,arma::vec& q_current_arg, arma::vec& dotq_current_arg, 
                                                    arma::vec& Kp_arg, arma::vec& Kd_arg, 
                                                    arma::vec& gravity_effort_arg, arma::vec& coriolis_effort_arg)
{
    std_msgs::Float64MultiArray gravity_effort_msg;
    std_msgs::Float64MultiArray coriolis_effort_msg;
    gravity_effort_msg.data.resize(joint_num);
    coriolis_effort_msg.data.resize(joint_num); 

    arma::vec result;
    result = arma::zeros<arma::vec>(joint_num);

    for (int i=0; i<joint_num;i++)
    {
        gravity_effort_msg.data[i] = gravity_effort_arg(i);
        coriolis_effort_msg.data[i] = coriolis_effort_arg(i);
        result(i) = Kp_arg(i)*(q_setpoint_arg(i)-q_current_arg(i)) - Kd_arg(i)*dotq_current_arg(i) + gravity_effort_arg(i) + coriolis_effort_arg(i);
        // if (result(i)<tau_lowerlimit(i))
        // {
        //     result(i)=tau_lowerlimit(i);
        // }
        // else if (result(i)>tau_upperlimit(i))
        // {
        //     result(i)=tau_upperlimit(i);
        // }
    }
    pub_gravity_effort.publish(gravity_effort_msg);
    pub_coriolis_effort.publish(coriolis_effort_msg);
    return result;
}

void ExplicitReferenceGovernor::sendTorqueCommands(arma::vec& tau_cmd_arg)
{
    for (int i=0; i<joint_num;i++)
    {   
        std_msgs::Float64 tau_cmd_msg;
        tau_cmd_msg.data = tau_cmd_arg(i);  
        pub_joint_effort_cmd[i].publish(tau_cmd_msg);             
    }

}
void ExplicitReferenceGovernor::sendArmPositionCommands(arma::vec& arm_pos_cmd_arg)
{
    for (int i=0; i<joint_num;i++)
    {   
        std_msgs::Float64 q_r_msg;
        q_r_msg.data = arm_pos_cmd_arg(i);  
        pub_joint_cmd[i].publish(q_r_msg);             
    }
}
void ExplicitReferenceGovernor::sendHandPositionCommands(arma::vec& finger_pos_cmd_arg)
{
    for (int i=0; i<finger_r.size();i++)
    {
        std_msgs::Float64 finger_r_msg;
        finger_r_msg.data = finger_pos_cmd_arg(i);
        pub_finger_cmd[i].publish(finger_r_msg);
    }
}

KDL::Frame ExplicitReferenceGovernor::forwardKinematics(arma::vec& q_arg)
{
    KDL::Frame result;

    kdl_q = armaToKdlVector(q_arg);
    ChainFkSolverPos_recursive fksolver(kdl_chain);
    kinematics_status = fksolver.JntToCart(kdl_q,result);
    // std::cout << "FORWARD KINEMATICS" << std::endl;
    if(kinematics_status>=0)
    {     
        // std::cout << "rotation matrix: \n" << result.M  << std::endl;
        // std::cout << "end-effector position: \n" << result.p << std::endl;
        return result;
    }
    else
    {
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }
    std::cout << std::endl;
}
arma::vec ExplicitReferenceGovernor::inverseKinematics(arma::vec& initial_q_arg, arma::vec& pos_arg, arma::vec& orient_arg)
{
    arma::vec result; 
    result  = arma::zeros<arma::vec>(joint_num);

    kdl_q_init = armaToKdlVector(initial_q_arg);
   
    ChainFkSolverPos_recursive fksolver(kdl_chain);
    ChainIkSolverVel_pinv ikvelsolver(kdl_chain);
    // ChainIkSolverPos_NR iksolver(kdl_chain, fksolver, ikvelsolver);
    ChainIkSolverPos_NR_JL iksolver(kdl_chain, kdl_q_lowerlimit, kdl_q_upperlimit, fksolver, ikvelsolver); 

    Rotation R;
    kdl_x = Frame(R.Quaternion(orient_arg(0),orient_arg(1),orient_arg(2),orient_arg(3)),Vector(pos_arg(0),pos_arg(1),pos_arg(2)));

    inversekinematics_status = iksolver.CartToJnt(kdl_q_init,kdl_x,kdl_q_r);

    if(inversekinematics_status>=0)
    {
        result = kdlToArmaVector(kdl_q_r); 
        return result;
    }
    else
    {
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }
}
arma::mat ExplicitReferenceGovernor::frameToJointPositionsMatrix(arma::vec& q_arg, arma::vec& panda_frame_ids)
{
    arma::mat result; 
    result  = arma::zeros<arma::mat>(3,panda_frame_ids.size());

    kdl_q = armaToKdlVector(q_arg);
    ChainFkSolverPos_recursive fksolver(kdl_chain);
    
    for (int i=0; i<panda_frame_ids.size(); i++)
    {
        kinematics_status = fksolver.JntToCart(kdl_q,kdl_x,panda_frame_ids(i));;
        for (int j=0; j<3; j++)
        {   
            result(j,i)=  kdl_x.p(j); 
        }
    }
    return result;
}
arma::mat ExplicitReferenceGovernor::computeLambda(arma::mat& panda_jointpositions, arma::mat& obst_spheres)
{
    arma::mat result; 
    result  = arma::zeros<arma::mat>(size(panda_jointpositions,1)-1,size(obst_spheres,1));

    for (int i=0; i<size(panda_jointpositions,1)-1;i++) // #joints (-1, because in code +1 to denote next joint)
    {   
        double denum = dot(panda_jointpositions.col(i+1)-panda_jointpositions.col(i),panda_jointpositions.col(i+1)-panda_jointpositions.col(i));
        for (int j=0; j<size(obst_spheres,1); j++)   // #obstacles     
        {
            if (denum <= 0.001) // because denum = norm >0 
            {
                result(i,j) = 0; // in case two consecutive frames are at same position, avoid deviding by zero -> lambda = nan
            }
            else
            {
                result(i,j) = dot(panda_jointpositions.col(i+1)-panda_jointpositions.col(i),obst_spheres.submat(0,j,2,j)-panda_jointpositions.col(i))/denum;
                if (result(i,j) < 0)
                {
                    result(i,j) = 0;
                }
                else if(result(i,j) >1)
                {
                    result(i,j) = 1;
                }
            }
        }       
    }
    return result; 
}
arma::mat ExplicitReferenceGovernor::computePij(arma::mat& panda_jointpositions, arma::mat& lambda)
{
    arma::mat result;
    result = arma::zeros<arma::mat>(3*size(lambda,0),size(lambda,1));
    for (int j=0; j<size(lambda,1); j++) // # obstacles
    {
        for (int i=0; i<size(lambda,0);i++) // # links
        {
            result.submat(3*i,j,3*i+2,j) = panda_jointpositions.col(i) + lambda(i,j) * (panda_jointpositions.col(i+1)-panda_jointpositions.col(i));
        }
    }
    return result;
}
arma::mat ExplicitReferenceGovernor::computeSphericalRepulsion(arma::mat& Pij, arma::mat& obst_spheres, double& zeta_obst_sphere, double& delta_obst_sphere)
{
    arma::mat result;
    result = arma::zeros<arma::mat>(size(Pij,0),size(Pij,1));

    for(int j=0; j<size(result,1); j++) // # obstacles
    {
        for (int i=0; i<size(result,0)/3; i++) // # links
        {
            arma::vec pijmincj;
            pijmincj =  (Pij.submat(3*i,j,3*i+2,j)-obst_spheres.submat(0,j,2,j));

            arma::vec multiplier;
            multiplier = (zeta_obst_sphere - (arma::norm(pijmincj)-obst_spheres.submat(3,j,3,j)))/(zeta_obst_sphere-delta_obst_sphere); 
            if (multiplier(0) <= 0.0)
            {
                multiplier = 0.0;
            }
            result.submat(3*i,j,3*i+2,j) = (multiplier*pijmincj.t()).t();
        }   
    }
    return result;
}
arma::vec ExplicitReferenceGovernor::obstSpereRepulsionField(arma::vec& q, arma::mat& obstSphereRep, arma::mat& lambda, arma::vec& panda_frame_ids)
{
    arma::mat result_temp;
    arma::mat result_norm;
    arma::vec result;
    result_temp = arma::zeros<arma::mat>(joint_num,size(obstSphereRep,1));
    result_norm = arma::zeros<arma::mat>(joint_num,size(obstSphereRep,1));
    result  = arma::zeros<arma::vec>(joint_num);

    int start_index = 2; // Jacobians for i<2 are zero --> pinv(Jac)=NaN
    for(int i = start_index; i< panda_frame_ids.size(); i++)
    {
        kdl_q = armaToKdlVector(q);
        jnt_to_jac_solver->JntToJac(kdl_q, kdl_jac, panda_frame_ids[i]);  // panda_frame_ids[i] = the "end-effector" to compute Jacobian with 
        panda_jacobian = kdlToArmaJacobian(kdl_jac); 

        for (int j = 0; j<size(obstSphereRep,1); j++) // # obstacles 
        {
            if (i==start_index)
            {
                result_temp.submat(0,j,joint_num-1,j) = result_temp.submat(0,j,joint_num-1,j)  + (1-lambda(i,j))*pinv(panda_jacobian)*obstSphereRep.submat(3*i,j,3*i+2,j);
            }
            else if (i==panda_frame_ids.size()-1)
            {   
                result_temp.submat(0,j,joint_num-1,j) = result_temp.submat(0,j,joint_num-1,j) + lambda(i-1,j)*pinv(panda_jacobian)*obstSphereRep.submat(3*(i-1),j,3*(i-1)+2,j);
            }
            else
            {
                result_temp.submat(0,j,joint_num-1,j) = result_temp.submat(0,j,joint_num-1,j) + (1-lambda(i,j))*pinv(panda_jacobian)*obstSphereRep.submat(3*i,j,3*i+2,j);
                result_temp.submat(0,j,joint_num-1,j) = result_temp.submat(0,j,joint_num-1,j) + lambda(i-1,j)*pinv(panda_jacobian)*obstSphereRep.submat(3*(i-1),j,3*(i-1)+2,j);
            }   
        }
        
        for (int j=0; j<size(obstSphereRep,1); j++) // # obstacles 
        {
            double denum;
            denum = std::max(arma::norm(result_temp.submat(0,j,joint_num-1,j)),0.001);
            for (int i=0; i<joint_num; i++) // # joints
            {
                result_norm(i,j) = result_temp(i,j)/denum;
            }
            result = result + result_norm.submat(0,j,joint_num-1,j);
        }
    }
    return result;
}
arma::cube ExplicitReferenceGovernor::computeMuNu(arma::mat& panda_jointpositions, arma::mat& obst_cylinders)
{
    arma::cube result;
    result = arma::zeros<arma::cube>(size(panda_jointpositions,1)-1,size(obst_cylinders,1),2);

    arma::vec a;
    arma::vec b;
    arma::vec c_0;
    arma::vec c_1;
    a = arma::zeros<arma::vec>(3);
    b = arma::zeros<arma::vec>(3);
    c_0 = arma::zeros<arma::vec>(3);
    c_1 = arma::zeros<arma::vec>(3);

    arma::vec cylinder_start;
    arma::vec cylinder_end;

    for (int i=0; i<size(panda_jointpositions,1)-1;i++) // #links
    {
        a = panda_jointpositions.col(i+1)-panda_jointpositions.col(i); 
        for (int j=0; j<size(obst_cylinders,1); j++)   // #obstacles   
        {
            cylinder_start = {obst_cylinders(0,j),obst_cylinders(1,j),obst_cylinders(2,j)-obst_cylinders(4,j)/2};
            cylinder_end = {obst_cylinders(0,j),obst_cylinders(1,j),obst_cylinders(2,j)+obst_cylinders(4,j)/2};
            b = cylinder_end - cylinder_start;
            c_0 = cylinder_start - panda_jointpositions.col(i);
            c_1 = cylinder_end - panda_jointpositions.col(i);

            if (arma::norm(a) <= 0.001) // case of consecutive frames
            {
                (result.slice(0))(i,j) = 0; // mu = 0, Sij = panda_jointpositions.col(i) 
                (result.slice(1))(i,j) = dot(b,(result.slice(0))(i,j)-cylinder_start)/dot(b,b); // nu computed as in point-line case
            }
            else if (arma::norm(arma::cross((a/arma::norm(a)),(b/arma::norm(b)))) < 0.01) // case of parallel segments
            {
                double d_0 = dot(a/arma::norm(a),c_0);
                double d_1 = dot(a/arma::norm(a),c_1);

                if(d_0<=0 && d_1<=0) // cylinder before link in z-direction (viewpoint of panda_jointpositions.col(i) to panda_jointpositions.col(i+1))
                {
                    (result.slice(0))(i,j) = 0; // mu = 0, Sij = panda_jointpositions.col(i) 
                    if (std::abs(d_0) < std::abs(d_1))
                    {
                        (result.slice(1))(i,j) = 0; // nu = 0, Tij = cylinder_start 
                    }
                    else if(std::abs(d_0) > std::abs(d_1))
                    {
                        (result.slice(1))(i,j) = 1; // nu = 1, Tij = cylinder_end 
                    }
                }
                else if(d_0>=arma::norm(a) && d_1>=arma::norm(a)) // cylinder after link in z-direction (viewpoint of panda_jointpositions.col(i) to panda_jointpositions.col(i+1))
                {
                    (result.slice(0))(i,j) = 1; // mu = 1, Sij = panda_jointpositions.col(i+1) 
                    if (std::abs(d_0)<std::abs(d_1))
                    {
                        (result.slice(1))(i,j) = 0; // nu = 0, Tij = cylinder_start 
                    }
                    else if(std::abs(d_0)>std::abs(d_1))
                    {
                        (result.slice(1))(i,j) = 1; // nu = 1, Tij = cylinder_end 
                    }
                }
                else // cylinder and link (partly) overlapping in z-direction
                {
                    double nu_parallel = dot(b,(panda_jointpositions.col(i)+panda_jointpositions.col(i+1))/2-cylinder_start)/dot(b,b);
                    if (0<=nu_parallel && nu_parallel <=1 ) 
                    { 
                        (result.slice(0))(i,j) = 0.5; // mu =0.5, Sij = (panda_jointpositions.col(i)+panda_jointpositions.col(i+1))/2
                        (result.slice(1))(i,j) = nu_parallel; // nu computed as in point-line case
                    }
                    else if(0<=d_0 && d_0<=arma::norm(a)) // = if nu_parallel < 0
                    {
                        if (d_1>arma::norm(a))
                        {
                            (result.slice(0))(i,j) = 1; // mu = 1, Sij = panda_jointpositions.col(i+1) 
                            arma::vec panda_segment = panda_jointpositions.col(i+1) ;
                            (result.slice(1))(i,j) = dot(b,panda_segment-cylinder_start)/dot(b,b); // nu computed as in point-line case
                        }
                        else if(d_1 < 0)
                        {
                            (result.slice(0))(i,j) = 0; // mu = 0, Sij = panda_jointpositions.col(i) 
                            arma::vec panda_segment = panda_jointpositions.col(i) ;
                            (result.slice(1))(i,j) = dot(b,panda_segment-cylinder_start)/dot(b,b); // nu computed as in point-line case
                        }
                    }
                    else if (0<=d_1 && d_1<=arma::norm(a)) // % = if nu_parallel > 1
                    {
                        if (d_0>arma::norm(a))
                        {
                            (result.slice(0))(i,j) = 1; // mu = 1, Sij = panda_jointpositions.col(i+1) 
                            arma::vec panda_segment = panda_jointpositions.col(i+1) ;
                            (result.slice(1))(i,j) = dot(b,panda_segment-cylinder_start)/dot(b,b); // nu computed as in point-line case
                        }
                        else if (d_0 < 0)
                        {
                            (result.slice(0))(i,j) = 0; // mu = 0, Sij = panda_jointpositions.col(i) 
                            arma::vec panda_segment = panda_jointpositions.col(i) ;
                            (result.slice(1))(i,j) = dot(b,panda_segment-cylinder_start)/dot(b,b); // nu computed as in point-line case
                        }
                    }                 
                }  
            }
            else // case of skew segments 
            {               
                (result.slice(0))(i,j) = (dot(b,b)*dot(c_0,a)-dot(c_0,b)*dot(b,a))/(dot(b,b)*dot(a,a)-dot(a,b)*dot(b,a)); // mu computed for skew line-line case
                if((result.slice(0))(i,j) < 0)
                {
                    (result.slice(0))(i,j) = 0; // mu = 0, Sij = panda_jointpositions.col(i)
                    arma::vec panda_segment = panda_jointpositions.col(i);
                    (result.slice(1))(i,j) = dot(b,panda_segment-cylinder_start)/dot(b,b); // nu computed as in point-line case
                }
                else if((result.slice(0))(i,j)>1)
                {
                    (result.slice(0))(i,j) = 1; // mu = 1, Sij = panda_jointpositions.col(i+1)
                    arma::vec panda_segment = panda_jointpositions.col(i+1);
                    (result.slice(1))(i,j) = dot(b,panda_segment-cylinder_start)/dot(b,b); // nu computed as in point-line case
                }
                else
                {
                    (result.slice(1))(i,j) = dot(a,a)/dot(b,a) * (dot(b,b)*dot(c_0,a)-dot(c_0,b)*dot(b,a))/(dot(b,b)*dot(a,a)-dot(b,a)*dot(b,a)) - dot(c_0,a)/dot(b,a); // nu computed for skew line-line case
                }

                if((result.slice(1))(i,j) < 0)
                {
                    (result.slice(1))(i,j) = 0;
                }
                else if((result.slice(1))(i,j) > 1)
                {
                    (result.slice(1))(i,j) = 1;
                }
            }  
        }
    }
    return result;
}


arma::cube ExplicitReferenceGovernor::computeSijTij(arma::mat& panda_jointpositions, arma::mat& obst_cylinders, arma::cube& munu)
{
    arma::cube result;
    result = arma::zeros<arma::cube>(3*(size(panda_jointpositions,1)-1),size(obst_cylinders,1),2);

    arma::vec cylinder_start;
    arma::vec cylinder_end;

    for (int i=0; i<size(panda_jointpositions,1)-1;i++) // #links
    {
        for (int j=0; j<size(obst_cylinders,1); j++)   // #obstacles   
        {
            cylinder_start = {obst_cylinders(0,j),obst_cylinders(1,j),obst_cylinders(2,j)-obst_cylinders(4,j)/2};
            cylinder_end = {obst_cylinders(0,j),obst_cylinders(1,j),obst_cylinders(2,j)+obst_cylinders(4,j)/2};

            (result.slice(0)).submat(3*i,j,3*i+2,j) = panda_jointpositions.col(i) + (munu.slice(0))(i,j)*(panda_jointpositions.col(i+1)-panda_jointpositions.col(i));
            (result.slice(1)).submat(3*i,j,3*i+2,j) = obst_cylinders.submat(0,j,2,j) + (munu.slice(1))(i,j)*(cylinder_end-cylinder_start);
        }
    }
    return result;
}

arma::mat ExplicitReferenceGovernor::computeCylindricalRepulsionField(arma::cube& SijTij, arma::mat& obst_cylinders, double& zeta_obst_cylinder, double& delta_obst_cylinder)
{
    arma::mat result;
    result = arma::zeros<arma::mat>(3*(size(panda_jointpositions,1)-1),size(obst_cylinders,1));

    for(int j=0; j<size(result,1); j++) // # obstacles
    {
        for (int i=0; i<size(result,0)/3; i++) // # links
        {
            arma::vec sijmintij;
            sijmintij =  (SijTij.slice(0)).submat(3*i,j,3*i+2,j)-(SijTij.slice(1)).submat(3*i,j,3*i+2,j);
            arma::vec multiplier;
            multiplier = (zeta_obst_cylinder - (arma::norm(sijmintij)-obst_cylinders.submat(3,j,3,j)))/(zeta_obst_cylinder-delta_obst_cylinder); 
            if (multiplier(0) <= 0.0)
            {
                multiplier = 0.0;
            }
            result.submat(3*i,j,3*i+2,j) = (multiplier*sijmintij.t()).t();
        }   
    }
    return result;
}

arma::vec ExplicitReferenceGovernor::obstCylinderRepulsionField(arma::vec& q, arma::mat& obstCylinderRep, arma::cube& munu, arma::vec& panda_frame_ids)
{
    arma::mat result_temp;
    arma::mat result_norm;
    arma::vec result;
    result_temp = arma::zeros<arma::mat>(joint_num,size(obstCylinderRep,1));
    result_norm = arma::zeros<arma::mat>(joint_num,size(obstCylinderRep,1));
    result  = arma::zeros<arma::vec>(joint_num);
    
    arma::mat mu; 
    mu = munu.slice(0);

    int start_index = 2; // Jacobians for i<2 are zero --> pinv(Jac)=NaN
    for(int i = start_index; i< panda_frame_ids.size(); i++)
    {
        kdl_q = armaToKdlVector(q);
        jnt_to_jac_solver->JntToJac(kdl_q, kdl_jac, panda_frame_ids[i]);  // panda_frame_ids[i] = the "end-effector" to compute Jacobian with 
        panda_jacobian = kdlToArmaJacobian(kdl_jac); 

        for (int j = 0; j<size(obstCylinderRep,1); j++) // # obstacles 
        {
            if (i==start_index)
            {
                result_temp.submat(0,j,joint_num-1,j) = result_temp.submat(0,j,joint_num-1,j)  + (1-mu(i,j))*pinv(panda_jacobian)*obstCylinderRep.submat(3*i,j,3*i+2,j);
            }
            else if (i==panda_frame_ids.size()-1)
            {   
                result_temp.submat(0,j,joint_num-1,j) = result_temp.submat(0,j,joint_num-1,j) + mu(i-1,j)*pinv(panda_jacobian)*obstCylinderRep.submat(3*(i-1),j,3*(i-1)+2,j);
            }
            else
            {
                result_temp.submat(0,j,joint_num-1,j) = result_temp.submat(0,j,joint_num-1,j) + (1-mu(i,j))*pinv(panda_jacobian)*obstCylinderRep.submat(3*i,j,3*i+2,j);  
                result_temp.submat(0,j,joint_num-1,j) = result_temp.submat(0,j,joint_num-1,j) + mu(i-1,j)*pinv(panda_jacobian)*obstCylinderRep.submat(3*(i-1),j,3*(i-1)+2,j);
            }   
        }
        
        for (int j=0; j<size(obstCylinderRep,1); j++) // # obstacles 
        {
            double denum;
            denum = std::max(arma::norm(result_temp.submat(0,j,joint_num-1,j)),0.001);
            for (int i=0; i<joint_num; i++) // # joints
            {
                result_norm(i,j) = result_temp(i,j)/denum;
            }
            result = result + result_norm.submat(0,j,joint_num-1,j);
        }
    }
    return result;
}

arma::vec ExplicitReferenceGovernor::attractionField(arma::vec& q_setpoint_arg, arma::vec& q_applied_arg, double& eta_arg)
{
    std_msgs::Float64MultiArray user_ref_msg;
    user_ref_msg.data.resize(joint_num);
    std_msgs::Float64MultiArray applied_ref_msg;
    applied_ref_msg.data.resize(joint_num);

    arma::vec  result;
    result  = arma::zeros<arma::vec>(joint_num);

    for (int i = 0; i < joint_num; i++)
    {
        user_ref_msg.data[i]=q_setpoint_arg(i);
        applied_ref_msg.data[i]=q_applied_arg(i);
        result(i) = (q_setpoint_arg(i) - q_applied_arg(i))/(std::max(norm(q_setpoint_arg,q_applied_arg),eta_arg)); 
    } 
    pub_user_ref.publish(user_ref_msg);
    pub_applied_ref.publish(applied_ref_msg);
    return result;
}
arma::vec ExplicitReferenceGovernor::qRepulsionField(arma::vec& q_setpoint_arg, 
                                                     arma::vec& q_lowerlimit_arg, arma::vec& q_upperlimit_arg, 
                                                     double& zeta_q_arg, double& delta_q_arg)
{   
    arma::vec result;
    result  = arma::zeros<arma::vec>(joint_num);
    for (int i = 0; i < joint_num; i++)
    {
        result(i) = std::max((zeta_q_arg-std::abs(q_setpoint_arg(i)-q_upperlimit_arg(i)))/(zeta_q_arg-delta_q_arg), 0.0) * (q_setpoint_arg[i]-q_upperlimit_arg[i])/arma::norm(q_setpoint_arg-q_upperlimit_arg)     
                  + std::max((zeta_q_arg-std::abs(q_setpoint_arg(i)-q_lowerlimit_arg(i)))/(zeta_q_arg-delta_q_arg), 0.0) * (q_setpoint_arg[i]-q_lowerlimit_arg[i])/arma::norm(q_setpoint_arg-q_lowerlimit_arg);
    } 
    return result;
}
arma::vec ExplicitReferenceGovernor::navigationField(arma::vec& rho_att_arg, arma::vec& rho_rep_q_arg, arma::vec& rho_rep_obst_sphere, arma::vec& rho_rep_obst_cylinder)
{
    arma::vec result;
    result  = arma::zeros<arma::vec>(joint_num);
    for (int i = 0; i < joint_num; i++)
    {
        result(i) = rho_att_arg(i) + rho_rep_q_arg(i) + rho_rep_obst_sphere(i) + rho_rep_obst_cylinder(i);
    }
    return result;
}

void ExplicitReferenceGovernor::forwardDynamics(arma::vec& q_current_arg){
    q_pred = q_current_arg;
    std::cout << "q_pred \n" << q_pred << std::endl;

    kdl_q_pred = armaToKdlVector(q_pred);
    dyn_param->JntToMass(kdl_q_pred, kdl_mass_matrix_pred);

    q_pred = kdlToArmaVector(kdl_q_pred);
    mass_matrix_pred = kdlToArmaMatrix(kdl_mass_matrix_pred);
    std::cout << "mass_matrix_pred \n" << mass_matrix_pred << std::endl;
}

void ExplicitReferenceGovernor::trajBasedDSM(arma::vec& q_setpoint_arg, arma::vec& q_current_arg, arma::vec& dotq_current_arg,
                                             arma::vec& i_error_arg, arma::vec& i_clamp_arg,
                                             arma::vec& Kp_arg, arma::vec& Ki_arg, arma::vec& Kd_arg,
                                             double& samplingFreq_arg, unsigned int& pred_horizon_arg, ros::Time& startTime_arg)
{
    if (printpred == 1)
    {
        printPredTime = ros::Time::now();
        // std::cout << "printPredTime = " << (printPredTime-startTime_arg).toSec() << std::endl;
    }
    // std::cout << (ros::Time::now()-startTime_arg).toSec() << std::endl;

    // initialization
    dotq_pred = dotq_current_arg;
    q_pred = q_current_arg;

    // std::cout << "q_pred \n" << q_pred << std::endl;

    i_error_pred = i_error_arg;

    // begin for loop over prediction horizon
    for(int k = 0; k < pred_horizon_arg; k++)
    {
        // update KDL vectors/matrices
        kdl_q_pred = armaToKdlVector(q_pred);
        kdl_dotq_pred = armaToKdlVector(dotq_pred);
        dyn_param->JntToMass(kdl_q_pred, kdl_mass_matrix_pred);
        dyn_param->JntToCoriolis(kdl_q_pred, kdl_dotq_pred, kdl_coriolis_effort_pred);
        dyn_param->JntToGravity(kdl_q_pred, kdl_gravity_effort_pred);

        // convert to Armadillo vectors/matrices
        q_pred = kdlToArmaVector(kdl_q_pred);
        dotq_pred = kdlToArmaVector(kdl_dotq_pred);
        mass_matrix_pred = kdlToArmaMatrix(kdl_mass_matrix_pred);
        coriolis_effort_pred = kdlToArmaVector(kdl_coriolis_effort_pred);
        gravity_effort_pred = kdlToArmaVector(kdl_gravity_effort_pred);

        tau_pred = PIDcontrol(q_setpoint_arg,q_pred,dotq_pred,i_error_pred,i_clamp,Kp,Ki,Kd,samplingFreq_pred,gravity_effort_pred);

        // ddotq_pred = mass_matrix_pred.colPivHouseholderQr().solve(tau_pred-coriolis_effort_pred-gravity_effort_pred); 
        // ddotq_pred = mass_matrix_pred.ldlt().solve(tau_pred-coriolis_effort_pred-gravity_effort_pred); 
        // ddotq_pred = mass_matrix_pred.llt().solve(tau_pred-coriolis_effort_pred-gravity_effort_pred); 
        // ddotq_pred = mass_matrix_pred.fullPivHouseholderQr().solve(tau_pred-coriolis_effort_pred-gravity_effort_pred);     

        // ddotq_pred = arma::solve(mass_matrix_pred, tau_pred-coriolis_effort_pred-gravity_effort_pred);    
        // ddotq_pred = arma::solve(mass_matrix_pred, tau_pred-coriolis_effort_pred-gravity_effort_pred, arma::solve_opts::fast);  
        ddotq_pred = arma::solve(mass_matrix_pred, tau_pred-coriolis_effort_pred-gravity_effort_pred, arma::solve_opts::likely_sympd);  
        // ddotq_pred = arma::solve(mass_matrix_pred, tau_pred-coriolis_effort_pred-gravity_effort_pred, arma::solve_opts::fast + arma::solve_opts::likely_sympd);     
        if(k==pred_horizon_arg-1){
            // std::cout << "mass_matrix_pred \n" << mass_matrix_pred << std::endl;
            // std::cout << "ddotq_pred \n" << ddotq_pred << std::endl;
        }
        // ddotq_pred = arma::solve(mass_matrix_pred*1000.0, tau_pred-coriolis_effort_pred-gravity_effort_pred, arma::solve_opts::fast + arma::solve_opts::likely_sympd);     
        // if(k==pred_horizon_arg-1){
        //     std::cout << "mass_matrix_pred * 1000 \n" << mass_matrix_pred * 1000.0 << std::endl;
        //     std::cout << "ddotq_pred * 1000 \n" << ddotq_pred * 1000 << std::endl;
        // }
        // ddotq_pred = ddotq_pred * 1000.0;

        // Simplectic Euler
        for (int i = 0; i < joint_num; i++)
        {
            dotq_pred(i) = dotq_pred(i) + ddotq_pred(i) * (1.0/samplingFreq_pred);
            q_pred(i) = q_pred(i) + dotq_pred(i) * (1.0/samplingFreq_pred);
        } 

        if (printpred == 1)
        {
            std_msgs::Float64MultiArray prediction_msg;
            prediction_msg.data.resize(15); // 1 predtime, 7 joint angles, 7 torques
            for (int i = 0; i < 15; i++)
            {   if (i ==0)
                {
                    // std::cout << "prediction " << k << " , tpred = " <<  (printPredTime-startTime_arg).toSec() + k/samplingFreq_pred << std::endl;
                    prediction_msg.data[i]=(printPredTime-startTime_arg).toSec() + k/samplingFreq_pred;
                }
                // else
                else if(i>0 && i<8)
                {
                    // std::cout << "else if  " << i << std::endl;
                    prediction_msg.data[i]=q_pred(i-1);
                }  
                else
                {
                //     // std::cout << "else " << i << std::endl;
                    prediction_msg.data[i]=tau_pred(i-8);
                }
                
            }
            pub_prediction.publish(prediction_msg);
            if(k==pred_horizon_arg-1)
            {
                printpred = 0;
            }
        }

        // torque constraints 
        for (int i = 0; i < joint_num; i++)
        {
            if (k==0)
            {
                DSM_tau_cmd_lowerlimit = tau_pred(i)- tau_lowerlimit(i); // initialize
                DSM_tau_cmd_upperlimit = tau_upperlimit(i) - tau_pred(i); // initialize
                if (i==0)
                {
                    DSM_tau_cmd = std::min(DSM_tau_cmd_lowerlimit, DSM_tau_cmd_upperlimit); // initialize
                }
                else 
                {
                    DSM_tau_cmd = std::min(DSM_tau_cmd, DSM_tau_cmd_lowerlimit);
                    DSM_tau_cmd = std::min(DSM_tau_cmd, DSM_tau_cmd_upperlimit);  
                }
            }
            else if (k>0)
            {
                DSM_tau_cmd_lowerlimit = std::min(DSM_tau_cmd_lowerlimit, tau_pred(i)-tau_lowerlimit(i));
                DSM_tau_cmd_upperlimit = std::min(DSM_tau_cmd_upperlimit, tau_upperlimit(i)-tau_pred(i)); 
                DSM_tau_cmd = std::min(DSM_tau_cmd, DSM_tau_cmd_lowerlimit);
                DSM_tau_cmd = std::min(DSM_tau_cmd, DSM_tau_cmd_upperlimit);
            }
        }

        // joint angle constraints 
        for (int i = 0; i < joint_num; i++)
        {
            if (k==0)
            {
                DSM_q_lowerlimit = q_pred(i) - q_lowerlimit(i); // initialize
                DSM_q_upperlimit = q_upperlimit(i) - q_pred(i); // initialize
                if (i==0)
                {
                    DSM_q = std::min(DSM_q_lowerlimit, DSM_q_upperlimit); // initialize
                }
                else 
                {
                    DSM_q = std::min(DSM_q, DSM_q_lowerlimit);
                    DSM_q = std::min(DSM_q, DSM_q_upperlimit);  
                }
            }
            else if (k>0)
            {
                DSM_q_lowerlimit = std::min(DSM_q_lowerlimit, q(i)-q_lowerlimit(i));
                DSM_q_upperlimit = std::min(DSM_q_upperlimit, q_upperlimit(i)-q(i)); 
                DSM_q = std::min(DSM_q, DSM_q_lowerlimit);
                DSM_q = std::min(DSM_q, DSM_q_upperlimit);
            }
        } 

        // spherical obstacle constraints 
        panda_jointpositions = frameToJointPositionsMatrix(q_pred, panda_frame_ids); 
        lambda = computeLambda(panda_jointpositions, obst_spheres);
        Pij = computePij(panda_jointpositions, lambda);
        int start_index = 1;
        for (int i=start_index; i<size(Pij,0)/3; i++) // # links, for j=0 -> link = frame 1 (you cannot move it away)
        {
            for (int j=0; j<size(Pij,1); j++) // # obstacles
            {
                arma::vec pijmincj;
                pijmincj =  Pij.submat(3*i,j,3*i+2,j)-obst_spheres.submat(0,j,2,j);
                if(i==start_index && j==0)
                {
                    DSM_obst_sphere = arma::norm(pijmincj)-(1+delta_obst_sphere)*obst_spheres(3,j);
                }
                else
                {
                    DSM_obst_sphere = std::min(DSM_obst_sphere, arma::norm(pijmincj)-(1+delta_obst_sphere)*obst_spheres(3,j));
                }
            }
        }

        // cylindrical obstacle constraints 
        panda_jointpositions = frameToJointPositionsMatrix(q_pred, panda_frame_ids); 
        munu = computeMuNu(panda_jointpositions, obst_cylinders); 
        SijTij = computeSijTij(panda_jointpositions, obst_cylinders, munu);  
        start_index = 1;
        for (int i=start_index; i<size((SijTij.slice(0)),0)/3; i++) // # links, for j=0 -> link = frame 1 (you cannot move it away)
        {
            for (int j=0; j<size((SijTij.slice(0)),1); j++) // # obstacles
            {
                arma::vec sijmintij;
                sijmintij =  (SijTij.slice(0)).submat(3*i,j,3*i+2,j)-(SijTij.slice(1)).submat(3*i,j,3*i+2,j);
                if(i==start_index && j==0)
                {
                    DSM_obst_cylinder = arma::norm(sijmintij)-(1+delta_obst_cylinder)*obst_cylinders(3,j);
                }
                else
                {
                    DSM_obst_cylinder = std::min(DSM_obst_cylinder, arma::norm(sijmintij)-(1+delta_obst_cylinder)*obst_cylinders(3,j));
                }
            }
        }

        DSM = std::min(kappa_tau_cmd*DSM_tau_cmd, kappa_q*DSM_q); 
        // DSM = std::min(DSM, kappa_obst_sphere*DSM_obst_sphere);
        // DSM = std::min(DSM, kappa_obst_cylinder*DSM_obst_cylinder);

        DSM = std::max(DSM, 0.0);  
    }
    std_msgs::Float64 DSM_tau_msg;
    DSM_tau_msg.data = kappa_tau_cmd*DSM_tau_cmd;
    pub_DSM_tau.publish(DSM_tau_msg);

    std_msgs::Float64 DSM_q_msg;
    DSM_q_msg.data = kappa_q*DSM_q;
    pub_DSM_q.publish(DSM_q_msg);

    std_msgs::Float64 DSM_obst_sphere_msg;
    DSM_obst_sphere_msg.data = kappa_obst_sphere*DSM_obst_sphere;
    pub_DSM_obst_sphere.publish(DSM_obst_sphere_msg);

    std_msgs::Float64 DSM_obst_cylinder_msg;
    DSM_obst_cylinder_msg.data = kappa_obst_cylinder*DSM_obst_cylinder;
    pub_DSM_obst_cylinder.publish(DSM_obst_cylinder_msg);

    std_msgs::Float64 DSM_msg;
    DSM_msg.data = DSM;
    pub_DSM.publish(DSM_msg);
}
arma::vec ExplicitReferenceGovernor::qvUpdate(arma::vec& q_applied_arg,
                                              arma::vec& rho_arg, double& DSM_arg,
                                              double& samplingFreq_arg)
{
    arma::vec result;
    result  = arma::zeros<arma::vec>(joint_num);
    for (int i = 0; i < joint_num; i++)
    {
        result(i) = q_applied_arg(i) + DSM_arg * rho_arg(i) * (1/samplingFreq_arg);
    } 
    return result;
}




KDL::JntArray ExplicitReferenceGovernor::armaToKdlVector(arma::vec& arma_vector_arg)
{
    KDL::JntArray result; 
    result.resize(joint_num);
    for (int i = 0; i < joint_num; i++)
    {
        result(i) = arma_vector_arg(i);
    }
    return result;
}

arma::vec ExplicitReferenceGovernor::kdlToArmaVector(KDL::JntArray& KDL_vector_arg)
{
    arma::vec result; 
    result  = arma::zeros<arma::vec>(joint_num);
    for (int i = 0; i < joint_num; i++)
    {
        result(i) = KDL_vector_arg(i);
    }
    return result;
}
arma::mat ExplicitReferenceGovernor::kdlToArmaMatrix(KDL::JntSpaceInertiaMatrix& KDL_matrix_arg)
{   
    arma::mat result;
    result  = arma::zeros<arma::mat>(joint_num,joint_num);
    for (int i = 0; i < joint_num; i++)
    {
        for (int j = 0; j < joint_num; j++)
        {
            result(i,j) = KDL_matrix_arg(i,j);
        }
    } 
    return result;
}

arma::mat ExplicitReferenceGovernor::kdlToArmaJacobian(KDL::Jacobian& KDL_jacobian_arg)
{
    arma::mat result;
    result  = arma::zeros<arma::mat>(3,joint_num); // only positions are important for repulsion field
    
    for (int i = 0; i < 3; i++) // x,y,z
    {
        for (int j = 0; j < joint_num; j++) // q1, q2, ..., q7
        {
            result(i, j) = KDL_jacobian_arg(i, j);
        }
    }
    return result;

}


double ExplicitReferenceGovernor::norm(arma::vec& a, arma::vec& b)
{
    if ((a.size() != b.size()) || (a.size() == 0 || b.size() == 0))
    {
        return -1;
    }
    double result = 0;
    for (int i = 0; i < a.size(); i++)
    {
        result += (a[i] - b[i])*(a[i] - b[i]);
    }
    return sqrt(result);
}
