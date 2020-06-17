#include <testlib/testlib.h>
#include <ros/package.h>

#include <string>
#include <cstdlib>
#include <algorithm>

using namespace std;
using namespace KDL;

TestLibrary::TestLibrary(string name) : n(name)
{
    q_lowerlimit = vector<double> {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}; // [rad]
    q_upperlimit = vector<double> {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973}; // [rad]
    q = vector<double> (7);
    dotq = vector<double> (7); 
    q_r = vector<double> (7); 
    q_v = vector<double> (7);  

    tau_cmd = vector<double> (7); 
    tau_lowerlimit = vector<double> {-87.0, -87.0, -87.0, -87.0, -12.0, -12.0, -12.0}; // [Nm]
    tau_upperlimit = vector<double> {87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0}; // [Nm]

    // Kp = vector<double> {100.0, 100.0, 100.0, 100.0, 18.0, 15.0, 18.0}; 
    Kp = vector<double> {120.0, 300.0, 180.0, 180.0, 70.0, 20.0, 20.0}; 
    // Ki = vector<double> {20.0, 20.0, 20.0, 20.0, 0.0, 0.0, 0.0}; 
    Ki = vector<double> {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
    // Kd = vector<double> {10.0, 10.0, 10.0, 10.0, 3.0, 2.0, 4.0}; 
    Kd = vector<double> {20.0, 20.0, 20.0, 20.0, 4.0, 3.0, 4.0}; 
    i_error = vector<double> {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    i_clamp = vector<double> {2.5, 4.0, 2.5, 2.5, 1.0, 1.0, 1.0}; 

    samplingFreq = 1000.0; //[Hz]

    print_q = false; // print current joint angles


    // Publishers and Subscribers
    sub_jointstate = n.subscribe("/panda/joint_states", 1, &TestLibrary::jointStatesCallback, this);
    for(int i=0; i<q.size(); i++)
    {
        pub_effort_cmd[i] = n.advertise<std_msgs::Float64>("/panda/joint" + std::to_string(i+1) + "_effort_controller/command",100);
    }



    // create KDL chain
    // if panda_arm_hand.urdf.xacro is adapted, then don't forget to run in a terminal with sourced workspace
    // rosrun xacro xacro --inorder -o model.urdf model.urdf.xacro
    // https://p4sser8y-words.readthedocs.io/ROS/orocos_kdl.html#usage
    // kdl_parser::treeFromFile("/home/kmerckae/src/constrained_control_robotarm/ros_ws/src/panda_description/urdf/panda_arm_hand.urdf", kdl_tree);
    string path = ros::package::getPath("panda_description");
    if (!kdl_parser::treeFromFile(path + "/urdf/panda_arm_hand.urdf", kdl_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        // return false;
    }
    if (!kdl_tree.getChain("panda_link0", "panda_hand", kdl_chain))
    {
        ROS_ERROR("Failed to get kdl chain");
    }
    ROS_INFO("Successfully get KDL Chain with %d joints", kdl_chain.getNrOfJoints());
    std::cout  << std::endl;

    joint_num = kdl_chain.getNrOfJoints();
    kdl_q.resize(joint_num);
    kdl_dotq.resize(joint_num);
    kdl_gravity_effort.resize(joint_num);
    gravity = Vector {0.0, 0.0, -9.81};
    dyn_param = make_shared<ChainDynParam>(kdl_chain, gravity);
    kdl_mass_matrix.resize(joint_num);
    kdl_coriolis_effort.resize(joint_num);
    // Create solver based on kinematic chain
    // http://www.orocos.org/kdl/examples
    // ChainFkSolverPos_recursive fksolver(kdl_chain); 
    // fksolver.JntToCart(kdl_q,kdl_x,5);
    // Construct the kdl solvers in non-realtime.
    // http://library.isr.ist.utl.pt/docs/roswiki/pr2_mechanism(2f)Tutorials(2f)Coding(20)a(20)realtime(20)Cartesian(20)controller(20)with(20)KDL.html
    jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(kdl_chain));
    kdl_jac.resize(joint_num);

    arma_jac_pos = arma::zeros<arma::mat>(3,joint_num);


    ddotq_pred = arma::zeros<arma::vec>(joint_num);
    q_pred = vector<double> (7);
    dotq_pred = vector<double> (7); 
    mass_matrix_pred = arma::zeros<arma::mat>(joint_num,joint_num);
    coriolis_effort_pred = arma::zeros<arma::vec>(joint_num);
    gravity_effort_pred = arma::zeros<arma::vec>(joint_num);
    tau_cmd_pred = arma::zeros<arma::vec>(joint_num);
    samplingFreq_pred = 100.0; //[Hz]
    i_error_pred = vector<double> {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    pred_horizon = 100; // number of samples
    DSM = 0.0;
    kappa_tau_cmd = 10.0;
    kappa_q = 100.0; 
    kappa_obst_sphere = 100.0; 


    // obstacles
    obst_sphere1_center = {0.0,0.5,0.5} ; // [m] 
    obst_sphere1_radius = 0.25; //[m] 

    frame_ids = vector<int> {1,2,3,4,5,6,7,8}; // [frame indices] see Franka doc
    armacube_frames = arma::zeros<arma::cube>(4,4,frame_ids.size()); // not really used right now
    armamat_pos = arma::zeros<arma::mat>(3,frame_ids.size()); // matrix with frame positions, each frame position = one column
    lambda = arma::zeros<arma::vec>(frame_ids.size()-1); // vec will become matrix when more obstacles
    Pij = arma::zeros<arma::vec>(3*lambda.size()); // vec will become matrix when more obstacles


    rho_att = vector<double> (7);
    eta = 0.1; 
    rho_rep_q = vector<double> (7);
    zeta_q=10*3.1415/180; //[rad]
    delta_q=0.0*3.1415/180; //[rad]
    zeta_obst_sphere = 0.2; //[m] 
    delta_obst_sphere = 0.0;//[m]
    rhomat_rep_obst_sphere = arma::zeros<arma::vec>(3*lambda.size()); // vec will become matrix when more obstacles
    rho = vector<double> (7);
    rho_rep_obst=  arma::zeros<arma::vec>(7);
    
    

    
}

TestLibrary::~TestLibrary()
{
}

void TestLibrary::jointStatesCallback(const sensor_msgs::JointState& msg)
{
    // ROS_INFO("in jointStatesCallback");
    for (int i=0; i<msg.position.size(); i++)
    {
        if(i==0 || i==1)
        {
            // i=0: panda_finger_joint1, i=1: panda_finger_joint_2
        }
        else
        {
            q[i-2] = msg.position[i]; // arm joints
            dotq[i-2] = msg.velocity[i];
        }
    } 
    if(print_q == false)
    {
        for (int i=0; i<q.size(); i++)
        {
            q_v[i]=q[i]; // initialization of q_v
            std::cout << "panda_joint_q" + std::to_string(i+1) + " : " + std::to_string(q_lowerlimit[i]) + " < " + std::to_string(q[i]) + " < " + std::to_string(q_upperlimit[i])  << std::endl;
        }
        std::cout << std::endl;
        print_q = true;
    }
}

void TestLibrary::PIDcontrol()
{
    for (int i=0; i<joint_num;i++)
    {
        i_error[i] = i_error[i] + (q_v[i] - q[i]) *(1.0/samplingFreq);//(ros::Time::now() - start).toSec();
        if (i_error[i] < -i_clamp[i])
        {
            i_error[i] = -i_clamp[i];
        }
        else if (i_error[i] > i_clamp[i])
        {
            i_error[i] = i_clamp[i];
        }
        updateKDLq(q);
        updateKDLdotq(dotq);
        updateKDLDynamics();
        tau_cmd[i] = Kp[i]*(q_v[i]-q[i]) - Kd[i]*dotq[i] + Ki[i]*i_error[i] + kdl_gravity_effort(i);
    }
    for (int i=0; i<joint_num;i++)
    {   
        std_msgs::Float64 tau_cmd_msg;
        tau_cmd_msg.data = tau_cmd[i];  
        pub_effort_cmd[i].publish(tau_cmd_msg);             
    }
}

void TestLibrary::predPIDcontrol()
{
    for (int i=0; i<joint_num;i++)
    {
        i_error_pred[i] = i_error_pred[i] + (q_v[i] - q_pred[i]) *(1.0/samplingFreq_pred);//(ros::Time::now() - start).toSec();
        if (i_error_pred[i] < -i_clamp[i])
        {
            i_error_pred[i] = -i_clamp[i];
        }
        else if (i_error_pred[i] > i_clamp[i])
        {
            i_error_pred[i] = i_clamp[i];
        }
        // updateKDLVectors(q_pred, dotq_pred);
        tau_cmd_pred[i] = Kp[i]*(q_v[i]-q_pred[i]) - Kd[i]*dotq_pred[i] + Ki[i]*i_error_pred[i] + gravity_effort_pred(i);
    }
}

void TestLibrary::updateKDLq(std::vector<double>& q_arg)
{
    for (int i = 0; i < joint_num; i++)
    {
        kdl_q(i) = q_arg[i];
    }
}

void TestLibrary::updateKDLdotq(std::vector<double>& dotq_arg)
{
    for (int i = 0; i < joint_num; i++)
    {
        kdl_dotq(i) = dotq_arg[i]; 
    }
}
void TestLibrary::updateKDLDynamics()
{
    dyn_param->JntToGravity(kdl_q, kdl_gravity_effort);
    dyn_param->JntToMass(kdl_q, kdl_mass_matrix);
    dyn_param->JntToCoriolis(kdl_q, kdl_dotq, kdl_coriolis_effort);
}

double TestLibrary::norm(vector<double>& a, vector<double>& b)
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

void TestLibrary::forwardKinematics() // you can use this for current, predicted, and applied joint config
{
    ChainFkSolverPos_recursive fksolver(kdl_chain); 
    for(int i=0; i<frame_ids.size(); i++)
    {
        kinematics_status = fksolver.JntToCart(kdl_q,kdl_x,frame_ids[i]);
        insertKDLxColInArmaMat(i);
         
        // kinematics_status = fksolver.JntToCart(kdl_q,kdl_x,segmentNumber);
        if(kinematics_status>=0)
        {
            // std::cout << "check insertKDLxColInArmaMat for segment " << frame_ids[i] << std::endl;
            // std::cout << armamat_pos.col(i) << std::endl; 

            // insertKDLxSliceInArmaMat(i);
            // std::cout << "check insertKDLxColInArmaMat for segment " << frame_ids[i] << std::endl;
            // std::cout << armacube_frames(0,0,i) << " " << armacube_frames(0,1,i) << " " << armacube_frames(0,2,i) << " " << armacube_frames(0,3,i)  << std::endl;
            // std::cout << armacube_frames(1,0,i) << " " << armacube_frames(1,1,i) << " " << armacube_frames(1,2,i) << " " << armacube_frames(1,3,i)  << std::endl;
            // std::cout << armacube_frames(2,0,i) << " " << armacube_frames(2,1,i) << " " << armacube_frames(2,2,i) << " " << armacube_frames(2,3,i)  << std::endl;
            // std::cout << armacube_frames(3,0,i) << " " << armacube_frames(3,1,i) << " " << armacube_frames(3,2,i) << " " << armacube_frames(3,3,i)  << std::endl;
            // std::cout << std::endl;

            // std::cout << "segment " << frame_ids[i] << std::endl;
            // std::cout << kdl_x(0,0) << " " << kdl_x(0,1) << " " << kdl_x(0,2) << " " << kdl_x(0,3)  << std::endl;
            // std::cout << kdl_x(1,0) << " " << kdl_x(1,1) << " " << kdl_x(1,2) << " " << kdl_x(1,3)  << std::endl;
            // std::cout << kdl_x(2,0) << " " << kdl_x(2,1) << " " << kdl_x(2,2) << " " << kdl_x(2,3)  << std::endl;
            // std::cout << kdl_x(3,0) << " " << kdl_x(3,1) << " " << kdl_x(3,2) << " " << kdl_x(3,3)  << std::endl;
            // std::cout << std::endl;
            // std::cout << kdl_x  << std::endl;
            // std::cout << std::endl;
            // std::cout << std::endl;

            // printf("%s \n","Succes, thanks KDL!");
        }
        else
        {
            // printf("%s \n","Error: could not calculate forward kinematics :(");
        }
    }
    // std::cout << armamat_pos.submat(0,0,2,frame_ids.size()-1) << std::endl;
}

void TestLibrary::insertKDLxColInArmaMat(int mat_col)
{
    for (int i=0; i<3; i++) // rows (x,y,z)
    {
        armamat_pos(i,mat_col) = kdl_x(i,3); // 4th columns of homogeneous transformation matrix = positions
    }
}

void TestLibrary::insertKDLxSliceInArmaCube(int cube_slice)
{
    for (int i=0; i<4; i++) // rows
    {
        for (int j=0; j<4; j++) //columns
        {
            armacube_frames(i,j,cube_slice) = kdl_x(i,j);
        }
    }
}


void TestLibrary::computeLambda() // #rows = #links, later #columns = #obstacles
{
    for (int i=0; i< frame_ids.size()-1; i++)
    {
        double denum = dot(armamat_pos.col(i+1)-armamat_pos.col(i),armamat_pos.col(i+1)-armamat_pos.col(i)) ; 
        if (denum <= 0.001) // because denum = norm >0 
        {
            lambda(i) = 0; // in case two consecutive frames are at same position, avoid deviding by zero -> lambda = nan
        }
        else
        {
            lambda(i) = dot(armamat_pos.col(i+1)-armamat_pos.col(i),obst_sphere1_center-armamat_pos.col(i))/denum;
        }
    }
}

void TestLibrary::computePij()
{
    for(int i=0; i<lambda.size(); i++)
    {
        if (lambda(i)<=0)
        {
            Pij.submat(3*i,0,3*i+2,0) = armamat_pos.submat(0,i,2,i);
        }
        else if (lambda(i)>= 1)
        {
            Pij.submat(3*i,0,3*i+2,0) = armamat_pos.submat(0,i+1,2,i+1);
        }
        else
        {
            Pij.submat(3*i,0,3*i+2,0) = armamat_pos.submat(0,i,2,i) + lambda(i) * (armamat_pos.submat(0,i+1,2,i+1)-armamat_pos.submat(0,i,2,i));
        }
    }
}

void TestLibrary::attractionField()
{
    for (int i = 0; i < joint_num; i++)
    {
        rho_att[i] = (q_r[i] - q_v[i])/(std::max(norm(q_r,q_v),eta)); 
    } 
}

void TestLibrary::qRepulsionField()
{
    for (int i = 0; i < joint_num; i++)
    {
        rho_rep_q[i] = std::max((zeta_q-std::abs(q_v[i]-q_upperlimit[i]))/(zeta_q-delta_q), 0.0) * (q_v[i]-q_upperlimit[i])/std::abs(q_v[i]-q_upperlimit[i])     
                     + std::max((zeta_q-std::abs(q_v[i]-q_lowerlimit[i]))/(zeta_q-delta_q), 0.0) * (q_v[i]-q_lowerlimit[i])/std::abs(q_v[i]-q_lowerlimit[i]);
    } 
}

void TestLibrary::obstSphereRepulsionField()
{
    updateKDLq(q_v); // updated kdl_q
    forwardKinematics(); // computes armamat_pos: matrix with frame positions in columns
    // std::cout << armamat_pos.submat(0,0,2,frame_ids.size()-1) << std::endl;
    // std::cout << std::endl;
    computeLambda(); // lambda = percentage between two consecutive frames used to compute Pij, #rows = #links, #columns = #obstacles (currently 1)
    // std::cout << lambda << std::endl;
    // std::cout << std::endl;
    computePij(); // point on link i which is closest to obstacle j, parametrized by lambda
    // std::cout << Pij << std::endl;

    // std::cout << rhomat_rep_obst_sphere << std::endl;

    for(int i=0; i<lambda.size(); i++)
    {   
        rhomat_rep_obst_sphere.submat(3*i,0,3*i+2,0) = std::max((zeta_obst_sphere-(arma::norm(Pij.submat(3*i,0,3*i+2,0)-obst_sphere1_center)-obst_sphere1_radius))/(zeta_obst_sphere-delta_obst_sphere), 0.0) *
                                                       (Pij.submat(3*i,0,3*i+2,0)-obst_sphere1_center)/arma::norm(Pij.submat(3*i,0,3*i+2,0)-obst_sphere1_center);
        // std::cout << "norm : " << arma::norm(Pij.submat(3*i,0,3*i+2,0)-obst_sphere1_center) << std::endl;
        // std::cout << "norm - radius : " << arma::norm(Pij.submat(3*i,0,3*i+2,0)-obst_sphere1_center)-obst_sphere1_radius<< std::endl;
        // std::cout << "zeta - (norm - radius) : " << zeta_obst_sphere-(arma::norm(Pij.submat(3*i,0,3*i+2,0)-obst_sphere1_center)-obst_sphere1_radius) << std::endl;
        // std::cout << std::endl;
    }
    // std::cout << rhomat_rep_obst_sphere << std::endl;

    rho_rep_obst = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // use zeros function, repulsion field in joint space 
    // std::cout << "before for loop" << std::endl; 
    // std::cout  << rho_rep_obst << std::endl;                                                                                                           
    for(int i=0; i<frame_ids.size()-2; i++)
    {
        // Compute the Jacobian and its pseudo-inverse at certain frame position  
        jnt_to_jac_solver->JntToJac(kdl_q, kdl_jac,frame_ids[i+2]);  // frame_ids[i] = the "end-effector" to compute Jacobian with 
        arma_jac_pos = updateKDLJac2ArmaJac(kdl_jac, arma_jac_pos);

        // transform the repulsion field to joint space
        if(i==frame_ids.size()-3)
        {
            // std::cout << "i = " << i << std::endl;
            // std::cout << "jacobian, " << frame_ids[i+2] << std::endl;
            // std::cout << "lambda, " << lambda[i+1] << std::endl;
            // std::cout << "rho_taskspace, " << std::endl;
            // std::cout << rhomat_rep_obst_sphere.submat(3*(i+1),0,3*(i+1)+2,0) << std::endl;
            rho_rep_obst = rho_rep_obst + 
                            lambda[i+1]*pinv(arma_jac_pos)*rhomat_rep_obst_sphere.submat(3*(i+1),0,3*(i+1)+2,0)/
                            std::max(arma::norm(lambda[i+1]*pinv(arma_jac_pos)*rhomat_rep_obst_sphere.submat(3*(i+1),0,3*(i+1)+2,0)),0.001);       
        }
        else
        {
            // std::cout << "i = " << i << std::endl;
            // std::cout << "jacobian, " << frame_ids[i+2] << std::endl;
            // std::cout << "lambda, " << lambda[i+1] << std::endl;
            // std::cout << "rho_taskspace, " << std::endl;
            // std::cout << rhomat_rep_obst_sphere.submat(3*(i+1),0,3*(i+1)+2,0) << std::endl;
            // std::cout << "1-lambda, " << lambda[i+2] << std::endl;
            // std::cout << "rho_taskspace, " << std::endl;
            // std::cout << rhomat_rep_obst_sphere.submat(3*(i+2),0,3*(i+2)+2,0) << std::endl;
            rho_rep_obst = rho_rep_obst + 
                            lambda[i+1]*pinv(arma_jac_pos)*rhomat_rep_obst_sphere.submat(3*(i+1),0,3*(i+1)+2,0)/
                            std::max(arma::norm(lambda[i+1]*pinv(arma_jac_pos)*rhomat_rep_obst_sphere.submat(3*(i+1),0,3*(i+1)+2,0)),0.001);
            rho_rep_obst = rho_rep_obst + 
                            (1-lambda[i+2])*pinv(arma_jac_pos)*rhomat_rep_obst_sphere.submat(3*(i+2),0,3*(i+2)+2,0)/
                            std::max(arma::norm((1-lambda[i+2])*pinv(arma_jac_pos)*rhomat_rep_obst_sphere.submat(3*(i+2),0,3*(i+2)+2,0)),0.001);
        }
        // std::cout << "i = " << i << std::endl;
        // std::cout  << rho_rep_obst << std::endl;
        
    }
    // std::cout << "final rho rep with norm" << std::endl;
    // std::cout << rho_rep_obst << std::endl;
    // std::cout << std::endl;
}

void TestLibrary::navigationField()
{
    std::cout << "rho" << std::endl;
    for (int i = 0; i < joint_num; i++)
    {
        rho[i] = rho_att[i] + rho_rep_q[i] + rho_rep_obst(i);
        std::cout << rho[i] << std::endl;
    }
    std::cout << std::endl;
    
}
void TestLibrary::qvUpdate()
{
    for (int i = 0; i < joint_num; i++)
    {
        // q_v[i] = q_v[i] + 1 * rho_att[i] * (1/samplingFreq);
        // q_v[i] = q_v[i] + DSM * rho_att[i] * (1/samplingFreq);  
        q_v[i] = q_v[i] + DSM * rho[i] * (1/samplingFreq);  
    } 
}

arma::mat TestLibrary::updateKDL2ArmaMatrix(KDL::JntSpaceInertiaMatrix KDL_matrix_arg, arma::mat arma_matrix_arg)
{
    for (int i = 0; i < joint_num; i++)
    {
        for (int j = 0; j < joint_num; j++)
        {
            arma_matrix_arg(i, j) = KDL_matrix_arg(i, j);
        }
    }
    return arma_matrix_arg;
}

 arma::vec TestLibrary::updateKDL2ArmaVector(KDL::JntArray KDL_vector_arg, arma::vec arma_vector_arg)
{
    for (int i = 0; i < joint_num; i++)
    {
        arma_vector_arg(i) = KDL_vector_arg(i); 
    }
    return arma_vector_arg;
}

arma::mat TestLibrary::updateKDLJac2ArmaJac(KDL::Jacobian KDL_jac_arg, arma::mat arma_jac_arg)
{
    for (int i = 0; i < 3; i++) // x,y,z
    {
        for (int j = 0; j < joint_num; j++) // q1, q2, ..., q7
        {
            arma_jac_arg(i, j) = KDL_jac_arg(i, j);
        }
    }
    return arma_jac_arg;
}

void TestLibrary::simDSM()
{
    // initialization
    dotq_pred = dotq;
    q_pred = q;

    i_error_pred = i_error;

    // begin for loop over prediction horizon
    for(int k = 0; k < pred_horizon; k++)
    {
        // update KDL vectors
        updateKDLq(q_pred); // updated kdl_q with q_pred
        updateKDLdotq(dotq_pred);
        updateKDLDynamics();

        mass_matrix_pred = updateKDL2ArmaMatrix(kdl_mass_matrix, mass_matrix_pred);
        coriolis_effort_pred = updateKDL2ArmaVector(kdl_coriolis_effort, coriolis_effort_pred);
        gravity_effort_pred = updateKDL2ArmaVector(kdl_gravity_effort, gravity_effort_pred);
        predPIDcontrol(); // computes tau_cmd_pred
        
        // if (k == 0 || k == pred_horizon-1)
        // {
        //     std::cout << "pred step = " << k << std::endl;
        //     mass_matrix_pred.print(); // OK
        //     // coriolis_effort_pred.print(); //OK
        //     // gravity_effort_pred.print(); // OK
        //     // tau_cmd_pred.print(); // OK
        //     std::cout << std::endl;
        // }

        // ddotq_pred with inverse Mass matrix (computationally efficient method)
        ddotq_pred = arma::solve(mass_matrix_pred, tau_cmd_pred-coriolis_effort_pred-gravity_effort_pred);

        // for loop over all joints
        for (int i = 0; i < joint_num; i++)
        {
            dotq_pred[i] = dotq_pred[i] + ddotq_pred(i) * (1/samplingFreq_pred);
            q_pred[i] = q_pred[i] + dotq_pred[i] * (1/samplingFreq_pred);
        } 
    
        // torque constraints 
        for (int i = 0; i < joint_num; i++)
        {
            if (k==0)
            {
                DSM_tau_cmd_lowerlimit = tau_cmd_pred(i) - tau_lowerlimit[i]; // initialize
                DSM_tau_cmd_upperlimit = tau_upperlimit[i] - tau_cmd_pred(i); // initialize
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
                DSM_tau_cmd_lowerlimit = std::min(DSM_tau_cmd_lowerlimit, tau_cmd_pred(i)-tau_lowerlimit[i]);
                DSM_tau_cmd_upperlimit = std::min(DSM_tau_cmd_upperlimit, tau_upperlimit[i]-tau_cmd_pred(i)); 
                DSM_tau_cmd = std::min(DSM_tau_cmd, DSM_tau_cmd_lowerlimit);
                DSM_tau_cmd = std::min(DSM_tau_cmd, DSM_tau_cmd_upperlimit);
            }
        } 

        // joint angle constraints 
        for (int i = 0; i < joint_num; i++)
        {
            if (k==0)
            {
                DSM_q_lowerlimit = q[i] - q_lowerlimit[i]; // initialize
                DSM_q_upperlimit = q_upperlimit[i] - q[i]; // initialize
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
                DSM_q_lowerlimit = std::min(DSM_q_lowerlimit, q[i]-q_lowerlimit[i]);
                DSM_q_upperlimit = std::min(DSM_q_upperlimit, q_upperlimit[i]-q[i]); 
                DSM_q = std::min(DSM_q, DSM_q_lowerlimit);
                DSM_q = std::min(DSM_q, DSM_q_upperlimit);
            }
        } 

        // spherical obstacle constraint
        forwardKinematics(); // computes armamat_pos: matrix with predicted frame positions in columns
        computeLambda(); // lambda = percentage between two consecutive frames used to compute Pij, #rows = #links, #columns = #obstacles (currently 1)
        computePij(); // point on predicted link i which is closest to obstacle j, parametrized by lambda
        for(int i=1; i<lambda.size(); i++) // from i=1, because we cannot move part for i=0 away
        {   
            if (i==1)
            {
                DSM_obst_sphere = arma::norm(Pij.submat(3*i,0,3*i+2,0)-obst_sphere1_center)-obst_sphere1_radius; // initialize
            }
            else
            {
                DSM_obst_sphere = std::min(DSM_obst_sphere, arma::norm(Pij.submat(3*i,0,3*i+2,0)-obst_sphere1_center)-obst_sphere1_radius);
            }
            
        }

        // overall DSM
        DSM = std::min(kappa_tau_cmd*DSM_tau_cmd, kappa_q*DSM_q);
        DSM = std::min(DSM, kappa_obst_sphere*DSM_obst_sphere);
        DSM = std::max(DSM, 0.0);

        std::cout << "DSM_tau_cmd: " << kappa_tau_cmd*DSM_tau_cmd << std::endl; 
        std::cout << "DSM_q: " << kappa_q*DSM_q << std::endl; 
        std::cout << "DSM_obst_sphere: " << kappa_obst_sphere*DSM_obst_sphere << std::endl; 
        std::cout << std::endl; 
    }
}