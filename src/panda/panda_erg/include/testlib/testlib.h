#include <cmath>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>

#include <sensor_msgs/JointState.h>

#include <armadillo/armadillo>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>


// #include <kdl/jntarray.hpp>

// #include <kdl/frames.hpp>
// #include <kdl/chainfksolverXXX_YYY.hpp>
// #include <kdl/chainiksolverXXX_YYY.hpp>
// #include <kdl/jntspaceinertiamatrix.hpp>
// #include <kdl/chaindynparam.hpp>


class TestLibrary
{
private:
  


public:
    TestLibrary(std::string name);
    ~TestLibrary();

    // ROS
    ros::NodeHandle n;
    
    // Publishers and Subscribers
    ros::Subscriber sub_jointstate;
    ros::Publisher pub_effort_cmd[7];
    
    // callback functions
    void jointStatesCallback(const sensor_msgs::JointState& msg);

    void PIDcontrol();
    void updateKDLq(std::vector<double>& q_arg);
    void updateKDLdotq(std::vector<double>& dotq_arg);
    void updateKDLDynamics();
    double norm(std::vector<double>& a, std::vector<double>& b);
    void forwardKinematics();
    void insertKDLxColInArmaMat(int mat_col);
    void insertKDLxSliceInArmaCube(int cube_slice);
    void computeLambda();
    void computePij();
    void obstSphereRepulsionField();
    void attractionField();
    void qRepulsionField();
    void navigationField();
    void qvUpdate();
    void predPIDcontrol();
    void simDSM();
    arma::mat updateKDL2ArmaMatrix(KDL::JntSpaceInertiaMatrix KDL_matrix_arg, arma::mat arma_matrix_arg);
    arma::vec updateKDL2ArmaVector(KDL::JntArray KDL_vector_arg, arma::vec arma_vector_arg);
    arma::mat updateKDLJac2ArmaJac(KDL::Jacobian KDL_jac_arg, arma::mat arma_jac_arg);

    std::vector<double> q_lowerlimit; 
    std::vector<double> q_upperlimit; 
    std::vector<double> q; 
    std::vector<double> dotq;
    std::vector<double> q_r;  
    std::vector<double> q_v; 

    std::vector<double> tau_cmd; 
    std::vector<double> tau_lowerlimit;
    std::vector<double> tau_upperlimit;

    std::vector<double> Kp; 
    std::vector<double> Ki;
    std::vector<double> Kd;

    std::vector<double> i_error; 
    std::vector<double> i_clamp;

    double samplingFreq; 

    bool print_q;  

    KDL::Tree kdl_tree;
    KDL::Chain kdl_chain;
    unsigned int joint_num;
    KDL::JntArray kdl_q; // represents actual, predicted, and applied joint angles
    KDL::JntArray kdl_dotq; // represents actual and predicted joint velocities
    KDL::JntArray kdl_gravity_effort;
    KDL::Vector gravity;
    std::shared_ptr<KDL::ChainDynParam> dyn_param;
    KDL::JntSpaceInertiaMatrix kdl_mass_matrix;
    KDL::JntArray kdl_coriolis_effort;
    // KDL::ChainFkSolverPos_recursive fksolver();
    KDL::Frame kdl_x; // cartesian position of joints
    bool kinematics_status;

    // KDL Solvers performing the actual computations   
    // http://library.isr.ist.utl.pt/docs/roswiki/pr2_mechanism(2f)Tutorials(2f)Coding(20)a(20)realtime(20)Cartesian(20)controller(20)with(20)KDL.html                                                                                                                            
    // boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
    std::shared_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
    // The variables (which need to be pre-allocated).
    KDL::Jacobian kdl_jac;            // Jacobian    

    arma::mat arma_jac_pos;

    arma::vec ddotq_pred;
    std::vector<double> dotq_pred;
    std::vector<double> q_pred;
    
    arma::mat mass_matrix_pred;
    arma::vec coriolis_effort_pred;
    arma::vec gravity_effort_pred;
    arma::vec tau_cmd_pred;

    double samplingFreq_pred;
    double pred_horizon;

    std::vector<double> i_error_pred;

    double DSM_tau_cmd_lowerlimit;
    double DSM_tau_cmd_upperlimit;
    double DSM_tau_cmd; 
    double kappa_tau_cmd;
    double DSM_q_lowerlimit;
    double DSM_q_upperlimit;
    double DSM_q; 
    double kappa_q;
    double DSM_obst_sphere;
    double kappa_obst_sphere;
    double DSM;

    std::vector<double> rho_att;
    double eta; 
    std::vector<double> rho_rep_q;
    double zeta_q;
    double delta_q;
    double zeta_obst_sphere; 
    double delta_obst_sphere;
    arma::vec rhomat_rep_obst_sphere;
    arma::vec rho_rep_obst;
    std::vector<double> rho;   

    // obstacles
    arma::vec obst_sphere1_center; 
    double obst_sphere1_radius; 
    std::vector<int> frame_ids;
    arma::cube armacube_frames;
    arma::mat armamat_pos;
    arma::vec lambda;
    arma::vec Pij;



};
