#include <cmath>
#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <armadillo/armadillo>

#include <chrono> 
#include <ctime> 

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

class ExplicitReferenceGovernor
{
private:
    ros::NodeHandle n;
    
public:
    ExplicitReferenceGovernor(std::string name);
    ~ExplicitReferenceGovernor();   

    ros::Time startTime;
    ros::Time printPredTime;
    bool printpred; 
    double reference_starttime;

    ros::Subscriber sub_reference;
    ros::Subscriber sub_state;
    ros::Publisher pub_joint_effort_cmd[7];
    ros::Publisher pub_joint_cmd[7];
    ros::Publisher pub_finger_cmd[2];
    ros::Publisher pub_user_ref;
    ros::Publisher pub_applied_ref;
    ros::Publisher pub_DSM;
    ros::Publisher pub_DSM_tau;
    ros::Publisher pub_DSM_q;
    ros::Publisher pub_DSM_obst_sphere;
    ros::Publisher pub_DSM_obst_cylinder;
    ros::Publisher pub_prediction;
    
    ros::Publisher pub_gravity_effort;
    ros::Publisher pub_coriolis_effort;

    ros::Publisher pub_chrono;

    bool is_q_printed;  
    bool kinematics_status;
    bool inversekinematics_status;

    unsigned int joint_num; 
    double samplingFreq;

    arma::vec finger_lowerlimit;
    arma::vec finger_upperlimit; 
    arma::vec finger;
    arma::vec finger_r; 

    arma::vec q_lowerlimit; 
    arma::vec q_upperlimit;
    arma::vec tau_lowerlimit;  
    arma::vec tau_upperlimit;

    arma::vec q_r_receive;
    arma::vec q_init;
    arma::vec q;
    arma::vec q_r; 
    arma::vec q_v;
    arma::vec dotq;
    
   
    arma::vec pos_r; 
    arma::vec orient_r;

    arma::vec Kp;
    arma::vec Ki;
    arma::vec Kd;
    arma::vec i_error;
    arma::vec i_clamp;
    arma::vec tau_cmd;
    arma::vec gravity_effort;
    arma::vec coriolis_effort;

    arma::vec panda_frame_ids;
    arma::cube panda_frames;
    arma::mat panda_jointpositions;
    arma::mat panda_jacobian;

    arma::mat obst_spheres;
    arma::mat lambda;
    arma::mat Pij;
    arma::mat obstSphereRep;

    arma::mat obst_cylinders;
    arma::cube munu;
    arma::cube SijTij; 
    arma::mat obstCylinderRep;

    arma::vec rho_att;
    double eta;
    arma::vec rho_rep_q;
    double zeta_q;
    double delta_q;
    arma::vec rho_rep_obst_sphere;
    double zeta_obst_sphere;
    double delta_obst_sphere;
    arma::vec rho_rep_obst_cylinder;
    double zeta_obst_cylinder;
    double delta_obst_cylinder;
    arma::vec rho;

    unsigned int pred_horizon;
    double samplingFreq_pred;
    arma::vec q_pred;
    arma::vec dotq_pred;
    arma::vec ddotq_pred;
    arma::vec i_error_pred;
    arma::mat mass_matrix_pred;
    arma::vec coriolis_effort_pred;
    arma::vec gravity_effort_pred;
    arma::vec tau_pred;
 
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
    double DSM_obst_cylinder;
    double kappa_obst_cylinder;
    double DSM;

    KDL::Tree kdl_tree;
    KDL::Chain kdl_chain;

    KDL::JntArray kdl_q_lowerlimit;
    KDL::JntArray kdl_q_upperlimit;
    KDL::JntArray kdl_q_init; 
    KDL::JntArray kdl_q;            
    KDL::JntArray kdl_q_r;
    KDL::JntArray kdl_dotq;
    
    std::shared_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
    KDL::Jacobian kdl_jac; 
    
    KDL::JntArray kdl_gravity_effort;
    KDL::Vector gravity;
    std::shared_ptr<KDL::ChainDynParam> dyn_param;
    KDL::JntSpaceInertiaMatrix kdl_mass_matrix;
    KDL::JntArray kdl_coriolis_effort;

    KDL::JntArray kdl_q_pred;
    KDL::JntArray kdl_dotq_pred;
    KDL::JntArray kdl_gravity_effort_pred;
    KDL::JntArray kdl_coriolis_effort_pred;
    KDL::JntSpaceInertiaMatrix kdl_mass_matrix_pred;

    KDL::Frame kdl_x;;
   
    void jointReferencesCallback(const std_msgs::Float64MultiArray& jointreferencesmsg);
    void jointStatesCallback(const sensor_msgs::JointState& jointstatesmsg);

    void getFingerLimitsRefs(); //finger_lowerlimit finger_upperlimit finger_r
    void getQLimits(); //q_lowerlimit, q_upperlimit
    void getTauLimits(); //tau_lowerlimit, tau_upperlimit
    void getQRefs(); // q_r 
    void getPoseRefs(); // pos x, pos y, pos z, quat x, quat y, quat z, quat w
    void getControlParameters(); // Kp, Ki, Kd, i_clamp
    void getObstacleSphereParameters(); // obst_spheres = centerx, centery, centerz, radius
    void getObstacleCylinderParameters(); // obst_cylinders = centerx, centery, centerz, radius, height
    void getErgNavParameters(); // eta, zeta_q, delta_q, zeta_obst_sphere, delta_obst_sphere, zeta_obst_cyl, delta_obst_cyl
    void getErgDsmParameters(); // pred_horizon, samplingFreq_pred, kappa_tau_cmd, kappa_q, kappa_obst_sphere  

    arma::vec PIDcontrol(arma::vec& q_setpoint_arg, arma::vec& q_current_arg, arma::vec& dotq_current_arg, 
                         arma::vec& i_error_arg, arma::vec& i_clamp_arg,
                         arma::vec& Kp_arg, arma::vec& Ki_arg, arma::vec& Kd_arg,
                         double samplingFreq_arg, arma::vec& gravity_effort_arg);
    arma::vec PDgcorControl(arma::vec& q_setpoint_arg,arma::vec& q_current_arg, arma::vec& dotq_current_arg, 
                            arma::vec& Kp_arg, arma::vec& Kd_arg, 
                            arma::vec& gravity_effort_arg, arma::vec& coriolis_effort_arg);


    void sendTorqueCommands(arma::vec& tau_cmd_arg);
    void sendArmPositionCommands(arma::vec& arm_pos_cmd_arg);
    void sendHandPositionCommands(arma::vec& finger_pos_cmd_arg);

    KDL::Frame forwardKinematics(arma::vec& q_arg);
    arma::vec inverseKinematics(arma::vec& initial_q_arg, arma::vec& pos_arg, arma::vec& orient_arg);

    arma::mat frameToJointPositionsMatrix(arma::vec& q_arg, arma::vec& panda_frame_ids);

    arma::mat computeLambda(arma::mat& panda_jointpositions, arma::mat& obst_spheres);
    arma::mat computePij(arma::mat& panda_jointpositions, arma::mat& lambda);
    arma::mat computeSphericalRepulsion(arma::mat& Pij, arma::mat& obst_spheres, double& zeta_obst, double& delta_obst);

    arma::cube computeMuNu(arma::mat& panda_jointpositions, arma::mat& obst_cylinders);
    arma::cube computeSijTij(arma::mat& panda_jointpositions, arma::mat& obst_cylinders, arma::cube& munu);
    arma::mat computeCylindricalRepulsionField(arma::cube& SijTij, arma::mat& obst_cylinders, double& zeta_obst, double& delta_obst);

    arma::vec attractionField(arma::vec& q_setpoint_arg, arma::vec& q_applied_arg, double& eta_arg);
    arma::vec qRepulsionField(arma::vec& q_setpoint_arg, 
                              arma::vec& q_lowerlimit_arg, arma::vec& q_upperlimit_arg, 
                              double& zeta_q_arg, double& delta_q_arg);
    arma::vec obstSpereRepulsionField(arma::vec& q, arma::mat& obstSphereRep, arma::mat& lambda, arma::vec& panda_frame_ids);
    arma::vec obstCylinderRepulsionField(arma::vec& q, arma::mat& obstCylinderRep, arma::cube& munu, arma::vec& panda_frame_ids);

    arma::vec navigationField(arma::vec& rho_att, arma::vec& rho_rep_q, arma::vec& rho_rep_obst_sphere, arma::vec& rho_rep_obst_cylinder);

    void forwardDynamics(arma::vec& q_current_arg);
    void trajBasedDSM(arma::vec& q_setpoint_arg, arma::vec& q_current_arg, arma::vec& dotq_current_arg,
                      arma::vec& i_error_arg, arma::vec& i_clamp_arg,
                      arma::vec& Kp_arg, arma::vec& Ki_arg, arma::vec& Kd_arg,
                      double& samplingFreq_arg, unsigned int& pred_horizon_arg, ros::Time& startTime_arg);
    arma::vec qvUpdate(arma::vec& q_applied_arg,
                                       arma::vec& rho_arg, double& DSM_arg,
                                       double& samplingFreq_arg);

    KDL::JntArray armaToKdlVector(arma::vec& arma_vector_arg);
    arma::vec kdlToArmaVector(KDL::JntArray& KDL_vector_arg);
    arma::mat kdlToArmaMatrix(KDL::JntSpaceInertiaMatrix& KDL_matrix_arg);
    arma::mat kdlToArmaJacobian(KDL::Jacobian& KDL_jacobian_arg);

    double norm(arma::vec& a, arma::vec& b);
};
