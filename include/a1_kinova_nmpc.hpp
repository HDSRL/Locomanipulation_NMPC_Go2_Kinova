#ifndef NMPC_A1_Kinova_HPP
#define NMPC_A1_Kinova_HPP

#include "a1_kinova_loco_structs.hpp"
#include <chrono>
#include "Parameters.hpp"



using namespace casadi;
namespace fs = std::filesystem;



// #define NUMBER_OF_AGENTS 1
// #define NUMBER_OF_OBS 9

class NMPC_A1_KINOVA  :  public Parameters 
{

private:
    
    Args args;     // Arguments for the solver //Need to redefine this
    Function f;
    double curr_time;
    Function f_SRB_;
    Function f_arm_;
    Function f_SRB_reduced_;
    Function f_arm_reduced_;
    Function f_arm_outside_;
    Function D_q_arm_sym_;
    Function solver_;
    Function Jacobian_holonomic_;
    std::string basePath;
    bool isDomainChanged_ = false;

    // NMPC variables
    DM qk_; // Current state (qk) in the other code
    DM qkAll_; // All states for N horizons (X0) in the other code
    int domain_ = 0;
    int control_tick_ = 0;
    DM traj_;//Desired trajectory

    DM q_des_; // Desired state (xd) in the other code
    DM uk_; // Current control input - n_inputs x 1 
    DM ukSRB_; // SRB control input - n_u_SRB x 1
    DM ukArm_ ; // Arm control input - n_u_arm x 1
    DM ukArmPD_; // Arm PD control input - n_u_arm x 1
    DM ukLambda_; // Interaction force control input - n_u_lambda_int x 1
    DM ukAll_; // All control inputs for N horizons (u0) in the other code
    DM ukAllSRB_; // SRB control inputs for N horizons (u0) in the other code
    DM ukAllArm_; // Arm control inputs for N horizons (u0) in the other code
    DM ukAllLambda_; // Interaction force control inputs for N horizons (u0) in the other code
    DM x_sol_; // Current state (xk) in the other code
    DM x_sol_N_; // solution for all states in shape n_states x N
    DM x_sol_N_SRB_; // SRB solution for all states in shape n_SRB x N
    DM x_sol_N_arm_; // Arm solution for all states in shape n_arm x N
    DM D_q_outside_; // Outside arm mass inertia matrix

    DM bez_alpha_; // Bezier coefficients
    DM bez_alpha_GRF_; // Bezier coefficients for arm
    
    // Logger variables
    DM uout;
    DM xout;
    DM feet;
    DM traj;
    DM tout;
    DM NMPC_solve_time;
    DM NMPC_solve_time_;
    DM xsolout;

    // From low-level controller
    double q[18] = {0};
    double dq[18] = {0};
    Eigen::Vector4d state_other; 
    int contactInd[4] = {1};
    Eigen::Matrix<double, 3, 4> toePos_;
    //From raisim
    double q_arm[4] = {0};
    double dq_arm[4] = {0};
    

    // Solver inputs
    DM g_ub_, g_lb_;

    // Eigen vars
    Eigen::MatrixXd x_sol_eigen_; // mpc_state_e_x_eventbased_ in the old code - this is only state solution for N domains
    Eigen::MatrixXd bez_alpha_eigen_; // alpha_COM_traj_e_ in the old code
    Eigen::MatrixXd bez_alpha_GRF_eigen_; // GRFs Bezier coefficients
    Eigen::MatrixXd ukAllSRB_eigen_; // GRFs
    
    Eigen::Matrix<double, 2*NUMBER_OF_AGENTS, 1> Pstart_;
    Eigen::Matrix<double, 2,NUMBER_OF_OBS> Pobs;
    Eigen::Matrix<double, 2,NUMBER_OF_OBS> Pobs_real;

    size_t agent_id_;
    Eigen::Vector2d agent_Initial_;
    
    Eigen::MatrixXd Pr_refined_;
    Eigen::MatrixXd Prd_refined_;

    std::fstream fid;
    int success ;

    double x0,y0,z0,xf,yf,zf,yaw0;
    float joystick_state[19] = {0};
    bool joystick_toggle_gait_ = false;
    bool joystick_sit_down_ = false; // Used to check if the joystick sit down is pressed
    int joystick_gait_counter_ = 0;
    bool joystick_send_gait_ = false;
    int gait_switch_counter_ = 0;
    bool reduce_swing_height_ = false; // Used to reduce the swing height of the LL
    bool first_stand_flag_ = true; // Used to check if the first stand is done
    bool sit_down_flag_ = true; // Used to check if the sit down is done
    double tsit0 = 0.0; // Used to store the time when the sit down starts
    double tsitf = 0.0; // Used to store the time when the sit down ends

public:
    Params params;

    NMPC_A1_KINOVA(int argc, char *argv[]) : Parameters(argc,argv)
    {
        // Initialize the parameters
        params.mpc_iter = 0;
        params.current_time = 0.0;
        params.dt = 0.0167;
        params.Fs = 1/params.dt;
        params.sim_length = SIM_TIME;
        // params.L = 1000; // not sure if this is needed
        params.N = HORIZON;   // Horizon length
        params.zmax = 250; // Maximum torque
        params.height = 0.28;   // stand height
        params.n_SRB = 12; // Number of SRB states
        params.n_arm = 20; // Number of arm states
        params.n_states = params.n_SRB + params.n_arm; // Number of states
        params.n_u_SRB = 12; // Number of SRB inputs
        params.n_u_arm = 4; // Number of arm inputs
        params.n_u_lambda_int = 6; // Number of interaction force inputs
        params.n_inputs = params.n_u_SRB + params.n_u_lambda_int + params.n_u_arm ; // Number of inputs (SRB torques, contact forces, and arm torques)
        params.n_controls =params.n_u_SRB + params.n_u_lambda_int + params.n_u_arm; //What is this?
        params.NumHolCon = 1*6; // Number of holonomic constraints: refer to the function GetHolonomicConstraints

        params.gait = 8;        // Trot gait
        params.G = 9.81;        // gravity
        params.mass = 12.4530;  // mass
        params.J = DM::zeros(3,3);  // inertia matrix
        params.pf = DM::zeros(3, 4);    // nominal foot position matrix
        params.p_hip = DM::zeros(3, 4); // nominal hip position matrix
        params.Pr_refined_ = Eigen::MatrixXd::Zero(4, 4);
        params.Prd_refined_ = Eigen::MatrixXd::Zero(4, 4);
        params.agentIndex = 1;  // agent index

        
        // Initialize the pf matrix within the constructor body
        params.pf(0,0) = 0.183;  params.pf(1,0) = -0.1321; params.pf(2,0) = 0.01;
        params.pf(0,1) = 0.183;  params.pf(1,1) = 0.1321;  params.pf(2,1) = 0.01;
        params.pf(0,2) = -0.183; params.pf(1,2) = -0.1321; params.pf(2,2) = 0.01;
        params.pf(0,3) = -0.183; params.pf(1,3) = 0.1321;  params.pf(2,3) = 0.01;

        params.p_hip(0,0) = 0.183;  params.p_hip(1,0) = -0.1321; params.p_hip(2,0) = 0.01;
        params.p_hip(0,1) = 0.183;  params.p_hip(1,1) = 0.1321;  params.p_hip(2,1) = 0.01;
        params.p_hip(0,2) = -0.183; params.p_hip(1,2) = -0.1321; params.p_hip(2,2) = 0.01;
        params.p_hip(0,3) = -0.183; params.p_hip(1,3) = 0.1321;  params.p_hip(2,3) = 0.01;

        
        params.J(0,0) = 0.01683993;  params.J(0,1) = 8.3902e-5;   params.J(0,2) = 0.000597679;
        params.J(1,0) = 8.3902e-5;   params.J(1,1) = 0.056579028; params.J(1,2) = 2.5134e-5;
        params.J(2,0) = 0.000597679; params.J(2,1) = 2.5134e-5;   params.J(2,2) = 0.064713601;

        params.dd(0) = -0.1;
        params.dd(1) = 0;
        params.dd(2) = 0.1;

        basePath = "tmp/";

        SX Qp = SX::vertcat({Parameters::a1_kinova_nmpc_params.Qpx, Parameters::a1_kinova_nmpc_params.Qpy, Parameters::a1_kinova_nmpc_params.Qpz});
        SX Qv = SX::vertcat({Parameters::a1_kinova_nmpc_params.Qvx, Parameters::a1_kinova_nmpc_params.Qvy, Parameters::a1_kinova_nmpc_params.Qvz});
        SX Qr = SX::vertcat({Parameters::a1_kinova_nmpc_params.Qrr, Parameters::a1_kinova_nmpc_params.Qrp, Parameters::a1_kinova_nmpc_params.Qry});
        SX Qw = SX::vertcat({Parameters::a1_kinova_nmpc_params.Qwr, Parameters::a1_kinova_nmpc_params.Qwp, Parameters::a1_kinova_nmpc_params.Qwy});

        // Combine into a block diagonal matrix Q
        params.Q_SRB = SX::vertcat({Qp, Qv, Qr, Qw}); // Corrected by adding braces to create a vector
        // writeMatrixToFile(params.Q_SRB, "Q_SRB.txt");

        //Terminal cost 
        params.P_SRB = SX::vertcat({Parameters::a1_kinova_nmpc_params.Pp*Qp, Parameters::a1_kinova_nmpc_params.Pv*Qv, Parameters::a1_kinova_nmpc_params.Pr*Qr, Parameters::a1_kinova_nmpc_params.Pw*Qw});
        // writeMatrixToFile(params.P_SRB, "P_SRB.txt");
       // Input cost component and R matrix
        SX R_tmp = SX::vertcat({1, 1, 1});
        params.R_SRB = SX::vertcat({Parameters::a1_kinova_nmpc_params.RFR*R_tmp,Parameters::a1_kinova_nmpc_params.RFL*R_tmp,Parameters::a1_kinova_nmpc_params.RRR*R_tmp,Parameters::a1_kinova_nmpc_params.RRL*R_tmp}); 
        // Corrected by adding braces to create a vector
        // writeMatrixToFile(params.R_SRB, "R_SRB.txt");
        //Kinova Arm
        //Stage cost
        SX Q_p_ab = SX::vertcat({Parameters::a1_kinova_nmpc_params.Qp_abx, Parameters::a1_kinova_nmpc_params.Qp_aby, Parameters::a1_kinova_nmpc_params.Qp_abz});
        SX Q_r_ab = SX::vertcat({Parameters::a1_kinova_nmpc_params.Qr_abx, Parameters::a1_kinova_nmpc_params.Qr_aby, Parameters::a1_kinova_nmpc_params.Qr_abz});
        SX Q_q_s = SX::vertcat({Parameters::a1_kinova_nmpc_params.Qq_s1, Parameters::a1_kinova_nmpc_params.Qq_s2, Parameters::a1_kinova_nmpc_params.Qq_s3, Parameters::a1_kinova_nmpc_params.Qq_s4});
    
        SX Q_dp_ab = SX::vertcat({Parameters::a1_kinova_nmpc_params.Qdp_abx, Parameters::a1_kinova_nmpc_params.Qdp_aby, Parameters::a1_kinova_nmpc_params.Qdp_abz});
        SX Q_omega_ab = SX::vertcat({Parameters::a1_kinova_nmpc_params.Qomega_abx, Parameters::a1_kinova_nmpc_params.Qomega_aby, Parameters::a1_kinova_nmpc_params.Qomega_abz});
        SX Q_dq_s = SX::vertcat({Parameters::a1_kinova_nmpc_params.Qdq_s1, Parameters::a1_kinova_nmpc_params.Qdq_s2, Parameters::a1_kinova_nmpc_params.Qdq_s3, Parameters::a1_kinova_nmpc_params.Qdq_s4});
  

        params.Q_arm = SX::vertcat({Q_p_ab, Q_r_ab, Q_q_s, Q_dp_ab, Q_omega_ab, Q_dq_s});
        // writeMatrixToFile(params.Q_arm, "Q_arm.txt");
        //Terminal cost
        params.P_arm = SX::vertcat({Parameters::a1_kinova_nmpc_params.Pp_ab*Q_p_ab, Parameters::a1_kinova_nmpc_params.Pr_ab*Q_r_ab, Parameters::a1_kinova_nmpc_params.Pq_s*Q_q_s, Parameters::a1_kinova_nmpc_params.Pdp_ab*Q_dp_ab, Parameters::a1_kinova_nmpc_params.Pomega_ab*Q_omega_ab, Parameters::a1_kinova_nmpc_params.Pdq_s*Q_dq_s});
        // writeMatrixToFile(params.P_arm, "P_arm.txt");
        // Input cost component and R matrix
        params.R_arm = SX::vertcat({Parameters::a1_kinova_nmpc_params.Ra1, Parameters::a1_kinova_nmpc_params.Ra2, Parameters::a1_kinova_nmpc_params.Ra3, Parameters::a1_kinova_nmpc_params.Ra4}); // Corrected by adding braces to create a vector
        // writeMatrixToFile(params.R_arm, "R_arm.txt");
        //Interaction wrench cost

        params.R_lambda = (SX::vertcat({Parameters::a1_kinova_nmpc_params.R_f1, Parameters::a1_kinova_nmpc_params.R_f2, Parameters::a1_kinova_nmpc_params.R_f3, Parameters::a1_kinova_nmpc_params.R_tau1, Parameters::a1_kinova_nmpc_params.R_tau2, Parameters::a1_kinova_nmpc_params.R_tau3})); // Corrected by adding braces to create a vector
        // writeMatrixToFile(params.R_lambda, "R_lambda.txt");
        params.solver_iter = Parameters::a1_kinova_nmpc_params.solver_iter;
        std::cout << "Solver iter: " << params.solver_iter << std::endl;

        //Set desired velocity
        params.desVel = {Parameters::motion_params.fwdSpeed, Parameters::motion_params.latSpeed,0};
        params.yawSpeed = Parameters::motion_params.yawSpeed;
        x0 = 0;
        y0 = 0;
        z0 = 0.055;
        xf = 0;
        yf = 0;
        zf = params.height;
        yaw0 = 0;
        

        /* // Initialize the joystick
        Joystick joystick("/dev/input/js0");
          // Ensure that it was found and that we can use it
        if (!joystick.isFound())
        {
            printf("open failed.\n");
        } */


    };
    ~NMPC_A1_KINOVA(){};

    casadi::DM bezier(const DM& afra, double s);
    casadi::DM bezierd(const DM& alpha, double s);

    casadi::SX Rotz(const casadi::SX& t);
    casadi::SX RotMat_XYZ(const casadi::SX& rpy);
    casadi::SX RotMat(const casadi::SX& t);
    casadi::SX skewSym(const casadi::SX& x);Eigen::MatrixXd skewSym(const Eigen::MatrixXd& x);
    
    casadi::SX rot_x(const casadi::SX& t);
    casadi::SX rot_y(const casadi::SX& t);
    casadi::SX rot_z(const casadi::SX& t);
    casadi::SX RotMat_ZYX(const casadi::SX& rpy);
    casadi::DM RotMat_ZYX(const casadi::DM& rpy);
    casadi::DM RotMat_XYZ(const casadi::DM& rpy);
    casadi::SX A_Theta_ZYX(const casadi::SX& rpy);
    casadi::SX A_Theta_XYZ(const casadi::SX& rpy);
    casadi::DM A_Theta_XYZ(const casadi::DM& rpy);
    casadi::DM transform_ZYX_to_XYZ(const casadi::DM& zyx_input);
    Eigen::Matrix<double,3,3> RotMat_XYZ(const Eigen::Matrix<double,3,1>& rpy);
    Eigen::Matrix<double,3,3> A_Theta_ZYX(const Eigen::Matrix<double,3,1>& rpy);
    Eigen::Matrix<double,3,3> A_Theta_XYZ(const Eigen::Matrix<double,3,1>& rpy);
    Eigen::Matrix<double,3,3> Ainv_Theta_XYZ(const Eigen::Matrix<double,3,1>& rpy);
   

    void get_Euler_XYZ_from_ZYX( casadi::DM& rpy_xyz, const casadi::DM& rpy_zyx);
    void get_Omega_XYZ_body_from_ZYX_world( casadi::DM& omega_xyz_body, const casadi::DM& omega_zyx_world, const casadi::DM& rpy_zyx);
    void get_Omega_XYZ_world_from_ZYX_world( casadi::DM& omega_xyz_world, const casadi::DM& omega_zyx_world, const casadi::DM& rpy_zyx);
    void get_Eulerrate_XYZ_world_from_ZYX_world( casadi::DM& omega_xyz_world, const casadi::DM& omega_zyx_world, const casadi::DM& rpy_xyz);
    void get_Eulerrate_XYZ_body_from_ZYX_world( casadi::DM& omega_xyz_world, const casadi::DM& omega_zyx_world, const casadi::DM& rpy_xyz);
    void get_Euler_ZYX_from_XYZ(Eigen::Matrix<double,3,1>& rpy_zyx, const Eigen::Matrix<double,3,1>& rpy_xyz);
    void get_Omega_ZYX_world_from_XYZ_body(Eigen::Matrix<double,3,1>& omega_zyx_world, const Eigen::Matrix<double,3,1>& omega_xyz_body, const Eigen::Matrix<double,3,1>& rpy_xyz);
    void get_Omega_ZYX_world_from_XYZ_world(Eigen::Matrix<double,3,1>& omega_zyx_world, const Eigen::Matrix<double,3,1>& omega_xyz_world, const Eigen::Matrix<double,3,1>& rpy_xyz);
    void get_Omega_ZYX_world_from_Eulerrate_XYZ_world(Eigen::Matrix<double,3,1>& omega_zyx_world, const Eigen::Matrix<double,3,1>& omega_xyz_world, const Eigen::Matrix<double,3,1>& rpy_xyz);
    void get_Omega_ZYX_world_from_Eulerrate_XYZ_body(Eigen::Matrix<double,3,1>& omega_zyx_world, const Eigen::Matrix<double,3,1>& omega_xyz_world, const Eigen::Matrix<double,3,1>& rpy_xyz);
    void transform_XYZ_to_ZYX(double *q, double *dq);
    void planner();
    void force_inequality(const casadi::SX& U, casadi::SX& g_force);
    void writeMatrixToFile(const casadi::SX& matrix, const std::string& filename);
    void appendMatrixToFile(const casadi::DM& matrix, const std::string& filename);
    void writeMatrixToFile(const casadi::DM& matrix, const std::string& filename);
    void writeMatrixToFile(const std::string& filename);
    void writeMatrixToFile(const Eigen::MatrixXd &matrix, const std::string &filename);
    void equalityconstraints(const casadi::SX& X, const casadi::SX& U, const casadi::SX& P, casadi::SX& obj, casadi::SX& g_eq, const Function& f_SRB, const Function& f_arm);
    void set_constraint_bounds();
    void set_statebounds();
    void update_statebounds();
    void force_inequality_bounds();
    void shift();
    int binomial(int n, int k);
    int binomialCoeff(int n, int k);
    void fitBezierTrajectory();
    void casadiToEigen(const casadi::DM& mat1, Eigen::MatrixXd& mat2);
    void updateState(double* qin, double* dqin, double *qin_arm, double *dqin_arm , int* ind, Eigen::Matrix<double, 3, 4> toePos, Eigen::Vector4d state_vec, const int domain, const int control_tick);
    void updateReference(float* joystickstate);
    void generateReferenceTrajectory();
    void writeData();


    void setPstart(Eigen::Matrix<double, 2*NUMBER_OF_AGENTS, 1>& Pstart);
    void setPobs(Eigen::Matrix<double, 2,NUMBER_OF_OBS>& Pobs);
    void setPobs_real(Eigen::Matrix<double, 2,NUMBER_OF_OBS>& Pobs);
    void setAgentID(size_t agent_id);

    void code_gen();
    void run_NMPC();
    void init();
    void solve_nlp();
    void logData();

    void jointevolution(double twave);
    
    inline std::vector<double> get_com_pos() {
        casadi::DM first_three = qk_(casadi::Slice(0, 3));  // Extract first three elements
        return std::vector<double>(first_three->begin(), first_three->end());
    }

    // Modify this function to return std::vector<double> for uk_
    inline std::vector<double> get_GRF() {
        return std::vector<double>(ukSRB_->begin(), ukSRB_->end());  // Convert the whole uk_ to std::vector
    }

    inline std::vector<double> get_arm_input_torques() {
        return std::vector<double>(ukArm_->begin(), ukArm_->end());
    }

    inline std::vector<double> get_arm_input_torques_PD() {
        return std::vector<double>(ukArmPD_->begin(), ukArmPD_->end());
    }



    inline std::vector<double> get_contacts() {
        return std::vector<double>(params.contact->begin(), params.contact->end());  // Convert the whole uk_ to std::vector
    }

    inline std::vector<double> get_reference_velocity() {
        casadi::DM first_three = q_des_(casadi::Slice(3, 6));  // Extract first three elements
        return std::vector<double>(first_three->begin(), first_three->end());
    }


    inline std::vector<double> get_Pf() {
        casadi::DM Pf = casadi::DM::vertcat({
            params.pf(Slice(), 0), // First column
            params.pf(Slice(), 1), // Second column
            params.pf(Slice(), 2), // Third column
            params.pf(Slice(), 3)  // Fourth column
        });
        return std::vector<double>(Pf->begin(), Pf->end());  // Convert the whole uk_ to std::vector
    }

    inline Eigen::MatrixXd get_alphaCOM() {return bez_alpha_eigen_;}
    inline Eigen::MatrixXd get_MPCsol() {return x_sol_eigen_;}
    inline Eigen::MatrixXd get_alphaGRF() {return bez_alpha_GRF_eigen_;}
    inline Eigen::MatrixXd get_MPCsolGRF() {return ukAllSRB_eigen_;}
    inline bool get_gait() {return joystick_send_gait_;}
    inline bool get_reduce_swing_height() {return reduce_swing_height_;}
};
#endif // NMPC_A1_Kinova_HPP