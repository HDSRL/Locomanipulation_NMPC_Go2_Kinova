#ifndef NMPC_C
#define NMPC_C

#include "loco_structs.hpp"
#include <chrono>
#include "iostream"
#include "fstream"
#include "Parameters.hpp"

using namespace casadi;
namespace fs = std::filesystem;

// #define NUMBER_OF_AGENTS 1
// #define NUMBER_OF_OBS 9

class NMPC : public Parameters 
{

private:
    
    Args args;     // Arguments for the solver
    Function f;
    double curr_time;
    Function f_dyn_;
    Function solver_;
    std::string basePath;
    bool isDomainChanged_ = false;

    // NMPC variables
    DM qk_; // Current state (qk) in the other code
    DM qkAll_; // All states for N horizons (X0) in the other code
    int domain_ = 0;
    int control_tick_ = 0;

    DM q_des_; // Desired state (xd) in the other code
    DM uk_; // Current control input - n_inputs x 1 
    DM ukAll_; // All control inputs for N horizons (u0) in the other code
    DM x_sol_; // Current state (xk) in the other code
    DM x_sol_N_; // solution for all states in shape n_states x N

    DM bez_alpha_; // Bezier coefficients
    DM x_trial; // Trial desired Trajectory

    // Logger variables
    DM uout;
    DM xout;
    DM feet;
    DM traj;
    DM tout;
    DM NMPC_solve_time;

    // From low-level controller
    double q[18] = {0};
    double dq[18] = {0};
    Eigen::Vector4d state_other; 
    int contactInd[4] = {1};
    Eigen::Matrix<double, 3, 4> toePos_;

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

public:
    Params params;

    NMPC(int argc, char *argv[]) : Parameters(argc,argv)
    {
        // Initialize the parameters
        params.mpc_iter = 0;
        params.current_time = 0.0;
        params.dt = 0.01;
        params.Fs = 1/params.dt;
        params.sim_length = SIM_TIME;
        // params.L = 1000; // not sure if this is needed
        params.N = HORIZON;   // Horizon length
        params.height = 0.28;   // stand height
        params.n_states = 12;   // states
        params.n_inputs = 12;   // control inputs
        params.n_controls = 12; // control inputs
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

        basePath = "tmp/";
        x_trial = DM::zeros(params.N, params.n_states);

        //Set desired velocity
        params.desVel = {Parameters::motion_params.fwdSpeed, Parameters::motion_params.latSpeed,0};
        params.yawSpeed = Parameters::motion_params.yawSpeed;
        SX Qp = SX::vertcat({Parameters::a1_nmpc_params.Qpx, Parameters::a1_nmpc_params.Qpy, Parameters::a1_nmpc_params.Qpz});
        SX Qv = SX::vertcat({Parameters::a1_nmpc_params.Qvx, Parameters::a1_nmpc_params.Qvy, Parameters::a1_nmpc_params.Qvz});
        SX Qr = SX::vertcat({Parameters::a1_nmpc_params.Qrr, Parameters::a1_nmpc_params.Qrp, Parameters::a1_nmpc_params.Qry});
        SX Qw = SX::vertcat({Parameters::a1_nmpc_params.Qwr, Parameters::a1_nmpc_params.Qwp, Parameters::a1_nmpc_params.Qwy});

        params.Q = SX::vertcat({Qp, Qv, Qr, Qw});

        //Terminal cost 
        params.P = SX::vertcat({Parameters::a1_kinova_nmpc_params.Pp*Qp, Parameters::a1_kinova_nmpc_params.Pv*Qv, Parameters::a1_kinova_nmpc_params.Pr*Qr, Parameters::a1_kinova_nmpc_params.Pw*Qw});
        SX R_tmp = SX::vertcat({1, 1, 1});
        params.R = SX::vertcat({Parameters::a1_kinova_nmpc_params.RFR*R_tmp,Parameters::a1_kinova_nmpc_params.RFL*R_tmp,Parameters::a1_kinova_nmpc_params.RRR*R_tmp,Parameters::a1_kinova_nmpc_params.RRL*R_tmp}); 
        


    };
~NMPC(){};

    casadi::DM bezier(const DM& afra, double s);
    casadi::DM bezierd(const DM& alpha, double s);

    casadi::SX Rotz(const casadi::SX& t);
    casadi::SX rot_x(const casadi::SX& t);
    casadi::SX rot_y(const casadi::SX& t);
    casadi::SX rot_z(const casadi::SX& t);
    casadi::SX RotMat_XYZ(const casadi::SX& rpy);
    casadi::DM RotMat_XYZ(const casadi::DM& rpy);
    casadi::SX skewSym(const casadi::SX& x);
    casadi::SX RotMat_ZYX(const casadi::SX& rpy);
    casadi::DM RotMat_ZYX(const casadi::DM& rpy);
    casadi::SX A_Theta_ZYX(const casadi::SX& rpy);
    casadi::DM A_Theta_ZYX(const casadi::DM& rpy);
    casadi::SX A_Theta_XYZ(const casadi::SX& rpy);
    casadi::DM A_Theta_XYZ(const casadi::DM& rpy);
    casadi::DM transform_ZYX_to_XYZ(const casadi::DM& zyx_input);
    Eigen::Matrix<double,3,3> RotMat_XYZ(const Eigen::Matrix<double,3,1>& rpy);
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
    void writeMatrixToFile(const casadi::DM& matrix, const std::string& filename);
    void writeMatrixToFile(const std::string& filename);
    void appendMatrixToFile(const casadi::DM& matrix, const std::string& filename);
    void equalityconstraints(const casadi::SX& X, const casadi::SX& U, const casadi::SX& P, casadi::SX& obj, casadi::SX& g_eq, const Function& f);
    void set_constraint_bounds();
    void set_statebounds();
    void force_inequality_bounds();
    void shift();
    int binomial(int n, int k);
    int binomialCoeff(int n, int k);
    void fitBezierTrajectory();
    void casadiToEigen(const casadi::DM& mat1, Eigen::MatrixXd& mat2);
    void updateState(double* qin, double* dqin, int* ind, Eigen::Matrix<double, 3, 4> toePos, Eigen::Vector4d state_vec, const int domain, const int control_tick);
    void generateReferenceTrajectory();

    void setPstart(Eigen::Matrix<double, 2*NUMBER_OF_AGENTS, 1>& Pstart);
    void setPobs(Eigen::Matrix<double, 2,NUMBER_OF_OBS>& Pobs);
    void setPobs_real(Eigen::Matrix<double, 2,NUMBER_OF_OBS>& Pobs);
    void setAgentID(size_t agent_id);

    void code_gen();
    void run_NMPC();
    void init();
    void solve_nlp();
    void logData();
    void writeData();
    inline std::vector<double> get_com_pos() {
        casadi::DM first_three = qk_(casadi::Slice(0, 3));  // Extract first three elements
        return std::vector<double>(first_three->begin(), first_three->end());
    }

    // Modify this function to return std::vector<double> for uk_
    inline std::vector<double> get_GRF() {
        return std::vector<double>(uk_->begin(), uk_->end());  // Convert the whole uk_ to std::vector
    }

    inline std::vector<double> get_contacts() {
        return std::vector<double>(params.contact->begin(), params.contact->end());  // Convert the whole uk_ to std::vector
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
};
#endif // NMPC_H