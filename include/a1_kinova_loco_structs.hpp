#ifndef A1_KINOVA_LOCO_STRUCTS_H
#define A1_KINOVA_LOCO_STRUCTS_H

#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <filesystem>


#define SIM_TIME 40.0
#define HORIZON 9
#define GOAL_X 10.0
#define GOAL_Y 0.0
#define NUMBER_OF_AGENTS 1
#define NUMBER_OF_OBS 10

using namespace casadi;
namespace fs = std::filesystem;

SX blkdiag(const std::vector<SX>& matrices);

struct Params {
    size_t mpc_iter = 0; // Iteration counter
    double sim_length = 10.0; // Default simulation time
    double current_time = 0.0; // Initialize to zero
    double dt; // Sample time
    double Fs; // Control Frequency, initialized in constructor
    double mu = 0.6; // Friction Coefficient
    size_t L; // Simulation length, initialized in constructor
    size_t N = HORIZON; // Horizon length
    double height = 0.28; // Stand height
    double zmax = 250; //Maximum torque
    DM arm_pos = {0.0, 0.9, 2.4, 0.0}; ; // Position vector of the arm's base w.r.t the A1 body frame

    DM dd = DM::zeros(3,1); // Arm base position relative to SRB COM

    size_t n_SRB = 12; // Number of SRB states
    size_t n_arm = 20; // Number of arm states
    size_t n_states = n_SRB + n_arm; // Number of states
    size_t n_u_SRB = 12; // Number of SRB inputs
    size_t n_u_arm = 4; // Number of arm inputs
    size_t n_u_lambda_int = 6; // Number of interaction force inputs
    size_t n_inputs = n_u_SRB + n_u_lambda_int + n_u_arm ; // Number of inputs (SRB torques, contact forces, and arm torques)
    size_t n_controls =n_u_SRB + n_u_lambda_int + n_u_arm; //What is this?
    size_t NumHolCon = 1*6; // Number of holonomic constraints: refer to the function GetHolonomicConstraints
    size_t gait = 8; // Trot


    double G = 9.81; // Gravity
    double mass = 12.4530; // Mass
    double mass_arm = 3.8; // Mass of the arm

    double Kp = 400.0; // Proportional gain
    double Kd = 40.0; // Derivative gain

    DM J = DM::zeros(3,3); // Assuming a 3x3 inertia matrix

    // Nominal hip position for most gaits
    DM pf = DM::zeros(3, 4); 
    DM p_hip = DM::zeros(3, 4);

    Eigen::MatrixXd Pr_refined_; // To be initialized as needed
    Eigen::MatrixXd Prd_refined_; // To be initialized as needed
    size_t agentIndex = 1;

    DM contact = DM::vertcat({1, 1, 1, 1});
    DM desVel = DM::vertcat({0.0, 0});
    DM yawSpeed = 0.0; // Yaw rate

    SX Q_SRB; // SRB State cost matrix
    SX Q_arm; // Kinova State cost matrix
    SX P_SRB; // SRB Terminal cost matrix
    SX P_arm; // Kinova Terminal cost matrix
    SX R_lambda; // Interaction wrench cost matrix
    SX R_SRB; // SRB Input cost matrix
    SX R_arm; // Kinova Input cost matrix

    size_t stepcnt = 0;
    int solver_iter = 10;

    // Constructor to initialize members that depend on dt
    explicit Params() {
        
        //SRB 
        //Stage cost
      /*   SX Qp = diag(SX::vertcat({1e6, 1e6, 8e6}));
        SX Qv = diag(SX::vertcat({1e3, 1e3, 1e3}));
        SX Qr = diag(SX::vertcat({8e8, 8e10, 3e8}));
        SX Qw = diag(SX::vertcat({8e8, 8e10, 8e8})); */
        SX Qp =  (SX::vertcat({1e6, 1e6, 8e6}));//diag(SX::vertcat({1e7, 1e8, 8e8}));
        SX Qv = (SX::vertcat({1e3, 1e3, 1e3}));
        SX Qr = (SX::vertcat({8e4, 8e4, 3e3}));
        SX Qw = SX::vertcat({1e0, 1e0, 1e0});

        // Combine into a block diagonal matrix Q
        Q_SRB = SX::vertcat({Qp, Qv, Qr, Qw}); // Corrected by adding braces to create a vector

       /*  //Terminal cost 
        SX Pp = 1e1*Qp;
        SX Pv = 1e1*Qv;
        SX Pr = 1e1*Qr;
        SX Pw = 1e1*Qw;
        P = blkdiag({Pp, Pv, Pr, Pw}); */


       // Input cost component and R matrix
        SX R_tmp = SX::vertcat({1e-2, 1e-2, 1e-2});
        R_SRB = SX::vertcat({R_tmp, R_tmp, R_tmp, R_tmp}); // Corrected by adding braces to create a vector

        //Kinova Arm
        //Stage cost
        SX Q_p_ab = (SX::vertcat({1e6,1e6,8e6}));
        SX Q_r_ab = (SX::vertcat({(8e5),(8e5),3e5}));
        SX Q_q_s = (SX::vertcat({8e6,8e6,8e6,3e6}));

        SX Q_dp_ab = (SX::vertcat({1e6,1e6,8e6}));
        SX Q_omega_ab = (SX::vertcat({8e5,8e5,3e5}));
        SX Q_dq_s = (SX::vertcat({8e6,8e6,8e3,8e3}));

        Q_arm = SX::vertcat({Q_p_ab, Q_r_ab, Q_q_s, Q_dp_ab, Q_omega_ab, Q_dq_s});

        SX R_tmp_arm = SX::vertcat({1e-3});
        R_arm = SX::vertcat({R_tmp_arm, R_tmp_arm, R_tmp_arm, R_tmp_arm});

        //Interaction wrench cost
        SX R_f1 = 0.01; SX R_f2 = 0.01; SX R_f3 = 0.01;
        SX R_tau1 = 0.1; SX R_tau2 = 0.1; SX R_tau3 = 0.1;

        R_lambda = (SX::vertcat({R_f1,R_f2,R_f3,R_tau1,R_tau2,R_tau3}));   
        
        // Initialize any other members that require dynamic calculation here
    }
};

struct Args {
    DM p, x0, lbx, ubx, lbg, ubg; 
};

#endif // A1_KINOVA_LOCO_STRUCTS_H