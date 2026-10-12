#ifndef NMPC_A1_Kinova
#define NMPC_P_A1_Kinova

#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include <cmath>
#include <fstream>
#include <iostream>

using namespace casadi;

SX blkdiag(const std::vector<SX>& matrices);
void writeMatrixToFile(const casadi::DM& matrix, const std::string& filename);

struct Params {
    size_t mpc_iter = 0; // Iteration counter
    double current_time = 0.0; // Initialize to zero
    double dt; // Sample time
    double Fs; // Control Frequency, initialized in constructor
    double mu = 0.4; // Friction Coefficient
    size_t L; // Simulation length, initialized in constructor
    size_t N; // Horizon length
    double height = 0.28; // Stand height
    double zmax = 250; //Maximum torque

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
    double mass = 18.00;//15.4530; // Mass
    double mass_arm = 5.5;//3.86; // Arm mass

    double Kp = 0;//0*900.0; // Proportional gain
    double Kd = 0;//0*60.0; // Derivative gain

    DM J = DM::zeros(3,3); // Assuming a 3x3 inertia matrix

    // Nominal hip position for most gaits
    DM pf = DM::zeros(3, 4); 
    DM p_hip = DM::zeros(3, 4);

    Eigen::MatrixXd Pr_refined_; // To be initialized as needed
    Eigen::MatrixXd Prd_refined_; // To be initialized as needed
    size_t agentIndex = 1;

    DM contact = DM::vertcat({1, 1, 1, 1});
    DM desVel = DM::vertcat({1.0, 0});

    SX Q_SRB; // SRB State cost matrix
    SX Q_arm; // Kinova State cost matrix
    SX R_lambda; // Interaction wrench cost matrix
    SX R_SRB; // SRB Input cost matrix
    SX R_arm; // Kinova Input cost matrix
    SX P_SRB; // Terminal cost matrix
    SX P_arm; // Terminal cost matrix


    size_t stepcnt = 0;

    // Constructor to initialize members that depend on dt
    explicit Params(double sampleTime) : dt(sampleTime), Fs(1.0 / dt), L(static_cast<int>(20 * Fs)), N(9) {
        // Initialize the pf matrix within the constructor body
        pf(0,0) = 0.183;  pf(1,0) = -0.1321; pf(2,0) = 0.01;
        pf(0,1) = 0.183;  pf(1,1) = 0.1321;  pf(2,1) = 0.01;
        pf(0,2) = -0.183; pf(1,2) = -0.1321; pf(2,2) = 0.01;
        pf(0,3) = -0.183; pf(1,3) = 0.1321;  pf(2,3) = 0.01;

        p_hip(0,0) = 0.183;  p_hip(1,0) = -0.1321; p_hip(2,0) = 0.01;
        p_hip(0,1) = 0.183;  p_hip(1,1) = 0.1321;  p_hip(2,1) = 0.01;
        p_hip(0,2) = -0.183; p_hip(1,2) = -0.1321; p_hip(2,2) = 0.01;
        p_hip(0,3) = -0.183; p_hip(1,3) = 0.1321;  p_hip(2,3) = 0.01;

        
        /* J(0,0) = 0.01683993; J(0,1) = 8.3902e-5; J(0,2) = 0.000597679;
        J(1,0) = 8.3902e-5; J(1,1) = 0.056579028; J(1,2) = 2.5134e-5;
        J(2,0) = 0.000597679; J(2,1) = 2.5134e-5; J(2,2) = 0.064713601; */

        //Go2
        J(0,0) = 0.02448; J(0,1) = 0.00012166; J(0,2) = 0.0014849;
        J(1,0) = 0.00012166; J(1,1) = 0.098077; J(1,2) = -3.12e-05;
        J(2,0) = 0.0014849; J(2,1) = -3.12e-05; J(2,2) = 0.107;

        dd(0) = -0.1;
        dd(1) = 0;
        dd(2) = 0.1;

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

        //Terminal cost 
        SX Pp = 1e1*Qp;
        SX Pv = 1e1*Qv;
        SX Pr = 1e1*Qr;
        SX Pw = 1e1*Qw;
        P_SRB = SX::vertcat({Pp, Pv, Pr, Pw});


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

        //Terminal cost
        SX P_p_ab = 1e1*Q_p_ab;
        SX P_r_ab = 1e1*Q_r_ab;
        SX P_q_s = 1e1*Q_q_s;
        SX P_dp_ab = 1e1*Q_dp_ab;
        SX P_omega_ab = 1e1*Q_omega_ab;
        SX P_dq_s = 1e1*Q_dq_s;

        P_arm = SX::vertcat({P_p_ab, P_r_ab, P_q_s, P_dp_ab, P_omega_ab, P_dq_s});

        SX R_tmp_arm = SX::vertcat({1e-3});
        R_arm = SX::vertcat({R_tmp_arm, R_tmp_arm, R_tmp_arm, R_tmp_arm});

        //Interaction wrench cost
        SX R_f1 = 0.01; SX R_f2 = 0.01; SX R_f3 = 0.01;
        SX R_tau1 = 0.1; SX R_tau2 = 0.1; SX R_tau3 = 0.1;

        R_lambda = (SX::vertcat({R_f1,R_f2,R_f3,R_tau1,R_tau2,R_tau3}));   
        
        // Initialize any other members that require dynamic calculation here
    }
};

SX blkdiag(const std::vector<SX>& matrices) {
    if (matrices.empty()) return SX();

    // Start with the first matrix
    SX result = matrices[0];
    for (size_t i = 1; i < matrices.size(); ++i) {
        // Pad the existing result and the next matrix with zeros to make them block diagonal
        SX topRight = SX::zeros(result.size1(), matrices[i].size2());
        SX bottomLeft = SX::zeros(matrices[i].size1(), result.size2());

        // Concatenate to form the new result
        result = vertcat(horzcat(result, topRight), 
                         horzcat(bottomLeft, matrices[i]));
    }

    return result;
}

struct Args {
    DM p, x0, lbx, ubx, lbg, ubg; 
};

DM bezier(const DM& afra, double s) {
    int n = afra.size1(); // Number of rows
    int m = afra.size2(); // Number of columns
    DM value = DM::zeros(n);
    int M = m - 1;

    std::vector<int> k; // Binomial coefficients
    switch (M) {
        case 3: k = {1, 3, 3, 1}; break;
        case 4: k = {1, 4, 6, 4, 1}; break;
        case 5: k = {1, 5, 10, 10, 5, 1}; break;
        case 6: k = {1, 6, 15, 20, 15, 6, 1}; break;
        case 7: k = {1, 7, 21, 35, 35, 21, 7, 1}; break;
        case 8: k = {1, 8, 28, 56, 70, 56, 28, 8, 1}; break;
        case 9: k = {1, 9, 36, 84, 126, 126, 84, 36, 9, 1}; break;
        case 10: k = {1, 10, 45, 120, 210, 252, 210, 120, 45, 10, 1}; break;
        case 20: k = {1, 20, 190, 1140, 4845, 15504, 38760, 77520, 125970, 167960, 184756, 167960, 125970, 77520, 38760, 15504, 4845, 1140, 190, 20, 1}; break;
        default: std::cerr << "Degree " << M << " not supported." << std::endl; return value; // Optionally handle other degrees dynamically
    }

    std::vector<double> x(M + 1, 1.0), y(M + 1, 1.0);
    for (int i = 1; i <= M; ++i) {
        x[i] = s * x[i - 1];
        y[i] = (1 - s) * y[i - 1];
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j <= M; ++j) {
            value(i) += afra(i, j) * k[j] * x[j] * y[M - j];
        }
    }

    return value;
}

DM bezierd(const DM& alpha, double s) {
    int n = alpha.size1(); // Number of rows
    int m = alpha.size2(); // Number of columns
    DM value = DM::zeros(n);
    int M = m - 1; // Degree of the Bezier curve

    std::vector<int> k; // Binomial coefficients for the derivative
    switch (M) {
        case 3: k = {3, 6, 3}; break;
        case 4: k = {4, 12, 12, 4}; break;
        case 5: k = {5, 20, 30, 20, 5}; break;
        case 6: k = {6, 30, 60, 60, 30, 6}; break;
        case 7: k = {7, 42, 105, 140, 105, 42, 7}; break;
        case 8: k = {8, 56, 168, 280, 280, 168, 56, 8}; break;
        case 9: k = {9, 72, 252, 504, 630, 504, 252, 72, 9}; break;
        case 10: k = {10, 45, 120, 210, 252, 210, 120, 45, 10, 1}; break;
        case 20: k = {20, 380, 3420, 19380, 77520, 232560, 542640, 1007760, 1511640, 1847560, 1847560, 1511640, 1007760, 542640, 232560, 77520, 19380, 3420, 380, 20}; break;
        default: std::cerr << "Degree " << M << " not supported for derivative calculation." << std::endl; return value;
    }

    std::vector<double> x(M, 1.0), y(M, 1.0); // Adjust for derivative size
    for (int i = 1; i < M; ++i) {
        x[i] = s * x[i - 1];
        y[i] = (1 - s) * y[i - 1];
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < M; ++j) {
            value(i) += (alpha(i, j + 1) - alpha(i, j)) * k[j] * x[j] * y[M - 1 - j];
        }
    }

    return value;
}

casadi::SX rot_x(const casadi::SX& t) {
    // Ensure t is a vector of size 1
    if (t.size1() != 1 || t.size2() != 1) {
        throw std::runtime_error("Input vector t must be of size 1.");
    }
    casadi::SX Rx = casadi::SX::vertcat({casadi::SX::horzcat({1, 0, 0}), casadi::SX::horzcat({0, cos(t), -sin(t)}), casadi::SX::horzcat({0, sin(t), cos(t)})});
    return Rx;
}

casadi::SX rot_y(const casadi::SX& t){
    // Ensure t is a vector of size 1
    if (t.size1() != 1 || t.size2() != 1) {
        throw std::runtime_error("Input vector t must be of size 1.");
    }

    casadi::SX Ry =  casadi::SX::vertcat({casadi::SX::horzcat({cos(t), 0, sin(t)}), casadi::SX::horzcat({0, 1, 0}), casadi::SX::horzcat({-sin(t), 0, cos(t)})});
   
    return Ry;

}

casadi::SX rot_z(const casadi::SX& t){
    // Ensure t is a vector of size 1
    if (t.size1() != 1 || t.size2() != 1) {
        throw std::runtime_error("Input vector t must be of size 1.");
    }

    casadi::SX Rz =  casadi::SX::vertcat({casadi::SX::horzcat({cos(t), -sin(t), 0}), casadi::SX::horzcat({sin(t), cos(t), 0}), casadi::SX::horzcat({0, 0, 1})});
   
    return Rz;

}

SX Rotz(const SX& t) {
    // Ensure t is a vector of size 3
    if (t.size1() != 3 || t.size2() != 1) {
        throw std::runtime_error("Input vector t must be of size 3.");
    }

    casadi::SX Rz = casadi::SX::zeros(3, 3);

    // Note: In CasADi, indexing is zero-based, similar to C++ and Eigen but different from MATLAB.
    Rz(0, 0) = cos(t(1)) * cos(t(2));
    Rz(0, 1) = -sin(t(2));
    Rz(0, 2) = 0;
    Rz(1, 0) = cos(t(1)) * sin(t(2));
    Rz(1, 1) = cos(t(2));
    Rz(1, 2) = 0;
    Rz(2, 0) = -sin(t(1));
    Rz(2, 1) = 0;
    Rz(2, 2) = 1;

    return Rz;
}

casadi::SX RotMat(const casadi::SX& rpy) {
    //ZYX convention Rz*Ry*Rx
    // Ensure rpy is a vector of size 3
    if (rpy.size1() != 3 || rpy.size2() != 1) {
        throw std::runtime_error("Input vector rpy must be of size 3.");
    }
    /* phi = rpy(0);
       theta = rpy(1);
       psi = rpy(2) */

    casadi::SX R = casadi::SX::zeros(3, 3);

    // Note: In CasADi, indexing is zero-based, similar to C++ and Eigen but different from MATLAB.
    R(0, 0) = cos(rpy(1)) * cos(rpy(2));
    R(0, 1) = cos(rpy(2))*sin(rpy(0))*sin(rpy(1)) - cos(rpy(0))*sin(rpy(2));
    R(0, 2) = sin(rpy(0))*sin(rpy(2)) + cos(rpy(0))*cos(rpy(2))*sin(rpy(1));
    R(1, 0) = cos(rpy(1)) * sin(rpy(2));
    R(1, 1) = cos(rpy(0))*cos(rpy(2)) + sin(rpy(0))*sin(rpy(2))*sin(rpy(1));
    R(1, 2) = cos(rpy(0))*sin(rpy(2))*sin(rpy(1)) - cos(rpy(2))*sin(rpy(0));
    R(2, 0) = -sin(rpy(1));
    R(2, 1) = cos(rpy(1))*sin(rpy(0));
    R(2, 2) = cos(rpy(1))*cos(rpy(0));

    return R;
}

casadi::SX A_Theta(const casadi::SX& rpy){
    //ZYX convention Rz*Ry*Rx
    // Ensure rpy is a vector of size 3
    if (rpy.size1() != 3 || rpy.size2() != 1) {
        throw std::runtime_error("Input vector rpy must be of size 3.");
    }
     /* phi = rpy(0);
       theta = rpy(1);
       psi = rpy(2) */
    casadi::SX A = casadi::SX::zeros(3, 3);
    A(0,0) = cos(rpy(2))/cos(rpy(1));
    A(0,1) = sin(rpy(2))/cos(rpy(1));
    A(0,2) = 0;
    A(1,0) = -sin(rpy(2));
    A(1,1) = cos(rpy(2));
    A(1,2) = 0;
    A(2,0) = cos(rpy(2))*tan(rpy(1));
    A(2,1) = sin(rpy(2))*tan(rpy(1));
    A(2,2) = 1;

    return A;

}



casadi::SX skewSym(const casadi::SX& x) {
    using namespace casadi;

    // Determine the size based on the input x
    int cols = x.size2();
    SX mat = SX::zeros(3, 3 * cols);

    // Loop through each column of x to create skew-symmetric matrices
    for (int n = 0; n < cols; ++n) {
        SX slice = x(Slice(), n);  // Get the nth column of x
        SX skew = SX::zeros(3, 3);
        // Fill the skew-symmetric matrix
        skew(0, 1) = -slice(2);
        skew(0, 2) = slice(1);
        skew(1, 0) = slice(2);
        skew(1, 2) = -slice(0);
        skew(2, 0) = -slice(1);
        skew(2, 1) = slice(0);

        // Place the skew matrix into the correct position in mat
        mat(Slice(0, 3), Slice(3*n, 3*(n+1))) = skew;
    }

    return mat;
}


void planner(double t, const DM& q, DM& q_des, int mode, Params& params) {
    int n_states = params.n_states; // Example: Number of states A1 + Kinova
    int n_controls = params.n_controls; // Example: Number of controls A1 + Kinova
    int n_u_SRB = params.n_u_SRB; // Number of SRB controls
    // const int N = params.N;
    DM xd = DM::zeros(n_states * params.N + n_u_SRB, 1); // Initialize the state and control vector

    if (t <= 1.0) { // Stand up duration
        mode = 1;
    }

    DM traj = q_des(casadi::Slice(0, n_states), 0); // Second parameter 0 is to indicate column index for a vector


    switch (mode) {
        case 1: {
            double t0 = 0, tf = 1.0; // Start and end times for mode 1
            double s = std::min((t - t0) / (tf - t0), 1.0); // Ensure s is within [0, 1]
            
            // Extract relevant parameters for easy access
            double xf = 0.0; // Initial x position
            double yf = 0.0; // Initial y position
            double zf = params.height; // Assuming the height is constant or predefined
            double z0 = 0.08;

            // Using DM::horzcat to create each row of the alpha matrix
            casadi::DM row1 = casadi::DM::horzcat({DM(0), DM(0), DM(0), DM(xf / 4), DM(3 * xf / 4), DM(xf), DM(xf), DM(xf)});
            casadi::DM row2 = casadi::DM::horzcat({DM(0), DM(0), DM(0), DM(yf / 4), DM(3 * yf / 4), DM(yf), DM(yf), DM(yf)});
            casadi::DM row3 = casadi::DM::horzcat({DM(z0), DM(z0), DM(z0), DM(zf / 4), DM(3 * zf / 4), DM(zf), DM(zf), DM(zf)});

            casadi::DM alpha = casadi::DM::vertcat({row1, row2, row3});

            // Update trajectory position
            casadi::DM position_update = bezier(alpha, s);

            for (int i = 0; i < 3; ++i) {
                traj(i) = position_update(i);
                traj(i + 12) = position_update(i) + params.dd(i); // Update arm base position
            }

            // Update trajectory velocity
            casadi::DM velocity_update = bezierd(alpha, s);

            for (int i = 0; i < 3; ++i) {
                traj(i + 3) = velocity_update(i);
                traj(i + 22) = velocity_update(i); // Update arm base velocity
            } 
           

            break;
        }
        case 8: {
            double domLen = 200.0; // Domain length in milliseconds
            double domLenSec = domLen / 1000.0; // Domain length in seconds
            DM R = RotMat(vertcat(q(6), q(7), q(8))); // Assuming q contains Euler angles in the last 3 elements
            // size_t currentStep = std::floor(t / params.dt) + 1;

            if (t >= (params.stepcnt * domLenSec + 1)) {
                // Update contact sequence
                if (static_cast<double>(params.contact(0)) == 1.0) {
                    params.contact = {0, 1, 1, 0};
                } else {
                    params.contact = {1, 0, 0, 1};
                }

                // Increase velocity in body frame
                traj(3) += 0.1; // Increase command velocity in body X
                traj(4) += 0.1; // Increase command velocity in body Y

                // Saturate the command velocities
                traj(3) = fmin(traj(3), params.desVel(0));
                traj(4) = fmin(traj(4), params.desVel(1));

                // Calculate step length in body frame and update step foot position
                DM stepLen = vertcat(traj(3) * domLenSec / 2, traj(4) * domLenSec / 2, 0.0);
                DM hipTmp = params.p_hip + stepLen;
                hipTmp = SX::mtimes(R, hipTmp); // Rotate to the world frame
                params.pf = hipTmp + vertcat(q(Slice(0, 2)), 0.0);

                // Set velocity in world frame
                DM cmdWrld = SX::mtimes(R, traj(Slice(3, 6))); // Convert to world frame

                // std::string basePath = "tmp/pathplanner/";
                // writeMatrixToFile(stepLen, basePath + "stepLen.txt");
                // writeMatrixToFile(params.p_hip, basePath + "params_p_hip.txt");
                // writeMatrixToFile(hipTmp, basePath + "hipTmp.txt");
                // writeMatrixToFile(q(Slice(0, 3)), basePath + "q_COM.txt");
                // writeMatrixToFile(cmdWrld, basePath + "cmdWrld.txt");
                // writeMatrixToFile(params.pf, basePath + "params_pf.txt");



                traj(Slice(3, 6)) = cmdWrld;

                // Increment step counter
                params.stepcnt++;
            }

            float heading = 0;

            // Update position and height based on velocity
            traj(0) = q(0) + params.dt * traj(3);
            traj(1) = q(1) + params.dt * traj(4);
            traj(2) = params.height; // Assuming height is constant or predefined
            traj(5) = 0; // Assuming zero vertical velocity

            for (size_t n = 1; n <= params.N; ++n) {
                // Compute the predicted state for each step in the horizon
                DM state = DM::zeros(n_states, 1);
                

                // Updating the state based on your MATLAB code logic
                state(0) = q(0) + n * params.dt * traj(3); // pos X
                state(1) = q(1) + n * params.dt * traj(3); // pos Y
                state(2) = params.height;                  // pos Z (constant height assumption)
                state(3) = traj(3);                        // vel X (constant velocity assumption)
                state(4) = traj(4);                        // vel Y
                state(5) = 0.0;                            // vel Z (assuming zero vertical velocity)
                // Assuming Euler angles (heading) and angular velocity are zero
                state(6) = 0.0; // Roll or phi
                state(7) = 0.0; // Pitch or theta
                state(8) = heading; // Yaw or psi
                state(9) = 0.0;  // Angular velocities (assuming zeros)
                state(10) = 0.0;
                state(11) = 0.0;

                //Arm states
                state(12) = q(0) + n * params.dt * traj(3) + params.dd(0); // pos X
                state(13) = q(1) + n * params.dt * traj(4) + params.dd(1); // pos Y
                state(14) = params.height + params.dd(2);                  // pos Z 

                state(15) = 0; //roll or phi
                state(16) = 0; //pitch or theta
                state(17) = 0; //yaw or psi

                state(18) = q_des(18); //joint 1 angle
                state(19) = q_des(19); //joint 2 angle
                state(20) = q_des(20); //joint 3 angle
                state(21) = q_des(21); //joint 4 angle

                state(22) = traj(3); //vel X
                state(23) = traj(4); //vel Y
                state(24) = 0; //vel Z

                state(25) = 0; //roll rate
                state(26) = 0; //pitch rate
                state(27) = 0; //yaw rate

                state(28) = q_des(28); //joint 1 rate
                state(29) = q_des(29); //joint 2 rate
                state(30) = q_des(30); //joint 3 rate
                state(31) = q_des(31); //joint 4 rate


                // Place the computed state into the appropriate segment of xd
                //xd.segment((n-1)*n_states, n_states) = state;
                casadi::DM grf_des_leg = DM::vertcat({0.0, 0.0, 0.5* (params.mass + params.mass_arm) * params.G}); // Assuming the mass is constant
                casadi::DM grf_des = DM::repmat(grf_des_leg, 4, 1); // Repeat the leg GRF for all legs     
                // writwMatrixToFile(grf_des, "tmp/grf_des.txt");   
                xd(Slice(static_cast<int>((n-1)*(n_states)), static_cast<int>(n*(n_states)+n_u_SRB))) = DM::vertcat({state, grf_des});
            }
            break;
        }
        // Handle other cases as necessary
        default:
            std::cerr << "Mode '" << mode << "' is not defined in the motion planner.\n";
    }
    

    if (mode != 8) {
        casadi::DM grf_des_leg = DM::vertcat({0.0, 0.0, 0.25* (params.mass + params.mass_arm) * params.G}); // Assuming the mass is constant
        casadi::DM grf_des = DM::repmat(grf_des_leg, 4, 1); // Repeat the leg GRF for all legs
        xd = DM::vertcat({repmat(traj, params.N, 1), grf_des});
    }

    q_des = xd; // Only xd needs to be returned
}

void force_inequality(const casadi::SX& U, const Params& params, casadi::SX& g_force, casadi::SX& g_ub, casadi::SX& g_lb) {
    // Use SX for symbolic constraint expressions
    casadi::SX g_force_sx = casadi::SX::vertcat(std::vector<casadi::SX>{}); //empty vertcat SX vectors so that appending works in future

    // g_ub and g_lb as symbolic vectors with appropriate bounds
    casadi::SX g_ub_sx = casadi::SX::vertcat(std::vector<casadi::SX>{});
    casadi::SX g_lb_sx = casadi::SX::vertcat(std::vector<casadi::SX>{});


    for (size_t k = 0; k < params.N; ++k) {
        for (size_t leg = 0; leg < 4; ++leg) {
            size_t base_idx = leg * 3;
            // Directly construct and append symbolic expressions
            // friction cone bounds separated into 4 equations from absolute value of friction cone
            g_force_sx = casadi::SX::vertcat({g_force_sx,
                U(base_idx, k) - (params.mu/1.414) * U(base_idx + 2, k),  // needs to me mu/sqrt(2)
                U(base_idx + 1, k) - (params.mu/1.414) * U(base_idx + 2, k),
                U(base_idx, k) + (params.mu/1.414) * U(base_idx + 2, k),
                U(base_idx + 1, k) + (params.mu/1.414) * U(base_idx + 2, k)
            });
            g_ub_sx = casadi::SX::vertcat({g_ub_sx, 0.0, 0.0, casadi::inf, casadi::inf}); 
            g_lb_sx = casadi::SX::vertcat({g_lb_sx, -casadi::inf, -casadi::inf, 0, 0});
        }

        //Add arm joint torque expression % Add arm joint limits based on the urdf file --> max torque = 60 N.m
        g_force_sx = casadi::SX::vertcat({g_force_sx,
        U(params.n_u_SRB,k),
        U(params.n_u_SRB + 1,k),
        U(params.n_u_SRB + 2,k),
        U(params.n_u_SRB + 3,k)
         });

        g_ub_sx = casadi::SX::vertcat({g_ub_sx, 60, 60, 60, 60});
        g_lb_sx = casadi::SX::vertcat({g_lb_sx, -60, -60, -60, -60});

        //Add interaction wrench expressions

       g_force_sx = casadi::SX::vertcat({g_force_sx,
        U(params.n_u_SRB + params.n_u_arm,k),
        U(params.n_u_SRB + params.n_u_arm +1,k),
        U(params.n_u_SRB + params.n_u_arm +2,k),
        U(params.n_u_SRB + params.n_u_arm +3,k),
        U(params.n_u_SRB + params.n_u_arm +4,k),
        U(params.n_u_SRB + params.n_u_arm +5,k)
         });

            // % Add interaction wrench limits -- considered free for now--but we can
    // % limit them in future to avoid enormously big interaction force on the
    // % base to avoid braking the base stand

        g_ub_sx = casadi::SX::vertcat({g_ub_sx, casadi::inf, casadi::inf, casadi::inf, casadi::inf, casadi::inf, casadi::inf});
        g_lb_sx = casadi::SX::vertcat({g_lb_sx, -casadi::inf, -casadi::inf, -casadi::inf, -casadi::inf, -casadi::inf, -casadi::inf});      
       

    }

    // Convert symbolic SX expressions to DM for g_ub and g_lb if necessary
    // Note: This might not be needed if keeping them symbolic is preferable
    g_force = g_force_sx; // g_force is already SX and can stay symbolic
    g_ub = g_ub_sx;
    g_lb = g_lb_sx;
}
//This function is unused
void force_inequality_bounds(const Params& params, casadi::DM& g_ub, casadi::DM& g_lb) {

    // g_ub and g_lb as symbolic vectors with appropriate bounds
    casadi::DM g_ub_sx;
    casadi::DM g_lb_sx;

    for (size_t k = 0; k < params.N; ++k) {
        for (size_t leg = 0; leg < 4; ++leg) {
            size_t base_idx = leg * 3;
            // Directly construct and append symbolic expressions
            g_ub_sx = casadi::SX::vertcat({g_ub_sx, 0.0, 0.0, casadi::inf, casadi::inf}); 
            g_lb_sx = casadi::SX::vertcat({g_lb_sx, -casadi::inf, -casadi::inf, 0, 0});
        }
    }
    g_ub = g_ub_sx;
    g_lb = g_lb_sx;
}

// Function to write casadi::SX matrix to file
void writeMatrixToFile(const casadi::SX& matrix, const std::string& filename) {
    std::ofstream file(filename);
    if (file.is_open()) {
        // Print matrix dimensions
        // file << "Matrix size: " << matrix.size1() << "x" << matrix.size2() << std::endl;
        // Print matrix content preserving original shape
        for (int i = 0; i < matrix.size1(); ++i) {
            for (int j = 0; j < matrix.size2(); ++j) {
                file << matrix(i, j);
                if (j < matrix.size2() - 1) file << ", ";
            }
            file << std::endl;
        }
        file.close();
    } else {
        std::cerr << "Unable to open file" << std::endl;
    }
}

// Function to write casadi::DM matrix to file
void writeMatrixToFile(const casadi::DM& matrix, const std::string& filename) {
    std::ofstream file(filename);
    if (file.is_open()) {
        // Print matrix dimensions
        // file << "Matrix size: " << matrix.size1() << "x" << matrix.size2() << std::endl;
        // Print matrix content preserving original shape
        for (int i = 0; i < matrix.size1(); ++i) {
            for (int j = 0; j < matrix.size2(); ++j) {
                file << matrix(i, j);
                if (j < matrix.size2() - 1) file << ", ";
            }
            file << std::endl;
        }
        file.close();
    } else {
        std::cerr << "Unable to open file" << std::endl;
    }
}

void appendMatrixToFile(const casadi::DM& matrix, const std::string& filename) {
    std::ofstream file(filename,std::ofstream::app);
    if (file.is_open()) {
        // Print matrix dimensions
        // file << "Matrix size: " << matrix.size1() << "x" << matrix.size2() << std::endl;
        // Print matrix content preserving original shape
        for (int i = 0; i < matrix.size1(); ++i) {
            for (int j = 0; j < matrix.size2(); ++j) {
                file << matrix(i, j);
                if (j < matrix.size2() - 1) file << ", ";
            }
            file << std::endl;
        }
        file.close();
    } else {
        std::cerr << "Unable to open file" << std::endl;
    }
}

// Overload to handle multiple DM matrices and one size_t variable
void writeMatrixToFile(const Params& params, const std::string& filename) {
    std::ofstream file(filename, std::ios::out | std::ios::trunc);  // Open for output and truncate to clear existing content
    if (file.is_open()) {
        // Helper lambda to write a single matrix
        auto writeMatrix = [&](const casadi::DM& matrix) {
            file << "Matrix size: " << matrix.size1() << "x" << matrix.size2() << "\n" << std::endl;
            for (int i = 0; i < matrix.size1(); ++i) {
                for (int j = 0; j < matrix.size2(); ++j) {
                    file << matrix(i, j);
                    if (j < matrix.size2() - 1) file << ", ";
                }
                file << std::endl;
            }
        };

        // Write each matrix with a header
        file << "params.pf:" << std::endl;
        writeMatrix(params.pf);
        file << "\nparams.contact:" << std::endl;
        writeMatrix(params.contact);
        file << "\nparams.desVel:" << std::endl;
        writeMatrix(params.desVel);
        
        // Additionally write the size_t variable
        file << "\nparams.stepcnt: " << params.stepcnt << std::endl;
        file << "\n\nparams.current_time: " << params.current_time << std::endl;
        file << "\n\nparams.mpc_iter: " << params.mpc_iter << std::endl;

        file.close();
    } else {
        std::cerr << "Unable to open file" << std::endl;
    }
}

void equalityconstraints(const SX &X, const SX &U, const SX &P, const Params &params, SX &obj, SX &g_eq, const Function &f_SRB, const Function &f_arm)
{

    std::string basePath = "./tmp/";
    int n_states = params.n_states;
    int n_SRB = params.n_SRB;
    int n_arm = params.n_arm;
    int n_u_SRB = params.n_u_SRB;
    int n_u_arm = params.n_u_arm;
    int n_u_lambda_int = params.n_u_lambda_int;
    int N = params.N;
    double dt = params.dt;
    //% Numeric variables
    double mass = params.mass;
    double g = params.G; // gravity
    DM J = params.J;
    DM dd = params.dd; // Position vector of the arm's base w.r.t the A1 body frame

    // Initial condition constraints
    SX st = X(Slice(0, n_states), 0);
    g_eq = st - P(Slice(0, n_states));

    //GRF desired trajectory
    SX GRF_des = P(Slice(n_states + n_states * N, n_states + n_states * N + 12));

    //Feet position
    SX feet_pos = P(Slice(n_states + n_states * N + 12, n_states + n_states * N + 12 + 12));

//Extracting the gains for cost function
    SX Q_SRB = diag(P(Slice(n_states + n_states * N + 12 + 12, n_states + n_states * N + 12 + 12 + n_SRB))); // A1 (SRB) desired state
    SX Q_arm = diag(P(Slice(n_states + n_states * N + 12 + 12 + n_SRB, n_states + n_states * N + 12 + 12 + n_SRB + n_arm))); // Kinova desired state
    
    
    SX R_SRB = diag(P(Slice(n_states + n_states * N + 12 + 12 + n_SRB + n_arm, n_states + n_states * N + 12 + 12 + n_SRB + n_arm + n_u_SRB))); // SRB control desired state
    SX R_arm = diag(P(Slice(n_states + n_states * N + 12 + 12 + n_SRB + n_arm + n_u_SRB, n_states + n_states * N + 12 + 12 + n_SRB + n_arm +  n_u_SRB + n_u_arm))); // Kinova control desired state
    SX R_lambda = diag(P(Slice(n_states + n_states * N + 12 + 12 + n_SRB + n_arm + n_u_SRB + n_u_arm, n_states + n_states * N + 12 + 12 + n_SRB + n_arm + n_u_SRB + n_u_arm + n_u_lambda_int))); // Interaction wrench desired state

    SX P_SRB = diag(P(Slice(n_states + n_states * N + 12 + 12 + n_SRB + n_arm + n_u_SRB + n_u_arm + n_u_lambda_int, n_states + n_states * N + 12 + 12 + n_SRB + n_arm + n_u_SRB + n_u_arm + n_u_lambda_int + n_SRB))); // Terminal cost for SRB
    SX P_arm = diag(P(Slice(n_states + n_states * N + 12 + 12 + n_SRB + n_arm + n_u_SRB + n_u_arm + n_u_lambda_int + n_SRB, n_states + n_states * N + 12 + 12 + n_SRB + n_arm + n_u_SRB + n_u_arm + n_u_lambda_int + n_SRB + n_arm))); // Terminal cost for Kinova
    
    SX D_q_outside = P(Slice(n_states + n_states * N + 12 + 12 + n_SRB + n_arm + n_u_SRB + n_u_arm + n_u_lambda_int + n_SRB + n_arm,n_states + n_states * N + 12 + 12 + n_SRB + n_arm + n_u_SRB + n_u_arm + n_u_lambda_int + n_SRB + n_arm + (n_arm/2)*(n_arm/2)));

    for (int k = 0; k < N; ++k)
    {

        // Evaluation of the objective function
        // SRB objective
        SX st_SRB = X(Slice(0, n_SRB), k);    // SRB states
        SX con_SRB = U(Slice(0, n_u_SRB), k); // SRB controls

        SX des_st_SRB = P(Slice(k * n_states, k * n_states + n_SRB)); // A1 (SRB) desired state

        // writeMatrixToFile(st, basePath + "st.txt");
        // writeMatrixToFile(des_st, basePath + "des_st.txt");
        // writeMatrixToFile(params.Q, basePath + "params_Q.txt");
        // writeMatrixToFile(con, basePath + "con.txt");
        // writeMatrixToFile(params.R, basePath + "params_R.txt");
        

        // Calculate objective for SRB
        SX st_err_SRB = st_SRB - des_st_SRB;
        SX grf_err = con_SRB - GRF_des;
        // writeMatrixToFile(st_err_SRB, basePath + "st_err_SRB.txt");
        // writeMatrixToFile(params.Q_SRB, basePath + "params_Q_SRB.txt");
        // writeMatrixToFile(params.R_SRB, basePath + "params_R_SRB.txt");
        // writeMatrixToFile(con_SRB, basePath + "con_SRB.txt");
        SX stage_cost_SRB = 0;
        if (k < N - 1)
        { //stage cost
            stage_cost_SRB = SX::mtimes(SX::mtimes(st_err_SRB.T(), Q_SRB), st_err_SRB) + SX::mtimes(SX::mtimes(grf_err.T(), R_SRB), grf_err);
            
        }
        else{
            // Terminal cost
            stage_cost_SRB = SX::mtimes(SX::mtimes(st_err_SRB.T(), P_SRB), st_err_SRB) + + SX::mtimes(SX::mtimes(grf_err.T(), R_SRB), grf_err);
        }

        // Kinova objective                     
        SX st_arm = X(Slice(n_SRB, n_SRB + n_arm), k);                                // Kinova states
        SX con_arm = U(Slice(n_u_SRB, n_u_SRB + n_u_arm), k);                         // Kinova controls
        SX des_st_arm = P(Slice(k * n_states + n_SRB, k * n_states + n_SRB + n_arm)); // Kinova desired state

        // calculate obj for the Kinova
        SX st_err_arm = st_arm - des_st_arm;
        // writeMatrixToFile(st_err_arm, basePath + "st_err_arm.txt");
        // writeMatrixToFile(params.Q_arm, basePath + "params_Q_arm.txt");
        // writeMatrixToFile(params.R_arm, basePath + "params_R_arm.txt");
        // writeMatrixToFile(con_arm, basePath + "con_arm.txt");
        SX stage_cost_arm = 0;
        if (k < N-1)
        {
            stage_cost_arm = SX::mtimes(SX::mtimes(st_err_arm.T(), Q_arm), st_err_arm) + SX::mtimes(SX::mtimes(con_arm.T(), R_arm), con_arm);

        }
        else{
            // Terminal cost
            stage_cost_arm = SX::mtimes(SX::mtimes(st_err_arm.T(), P_arm), st_err_arm) + SX::mtimes(SX::mtimes(con_arm.T(), R_arm), con_arm);
        }
        
        
        // Interaction force objective
        SX con_lambda = U(Slice(n_u_SRB + n_u_arm, n_u_SRB + n_u_arm + n_u_lambda_int), k); // Interaction forces
        // TODO: ????? Scalar weight higher gain for torques
        SX stage_cost_lambda = SX::mtimes(SX::mtimes(con_lambda.T(), R_lambda), con_lambda); //% calculate obj for the interaction wrench

        // Final objective
        obj = obj + stage_cost_SRB + stage_cost_arm + stage_cost_lambda;

        // SX st_err = st - des_st;
        // // writeMatrixToFile(st_err, basePath + "st_err.txt");
        // SX stage_cost = SX::mtimes(SX::mtimes(st_err.T(), params.Q), st_err);
        // SX terminal_cost = SX::mtimes(SX::mtimes(con.T(), params.R), con);

        // // writeMatrixToFile(stage_cost, basePath + "stage_cost.txt");
        // // writeMatrixToFile(terminal_cost, basePath + "terminal_cost.txt");

        // obj = obj + stage_cost + terminal_cost;

        //---------------------Equality constraints for SRB dynamics----------------------//

        //  Next state for SRB
        SX st_next_SRB = X(Slice(0, n_SRB), k + 1);
        std::map<std::string, SX> arg_SRB;
        arg_SRB["st"] = st_SRB;
        arg_SRB["con"] = con_SRB;
        arg_SRB["int"] = con_lambda;
        arg_SRB["pf"] = SX::reshape(feet_pos.T(),3,4); //reshape so that it is a 3x4 matrix

        std::map<std::string, SX> result_SRB = f_SRB(arg_SRB);
        SX f_value_SRB = result_SRB.at("rhs");
        SX st_next_euler_SRB = st_SRB + dt * f_value_SRB;
        // Add equality constraints
        g_eq = SX::vertcat({g_eq, st_next_SRB - st_next_euler_SRB});

        //---------------------Equality constraints for arm dynamics----------------------//

        // Next state for arm
        SX st_next_arm = X(Slice(n_SRB, n_SRB + n_arm), k + 1);
        std::map<std::string, SX> arg_arm;
        arg_arm["q"] = st_arm(Slice(0, 10));
        arg_arm["dq"] = st_arm(Slice(10, 20));
        arg_arm["con"] = con_arm;
        arg_arm["int"] = con_lambda;
        arg_arm["D_q"] = SX::reshape(D_q_outside, n_arm/2, n_arm/2); //reshape so that it is a 10x10 matrix       
        
        std::map<std::string, SX> result_arm = f_arm(arg_arm);
        SX f_value_arm = result_arm.at("rhs");
        SX st_next_euler_arm = st_arm + dt * f_value_arm;
        // Add equality constraints
        g_eq = SX::vertcat({g_eq, st_next_arm - st_next_euler_arm});

        //------------------------Get Holonomic constraints------------------------//
        // SRB
        // SX i3 = SX::eye(3);
        // SX z3 = SX::zeros(3, 3);
        // Position vector of A1 w.r.t Inertial frame
        SX p = st_SRB(Slice(0, 3));
        SX dp = st_SRB(Slice(3, 6));
        //% ddp = [ddx ddy ddz]'; % TODO: To be substituted
        // SX ddp = 1 / mass * SX::mtimes(SX::horzcat({i3, i3, i3, i3, i3, z3}), SX::vertcat({con_SRB, con_lambda})) + SX::vertcat({0, 0, -g}); //% Assuming the mass is constant

        //% Euler angles vector
        SX theta = st_SRB(Slice(6, 9));
        SX omega = st_SRB(Slice(9, 12));

        // Rotation matrix from Euler angles
        SX R = RotMat(theta);
        SX A = A_Theta(theta);

        // SX dtheta = SX::mtimes(A, st_SRB(Slice(9, 12)));

        // SX pf = params.pf;                //% p_foot in MATLAB code
        // SX rd = SX::reshape(P(Slice(n_states + n_states * N,n_states + n_states * N + 12)).T(),3,4) - st_SRB(Slice(0, 3)); // Distance from foot placement (?) to COM
        // SX IwI = SX::mtimes(SX::mtimes(SX::mtimes(R, SX::solve(J, R.T())), skewSym(omega)), SX::mtimes(R, SX::mtimes(J, R.T())));
        // SX Exp1 = SX::mtimes(SX::horzcat({SX::mtimes(SX::mtimes(R, SX::solve(J, R.T())), skewSym(rd)), SX::mtimes(SX::mtimes(R, SX::solve(J, R.T())), skewSym(dd)), SX::mtimes(R, SX::solve(J, R.T()))}), SX::vertcat({con_SRB, con_lambda}));
        // SX domega = Exp1 - SX::mtimes(IwI, omega);
        // SX ddtheta =  SX::mtimes(SX::jacobian(SX::mtimes(A,omega), theta),dtheta) + SX::mtimes(A,domega);

        //Constraints related to SRB
        SX ddp = f_value_SRB(Slice(3, 6));
        SX dtheta = f_value_SRB(Slice(6, 9));
        SX domega = f_value_SRB(Slice(9, 12));
        SX ddtheta =  SX::mtimes(SX::jacobian(SX::mtimes(A,omega), theta),dtheta) + SX::mtimes(A,domega);
        


        //Constraints related to the Kinova arm
        SX dp_ab = f_value_arm(Slice(0, 3));
        SX ddp_ab = f_value_arm(Slice(10,13));

        SX dtheta_ab = f_value_arm(Slice(3,6));
        SX ddtheta_ab = f_value_arm(Slice(13,16));

        // ----------------------1 st Holonomic Constraint (Position)----------------------//
        SX HolonomicPosition = p + SX::mtimes(R, dd) - st_arm(Slice(0, 3));
        SX HolonomicOrientation = theta - st_arm(Slice(3, 6));

        // ----------------------2 nd Holonomic Constraint (Velocity)----------------------//
        SX HolonomicVelocity = dp + SX::mtimes(SX::jacobian(SX::mtimes(R,dd),theta),dtheta) - dp_ab;
        SX HolonomicAngularVelocity = dtheta - dtheta_ab;

        // ----------------------3 rd Holonomic Constraint (Acceleration)----------------------//
        SX HolonomicAcceleration  = ddp + SX::mtimes(SX::jacobian(SX::mtimes(R,dd),theta),ddtheta) +  SX::mtimes(SX::jacobian((SX::jacobian(SX::mtimes(R,dd),theta),dtheta),theta),dtheta) - ddp_ab;
        // SX HolonomicAcceleration  = ddp + SX::mtimes(SX::jacobian(SX::mtimes(R,dd),theta),ddtheta)  - ddp_ab;

        SX HolonomicAngularAcceleration = ddtheta - ddtheta_ab;
        SX bias_position = SX::vertcat({0, 0, 0.0});
        SX bias_orientation = SX::vertcat({0, 0, 0});
        // Add holonomic constraints
        // g_eq = SX::vertcat({g_eq, HolonomicPosition, HolonomicOrientation, HolonomicVelocity, HolonomicAngularVelocity, HolonomicAcceleration, HolonomicAngularAcceleration});
        g_eq = SX::vertcat({g_eq, HolonomicAcceleration+ params.Kp * (HolonomicPosition - bias_position) + params.Kd * HolonomicVelocity, HolonomicAngularAcceleration + params.Kp * (HolonomicOrientation - bias_orientation) + params.Kd * HolonomicAngularVelocity});


        // // writeMatrixToFile(st_next, basePath + "st_next.txt");
        // // writeMatrixToFile(f_value, basePath + "f_value.txt");

        // // Compute constraints
        // SX st_next_euler = st + dt * f_value;
        // g_eq = SX::vertcat({g_eq, st_next - st_next_euler});

        // // writeMatrixToFile(st_next_euler, basePath + "st_next_euler.txt");
        // writeMatrixToFile(g_eq, basePath + "g_eq.txt");
     
    }
}



void set_constraint_bounds(Args& args, const DM& g_lb, const DM& g_ub, int n_states, int N, int NumHolCon) {
    // Calculate the start index for setting lower and upper bounds
    int startIndex = n_states * (N + 1) + NumHolCon * N ; // Adjust for zero-based indexing
    // int startIndex = n_states * (N + 1) ; // Adjust for zero-based indexing

    // Resize the lbg and ubg if necessary
    if (args.lbg.is_empty() || args.lbg.size1() < startIndex + g_lb.size1()) {
        args.lbg = DM::zeros(startIndex + g_lb.size1(), 1); // Adjust total size if needed
    }
   
    if (args.ubg.is_empty() || args.ubg.size1() < startIndex + g_ub.size1()) {
        args.ubg = DM::zeros(startIndex + g_ub.size1(), 1); // Adjust total size if needed
    }

   /*  for (size_t i = 0; i < N; ++i)  
    {
        args.lbg(Slice(static_cast<int>(n_states + (i+1)*n_states),static_cast<int>(n_states + (i+1)*n_states+NumHolCon))) = DM::vertcat({-0.05,-0.05,-0.05,-0.1,-0.1,-0.1});
        args.ubg(Slice(static_cast<int>(n_states + (i+1)*n_states),static_cast<int>(n_states + (i+1)*n_states+NumHolCon))) = DM::vertcat({0.05,0.05,0.05,0.1,0.1,0.1});           
    } */

    // Set values for lower bounds
    for (int i = 0; i < g_lb.size1(); ++i) {
        args.lbg(startIndex + i) = g_lb(i);
    }

    // Set values for upper bounds
    for (int i = 0; i < g_ub.size1(); ++i) {
        args.ubg(startIndex + i) = g_ub(i);
    }
}

void set_statebounds(int mpciter, const Params& params, Args& args) {
    /*  LB and UB includes the ranges for all the decision variables for all
% future horizons
% [states;
%   inputs]
% In the previous code, we had no constriants on our states, but here we
% do. */
    int n_states = params.n_states;  // Number of states
    int n_inputs = params.n_inputs;  // Number of inputs
    int N = params.N;
    double zmax = params.zmax;
    int n_SRB = params.n_SRB;
    int n_arm = params.n_arm;
    int n_u_SRB = params.n_u_SRB;
    int n_u_arm = params.n_u_arm;
    int n_u_lambda_int = params.n_u_lambda_int;


    // Initialize LB and UB
    args.lbx = DM::zeros(n_states*(N+1) + n_inputs*N, 1);
    args.ubx = DM::zeros(n_states*(N+1) + n_inputs*N, 1); //%state x lower bound % new - adapt the bound
    args.lbx(Slice()) = -DM::inf();
    args.ubx(Slice()) = DM::inf();

    //%% Introducing lower and upper bounds of arm joints q_s shape variables
    // %%%% Arm joints q_s = [q1 q2 q3 q4] in (rad) from the URDF file
    SX q_s_LB = SX::vertcat({-6.28318530718, 0.820304748437, 0.331612557879, -6.28318530718});
    SX q_s_UB = SX::vertcat({6.28318530718, 5.46288055874, 5.9515727493, 6.28318530718});
    //%%%% Arm joints rates dq_s = [dq1 dq2 dq3 dq4] in (rad/sec) from the URDF file
    SX dq_s_LB = -1;
    SX dq_s_UB = 1;

    //%% State bounds for the 1st horizon: k=1
    // %%% SRB state bounds: always free
    args.lbx(Slice(0, n_SRB)) = -DM::inf();
    args.ubx(Slice(0, n_SRB)) = DM::inf();

    /* %%% Kinova state bounds
    %%%% Arm base position and orientation
    % These are going to be addressed by 
    % the Holonomic constraints, so we set them free for now */
    args.lbx(Slice(n_SRB, n_SRB + 6)) = -DM::inf();
    args.ubx(Slice(n_SRB, n_SRB + 6)) = DM::inf();

    // %%%% Arm base position and orientation rates
    args.lbx(Slice(n_SRB + 7+4-1, n_SRB + 7+4+5)) = -DM::inf();
    args.ubx(Slice(n_SRB + 7+4-1, n_SRB + 7+4+5)) = DM::inf();

    // %%%% Arm joints q_s = [q1 q2 q3 q4] in (rad) from the URDF file
    args.lbx(Slice(n_SRB + 7-1, n_SRB + 7+3)) = q_s_LB;
    args.ubx(Slice(n_SRB + 7-1, n_SRB + 7+3)) = q_s_UB;

    // %%%% Arm joints rates dq_s = [dq1 dq2 dq3 dq4] in (rad/sec) from the URDF file
    args.lbx(Slice(n_SRB + 7+4+5+1-1, n_SRB + n_arm)) = SX::vertcat({dq_s_LB, dq_s_LB, dq_s_LB, dq_s_LB});
    args.ubx(Slice(n_SRB + 7+4+5+1-1, n_SRB + n_arm)) = SX::vertcat({dq_s_UB, dq_s_UB, dq_s_UB, dq_s_UB});

 /*    %% State bounds for k>1

    % This for-loop will assign lower and upper bounds for the vertical
    % components of SRB inputs (also referred to as ground reaction forces GRFs) */

    for (int k = 0; k < N; ++k) {
    //    %     contact_index = rem(mpciter+k-1,40)+1;
        // %% States section

        // %%% SRB 
        args.lbx(Slice(n_states*(k+1), n_states*(k+1) + n_SRB)) = -DM::inf();
        args.ubx(Slice(n_states*(k+1), n_states*(k+1) + n_SRB)) = DM::inf();

        // %%% Kinova
        // %%%% Arm base position and orientation
        args.lbx(Slice(n_states*(k+1) + n_SRB, n_states*(k+1) + n_SRB + 6)) = -DM::inf();
        args.ubx(Slice(n_states*(k+1) + n_SRB, n_states*(k+1) + n_SRB + 6)) = DM::inf();

        // %%%% Arm base pos. & ori. rates
        args.lbx(Slice(n_states*(k+1) + n_SRB + 7+4-1, n_states*(k+1) + n_SRB + 7+4+5)) = -DM::inf();
        args.ubx(Slice(n_states*(k+1) + n_SRB + 7+4-1, n_states*(k+1) + n_SRB + 7+4+5)) = DM::inf();

        // %%%% Arm joints q_s = [q1 q2 q3 q4] in (rad) from the URDF file
        args.lbx(Slice(n_states*(k+1) + n_SRB + 7-1, n_states*(k+1) + n_SRB + 7+3)) = q_s_LB;
        args.ubx(Slice(n_states*(k+1) + n_SRB + 7-1, n_states*(k+1) + n_SRB + 7+3)) = q_s_UB;

        // %%%% Arm joints rates dq_s = [dq1 dq2 dq3 dq4] in (rad/sec) from the URDF file
        args.lbx(Slice(n_states*(k+1) + n_SRB + 7+4+5+1-1, n_states*(k+1) + n_SRB + n_arm)) = SX::vertcat({dq_s_LB, dq_s_LB, dq_s_LB, dq_s_LB});
        args.ubx(Slice(n_states*(k+1) + n_SRB + 7+4+5+1-1, n_states*(k+1) + n_SRB + n_arm)) = SX::vertcat({dq_s_UB, dq_s_UB, dq_s_UB, dq_s_UB});

        // %% This part will assign lower and upper bound for the vertical component of the GRFs: "f_z"
        // %%% SRB
        int contact_index = (mpciter + k) % 40; // Adjusting MATLAB 1-based to C++ 0-based index
        for (int leg = 0; leg < 4; ++leg) {
            int index = n_states*(N+1) + n_inputs*k + 3*leg + 3-1; // Adjusted for 0-based indexing
            args.lbx(index) = 0;
            args.ubx(index) = params.contact(leg) * zmax; // Assuming contact is std::vector<double>
        }
    }
}

void shift(double& t0, DM& qk, DM& u0, double dt, const DM& u, const Function& f_SRB, const Function& f_arm, const Params& params) {

    /* % This function applies the 1st control input to the nonlinear dynamics of
    % the system and updates the current states using Euler forward integration
    % approximation. 

    % Also, deletes the used control input and duplicates the last control
    % input */
    
    //Separate the SRB and arm states, control inputs and interaction forces
    int n_states = params.n_states;
    int n_SRB = params.n_SRB;
    int n_arm = params.n_arm;
    int n_u_SRB = params.n_u_SRB;
    int n_u_arm = params.n_u_arm;
    int n_u_lambda_int = params.n_u_lambda_int;

    //SRB states and controls - Only the first control input is used
    DM qk_SRB = qk(Slice(0, n_SRB));
    DM u_SRB = u(0,Slice(0, n_u_SRB)).T();  // First control input, transposed to match MATLAB

    //Arm states and controls -  Only the first control input is used
    DM qk_arm = qk(Slice(n_SRB, n_SRB + n_arm));
    DM q_arm = qk_arm(Slice(0, 10));
    DM dq_arm = qk_arm(Slice(10, 20));
    DM u_arm = u(0,Slice(n_u_SRB, n_u_SRB + n_u_arm)).T();  // First control input, transposed to match MATLAB

    //Interaction forces -  Only the first control input is used
    DM u_lambda_int = u(0,Slice(n_u_SRB + n_u_arm, n_u_SRB + n_u_arm + n_u_lambda_int)).T();  // First control input, transposed to match MATLAB
    
    //---------------------SRB Dynamics----------------------//
    // Prepare input for the function f_SRB
    std::map<std::string, DM> arg_SRB;
    arg_SRB["st"] = qk_SRB;  // Current state
    arg_SRB["con"] = u_SRB; 
    arg_SRB["int"] = u_lambda_int;
    arg_SRB["pf"] = params.pf;

    // Call the function f
    std::map<std::string, DM> result_SRB = f_SRB(arg_SRB);
    DM f_value_SRB = result_SRB["rhs"];  // Assuming the output is named 'rhs'

    // Update state
    qk_SRB = qk_SRB + DM(dt) * f_value_SRB;

    //---------------------Arm Dynamics----------------------//
    // Prepare input for the function f_arm
    std::map<std::string, DM> arg_arm;
    arg_arm["q"] = q_arm;
    arg_arm["dq"] = dq_arm;
    arg_arm["con"] = u_arm;
    arg_arm["int"] = u_lambda_int;

    // Call the function f
    std::map<std::string, DM> result_arm = f_arm(arg_arm);
    DM f_value_arm = result_arm["rhs"];  // Assuming the output is named 'rhs'

    // Update state
    qk_arm = qk_arm + DM(dt) * f_value_arm;

    // Update the full state
    qk = DM::vertcat({qk_SRB, qk_arm});


    // Increment time
    t0 += dt;

    // Shift control inputs
    DM u_new = vertcat(u(Slice(1, u.size1()), Slice()),  // Skip the first control set
                       u(Slice(u.size1()-1, u.size1()), Slice()));  // Duplicate the last control set
    u0 = u_new;
}

// void getKinovaDynamics(SX& D_q, SX& Gravity, SX& B, SX& J, SX& q, SX& dq, SX& q_in, SX& dq_in){
//    /*  % This version is based on Ali's notations, which considers the arm's dynamics exlusively.

//     % This code calculates the inertial and gravity matrices for the 4-DOF
//     % Kinova arm dynamcs
//     % The Coriolis terms have been disregarded
//     % The arm is assumed to be in a relative posori of [armx army armz, roll pitch yaw] w.r.t to the A1 
//     % The commented lines by Jeeseop have been deleted, and the current
//     % commented lines have been commented by Ali */
//      // %floating base (A1's position and orientation)
//     SX x_b = q_in(0); SX y_b = q_in(1); SX z_b = q_in(2);
//     SX phi_b = q_in(3); SX theta_b = q_in(4); SX psi_b = q_in(5);
//     SX dx_b = dq_in(0); SX dy_b = dq_in(1); SX dz_b = dq_in(2);
//     SX dphi_b = dq_in(3); SX dtheta_b = dq_in(4); SX dpsi_b = dq_in(5);
//     SX q1 = q_in(6); SX q2 = q_in(7); SX q3 = q_in(8); SX q4 = q_in(9);
//     SX dq1 = dq_in(6); SX dq2 = dq_in(7); SX dq3 = dq_in(8); SX dq4 = dq_in(9);
//     SX q10 = q_in(10); SX q11 = q_in(11); SX q12 = q_in(12); SX q13 = q_in(13); SX q14 = q_in(14); SX q15 = q_in(15);
//     SX dq10 = dq_in(10); SX dq11 = dq_in(11); SX dq12 = dq_in(12); SX dq13 = dq_in(13); SX dq14 = dq_in(14); SX dq15 = dq_in(15);
  
//     // % arminitialposori = [armx; army; armz; roll; pitch; yaw]; 
//     // %roll pitch yaw values as an extrinsic rotation which commonly used in URDF 
//     SX q = SX::vertcat({x_b, y_b, z_b, phi_b, theta_b, psi_b, q1, q2, q3, q4});
//     SX dq = SX::vertcat({dx_b, dy_b, dz_b, dphi_b, dtheta_b, dpsi_b, dq1, dq2, dq3, dq4});

// /*     %note: every rotation axis is aligned as Z axis: DH parameter-like

//     % framename: I => (x_b)-I1 => (y_b)-I2 => (z_b)-I3 => (phi_b)-I4 => (theta)-I5 => 
//     %            (psi_b)-B(base) => (q1)-L1 => (q2)-L2 => (q3)-L3 => (q4)-L4 =>

    
//     % H-transform format: [[rotation mtx],[translation vector]';[0 0 0 1]] */
//     // %% Homogeneous Transform
//     SX T_II3 = SX::vertcat({SX::horzcat({SX::eye(3,3),SX::vertcat({x_b,y_b,z_b})}), SX::horzcat({SX::zeros(1,3), 1})});
//     SX T_II4 = SX::mtimes(T_II3, SX::vertcat({SX::horzcat({rot_z(psi_b),SX::zeros(3,1)}), SX::horzcat({SX::zeros(1,3), 1})}));
//     SX T_II5 = SX::mtimes(T_II4, SX::vertcat({SX::horzcat({rot_y(theta_b),SX::zeros(3,1)}), SX::horzcat({SX::zeros(1,3), 1})}));

//     // %%% TODO: why don't we add an explicit rotation around z-axis here?
//     // %%%%%%%%%%%%--???????????--%%%%%%%%%%%%%%%%

//     //====================arm pos changable (Transformation from arm's base to the world Inertia frame)====================//
//     //%ZYX convention
//     SX T_IB = SX::mtimes(T_II5, SX::vertcat({SX::horzcat({rot_x(phi_b),SX::zeros(3,1)}), SX::horzcat({SX::zeros(1,3), 1})}));

//     SX T_BL1_1 = SX::vertcat({SX::horzcat({rot_y(SX::pi),SX::vertcat({0,0,0.15675})}),SX::horzcat({SX::zeros(1,3),1})});
//     SX T_BL1_2 = SX::vertcat({SX::horzcat({rot_z(q1),SX::zeros(3,1)}),SX::horzcat({SX::zeros(1,3),1})});
//     SX T_BL1 = SX::mtimes(T_BL1_1, T_BL1_2);

//     SX T_L1L2_1 = SX::vertcat({SX::horzcat({SX::eye(3),SX::vertcat({0,0.0016,-0.11875})}),SX::horzcat({SX::zeros(1,3),1})});
//     SX T_L1L2_2 = SX::vertcat({SX::horzcat({rot_z(SX::pi),SX::zeros(3,1)}),SX::horzcat({SX::zeros(1,3),1})});
//     SX T_L1L2_3 = SX::vertcat({SX::horzcat({rot_x(-SX::half_pi),SX::zeros(3,1)}),SX::horzcat({SX::zeros(1,3),1})});
//     SX T_L1L2_4 = SX::vertcat({SX::horzcat({rot_z(q2),SX::zeros(3,1)}),SX::horzcat({SX::zeros(1,3),1})});
//     SX T_L1L2 = SX::mtimes(T_L1L2_1, SX::mtimes(T_L1L2_2, SX::mtimes(T_L1L2_3, T_L1L2_4)));

//     SX T_L2L3_1 = SX::vertcat({SX::horzcat({SX::eye(3),SX::vertcat({0,-0.410,0})}),SX::horzcat({SX::zeros(1,3),1})});
//     SX T_L2L3_2 = SX::vertcat({SX::horzcat({rot_y(SX::pi),SX::zeros(3,1)}),SX::horzcat({SX::zeros(1,3),1})});
//     SX T_L2L3_3 = SX::vertcat({SX::horzcat({rot_Z(q3),SX::zeros(3,1)}),SX::horzcat({SX::zeros(1,3),1})});
//     SX T_L2L3 = SX::mtimes(T_L2L3_1, SX::mtimes(T_L2L3_2, T_L2L3_3));

//     SX T_L3L4_1 = SX::vertcat({SX::horzcat({SX::eye(3),SX::vertcat({0,0.2073,-0.0114})}),SX::horzcat({SX::zeros(1,3),1})});
//     SX T_L3L4_2 = SX::vertcat({SX::horzcat({rot_z(SX::pi),SX::zeros(3,1)}),SX::horzcat({SX::zeros(1,3),1})});
//     SX T_L3L4_3 = SX::vertcat({SX::horzcat({rot_x(-SX::half_pi),SX::zeros(3,1)}),SX::horzcat({SX::zeros(1,3),1})});
//     SX T_L3L4_4 = SX::vertcat({SX::horzcat({rot_z(q4),SX::zeros(3,1)}),SX::horzcat({SX::zeros(1,3),1})});
//     SX T_L3L4 = SX::mtimes(T_L3L4_1, SX::mtimes(T_L3L4_2, SX::mtimes(T_L3L4_3, T_L3L4_4)));

//     SX T_L4EE_1 = SX::vertcat({SX::horzcat({SX::eye(3),SX::vertcat({0,0,-0.16})}),SX::horzcat({SX::zeros(1,3),1})});
//     SX T_L4EE_2 = SX::vertcat({SX::horzcat({rot_z(SX::half_pi),SX::zeros(3,1)}),SX::horzcat({SX::zeros(1,3),1})});
//     SX T_L4EE_3 = SX::vertcat({SX::horzcat({rot_x(SX::pi),SX::zeros(3,1)}),SX::horzcat({SX::zeros(1,3),1})});
//     SX T_L4EE = SX::mtimes(T_L4EE_1, SX::mtimes(T_L4EE_2, T_L4EE_3));

//     SX T_IL1 = SX::mtimes(T_IB, T_BL1);
//     SX T_IL2 = SX::mtimes(T_IL1, T_L1L2);
//     SX T_IL3 = SX::mtimes(T_IL2, T_L2L3);
//     SX T_IL4 = SX::mtimes(T_IL3, T_L3L4);
//     SX T_IEE = SX::mtimes(T_IL4, T_L4EE);

//     //forward kinematics

//     SX I_EEposition = T_IEE(Slice(0, 3), 3);

//     // %% end effector Jacobian  (diff --> jacobian  in CaSaDi)
//     SX J_v = SX::horzcat({SX::jacobian(I_EEposition, x_b), SX::jacobian(I_EEposition, y_b), SX::jacobian(I_EEposition, z_b),
//                           SX::jacobian(I_EEposition, phi_b), SX::jacobian(I_EEposition, theta_b), SX::jacobian(I_EEposition, psi_b),
//                           SX::jacobian(I_EEposition, q1), SX::jacobian(I_EEposition, q2), SX::jacobian(I_EEposition, q3), SX::jacobian(I_EEposition, q4),
//                           SX::jacobian(I_EEposition, q10), SX::jacobian(I_EEposition, q11), 
//                           SX::jacobian(I_EEposition, q12), SX::jacobian(I_EEposition, q13),
//                            SX::jacobian(I_EEposition, q14), SX::jacobian(I_EEposition, q15)});

//     SX J_v_wo_fingers = J_v(Slice(), Slice(0, 10));

//     // %% Jacobian Dot  (diff --> jacobian  in CaSaDi)

//     // % jacobiandot은simplified homogeneous transform으로했을때만카피페이스트가능하게계산된다
//     SX J_vqdot = SX::mtimes(J_v, SX::vertcat({dx_b, dy_b, dz_b, dphi_b, dtheta_b, dpsi_b, dq1, dq2, dq3, dq4, dq10, dq11, dq12, dq13, dq14, dq15}));
//     J_vqdot_subs = substitute(J_vqdot,SX::horzcat({q10,q11,q12,q13,q14,q15,dq10, dq11, dq12, dq13, dq14, dq15}),
//                                     SX::zeros(1,12));

//     SX roundJ_v =  SX::horzcat({SX::jacobian(J_vqdot_subs,x_b),SX::jacobian(J_vqdot_subs,y_b),SX::jacobian(J_vqdot_subs,z_b),
//                                 SX::jacobian(J_vqdot_subs,phi_b),SX::jacobian(J_vqdot_subs,theta_b),SX::jacobian(J_vqdot_subs,psi_b),
//                                 SX::jacobian(J_vqdot_subs,q1),SX::jacobian(J_vqdot_subs,q2),SX::jacobian(J_vqdot_subs,q3),SX::jacobian(J_vqdot_subs,q4)
//                                 SX::jacobian(J_vqdot_subs,q10),SX::jacobian(J_vqdot_subs,q11),
//                                 SX::jacobian(J_vqdot_subs,q12),SX::jacobian(J_vqdot_subs,q13),
//                                 SX::jacobian(J_vqdot_subs,q14),SX::jacobian(J_vqdot_subs,q15)});       
//     SX longdJ_v = SX::mtimes(roundJ_v, SX::vertcat({dx_b, dy_b, dz_b, dphi_b, dtheta_b, dpsi_b, dq1, dq2, dq3, dq4, dq10, dq11, dq12, dq13, dq14, dq15}));  
//     SX dJ_v =   substitute(longdJ_v,SX::horzcat({q10,q11,q12,q13, q14, q15, dq10, dq11, dq12, dq13, dq14, dq15}),
//                                     SX::zeros(1,12));                                    
    
//     //------------------------------ Mass and Inertia matrix for each link------------------------------------//

//     SX mass_B = 0.46784;
//     SX comvector_B = SX::horzcat({0.0, 0.0, 0.1255});
//     SX inertia_B = SX::diag(SX::vertcat({0.000951270861568, 0.000951270861568, 0.000374272}));

//     SX mass_L1 = 0.7477;
//     SX comvector_L1 = SX::horzcat({0, -0.002, -0.0605});
//     SX inertia_L1 = SX::diag(SX::vertcat({0.00152031725204, 0.00152031725204, 0.00059816}));

//     SX mass_L2 = 0.99;
//     SX comvector_L2 = SX::horzcat({0 ,-0.2065, -0.01});
//     SX inertia_L2 = SX::diag(SX::vertcat({0.010502207991, 0.000792, 0.010502207991}));

//     SX mass_L3 = 0.6763;
//     SX comvector_L3 = SX::horzcat({0, 0.081, -0.0086});
//     SX inertia_L3 = SX::diag(SX::vertcat({0.00142022431908, 0.000304335, 0.00142022431908}));

//     SX mass_L4 = 0.99;
//     SX comvector_L4 = SX::horzcat({0 ,0 ,-0.06});
//     SX inertia_L4 = SX::diag(SX::vertcat({0.0003453236187, 0.0003453236187, 0.0005816}));

//     // COM Pos Transforms
//     SX T_IcomB = SX::mtimes(T_IB, SX::vertcat({SX::horzcat({SX::eye(3), comvector_B.T()}), SX::horzcat({SX::zeros(1,3), 1})}));
//     SX T_IcomL1 = SX::mtimes(T_IL1, SX::vertcat({SX::horzcat({SX::eye(3), comvector_L1.T()}), SX::horzcat({SX::zeros(1,3), 1})}));
//     SX T_IcomL2 = SX::mtimes(T_IL2, SX::vertcat({SX::horzcat({SX::eye(3), comvector_L2.T()}), SX::horzcat({SX::zeros(1,3), 1})}));
//     SX T_IcomL3 = SX::mtimes(T_IL3, SX::vertcat({SX::horzcat({SX::eye(3), comvector_L3.T()}), SX::horzcat({SX::zeros(1,3), 1})}));
//     SX T_IcomL4 = SX::mtimes(T_IL4, SX::vertcat({SX::horzcat({SX::eye(3), comvector_L4.T()}), SX::horzcat({SX::zeros(1,3), 1})}));

//     SX I_comBpos = T_IcomB(Slice(0, 3), 3);
//     SX I_comL1pos = T_IcomL1(Slice(0, 3), 3);
//     SX I_comL2pos = T_IcomL2(Slice(0, 3), 3);
//     SX I_comL3pos = T_IcomL3(Slice(0, 3), 3);
//     SX I_comL4pos = T_IcomL4(Slice(0, 3), 3);

//     //COM of whole arm
//     SX longcompos = (mass_B * I_comBpos + mass_L1 * I_comL1pos + mass_L2 * I_comL2pos + mass_L3 * I_comL3pos + mass_L4 * I_comL4pos) / (mass_B + mass_L1 + mass_L2 + mass_L3 + mass_L4);

//     // %% COM Jacobian Jv   (diff --> jacobian  in CaSaDi)
//     SX J_v_comB = SX::horzcat({SX::jacobian(I_comBpos, x_b), SX::jacobian(I_comBpos, y_b), SX::jacobian(I_comBpos, z_b),
//                                SX::jacobian(I_comBpos, phi_b), SX::jacobian(I_comBpos, theta_b), SX::jacobian(I_comBpos, psi_b),
//                                SX::jacobian(I_comBpos, q1), SX::jacobian(I_comBpos, q2), SX::jacobian(I_comBpos, q3), SX::jacobian(I_comBpos, q4),
//                                SX::jacobian(I_comBpos, q10), SX::jacobian(I_comBpos, q11),
//                                SX::jacobian(I_comBpos, q12), SX::jacobian(I_comBpos, q13),
//                                SX::jacobian(I_comBpos, q14), SX::jacobian(I_comBpos, q15)});

//     SX J_v_comL1 = SX::horzcat({SX::jacobian(I_comL1pos, x_b), SX::jacobian(I_comL1pos, y_b), SX::jacobian(I_comL1pos, z_b),
//                                 SX::jacobian(I_comL1pos, phi_b), SX::jacobian(I_comL1pos, theta_b), SX::jacobian(I_comL1pos, psi_b),
//                                 SX::jacobian(I_comL1pos, q1), SX::jacobian(I_comL1pos, q2), SX::jacobian(I_comL1pos, q3), SX::jacobian(I_comL1pos, q4),
//                                 SX::jacobian(I_comL1pos, q10), SX::jacobian(I_comL1pos, q11),
//                                 SX::jacobian(I_comL1pos, q12), SX::jacobian(I_comL1pos, q13),
//                                 SX::jacobian(I_comL1pos, q14), SX::jacobian(I_comL1pos, q15)});

//     SX J_v_comL2 = SX::horzcat({SX::jacobian(I_comL2pos, x_b), SX::jacobian(I_comL2pos, y_b), SX::jacobian(I_comL2pos, z_b),
//                                 SX::jacobian(I_comL2pos, phi_b), SX::jacobian(I_comL2pos, theta_b), SX::jacobian(I_comL2pos, psi_b),
//                                 SX::jacobian(I_comL2pos, q1), SX::jacobian(I_comL2pos, q2), SX::jacobian(I_comL2pos, q3), SX::jacobian(I_comL2pos, q4),
//                                 SX::jacobian(I_comL2pos, q10), SX::jacobian(I_comL2pos, q11),
//                                 SX::jacobian(I_comL2pos, q12), SX::jacobian(I_comL2pos, q13),
//                                 SX::jacobian(I_comL2pos, q14), SX::jacobian(I_comL2pos, q15)});                                                               

//     SX J_v_comL3 = SX::horzcat({SX::jacobian(I_comL3pos, x_b), SX::jacobian(I_comL3pos, y_b), SX::jacobian(I_comL3pos, z_b),
//                                 SX::jacobian(I_comL3pos, phi_b), SX::jacobian(I_comL3pos, theta_b), SX::jacobian(I_comL3pos, psi_b),
//                                 SX::jacobian(I_comL3pos, q1), SX::jacobian(I_comL3pos, q2), SX::jacobian(I_comL3pos, q3), SX::jacobian(I_comL3pos, q4),
//                                 SX::jacobian(I_comL3pos, q10), SX::jacobian(I_comL3pos, q11),
//                                 SX::jacobian(I_comL3pos, q12), SX::jacobian(I_comL3pos, q13),
//                                 SX::jacobian(I_comL3pos, q14), SX::jacobian(I_comL3pos, q15)});

//     SX J_v_comL4 = SX::horzcat({SX::jacobian(I_comL4pos, x_b), SX::jacobian(I_comL4pos, y_b), SX::jacobian(I_comL4pos, z_b),
//                                 SX::jacobian(I_comL4pos, phi_b), SX::jacobian(I_comL4pos, theta_b), SX::jacobian(I_comL4pos, psi_b),
//                                 SX::jacobian(I_comL4pos, q1), SX::jacobian(I_comL4pos, q2), SX::jacobian(I_comL4pos, q3), SX::jacobian(I_comL4pos, q4),
//                                 SX::jacobian(I_comL4pos, q10), SX::jacobian(I_comL4pos, q11),
//                                 SX::jacobian(I_comL4pos, q12), SX::jacobian(I_comL4pos, q13),
//                                 SX::jacobian(I_comL4pos, q14), SX::jacobian(I_comL4pos, q15)});          

//     //--------------------------------------COM vel------------------------------------//

//     SX I_comBvel = SX::mtimes(J_v_comB, SX::vertcat({dx_b, dy_b, dz_b, dphi_b, dtheta_b, dpsi_b, dq1, dq2, dq3, dq4, dq10, dq11, dq12, dq13, dq14, dq15}));  
//     SX I_comL1vel = SX::mtimes(J_v_comL1, SX::vertcat({dx_b, dy_b, dz_b, dphi_b, dtheta_b, dpsi_b, dq1, dq2, dq3, dq4, dq10, dq11, dq12, dq13, dq14, dq15}));       
//     SX I_comL2vel = SX::mtimes(J_v_comL2, SX::vertcat({dx_b, dy_b, dz_b, dphi_b, dtheta_b, dpsi_b, dq1, dq2, dq3, dq4, dq10, dq11, dq12, dq13, dq14, dq15}));
//     SX I_comL3vel = SX::mtimes(J_v_comL3, SX::vertcat({dx_b, dy_b, dz_b, dphi_b, dtheta_b, dpsi_b, dq1, dq2, dq3, dq4, dq10, dq11, dq12, dq13, dq14, dq15}));
//     SX I_comL4vel = SX::mtimes(J_v_comL4, SX::vertcat({dx_b, dy_b, dz_b, dphi_b, dtheta_b, dpsi_b, dq1, dq2, dq3, dq4, dq10, dq11, dq12, dq13, dq14, dq15}));

//     SX longcomvel = (mass_B * I_comBvel + mass_L1 * I_comL1vel + mass_L2 * I_comL2vel + mass_L3 * I_comL3vel + mass_L4 * I_comL4vel) / (mass_B + mass_L1 + mass_L2 + mass_L3 + mass_L4);  


//     //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%COM Jacobian method 1%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
//     // %Everything in world frame     
//     SX w_bB = SX::solve(A_Theta(SX::vertcat({phi_b, theta_b, psi_b})), SX::vertcat({dphi_b, dtheta_b, dpsi_b}));
//     SX w_bL1 = w_bB + SX::mtimes(T_IL1(Slice(0, 3), Slice(0, 3)), SX::vertcat({0,0,1})*q1); 
//     SX w_bL2 = w_bL1 + SX::mtimes(T_IL2(Slice(0, 3), Slice(0, 3)), SX::vertcat({0,0,1})*q2);
//     SX w_bL3 = w_bL2 + SX::mtimes(T_IL3(Slice(0, 3), Slice(0, 3)), SX::vertcat({0,0,1})*q3);
//     SX w_bL4 = w_bL3 + SX::mtimes(T_IL4(Slice(0, 3), Slice(0, 3)), SX::vertcat({0,0,1})*q4);

//     // %Jacobian w.r.t \dot(state)
//     SX J_w_comB = SX::horzcat({SX::jacobian(w_bB, dx_b), SX::jacobian(w_bB, dy_b), SX::jacobian(w_bB, dz_b),
//                               SX::jacobian(w_bB, dphi_b), SX::jacobian(w_bB, dtheta_b), SX::jacobian(w_bB, dpsi_b),
//                               SX::jacobian(w_bB, dq1), SX::jacobian(w_bB, dq2), SX::jacobian(w_bB, dq3), SX::jacobian(w_bB, dq4),
//                               SX::jacobian(w_bB, dq10), SX::jacobian(w_bB, dq11),
//                               SX::jacobian(w_bB, dq12), SX::jacobian(w_bB, dq13),
//                               SX::jacobian(w_bB, dq14), SX::jacobian(w_bB, dq15)});

//     SX J_w_comL1 = SX::horzcat({SX::jacobian(w_bL1, dx_b), SX::jacobian(w_bL1, dy_b), SX::jacobian(w_bL1, dz_b),
//                                SX::jacobian(w_bL1, dphi_b), SX::jacobian(w_bL1, dtheta_b), SX::jacobian(w_bL1, dpsi_b),
//                                SX::jacobian(w_bL1, dq1), SX::jacobian(w_bL1, dq2), SX::jacobian(w_bL1, dq3), SX::jacobian(w_bL1, dq4),
//                                SX::jacobian(w_bL1, dq10), SX::jacobian(w_bL1, dq11),
//                                SX::jacobian(w_bL1, dq12), SX::jacobian(w_bL1, dq13),
//                                SX::jacobian(w_bL1, dq14), SX::jacobian(w_bL1, dq15)});            

//     SX J_w_comL2 = SX::horzcat({SX::jacobian(w_bL2, dx_b), SX::jacobian(w_bL2, dy_b), SX::jacobian(w_bL2, dz_b),
//                                SX::jacobian(w_bL2, dphi_b), SX::jacobian(w_bL2, dtheta_b), SX::jacobian(w_bL2, dpsi_b),
//                                SX::jacobian(w_bL2, dq1), SX::jacobian(w_bL2, dq2), SX::jacobian(w_bL2, dq3), SX::jacobian(w_bL2, dq4),
//                                SX::jacobian(w_bL2, dq10), SX::jacobian(w_bL2, dq11),
//                                SX::jacobian(w_bL2, dq12), SX::jacobian(w_bL2, dq13),
//                                SX::jacobian(w_bL2, dq14), SX::jacobian(w_bL2, dq15)});

//     SX J_w_comL3 = SX::horzcat({SX::jacobian(w_bL3, dx_b), SX::jacobian(w_bL3, dy_b), SX::jacobian(w_bL3, dz_b),
//                                SX::jacobian(w_bL3, dphi_b), SX::jacobian(w_bL3, dtheta_b), SX::jacobian(w_bL3, dpsi_b),
//                                SX::jacobian(w_bL3, dq1), SX::jacobian(w_bL3, dq2), SX::jacobian(w_bL3, dq3), SX::jacobian(w_bL3, dq4),
//                                SX::jacobian(w_bL3, dq10), SX::jacobian(w_bL3, dq11),
//                                SX::jacobian(w_bL3, dq12), SX::jacobian(w_bL3, dq13),
//                                SX::jacobian(w_bL3, dq14), SX::jacobian(w_bL3, dq15)});

//     SX J_w_comL4 = SX::horzcat({SX::jacobian(w_bL4, dx_b), SX::jacobian(w_bL4, dy_b), SX::jacobian(w_bL4, dz_b),
//                                 SX::jacobian(w_bL4, dphi_b), SX::jacobian(w_bL4, dtheta_b), SX::jacobian(w_bL4, dpsi_b),
//                                 SX::jacobian(w_bL4, dq1), SX::jacobian(w_bL4, dq2), SX::jacobian(w_bL4, dq3), SX::jacobian(w_bL4, dq4),
//                                 SX::jacobian(w_bL4, dq10), SX::jacobian(w_bL4, dq11),
//                                 SX::jacobian(w_bL4, dq12), SX::jacobian(w_bL4, dq13),
//                                 SX::jacobian(w_bL4, dq14), SX::jacobian(w_bL4, dq15)});          

//     // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Inertia Matrix%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//

//     SX longinertiamatrix = mass_B*SX::mtimes(J_v_comB.T(),J_v_comB) + mass_L1*SX::mtimes(J_v_comL1.T(),J_v_comL1) + mass_L2*SX::mtimes(J_v_comL2.T(),J_v_comL2) 
//                             + mass_L3*SX::mtimes(J_v_comL3.T(),J_v_comL3) + mass_L4*SX::mtimes(J_v_comL4.T(),J_v_comL4) 
//                             + SX::mtimes(SX::mtimes(J_w_comB.T(),T_IB(Slice(0,3),Slice(0,3))),SX::mtimes(inertia_B,SX::mtimes(T_IB(Slice(0,3),Slice(0,3)).T(),J_w_comB)))
//                             + SX::mtimes(SX::mtimes(J_w_comL1.T(),T_IL1(Slice(0,3),Slice(0,3))),SX::mtimes(inertia_L1,SX::mtimes(T_IL1(Slice(0,3),Slice(0,3)).T(),J_w_comL1)))
//                             + SX::mtimes(SX::mtimes(J_w_comL2.T(),T_IL2(Slice(0,3),Slice(0,3))),SX::mtimes(inertia_L2,SX::mtimes(T_IL2(Slice(0,3),Slice(0,3)).T(),J_w_comL2)))
//                             + SX::mtimes(SX::mtimes(J_w_comL3.T(),T_IL3(Slice(0,3),Slice(0,3))),SX::mtimes(inertia_L3,SX::mtimes(T_IL3(Slice(0,3),Slice(0,3)).T(),J_w_comL3)))
//                             + SX::mtimes(SX::mtimes(J_w_comL4.T(),T_IL4(Slice(0,3),Slice(0,3))),SX::mtimes(inertia_L4,SX::mtimes(T_IL4(Slice(0,3),Slice(0,3)).T(),J_w_comL4)));

//     SX D_q = longinertiamatrix(Slice(0,10),Slice(0,10)); // % 10x10 Inertia matrix       % For now, disregard fingers' dynamics            

//     // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Nonlinearity vector%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//

//     // %%%% gravity vector  
//     SX G = 9.81;
//     SX long_Gravity = - SX::mtimes(J_v_comB.T(),SX::vertcat({0,0,-mass_B*G}))
//                         - SX::mtimes(J_v_comL1.T(),SX::vertcat({0,0,-mass_L1*G}))     
//                         - SX::mtimes(J_v_comL2.T(),SX::vertcat({0,0,-mass_L2*G}))
//                         - SX::mtimes(J_v_comL3.T(),SX::vertcat({0,0,-mass_L3*G}))
//                         - SX::mtimes(J_v_comL4.T(),SX::vertcat({0,0,-mass_L4*G}));

//     SX Gravity = long_Gravity(Slice(0,10),0); // % 10x1 gravity vector      % For now, disregard fingers' dynamics

//     /* % TODO: get the true gear ratio values for the arm joints from the urdf file
//     % Refer to Kinova URDF
//     % g1=160;
//     % g2=160;
//     % g3=160;
//     % g4=160;                    */

//     SX g1 = 1; SX g2 = 1; SX g3 = 1; SX g4 = 1; // % gear ratio

//     SX B_bar = blkdiag(g1,g2,g3,g4); // % gear ratio matrix
//     SX B  = SX::vertcat({SX::zeros(6,4),B_bar}); // % gear ratio matrix

//     SX J = SX::vertcat({SX::horzcat({SX::eye(3),SX::zeros(3,3),SX::zeros(3,4)}),SX::horzcat({SX::zeros(3,3),SX::eye(3),SX::zeros(3,4)})});
 

// }



#endif // NMPC_A1_Kinova