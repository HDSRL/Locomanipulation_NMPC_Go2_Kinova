#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <filesystem>
#include "a1_kinova_nmpc_params.h"
#include <array>
#include <vector>
#include <chrono>
#include <random>

using namespace casadi;
namespace fs = std::filesystem;


// int main(){
//     /*  Test problem
//      *
//      *    min x0^2 + x1^2
//      *    s.t.    x0 + x1 - 10 = 0
//      */


//     // file name
//     std::string file_name = "nlp_code";
//     // code predix
//     std::string prefix_code = fs::current_path().string() + "/";
//     // shared library prefix
//     std::string prefix_lib = fs::current_path().string() + "/";

//     // Create a new NLP solver instance from the compiled code
//     std::string lib_name = prefix_lib + file_name + ".so";
//     casadi::Function solver = casadi::nlpsol("solver", "ipopt", lib_name);

//     // Bounds and initial guess
//     std::map<std::string, casadi::DM> arg, res;
//     arg["lbx"] = -casadi::DM::inf();
//     arg["ubx"] =  casadi::DM::inf();
//     arg["lbg"] =  0;
//     arg["ubg"] =  casadi::DM::inf();
//     arg["x0"] = 0;

//     // Solve the NLP
//     res = solver(arg);

//     // Print solution
//     std::cout << "-----" << std::endl;
//     std::cout << "objective at solution = " << res.at("f") << std::endl;
//     std::cout << "primal solution = " << res.at("x") << std::endl;
//     std::cout << "dual solution (x) = " << res.at("lam_x") << std::endl;
//     std::cout << "dual solution (g) = " << res.at("lam_g") << std::endl;

//     return 0;
// }

int main(){
    // File path setup
    // bool code_gen = 1; // Set to 1 to generate code
    std::string basePath = "tmp/";
    fs::create_directories(basePath); // Ensure logging directory exists



    // casadi::Function solver = casadi::nlpsol("solver","ipopt", "nlp_code.c");

    // =========================== SRB NMPC =========================== //
    Params params(0.01); // 0.01 is the sample time for MPC 
    const double sim_time = 200.0; 
    // const size_t N = 12; // Prediction horizon
    const double dt = 0.01; // Sample time
    // const double Fs = 1.0/dt; // Control Frequency
    // const double mu = 0.6; // Friction Coefficient
    const float sim_length = sim_time/dt; // Simulation length
    // const double height = 0.28; // Stand height
    const int n_states = 12+20; // Number of states
    const int n_inputs = 12+4+6; // Number of inputs
    const int n_controls = 12+4+6; // Number of inputs
    const int gait = 8; // Gait type
    // const double G = 9.81; // Gravity
    // const double mass = 12.4530; // Mass
    // const int agentIndex = 1;

    DM qk = DM::zeros(n_states);

    // int indexOffset = (agentIndex - 1) * 2;

    qk(2) = 0.08; // Assuming z position is constant or predefined

    // %%%%--arm ICs--%%%%
    // % The initial condition for the arm's base should match those of the
    // % holonomically fixed to the A1
    qk(Slice(12,15)) = qk(Slice(0,3)) + DM::mtimes(RotMat(qk(Slice(6,9))),SX(params.dd));

    // % Arm's base orientation
    qk(Slice(15,18)) = qk(Slice(6,9));

    //     % Shape variable ICs: for now, consider the arm with fixed joint angles
    // % x0(19:22) = [ZZero; 2; 5;ZZero];  % q_s = [q1 q2 q3 q4] arm's joints (rad)
    qk(Slice(18,22)) = DM::vertcat({0.0, 0.9, 2.4, 0.0});//% q_s = [q1 q2 q3 q4] arm's joints (rad)

    // % Arm's base velocity
    qk(Slice(22,25)) = qk(Slice(3,6)) + DM::mtimes(skewSym(qk(Slice(9,12))),SX::mtimes(RotMat(qk(Slice(6,9))),SX(params.dd)));

    //% Arm's base orientation rate
    qk(Slice(25,28)) = DM::mtimes(A_Theta(qk(Slice(6,9))),qk(Slice(9,12)));

//     % x0(26:28) = [ZZero; ZZero; ZZero];
// x0(29:32) = [ZZero;ZZero;ZZero;ZZero];  % dq_s = [dq1 dq2 dq3 dq4] arm's joints rates (rad/s)

    double t0 = 0.0;

    // Initialize matrices for storing states and control inputs
    // Each column represents the state vector or control input vector at a specific timestep
    // DM xx(n_states, sim_length);
    // xx(Slice(0, n_states), 0) = qk;  

    DM u0 = DM::zeros(params.N, params.n_inputs); // Control inputs for N horizon steps, not sim_steps
    std::cout<<"params.N: "<<params.N<<std::endl;
    

    // States decision variables initialization
    DM X0 = repmat(qk, 1, params.N+1);
    DM xd = repmat(qk, params.N, 1);

    double current_time = 0.0;
    size_t mpciter = 0;

    Args args;
    // % The 1st part will remain zero, because lower and upper bound of "equality"
    // % constraints are always zero
    // % The 2nd part: we have zeros for lower and upper bound of Holonomic Constraints
    // % The 3rd part: i.e. 4*4*N is for friction cone of "f_x" and "f_y": having 4 feet, each having 4 expressions
    // % The 4th part: arm torques (4) plus the interaction wrench (6)
    args.lbg = DM::zeros(n_states * (params.N + 1) + params.NumHolCon * params.N + 4*4*params.N + (4 + 6) * params.N, 1);
    args.ubg = DM::zeros(n_states * (params.N + 1) + params.NumHolCon * params.N + 4*4*params.N + (4 + 6) * params.N, 1);
        
    DM uout =  DM::zeros(params.n_controls, sim_length); // Vector to store control outputs
    DM xout =  DM::zeros(params.n_states, sim_length);                                      
    DM feet =  DM::zeros(12, sim_length); // Vector to store foot positions
    DM traj =  DM::zeros(12+20, sim_length); // Vector to store desired trajectory
    DM tout =  DM::zeros(1, sim_length);                                                  
    DM NMPC_solve_time = DM::zeros(1, sim_length);  

    SX U = SX::sym("U", n_controls, params.N); //% Decision variables (controls) for all horizons
    SX P = SX::sym("P", n_states + n_states * params.N + params.n_u_SRB +  12 + params.n_SRB + params.n_arm + params.n_SRB + params.n_arm); //% a parameter to store the current state and the desired future (for all horizons)
// % This is a multipl-shooting in which we strore both states and inputs
    SX X = SX::sym("X", n_states, params.N + 1); //% A Matrix that represents the states over the optimization problem.
// % N+1: Because we store the current states in the 1st column
    Dict opts;
    // opts["snopt.Iterations limit"] = iter_lim;
    // opts["snopt.Major Print level"] = 3;
    // opts["snopt.Minor Print level"] = 3;
    // opts["snopt.Derivative option"] = 0;
    // opts["snopt.Major feasibility tolerance"] = 1.0e-2;
    // opts["snopt.Minor feasibility tolerance"] = 1.0e-2;
    // opts["snopt.Major optimality tolerance"] = 1.0e-2;
    // file name
    std::string file_name = "a1_kinova_nlp_code";
    std::string prefix_code = fs::current_path().string() + "/";
    std::string prefix_lib = fs::current_path().string() + "/";
    std::string lib_name = prefix_lib + file_name + ".so";
    // 
    // casadi::Function solver = external("solver", lib_name);

    opts["ipopt.max_iter"] = 10;  // Replace Max_mpciter with its actual value
    opts["ipopt.print_level"] = 3;  // Can be changed to 0 or 3 based on verbosity required
    opts["print_time"] = 0;  // Disable printing solver time
    opts["ipopt.acceptable_tol"] = 1e-2;  // Tolerance for stopping criterion
    opts["ipopt.acceptable_obj_change_tol"] = 1e-2;  // Objective change tolerance for stopping

    casadi::Function solver = casadi::nlpsol("solver","ipopt", lib_name, opts);
    lib_name = prefix_lib + file_name + "_f" ".so";
    casadi::Function f_SRB = external("f_SRB", lib_name);
    casadi::Function f_arm = external("f_arm", lib_name);
    casadi::Function f_SRB_reduced = external("f_SRB_reduced", lib_name);
    casadi::Function f_arm_reduced = external("f_arm_reduced", lib_name);

    while(mpciter < sim_time / dt) {

        // DEBUG SESSION //
        params.current_time = current_time;
        params.mpc_iter = mpciter;
        
        // Updates the xd to obtain desired 12-state reference for next N domains
        planner(current_time, qk, xd, gait, params);

        writeMatrixToFile(qk, basePath + "qk.txt");
        writeMatrixToFile(xd, basePath + "xd.txt"); 
        writeMatrixToFile(params, basePath + "params.txt");

        // DM J = DM::zeros(3,3); // Assuming a paramse-5; J(2,2) = 0.064713601;.

        

        // auto iter_lim = 0;

        // if(current_time > 1.1)
        //     iter_lim = 100;
        // else
        //     iter_lim = 200;

        

        // Declare variables to hold the outputs
        SX g_force, g_ub, g_lb;

        // Call the function
        force_inequality(U, params, g_force, g_ub, g_lb);

        // writeMatrixToFile(g_ub, basePath + "g_ub.txt");
        // writeMatrixToFile(g_lb, basePath + "g_lb.txt");
        // writeMatrixToFile(params, basePath + "params.txt");

        /* g is already defined in the solver object 
        SX obj = 0, g_eq; 
        equalityconstraints(X, U, P, params, obj, g_eq, f_SRB, f_arm);

        // writeMatrixToFile(obj, basePath + "obj.txt");
        // writeMatrixToFile(g_eq, basePath + "g_eq.txt");

        SX g = vertcat(g_eq, g_force); */

        // args.lbg = DM::zeros(n_states * (N + 1) + 4*4*N, 1);
        // args.ubg = DM::zeros(n_states * (N + 1) + 4*4*N, 1);
        set_statebounds(mpciter, params, args);
        set_constraint_bounds(args, g_lb, g_ub, n_states, params.N, params.NumHolCon);

        // set_statebounds(mpciter, params, args);
        // set_constraint_bounds(args, g_lb, g_ub, n_states, params.N,params.NumHolCon);


        

        // Reshape X0 and u0 transposed into column vectors
        DM X0_col = reshape(X0.T(), n_states * (params.N + 1), 1);
        DM u0_col = reshape(u0.T(), n_inputs * params.N, 1);

        // Concatenate reshaped vectors vertically
        DM x0u0= DM::vertcat({X0_col, u0_col});
        args.x0 = x0u0;
        args.p = DM::vertcat({qk, xd,reshape(params.pf,12,1), params.Q_SRB, params.Q_arm, params.R_SRB, params.R_arm, params.R_lambda, params.P_SRB, params.P_arm});
        // Solve the problem
        // DMDict arg = {{"x0", args.x0}, {"lbx", args.lbx}, {"ubx", args.ubx}, {"lbg", args.lbg}, {"ubg", args.ubg}, {
        //     "p", args.p}};
        // DMDict res = solver(arg);

        // Bounds and initial guess
        std::map<std::string, casadi::DM> arg, res;
        arg["lbx"] = args.lbx;
        arg["ubx"] =  args.ubx;
        arg["lbg"] =  args.lbg;
        arg["ubg"] =  args.ubg;
        arg["x0"] = args.x0;
        arg["p"] = args.p;

        writeMatrixToFile(args.lbx, basePath + "args_lbx.txt");
        writeMatrixToFile(args.ubx, basePath + "args_ubx.txt");
        writeMatrixToFile(args.lbg, basePath + "args_lbg.txt");
        writeMatrixToFile(args.ubg, basePath + "args_ubg.txt");
        writeMatrixToFile(args.x0, basePath + "args_x0.txt");
        writeMatrixToFile(args.p, basePath + "args_p.txt");
        
        auto start = std::chrono::high_resolution_clock::now();
        res = solver(arg);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << "NMPC Solve Time: " << duration.count() << "ms" << std::endl;


        DM sol = DM::zeros(n_states * (params.N + 1) + n_controls * params.N, 1); // sol.x should be initialized with actual solution data
        sol = res.at("x");

        casadi_int start_index_u = static_cast<casadi_int>(n_states * (params.N + 1));
        casadi_int end_index = static_cast<casadi_int>(sol.numel());

        DM u = reshape(sol(Slice(start_index_u, end_index)).T(), n_controls, params.N).T();
        DM sol_x_N = reshape(sol(Slice(0, static_cast<casadi_int>(n_states * (params.N + 1)))).T(), n_states, params.N + 1).T();
        appendMatrixToFile(sol_x_N, basePath + "sol_x_N.txt");

        uout(Slice(), mpciter) = u(0, Slice());     // Store current control outputs
        shift(t0, qk, u0, dt, u, f_SRB, f_arm, params);
       /*  std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<double> d(0, 0.1);
        DM noise = DM::zeros(32,1);
        for(int i = 0; i < 32; i++){
            noise(i) = d(gen);
        } */
        // std::cout<<"Noise: "<<noise<<std::endl;
        X0 = DM::vertcat({sol_x_N(Slice(1, sol_x_N.size1()), Slice()), sol_x_N(Slice(sol_x_N.size1() - 1, sol_x_N.size1()), Slice())});
        
        // Data Logging

        xout(Slice(), mpciter) = qk.T(); // Store current states, transposed
        feet(Slice(), mpciter) = reshape(params.pf, 1, 12); // Store reshaped foot positions
        tout(0, mpciter) = t0;          // Store current time
        NMPC_solve_time(0, mpciter) = DM(duration.count()); // Store NMPC solve time

        
        // Function f = Function("f", {states, controls}, {rhs}, {"states", "controls"}, {"rhs"});

        mpciter += 1;
        current_time += dt;
        std::cout << "Current Time: " << current_time << std::endl;
        
        // SX g_force; DM g_ub, g_lb;
        // force_inequality(U, params, g_force, g_ub, g_lb);

    }

    writeMatrixToFile(xout.T(), basePath + "xout_use.txt");
    writeMatrixToFile(feet.T(), basePath + "feet_use.txt");
    writeMatrixToFile(tout.T(), basePath + "tout_use.txt");
    writeMatrixToFile(uout.T(), basePath + "uout_use.txt");
    writeMatrixToFile(NMPC_solve_time.T(), basePath + "NMPC_solve_time_use.txt");

    // return 0;
}

