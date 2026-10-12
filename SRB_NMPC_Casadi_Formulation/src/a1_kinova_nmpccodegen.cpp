#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <filesystem>
#include "a1_kinova_nmpc_params.h"
#include <array>
#include <vector>
#include <D_mat_pin.hpp>
#include <G_vec_pin.hpp>

using namespace casadi;
namespace fs = std::filesystem;


int main(){

    // File path setup
    bool code_gen = 1; // Set to 1 to generate code
    std::string basePath = "tmp/";
    fs::create_directories(basePath); // Ensure directory exists
    
    std::string file_name_Dq = "D_mat_pin";
    std::string file_name_grav = "G_vec_pin";
    std::string prefix_lib = fs::current_path().string() + "/../dynamics/"; // run from SRB_NMPC_Casadi_Formulation/build
    std::string lib_path_Dq = prefix_lib + file_name_Dq + ".casadi";
    std::string lib_path_grav = prefix_lib + file_name_grav + ".casadi";
    // =========================== SRB NMPC =========================== //
    Params params(0.0167); // 0.01 is the sample time for MPC 
    params.dd = {-0.04,0,0.171}; // Position vector of the arm's base w.r.t the A1 body frame
    const double sim_time = 20.0; 
    // const size_t N = 12; // Prediction horizon
    const double dt = 0.0167; // Sample time
    // const double Fs = 1.0/dt; // Control Frequency
    // const double mu = 0.6; // Friction Coefficient
    const float sim_length = sim_time/dt; // Simulation length
    // const double height = 0.28; // Stand height
    // const int n_states = 12 + 20; // Number of states
    // const int n_inputs = 12 + 4 + 6; // Number of inputs
    // const int n_controls = 12 + 4 + 6; // Number of inputs
    const int gait = 8; // Gait type
    const double G = 9.81; // Gravity
    DM arm_pos = {0.0,3.49066,5.67232,0.0};//{0.0,2.68, 1.051,0.0};//{0.0,3.75066,5.67232,0.0};//{0.0,2.68, 1.051,0.0};//{0.0,2.68, 1.051,0.0};//{0.0, 0.9, 2.4, 0.0}; // Position vector of the arm's base w.r.t the A1 body frame
    // const int agentIndex = 1;
    //Wrap the desired position to the range of [-pi, pi]
    /* for (size_t i = 0; i < 4; ++i)
    {
        arm_pos(i) = (static_cast<double>(arm_pos(i)) >= M_PI)? arm_pos(i) - 2*M_PI : (static_cast<double>(arm_pos(i)) <= -M_PI)?  arm_pos(i) + 2*M_PI :arm_pos(i) ;
    } */
    // Symbolic variables 
    // %% Define symbolic states

    // %%%%%--A1--%%%%%
    // % We have many x's in this code, so I named A1's position as  "Xx"
    casadi::SXVector states_SRB_vec = {
        SX::sym("Xx"), SX::sym("y"), SX::sym("z"), // % A1's position w.r.t Inertial frame
        SX::sym("dx"), SX::sym("dy"), SX::sym("dz"), //% A1's velocity w.r.t Inertial frame
        SX::sym("phi"), SX::sym("theta"), SX::sym("psi"), // % A1's orientation w.r.t Inertial frame
        SX::sym("omega_x"), SX::sym("omega_y"), SX::sym("omega_z") //% A1's angular velocity w.r.t Inertial frame
    };

   
    casadi::SX states_SRB = casadi::SX::vertcat(states_SRB_vec);
    // %%%%%--Arm--%%%%% 
      // %floating base (A1's position and orientation)
    SX x_b = SX::sym("x_b"); SX y_b = SX::sym("y_b");SX z_b = SX::sym("z_b");
    SX phi_b = SX::sym("phi_b"); SX theta_b = SX::sym("theta_b"); SX psi_b = SX::sym("psi_b"); //% syms Xx y z  real; % syms  phi_b theta_b psi_b real
    SX dx_b = SX::sym("dx_b"); SX dy_b = SX::sym("dy_b"); SX dz_b = SX::sym("dz_b");
    SX dphi_b = SX::sym("dphi_b"); SX dtheta_b = SX::sym("dtheta_b"); SX dpsi_b = SX::sym("dpsi_b"); //% syms dx_b dy_b dz_b dphi_b dtheta_b dpsi_b real
    SX q1 = SX::sym("q1"); SX q2 = SX::sym("q2"); SX q3 = SX::sym("q3"); SX q4 = SX::sym("q4"); //% syms q1 q2 q3 q4 real 
    SX q10 = SX::sym("q10"); SX q11 = SX::sym("q11"); SX q12 = SX::sym("q12"); SX q13 = SX::sym("q13"); SX q14 = SX::sym("q14"); SX q15 = SX::sym("q15");//Fingers
    SX dq1 = SX::sym("dq1"); SX dq2 = SX::sym("dq2"); SX dq3 = SX::sym("dq3"); SX dq4 = SX::sym("dq4"); //% syms dq1 dq2 dq3 dq4 real
    SX dq10 = SX::sym("dq10"); SX dq11 = SX::sym("dq11"); SX dq12 = SX::sym("dq12"); SX dq13 = SX::sym("dq13"); SX dq14 = SX::sym("dq14"); SX dq15 = SX::sym("dq15");//Fingers
    
    SX q_in = SX::vertcat({x_b, y_b, z_b, phi_b, theta_b, psi_b, q1, q2, q3, q4, q10, q11, q12, q13, q14, q15});
    SX dq_in = SX::vertcat({dx_b, dy_b, dz_b, dphi_b, dtheta_b, dpsi_b, dq1, dq2, dq3, dq4, dq10, dq11, dq12, dq13, dq14, dq15});
    
    SX D_q_arm, Gravity_arm, B_arm, J_arm, q_arm, dq_arm;

    q_arm = SX::vertcat({x_b, y_b, z_b, phi_b, theta_b, psi_b, q1, q2, q3, q4});
    dq_arm = SX::vertcat({dx_b, dy_b, dz_b, dphi_b, dtheta_b, dpsi_b, dq1, dq2, dq3, dq4});

    SX D_q_arm_pin = D_mat_pin(q_arm);
    SX Gravity_arm_pin = G_vec_pin(q_arm);
    //%%%%%%%%%%%%%%%%%%%%%% KINOVA DYNAMICS PINOCCHIO %%%%%%%%%%%%%%%%%%%%%%%%%%% 
    /* Function Dq_arm_pin = Function::load(lib_path_Dq);   
    Function Gravity_arm_pin = Function::load(lib_path_grav);   
    

    std::map<std::string, SX> arg_D_q_arm_pin;
    arg_D_q_arm_pin["q_eul"] = q_arm;
    std::map<std::string, SX> result_D_q_arm_pin= Dq_arm_pin(arg_D_q_arm_pin);
    SX D_q_arm_pin_sx = result_D_q_arm_pin["D_mat"];

    std::map<std::string, SX> arg_Gravity_arm_pin;
    arg_Gravity_arm_pin["q_eul"] = q_arm;
    std::map<std::string, SX> result_Gravity_arm_pin= Gravity_arm_pin(arg_Gravity_arm_pin);
    SX Gravity_arm_pin_sx = result_Gravity_arm_pin["G_vec"]; */

/*     %note: every rotation axis is aligned as Z axis: DH parameter-like

    % framename: I => (x_b)-I1 => (y_b)-I2 => (z_b)-I3 => (phi_b)-I4 => (theta)-I5 => 
    %            (psi_b)-B(base) => (q1)-L1 => (q2)-L2 => (q3)-L3 => (q4)-L4 =>

    
    % H-transform format: [[rotation mtx],[translation vector]';[0 0 0 1]] */
    // %% Homogeneous Transform
    SX T_II3 = SX::vertcat({SX::horzcat({SX::eye(3),SX::vertcat({x_b,y_b,z_b})}), SX::horzcat({SX::zeros(1,3), 1})});
    SX T_II4 = SX::mtimes(T_II3, SX::vertcat({SX::horzcat({rot_z(psi_b),SX::zeros(3,1)}), SX::horzcat({SX::zeros(1,3), 1})}));
    SX T_II5 = SX::mtimes(T_II4, SX::vertcat({SX::horzcat({rot_y(theta_b),SX::zeros(3,1)}), SX::horzcat({SX::zeros(1,3), 1})}));

    // %%% TODO: why don't we add an explicit rotation around z-axis here?
    // %%%%%%%%%%%%--???????????--%%%%%%%%%%%%%%%%
    casadi::SX pi = casadi::SX(M_PI);
    casadi::SX half_pi = casadi::SX(M_PI/2);

    //====================arm pos changable (Transformation from arm's base to the world Inertia frame)====================//
    //%ZYX convention
    SX T_IB = SX::mtimes(T_II5, SX::vertcat({SX::horzcat({rot_x(phi_b),SX::zeros(3,1)}), SX::horzcat({SX::zeros(1,3), 1})}));
    //Likely incorrect
    // SX T_IB = T_II3;

    SX T_BL1_1 = SX::vertcat({SX::horzcat({rot_y(pi),SX::vertcat({0,0,0.15675})}),SX::horzcat({SX::zeros(1,3),1})});
    SX T_BL1_2 = SX::vertcat({SX::horzcat({rot_z(q1),SX::zeros(3,1)}),SX::horzcat({SX::zeros(1,3),1})});
    SX T_BL1 = SX::mtimes(T_BL1_1, T_BL1_2);

    SX T_L1L2_1 = SX::vertcat({SX::horzcat({SX::eye(3),SX::vertcat({0,0.0016,-0.11875})}),SX::horzcat({SX::zeros(1,3),1})});
    SX T_L1L2_2 = SX::vertcat({SX::horzcat({rot_z(pi),SX::zeros(3,1)}),SX::horzcat({SX::zeros(1,3),1})});
    SX T_L1L2_3 = SX::vertcat({SX::horzcat({rot_x(-half_pi),SX::zeros(3,1)}),SX::horzcat({SX::zeros(1,3),1})});
    SX T_L1L2_4 = SX::vertcat({SX::horzcat({rot_z(q2),SX::zeros(3,1)}),SX::horzcat({SX::zeros(1,3),1})});
    SX T_L1L2 = SX::mtimes(T_L1L2_1, SX::mtimes(T_L1L2_2, SX::mtimes(T_L1L2_3, T_L1L2_4)));

    SX T_L2L3_1 = SX::vertcat({SX::horzcat({SX::eye(3),SX::vertcat({0,-0.410,0})}),SX::horzcat({SX::zeros(1,3),1})});
    SX T_L2L3_2 = SX::vertcat({SX::horzcat({rot_y(pi),SX::zeros(3,1)}),SX::horzcat({SX::zeros(1,3),1})});
    SX T_L2L3_3 = SX::vertcat({SX::horzcat({rot_z(q3),SX::zeros(3,1)}),SX::horzcat({SX::zeros(1,3),1})});
    SX T_L2L3 = SX::mtimes(T_L2L3_1, SX::mtimes(T_L2L3_2, T_L2L3_3));

    SX T_L3L4_1 = SX::vertcat({SX::horzcat({SX::eye(3),SX::vertcat({0,0.2073,-0.0114})}),SX::horzcat({SX::zeros(1,3),1})});
    SX T_L3L4_2 = SX::vertcat({SX::horzcat({rot_z(pi),SX::zeros(3,1)}),SX::horzcat({SX::zeros(1,3),1})});
    SX T_L3L4_3 = SX::vertcat({SX::horzcat({rot_x(-half_pi),SX::zeros(3,1)}),SX::horzcat({SX::zeros(1,3),1})});
    SX T_L3L4_4 = SX::vertcat({SX::horzcat({rot_z(q4),SX::zeros(3,1)}),SX::horzcat({SX::zeros(1,3),1})});
    SX T_L3L4 = SX::mtimes(T_L3L4_1, SX::mtimes(T_L3L4_2, SX::mtimes(T_L3L4_3, T_L3L4_4)));

    SX T_L4EE_1 = SX::vertcat({SX::horzcat({SX::eye(3),SX::vertcat({0,0,-0.16})}),SX::horzcat({SX::zeros(1,3),1})});
    SX T_L4EE_2 = SX::vertcat({SX::horzcat({rot_z(half_pi),SX::zeros(3,1)}),SX::horzcat({SX::zeros(1,3),1})});
    SX T_L4EE_3 = SX::vertcat({SX::horzcat({rot_x(pi),SX::zeros(3,1)}),SX::horzcat({SX::zeros(1,3),1})});
    SX T_L4EE = SX::mtimes(T_L4EE_1, SX::mtimes(T_L4EE_2, T_L4EE_3));

    SX T_IL1 = SX::mtimes(T_IB, T_BL1);
    SX T_IL2 = SX::mtimes(T_IL1, T_L1L2);
    SX T_IL3 = SX::mtimes(T_IL2, T_L2L3);
    SX T_IL4 = SX::mtimes(T_IL3, T_L3L4);
    SX T_IEE = SX::mtimes(T_IL4, T_L4EE);

    //forward kinematics

    SX I_EEposition = T_IEE(Slice(0, 3), 3);

    // %% end effector Jacobian  (diff --> jacobian  in CaSaDi)
    SX J_v = SX::horzcat({SX::jacobian(I_EEposition, x_b), SX::jacobian(I_EEposition, y_b), SX::jacobian(I_EEposition, z_b),
                          SX::jacobian(I_EEposition, phi_b), SX::jacobian(I_EEposition, theta_b), SX::jacobian(I_EEposition, psi_b),
                          SX::jacobian(I_EEposition, q1), SX::jacobian(I_EEposition, q2), SX::jacobian(I_EEposition, q3), SX::jacobian(I_EEposition, q4),
                          SX::jacobian(I_EEposition, q10), SX::jacobian(I_EEposition, q11), 
                          SX::jacobian(I_EEposition, q12), SX::jacobian(I_EEposition, q13),
                           SX::jacobian(I_EEposition, q14), SX::jacobian(I_EEposition, q15)});

    SX J_v_val = SX::substitute(J_v, SX::horzcat({x_b,y_b,z_b, phi_b,theta_b,psi_b,q1,q2,q3,q4,q10,q11,q12,q13,q14,q15}), SX::horzcat({7.56355e-21, 0.4, 0.379941, 0.0, 0.0, 0, 3.8497e-20, 0.9, 2.4, 7.96129e-21, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
    writeMatrixToFile(J_v_val, basePath + "J_v_val.txt");

    SX J_v_wo_fingers = J_v(Slice(), Slice(0, 10));

    // %% Jacobian Dot  (diff --> jacobian  in CaSaDi)

    // % jacobiandot은simplified homogeneous transform으로했을때만카피페이스트가능하게계산된다
    SX J_vqdot = SX::mtimes(J_v, SX::vertcat({dx_b, dy_b, dz_b, dphi_b, dtheta_b, dpsi_b, dq1, dq2, dq3, dq4, dq10, dq11, dq12, dq13, dq14, dq15}));
    SX J_vqdot_subs = SX::substitute(J_vqdot,SX::horzcat({q10,q11,q12,q13,q14,q15,dq10, dq11, dq12, dq13, dq14, dq15}),
                                    SX::zeros(1,12));

    SX roundJ_v =  SX::horzcat({SX::jacobian(J_vqdot_subs,x_b),SX::jacobian(J_vqdot_subs,y_b),SX::jacobian(J_vqdot_subs,z_b),
                                SX::jacobian(J_vqdot_subs,phi_b),SX::jacobian(J_vqdot_subs,theta_b),SX::jacobian(J_vqdot_subs,psi_b),
                                SX::jacobian(J_vqdot_subs,q1),SX::jacobian(J_vqdot_subs,q2),SX::jacobian(J_vqdot_subs,q3),SX::jacobian(J_vqdot_subs,q4),
                                SX::jacobian(J_vqdot_subs,q10),SX::jacobian(J_vqdot_subs,q11),
                                SX::jacobian(J_vqdot_subs,q12),SX::jacobian(J_vqdot_subs,q13),
                                SX::jacobian(J_vqdot_subs,q14),SX::jacobian(J_vqdot_subs,q15)});       
    SX longdJ_v = SX::mtimes(roundJ_v, SX::vertcat({dx_b, dy_b, dz_b, dphi_b, dtheta_b, dpsi_b, dq1, dq2, dq3, dq4, dq10, dq11, dq12, dq13, dq14, dq15}));  
    SX dJ_v =   SX::substitute(longdJ_v,SX::horzcat({q10,q11,q12,q13, q14, q15, dq10, dq11, dq12, dq13, dq14, dq15}),
                                    SX::zeros(1,12));     

    // writeMatrixToFile(J_v, basePath + "J_v.txt");                               
    
    //------------------------------ Mass and Inertia matrix for each link------------------------------------//
    SX gain = 1;
    SX mass_B = gain*0.46784;
    SX comvector_B = SX::horzcat({0.0, 0.0, 0.1255});
    SX inertia_B = gain*SX::diag(SX::vertcat({0.000951270861568, 0.000951270861568, 0.000374272}));

    SX mass_L1 = gain*0.7477;
    SX comvector_L1 = SX::horzcat({0, -0.002, -0.0605});
    SX inertia_L1 = gain*SX::diag(SX::vertcat({0.00152031725204, 0.00152031725204, 0.00059816}));

    SX mass_L2 = gain*0.99;
    SX comvector_L2 = SX::horzcat({0 ,-0.2065, -0.01});
    SX inertia_L2 = gain*SX::diag(SX::vertcat({0.010502207991, 0.000792, 0.010502207991}));

    SX mass_L3 = gain*0.6763;
    SX comvector_L3 = SX::horzcat({0, 0.081, -0.0086});
    SX inertia_L3 = gain*SX::diag(SX::vertcat({0.00142022431908, 0.000304335, 0.00142022431908}));

    SX mass_L4 = gain*0.99;
    SX comvector_L4 = SX::horzcat({0 ,0 ,-0.06});
    SX inertia_L4 = gain*SX::diag(SX::vertcat({0.0003453236187, 0.0003453236187, 0.0005816}));

    // COM Pos Transforms
    SX T_IcomB = SX::mtimes(T_IB, SX::vertcat({SX::horzcat({SX::eye(3), comvector_B.T()}), SX::horzcat({SX::zeros(1,3), 1})}));
    SX T_IcomL1 = SX::mtimes(T_IL1, SX::vertcat({SX::horzcat({SX::eye(3), comvector_L1.T()}), SX::horzcat({SX::zeros(1,3), 1})}));
    SX T_IcomL2 = SX::mtimes(T_IL2, SX::vertcat({SX::horzcat({SX::eye(3), comvector_L2.T()}), SX::horzcat({SX::zeros(1,3), 1})}));
    SX T_IcomL3 = SX::mtimes(T_IL3, SX::vertcat({SX::horzcat({SX::eye(3), comvector_L3.T()}), SX::horzcat({SX::zeros(1,3), 1})}));
    SX T_IcomL4 = SX::mtimes(T_IL4, SX::vertcat({SX::horzcat({SX::eye(3), comvector_L4.T()}), SX::horzcat({SX::zeros(1,3), 1})}));

    SX I_comBpos = T_IcomB(Slice(0, 3), 3);
    SX I_comL1pos = T_IcomL1(Slice(0, 3), 3);
    SX I_comL2pos = T_IcomL2(Slice(0, 3), 3);
    SX I_comL3pos = T_IcomL3(Slice(0, 3), 3);
    SX I_comL4pos = T_IcomL4(Slice(0, 3), 3);

    //COM of whole arm
    SX longcompos = (mass_B * I_comBpos + mass_L1 * I_comL1pos + mass_L2 * I_comL2pos + mass_L3 * I_comL3pos + mass_L4 * I_comL4pos) / (mass_B + mass_L1 + mass_L2 + mass_L3 + mass_L4);
    SX mass_arm = mass_B + mass_L1 + mass_L2 + mass_L3 + mass_L4;


    
    // %% COM Jacobian Jv   (diff --> jacobian  in CaSaDi)
    SX J_v_comB = SX::horzcat({SX::jacobian(I_comBpos, x_b), SX::jacobian(I_comBpos, y_b), SX::jacobian(I_comBpos, z_b),
                               SX::jacobian(I_comBpos, phi_b), SX::jacobian(I_comBpos, theta_b), SX::jacobian(I_comBpos, psi_b),
                               SX::jacobian(I_comBpos, q1), SX::jacobian(I_comBpos, q2), SX::jacobian(I_comBpos, q3), SX::jacobian(I_comBpos, q4),
                               SX::jacobian(I_comBpos, q10), SX::jacobian(I_comBpos, q11),
                               SX::jacobian(I_comBpos, q12), SX::jacobian(I_comBpos, q13),
                               SX::jacobian(I_comBpos, q14), SX::jacobian(I_comBpos, q15)});

    SX J_v_comB_val = SX::substitute(J_v_comB, SX::horzcat({x_b,y_b,z_b, phi_b,theta_b,psi_b,q1,q2,q3,q4,q10,q11,q12,q13,q14,q15}), SX::horzcat({7.56355e-21, 0.4, 0.379941, 0.0, 0.0, 1.57, 3.8497e-20, 0.9, 2.4, 7.96129e-21, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
    writeMatrixToFile(J_v_comB_val, basePath + "J_v_comB_val.txt");

    SX J_v_comL1 = SX::horzcat({SX::jacobian(I_comL1pos, x_b), SX::jacobian(I_comL1pos, y_b), SX::jacobian(I_comL1pos, z_b),
                                SX::jacobian(I_comL1pos, phi_b), SX::jacobian(I_comL1pos, theta_b), SX::jacobian(I_comL1pos, psi_b),
                                SX::jacobian(I_comL1pos, q1), SX::jacobian(I_comL1pos, q2), SX::jacobian(I_comL1pos, q3), SX::jacobian(I_comL1pos, q4),
                                SX::jacobian(I_comL1pos, q10), SX::jacobian(I_comL1pos, q11),
                                SX::jacobian(I_comL1pos, q12), SX::jacobian(I_comL1pos, q13),
                                SX::jacobian(I_comL1pos, q14), SX::jacobian(I_comL1pos, q15)});

    SX J_v_comL2 = SX::horzcat({SX::jacobian(I_comL2pos, x_b), SX::jacobian(I_comL2pos, y_b), SX::jacobian(I_comL2pos, z_b),
                                SX::jacobian(I_comL2pos, phi_b), SX::jacobian(I_comL2pos, theta_b), SX::jacobian(I_comL2pos, psi_b),
                                SX::jacobian(I_comL2pos, q1), SX::jacobian(I_comL2pos, q2), SX::jacobian(I_comL2pos, q3), SX::jacobian(I_comL2pos, q4),
                                SX::jacobian(I_comL2pos, q10), SX::jacobian(I_comL2pos, q11),
                                SX::jacobian(I_comL2pos, q12), SX::jacobian(I_comL2pos, q13),
                                SX::jacobian(I_comL2pos, q14), SX::jacobian(I_comL2pos, q15)});                                                               

    SX J_v_comL3 = SX::horzcat({SX::jacobian(I_comL3pos, x_b), SX::jacobian(I_comL3pos, y_b), SX::jacobian(I_comL3pos, z_b),
                                SX::jacobian(I_comL3pos, phi_b), SX::jacobian(I_comL3pos, theta_b), SX::jacobian(I_comL3pos, psi_b),
                                SX::jacobian(I_comL3pos, q1), SX::jacobian(I_comL3pos, q2), SX::jacobian(I_comL3pos, q3), SX::jacobian(I_comL3pos, q4),
                                SX::jacobian(I_comL3pos, q10), SX::jacobian(I_comL3pos, q11),
                                SX::jacobian(I_comL3pos, q12), SX::jacobian(I_comL3pos, q13),
                                SX::jacobian(I_comL3pos, q14), SX::jacobian(I_comL3pos, q15)});

    SX debug_J_v_comL3 = SX::substitute(J_v_comL3, SX::horzcat({x_b,y_b,z_b, phi_b,theta_b,psi_b,q1,q2,q3,q4,q10,q11,q12,q13,q14,q15}), SX::horzcat({7.56355e-21, 0.4, 0.379941, 0.0, 0.0, 1.57, 3.8497e-20, 0.9, 2.4, 7.96129e-21, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
    writeMatrixToFile(debug_J_v_comL3, basePath + "debug_J_v_comL3.txt");

    SX J_v_comL4 = SX::horzcat({SX::jacobian(I_comL4pos, x_b), SX::jacobian(I_comL4pos, y_b), SX::jacobian(I_comL4pos, z_b),
                                SX::jacobian(I_comL4pos, phi_b), SX::jacobian(I_comL4pos, theta_b), SX::jacobian(I_comL4pos, psi_b),
                                SX::jacobian(I_comL4pos, q1), SX::jacobian(I_comL4pos, q2), SX::jacobian(I_comL4pos, q3), SX::jacobian(I_comL4pos, q4),
                                SX::jacobian(I_comL4pos, q10), SX::jacobian(I_comL4pos, q11),
                                SX::jacobian(I_comL4pos, q12), SX::jacobian(I_comL4pos, q13),
                                SX::jacobian(I_comL4pos, q14), SX::jacobian(I_comL4pos, q15)});          

    //--------------------------------------COM vel------------------------------------//

    SX I_comBvel = SX::mtimes(J_v_comB, SX::vertcat({dx_b, dy_b, dz_b, dphi_b, dtheta_b, dpsi_b, dq1, dq2, dq3, dq4, dq10, dq11, dq12, dq13, dq14, dq15}));  
    SX I_comL1vel = SX::mtimes(J_v_comL1, SX::vertcat({dx_b, dy_b, dz_b, dphi_b, dtheta_b, dpsi_b, dq1, dq2, dq3, dq4, dq10, dq11, dq12, dq13, dq14, dq15}));       
    SX I_comL2vel = SX::mtimes(J_v_comL2, SX::vertcat({dx_b, dy_b, dz_b, dphi_b, dtheta_b, dpsi_b, dq1, dq2, dq3, dq4, dq10, dq11, dq12, dq13, dq14, dq15}));
    SX I_comL3vel = SX::mtimes(J_v_comL3, SX::vertcat({dx_b, dy_b, dz_b, dphi_b, dtheta_b, dpsi_b, dq1, dq2, dq3, dq4, dq10, dq11, dq12, dq13, dq14, dq15}));
    SX I_comL4vel = SX::mtimes(J_v_comL4, SX::vertcat({dx_b, dy_b, dz_b, dphi_b, dtheta_b, dpsi_b, dq1, dq2, dq3, dq4, dq10, dq11, dq12, dq13, dq14, dq15}));

    SX longcomvel = (mass_B * I_comBvel + mass_L1 * I_comL1vel + mass_L2 * I_comL2vel + mass_L3 * I_comL3vel + mass_L4 * I_comL4vel) / (mass_B + mass_L1 + mass_L2 + mass_L3 + mass_L4);  


    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%COM Jacobian method 1%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
    // %Everything in world frame     
    SX w_bB = SX::solve(A_Theta(SX::vertcat({phi_b, theta_b, psi_b})), SX::vertcat({dphi_b, dtheta_b, dpsi_b}));
    SX w_bL1 = w_bB + SX::mtimes(T_IL1(Slice(0, 3), Slice(0, 3)), SX::vertcat({0,0,1})*dq1); 
    SX w_bL2 = w_bL1 + SX::mtimes(T_IL2(Slice(0, 3), Slice(0, 3)), SX::vertcat({0,0,1})*dq2);
    SX w_bL3 = w_bL2 + SX::mtimes(T_IL3(Slice(0, 3), Slice(0, 3)), SX::vertcat({0,0,1})*dq3);
    SX w_bL4 = w_bL3 + SX::mtimes(T_IL4(Slice(0, 3), Slice(0, 3)), SX::vertcat({0,0,1})*dq4);

    // %Jacobian w.r.t \dot(state)
    SX J_w_comB = SX::horzcat({SX::jacobian(w_bB, dx_b), SX::jacobian(w_bB, dy_b), SX::jacobian(w_bB, dz_b),
                              SX::jacobian(w_bB, dphi_b), SX::jacobian(w_bB, dtheta_b), SX::jacobian(w_bB, dpsi_b),
                              SX::jacobian(w_bB, dq1), SX::jacobian(w_bB, dq2), SX::jacobian(w_bB, dq3), SX::jacobian(w_bB, dq4),
                              SX::jacobian(w_bB, dq10), SX::jacobian(w_bB, dq11),
                              SX::jacobian(w_bB, dq12), SX::jacobian(w_bB, dq13),
                              SX::jacobian(w_bB, dq14), SX::jacobian(w_bB, dq15)});

    SX J_w_comL1 = SX::horzcat({SX::jacobian(w_bL1, dx_b), SX::jacobian(w_bL1, dy_b), SX::jacobian(w_bL1, dz_b),
                               SX::jacobian(w_bL1, dphi_b), SX::jacobian(w_bL1, dtheta_b), SX::jacobian(w_bL1, dpsi_b),
                               SX::jacobian(w_bL1, dq1), SX::jacobian(w_bL1, dq2), SX::jacobian(w_bL1, dq3), SX::jacobian(w_bL1, dq4),
                               SX::jacobian(w_bL1, dq10), SX::jacobian(w_bL1, dq11),
                               SX::jacobian(w_bL1, dq12), SX::jacobian(w_bL1, dq13),
                               SX::jacobian(w_bL1, dq14), SX::jacobian(w_bL1, dq15)});            

    SX J_w_comL2 = SX::horzcat({SX::jacobian(w_bL2, dx_b), SX::jacobian(w_bL2, dy_b), SX::jacobian(w_bL2, dz_b),
                               SX::jacobian(w_bL2, dphi_b), SX::jacobian(w_bL2, dtheta_b), SX::jacobian(w_bL2, dpsi_b),
                               SX::jacobian(w_bL2, dq1), SX::jacobian(w_bL2, dq2), SX::jacobian(w_bL2, dq3), SX::jacobian(w_bL2, dq4),
                               SX::jacobian(w_bL2, dq10), SX::jacobian(w_bL2, dq11),
                               SX::jacobian(w_bL2, dq12), SX::jacobian(w_bL2, dq13),
                               SX::jacobian(w_bL2, dq14), SX::jacobian(w_bL2, dq15)});

    SX J_w_comL3 = SX::horzcat({SX::jacobian(w_bL3, dx_b), SX::jacobian(w_bL3, dy_b), SX::jacobian(w_bL3, dz_b),
                               SX::jacobian(w_bL3, dphi_b), SX::jacobian(w_bL3, dtheta_b), SX::jacobian(w_bL3, dpsi_b),
                               SX::jacobian(w_bL3, dq1), SX::jacobian(w_bL3, dq2), SX::jacobian(w_bL3, dq3), SX::jacobian(w_bL3, dq4),
                               SX::jacobian(w_bL3, dq10), SX::jacobian(w_bL3, dq11),
                               SX::jacobian(w_bL3, dq12), SX::jacobian(w_bL3, dq13),
                               SX::jacobian(w_bL3, dq14), SX::jacobian(w_bL3, dq15)});

    SX J_w_comL4 = SX::horzcat({SX::jacobian(w_bL4, dx_b), SX::jacobian(w_bL4, dy_b), SX::jacobian(w_bL4, dz_b),
                                SX::jacobian(w_bL4, dphi_b), SX::jacobian(w_bL4, dtheta_b), SX::jacobian(w_bL4, dpsi_b),
                                SX::jacobian(w_bL4, dq1), SX::jacobian(w_bL4, dq2), SX::jacobian(w_bL4, dq3), SX::jacobian(w_bL4, dq4),
                                SX::jacobian(w_bL4, dq10), SX::jacobian(w_bL4, dq11),
                                SX::jacobian(w_bL4, dq12), SX::jacobian(w_bL4, dq13),
                                SX::jacobian(w_bL4, dq14), SX::jacobian(w_bL4, dq15)});          

    // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Inertia Matrix%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//

    SX longinertiamatrix = mass_B*SX::mtimes(J_v_comB.T(),J_v_comB) + mass_L1*SX::mtimes(J_v_comL1.T(),J_v_comL1) + mass_L2*SX::mtimes(J_v_comL2.T(),J_v_comL2) 
                            + mass_L3*SX::mtimes(J_v_comL3.T(),J_v_comL3) + mass_L4*SX::mtimes(J_v_comL4.T(),J_v_comL4) 
                            + SX::mtimes(SX::mtimes(J_w_comB.T(),T_IB(Slice(0,3),Slice(0,3))),SX::mtimes(inertia_B,SX::mtimes(T_IB(Slice(0,3),Slice(0,3)).T(),J_w_comB)))
                            + SX::mtimes(SX::mtimes(J_w_comL1.T(),T_IL1(Slice(0,3),Slice(0,3))),SX::mtimes(inertia_L1,SX::mtimes(T_IL1(Slice(0,3),Slice(0,3)).T(),J_w_comL1)))
                            + SX::mtimes(SX::mtimes(J_w_comL2.T(),T_IL2(Slice(0,3),Slice(0,3))),SX::mtimes(inertia_L2,SX::mtimes(T_IL2(Slice(0,3),Slice(0,3)).T(),J_w_comL2)))
                            + SX::mtimes(SX::mtimes(J_w_comL3.T(),T_IL3(Slice(0,3),Slice(0,3))),SX::mtimes(inertia_L3,SX::mtimes(T_IL3(Slice(0,3),Slice(0,3)).T(),J_w_comL3)))
                            + SX::mtimes(SX::mtimes(J_w_comL4.T(),T_IL4(Slice(0,3),Slice(0,3))),SX::mtimes(inertia_L4,SX::mtimes(T_IL4(Slice(0,3),Slice(0,3)).T(),J_w_comL4)));

    D_q_arm = longinertiamatrix(Slice(0,10),Slice(0,10)); // % 10x10 Inertia matrix       % For now, disregard fingers' dynamics       
    SX D_q_arm_reduced =   SX::substitute(D_q_arm,SX::horzcat({phi_b,theta_b,psi_b,q1,q2,q3,q4}),SX::horzcat({0,0,0, arm_pos.T()}));   
    writeMatrixToFile(D_q_arm_reduced, basePath + "D_q_arm_reduced.txt");
    SX D_q_arm_val = SX::substitute(D_q_arm, SX::horzcat({x_b,y_b,z_b, phi_b,theta_b,psi_b,q1,q2,q3,q4}), SX::horzcat({7.56355e-21, 0.4, 0.379941, 0.0, 0.0, 0, 3.8497e-20, 0.9, 2.4, 7.96129e-21}));                  
    writeMatrixToFile(D_q_arm_val, basePath + "D_q_arm_val.txt");
    // D_q_arm = D_q_arm_pin; // % for further use in dynamics function
    // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Nonlinearity vector%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//

    // %%%% gravity vector  
    SX G_sx = 9.81;
    SX long_Gravity = - SX::mtimes(J_v_comB.T(),SX::vertcat({0,0,-mass_B*G_sx}))
                        - SX::mtimes(J_v_comL1.T(),SX::vertcat({0,0,-mass_L1*G_sx}))     
                        - SX::mtimes(J_v_comL2.T(),SX::vertcat({0,0,-mass_L2*G_sx}))
                        - SX::mtimes(J_v_comL3.T(),SX::vertcat({0,0,-mass_L3*G_sx}))
                        - SX::mtimes(J_v_comL4.T(),SX::vertcat({0,0,-mass_L4*G_sx}));
    
    // SX debug_Gravity  = - SX::mtimes(J_v_comB.T(),SX::vertcat({0,0,-mass_B*G_sx}));
    // SX debug_Gravity_val = SX::substitute(debug_Gravity, SX::horzcat({x_b,y_b,z_b, phi_b,theta_b,psi_b,q1,q2,q3,q4}), SX::horzcat({7.56355e-21, 0.4, 0.379941, 0.0, 0.0, 1.57, 3.8497e-20, 0.9, 2.4, 7.96129e-21}));
    // writeMatrixToFile(debug_Gravity_val, basePath + "debug_Gravity_val.txt");
    SX Potential_Energy = SX::mtimes(mass_arm, SX::mtimes(SX::vertcat({0,0,G_sx}).T(), longcompos));
    SX Gravity_PE_long = SX::jacobian(Potential_Energy, casadi::SX::vertcat({x_b, y_b, z_b, phi_b, theta_b, psi_b,  q1, q2, q3, q4, q10, q11, q12, q13, q14, q15}));
    SX Gravity_PE = Gravity_PE_long(Slice(0,10)); // 10 x 1 gravity vector       % For now, disregard fingers' dynamics
    SX Gravity_PE_val = SX::substitute(Gravity_PE, SX::horzcat({x_b,y_b,z_b, phi_b,theta_b,psi_b,q1,q2,q3,q4}), SX::horzcat({7.56355e-21, 0.4, 0.379941, 0.0, 0.0, 1.57, 3.8497e-20, 0.9, 2.4, 7.96129e-21}));
    writeMatrixToFile(Gravity_PE_val, basePath + "Gravity_PE_val.txt");

    SX debug_Gravity_L3 = - SX::mtimes(J_v_comL3.T(),SX::vertcat({0,0,-mass_L3*G_sx}));
    SX debug_Gravity_L3_val = SX::substitute(debug_Gravity_L3, SX::horzcat({x_b,y_b,z_b, phi_b,theta_b,psi_b,q1,q2,q3,q4}), SX::horzcat({7.56355e-21, 0.4, 0.379941, 0.0, 0.0, 0, 3.8497e-20, 0.9, 2.4, 7.96129e-21}));
    writeMatrixToFile(debug_Gravity_L3_val, basePath + "debug_Gravity_L3_val.txt");

    Gravity_arm = long_Gravity(Slice(0,10),0); // % 10x1 gravity vector      % For now, disregard fingers' dynamics
    // Gravity_arm(Slice(3,6)) = SX::mtimes(RotMat(SX::vertcat({phi_b,theta_b,psi_b})),Gravity_arm(Slice(3,6))); // Convert to Euler angle rates
    
    SX Gravity_arm_val = SX::substitute(Gravity_arm, SX::horzcat({x_b,y_b,z_b, phi_b,theta_b,psi_b,q1,q2,q3,q4}), SX::horzcat({7.56355e-21, 0.4, 0.379941, 0.0, 0.0, 1.57, 3.8497e-20, 0.9, 2.4, 7.96129e-21}));
    writeMatrixToFile(Gravity_arm_val, basePath + "Gravity_arm_val.txt");
    // Gravity_arm = Gravity_arm_pin;
    // writeMatrixToFile(Gravity_arm, basePath + "Gravity_arm.txt");
        
    /* % TODO: get the true gear ratio values for the arm joints from the urdf file
    % Refer to Kinova URDF
    % g1=160;
    % g2=160;
    % g3=160;
    % g4=160;                    */

    SX g1 = 1; SX g2 = 1; SX g3 = 1; SX g4 = 1; // % gear ratio
    std::vector<casadi::SX> g_list = {g1, g2, g3, g4};

    SX B_bar = blkdiag(g_list); // % gear ratio matrix
    B_arm  = SX::vertcat({SX::zeros(6,4),B_bar}); // % gear ratio matrix // 10 x 4

    // J_arm = SX::vertcat({SX::horzcat({SX::eye(3),SX::zeros(3,3),SX::zeros(3,4)}),SX::horzcat({SX::zeros(3,3),A_Theta(SX::vertcat({phi_b,theta_b,psi_b})),SX::zeros(3,4)})}); // 6 x 10 //A_Theta since coordinates are in base euler rates not omega
    J_arm = SX::vertcat({SX::horzcat({SX::eye(3),SX::zeros(3,3),SX::zeros(3,4)}),SX::horzcat({SX::zeros(3,3),SX::eye(3),SX::zeros(3,4)})}); // 6 x 10 //A_Theta since coordinates are in base euler rates not omega
 
    // getKinovaDynamics(D_q_arm, Gravity_arm, B_arm, J_arm, q_arm, dq_arm, q_in, dq_in);

    // SX x_b = q_arm(0); SX y_b = q_arm(1); SX z_b = q_arm(2); //% Position of the arm base
    // SX phi_b = q_arm(3); SX theta_b = q_arm(4); SX psi_b = q_arm(5); //% Orientation of the arm base
    // SX q1 = q_arm(6); SX q2 = q_arm(7); SX q3 = q_arm(8); SX q4 = q_arm(9); //% Joint angles

    // SX dx_b = dq_arm(0); SX dy_b = dq_arm(1); SX dz_b = dq_arm(2); //% Velocity of the arm base
    // SX dphi_b = dq_arm(3); SX dtheta_b = dq_arm(4); SX dpsi_b = dq_arm(5); //% Angular velocity of the arm base
    // SX dq1 = dq_arm(6); SX dq2 = dq_arm(7); SX dq3 = dq_arm(8); SX dq4 = dq_arm(9); //% Joint velocities

    // casadi::SX states_arm = casadi::SX::vertcat({x_b, y_b, z_b, phi_b, theta_b, psi_b, q1, q2, q3, q4, dx_b, dy_b, dz_b, dphi_b, dtheta_b, dpsi_b, dq1, dq2, dq3, dq4});
    casadi::SX states_arm = SX::vertcat({q_arm, dq_arm});

    // %%%%%--Augment states--%%%%%//
    SX states = casadi::SX::vertcat({states_SRB, states_arm}); //32 x 1
    size_t n_states = states.size1();

    // %%%%%%%%%%%%%%%%%% Define Inputs%%%%%%%%%%%%%%%%%%%%%//
    // %%%%%--A1--%%%%%
     // Define controls similarly
    casadi::SXVector controls_SRB_vec = {
        SX::sym("u1x"), SX::sym("u1y"), SX::sym("u1z"),
        SX::sym("u2x"), SX::sym("u2y"), SX::sym("u2z"),
        SX::sym("u3x"), SX::sym("u3y"), SX::sym("u3z"),
        SX::sym("u4x"), SX::sym("u4y"), SX::sym("u4z")
    };
    casadi::SX controls_SRB = casadi::SX::vertcat(controls_SRB_vec);

    //Footholds
    casadi::SXVector footholds_vec = {
        SX::sym("pf1x"), SX::sym("pf1y"), SX::sym("pf1z"),
        SX::sym("pf2x"), SX::sym("pf2y"), SX::sym("pf2z"),
        SX::sym("pf3x"), SX::sym("pf3y"), SX::sym("pf3z"),
        SX::sym("pf4x"), SX::sym("pf4y"), SX::sym("pf4z")
    };
    casadi::SX footholds = casadi::SX::vertcat(footholds_vec);

    //%%%%%--Arm--%%%%%
    casadi::SXVector controls_arm_vec = {
        SX::sym("tau1"), SX::sym("tau2"), SX::sym("tau3"), SX::sym("tau4")
    };
    casadi::SX controls_arm = casadi::SX::vertcat(controls_arm_vec);

    // %%%%%--Interaction wrench (lambda_int)--%%%%%
    casadi::SXVector lambda_int_vec = {
        SX::sym("f_int_x"), SX::sym("f_int_y"), SX::sym("f_int_z"), SX::sym("tau_int_x"), SX::sym("tau_int_y"), SX::sym("tau_int_z")
    };
    casadi::SX controls_lambda_int = casadi::SX::vertcat(lambda_int_vec);

    SX controls = casadi::SX::vertcat({controls_SRB, controls_arm, controls_lambda_int}); // 22 x 1
    size_t n_controls = controls.size1();

    // %%%%%%%%%%%%%%%%%% Define Cost Function Gains %%%%%%%%%%%%%%%%%%%%%%//
    // %%%%%--A1--%%%%%
    casadi::SXVector Q_SRB_vec = {
        SX::sym("Q_SRB_x"), SX::sym("Q_SRB_y"), SX::sym("Q_SRB_z"),
        SX::sym("Q_SRB_vx"), SX::sym("Q_SRB_vy"), SX::sym("Q_SRB_vz"),
        SX::sym("Q_SRB_phi"), SX::sym("Q_SRB_theta"), SX::sym("Q_SRB_psi"),
        SX::sym("Q_SRB_wx"), SX::sym("Q_SRB_wy"), SX::sym("Q_SRB_wz"),
        };
    casadi::SX Q_SRB = casadi::SX::vertcat(Q_SRB_vec);

    casadi::SXVector Q_arm_vec = {
        SX::sym("Q_arm_x"), SX::sym("Q_arm_y"), SX::sym("Q_arm_z"),
        SX::sym("Q_arm_phi"), SX::sym("Q_arm_theta"), SX::sym("Q_arm_psi"),
        SX::sym("Q_arm_q1"), SX::sym("Q_arm_q2"), SX::sym("Q_arm_q3"), SX::sym("Q_arm_q4"), 
        SX::sym("Q_arm_vx"), SX::sym("Q_arm_vy"), SX::sym("Q_arm_vz"),
        SX::sym("Q_arm_wx"), SX::sym("Q_arm_wy"), SX::sym("Q_arm_wz"),
        SX::sym("Q_arm_dq1"), SX::sym("Q_arm_dq2"), SX::sym("Q_arm_dq3"), SX::sym("Q_arm_dq4")
        };
    casadi::SX Q_arm = casadi::SX::vertcat(Q_arm_vec);

    casadi::SXVector R_SRB_vec = {
        SX::sym("R_SRB_1x"), SX::sym("R_SRB_1y"), SX::sym("R_SRB_1z"),
        SX::sym("R_SRB_2x"), SX::sym("R_SRB_2y"), SX::sym("R_SRB_2z"),
        SX::sym("R_SRB_3x"), SX::sym("R_SRB_3y"), SX::sym("R_SRB_3z"),
        SX::sym("R_SRB_4x"), SX::sym("R_SRB_4y"), SX::sym("R_SRB_4z")
        };
    casadi::SX R_SRB = casadi::SX::vertcat(R_SRB_vec);

    casadi::SXVector R_arm_vec = {
        SX::sym("R_arm_tau1"), SX::sym("R_arm_tau2"), SX::sym("R_arm_tau3"), SX::sym("R_arm_tau4")
        };

    casadi::SX R_arm = casadi::SX::vertcat(R_arm_vec);

    casadi::SXVector R_lambda_int_vec = {
        SX::sym("R_lambda_int_f_x"), SX::sym("R_lambda_int_f_y"), SX::sym("R_lambda_int_f_z"), SX::sym("R_lambda_int_tau_x"), SX::sym("R_lambda_int_tau_y"), SX::sym("R_lambda_int_tau_z")
        };
    casadi::SX R_lambda_int = casadi::SX::vertcat(R_lambda_int_vec);

    //Terminal cost function
    casadi::SXVector P_SRB_vec = {
        SX::sym("P_SRB_x"), SX::sym("P_SRB_y"), SX::sym("P_SRB_z"),
        SX::sym("P_SRB_vx"), SX::sym("P_SRB_vy"), SX::sym("P_SRB_vz"),
        SX::sym("P_SRB_phi"), SX::sym("P_SRB_theta"), SX::sym("P_SRB_psi"),
        SX::sym("P_SRB_wx"), SX::sym("P_SRB_wy"), SX::sym("P_SRB_wz")
        };
    
    casadi::SX P_SRB = casadi::SX::vertcat(P_SRB_vec);

    casadi::SXVector P_arm_vec = {
        SX::sym("P_arm_x"), SX::sym("P_arm_y"), SX::sym("P_arm_z"),
        SX::sym("P_arm_phi"), SX::sym("P_arm_theta"), SX::sym("P_arm_psi"),
        SX::sym("P_arm_q1"), SX::sym("P_arm_q2"), SX::sym("P_arm_q3"), SX::sym("P_arm_q4"), 
        SX::sym("P_arm_vx"), SX::sym("P_arm_vy"), SX::sym("P_arm_vz"),
        SX::sym("P_arm_wx"), SX::sym("P_arm_wy"), SX::sym("P_arm_wz"),
        SX::sym("P_arm_dq1"), SX::sym("P_arm_dq2"), SX::sym("P_arm_dq3"), SX::sym("P_arm_dq4")
        };
    casadi::SX P_arm = casadi::SX::vertcat(P_arm_vec);

    casadi::SX D_q_arm_outside = casadi::SX::sym("D_q_arm_outside", params.n_arm/2, params.n_arm/2);

    SX U = SX::sym("U", n_controls, params.N); //% Decision variables (controls) for all horizons
    SX P1 = SX::sym("P1", n_states + n_states * params.N + params.n_u_SRB); //% a parameter to store the current state and the desired future (for all horizons)
    SX P2 = reshape(footholds,12,1); //% a parameter to store the current state and the desired future (for all horizons)
    //Storing gains for cost function
    SX P3 = SX::vertcat({Q_SRB, Q_arm, R_SRB, R_arm, R_lambda_int, P_SRB,P_arm}); //% to store cost function gains
    
    SX P4  = reshape(D_q_arm_outside, (params.n_arm/2)*(params.n_arm/2),1); //% a parameter to store the current state and the desired future (for all horizons)

    SX P = SX::vertcat({P1, P2, P3, P4}); //% a parameter to store the current state and the desired future (for all horizons)
// % This is a multipl-shooting in which we strore both states and inputs
    SX X = SX::sym("X", n_states, params.N + 1); //% A Matrix that represents the states over the optimization problem.
// % N+1: Because we store the current states in the 1st column
    SX OPT_variables = vertcat(reshape(X, n_states * (params.N + 1), 1), reshape(U, params.n_inputs * params.N, 1)); //% make the decision variable one column  vector ((32*11)+(22*10))

    // %% THE SIMULATION LOOP SHOULD START FROM HERE
    // %%%%%--Initial conditions--%%%%%

    // %%--SRB ICs--%%

    DM qk = DM::zeros(n_states);
    qk(2) = 0.08; // Assuming SRB z position is constant or predefined

    // %%%%--arm ICs--%%%%
    // % The initial condition for the arm's base should match those of the
    // % holonomically fixed to the A1
    qk(Slice(12,15)) = qk(Slice(0,3)) + DM::mtimes(RotMat(qk(Slice(6,9))),SX(params.dd));

    // % Arm's base orientation
    qk(Slice(15,18)) = qk(Slice(6,9));

    //     % Shape variable ICs: for now, consider the arm with fixed joint angles
    // % x0(19:22) = [ZZero; 2; 5;ZZero];  % q_s = [q1 q2 q3 q4] arm's joints (rad)
    // qk(Slice(18,22)) = DM::vertcat({DM(arm_pos(0)),DM(arm_pos(1)),DM(arm_pos(2)),DM(arm_pos(3))});//% q_s = [q1 q2 q3 q4] arm's joints (rad)
    qk(Slice(18,22)) = arm_pos;//% q_s = [q1 q2 q3 q4] arm's joints (rad)
    // % Arm's base velocity
    qk(Slice(22,25)) = qk(Slice(3,6)) + DM::mtimes(skewSym(qk(Slice(9,12))),SX::mtimes(RotMat(qk(Slice(6,9))),SX(params.dd)));

    //% Arm's base orientation rate
    qk(Slice(25,28)) = DM::mtimes(A_Theta(qk(Slice(6,9))),qk(Slice(9,12)));

//     % x0(26:28) = [ZZero; ZZero; ZZero];
// x0(29:32) = [ZZero;ZZero;ZZero;ZZero];  % dq_s = [dq1 dq2 dq3 dq4] arm's joints rates (rad/s)


    double t0 = 0.0;
    DM xx(n_states, sim_length);
    xx(Slice(0, static_cast<int>(n_states)), 0) = qk;  

    DM u0 = DM::zeros(params.N, params.n_inputs); // Control inputs for N horizon steps, not sim_steps
    

    // States decision variables initialization
    DM X0 = DM::repmat(qk, 1, params.N+1); 
    DM xd = DM::repmat(qk, params.N, 1);

    // Write initial symbolic states and controls to file
    // writeMatrixToFile(states, basePath + "states.txt");
    // writeMatrixToFile(controls, basePath + "controls.txt");
    // writeMatrixToFile(qk, basePath + "qk.txt");
    // writeMatrixToFile(xd, basePath + "xd.txt"); 
    // writeMatrixToFile(xx, basePath + "xx.txt");
    // writeMatrixToFile(X, basePath + "X.txt");
    // writeMatrixToFile(P, basePath + "P.txt");
    // writeMatrixToFile(U, basePath + "U.txt");
    // writeMatrixToFile(OPT_variables, basePath + "OPT_variables.txt");
    // writeMatrixToFile(params, basePath + "params.txt");

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

    DM uout = DM::zeros(n_controls, sim_length); // Vector to store control outputs
    DM xout = DM::zeros(n_states, sim_length);
    DM feet = DM::zeros(12, sim_length); // Vector to store foot positions
    DM traj = DM::zeros(12+20, sim_length); // Vector to store desired trajectory
    DM tout = DM::zeros(1, sim_length);
    DM NMPC_solve_time = DM::zeros(1, sim_length);  // Vector to store NMPC solve time

        //=======================================SRB Dynamics=======================================//
    //  define the nonlinear right-hand-side "f"  for A1 (SRB dynamics)
    
    SX t_rot = casadi::SX::vertcat({states(6), states(7), states(8)});  // Extract phi, theta, and psi into a column vector for Rotz
    SX rot_mat = RotMat(t_rot);    // Call RotMat with the column vector
    // writeMatrixToFile(rot_mat, basePath + "rot_mat.txt");
    SX z3 = casadi::SX::zeros(3, 3);    // Create a 3x3 zero matrix
    SX i3 = casadi::SX::eye(3);         // Create a 3x3 identity matrix


    // DM J = DM::zeros(3,3); // Assuming a paramse-5; J(2,2) = 0.064713601;
    DM J = params.J;
    // writeMatrixToFile(J, basePath + "J.txt");


    SX omega = casadi::SX::vertcat({states(9), states(10), states(11)}); // Extracting angular velocities (dphi, dtheta, dpsi)
    SX skew_omega = skewSym(omega);
    // writeMatrixToFile(skew_omega, basePath + "skew_omega.txt");
    SX J_sx = SX(J); // Convert J from DM to SX
    
    // writeMatrixToFile(J_sx, basePath + "J_sx.txt");

    // SX rot_mat_val = SX::substitute(rot_mat, SX::horzcat({states_SRB(6),states_SRB(7),states_SRB(8),
    //                         states_SRB(9),states_SRB(10), states_SRB(11)}),
    //                          SX::horzcat({1,2,3,4,5,6}));
    // writeMatrixToFile(rot_mat_val, basePath + "rot_mat_val.txt");

    // SX inverseJ_rotmat_transpose = solve(J_sx,rot_mat.T());
    // SX inverseJ_rotmat_transpose_val = SX::substitute(inverseJ_rotmat_transpose, SX::horzcat({states_SRB(6),states_SRB(7),states_SRB(8),
    //                         states_SRB(9),states_SRB(10), states_SRB(11)}),
    //                          SX::horzcat({1,2,3,4,5,6}));
    // writeMatrixToFile(inverseJ_rotmat_transpose_val, basePath+ "inverseJ_rotmat_transpose_val.txt");


    // SX IwI_1 = SX::mtimes(rot_mat, solve(J_sx,rot_mat.T()));
    // SX IwI_1_val = SX::substitute(IwI_1, SX::horzcat({states_SRB(6),states_SRB(7),states_SRB(8),
    //                         states_SRB(9),states_SRB(10), states_SRB(11)}),
    //                          SX::horzcat({1,2,3,4,5,6}));
    // writeMatrixToFile(IwI_1_val, basePath + "IwI_1_val.txt");

    // SX IwI_2 = SX::mtimes(SX::mtimes(rot_mat, J_sx), rot_mat.T());
    // SX IwI_2_val = SX::substitute(IwI_2, SX::horzcat({states_SRB(6),states_SRB(7),states_SRB(8),
    //                         states_SRB(9),states_SRB(10), states_SRB(11)}),
    //                          SX::horzcat({1,2,3,4,5,6}));
    // writeMatrixToFile(IwI_2_val, basePath + "IwI_2_val.txt");

    // SX IwI = SX::mtimes(IwI_1, mtimes(skew_omega, IwI_2));
    // SX IwI_val = SX::substitute(IwI, SX::horzcat({states_SRB(6),states_SRB(7),states_SRB(8),
    //                         states_SRB(9),states_SRB(10), states_SRB(11)}),
    //                          SX::horzcat({1,2,3,4,5,6}));
    SX IwI = SX::mtimes(SX::mtimes(SX::mtimes(rot_mat, solve(J_sx,rot_mat.T())), skew_omega), SX::mtimes(SX::mtimes(rot_mat, J_sx), rot_mat.T()));
    // writeMatrixToFile(IwI, basePath + "IwI.txt");
    // writeMatrixToFile(IwI_val, basePath + "IwI_val.txt");

    // writeMatrixToFile(Rz, basePath + "Rz.txt");
    // writeMatrixToFile(-IwI, basePath + "IwI.txt");
    // writeMatrixToFile(J_sx, basePath + "J_sx.txt");

    SX A_SRB = SX::zeros(12, 12); // A is a 12x12 matrix given your block structure
    // Fill in the blocks of A
    A_SRB(Slice(0, 3), Slice(0, 3)) = z3;
    A_SRB(Slice(0, 3), Slice(3, 6)) = i3;
    A_SRB(Slice(0, 3), Slice(6, 9)) = z3;
    A_SRB(Slice(0, 3), Slice(9, 12)) = z3;

    A_SRB(Slice(3, 6), Slice(0, 3)) = z3;
    A_SRB(Slice(3, 6), Slice(3, 6)) = z3;
    A_SRB(Slice(3, 6), Slice(6, 9)) = z3;
    A_SRB(Slice(3, 6), Slice(9, 12)) = z3;

    A_SRB(Slice(6, 9), Slice(0, 3)) = z3;
    A_SRB(Slice(6, 9), Slice(3, 6)) = z3;
    A_SRB(Slice(6, 9), Slice(6, 9)) = z3;
    A_SRB(Slice(6, 9), Slice(9, 12)) = A_Theta(states(Slice(6,9)));

    A_SRB(Slice(9, 12), Slice(0, 3)) = z3;
    A_SRB(Slice(9, 12), Slice(3, 6)) = z3;
    A_SRB(Slice(9, 12), Slice(6, 9)) = z3;
    A_SRB(Slice(9, 12), Slice(9, 12)) = -IwI; //-IwI

    SX A_SRB_reduced = A_SRB;
    A_SRB_reduced(Slice(9, 12), Slice(9, 12)) = z3;

    // writeMatrixToFile(A, basePath + "A.txt");
    // SX p_foot_sx = SX(params.pf);
    SX p_foot_sx = reshape(footholds.T(), 3, 4); // Reshape the footholds into a 3x4 matrix
    writeMatrixToFile(p_foot_sx, basePath + "p_foot_sx.txt");

    SX com_vector = SX::vertcat({states(0), states(1), states(2)});
    SX com_vector_expanded = repmat(com_vector, 1, 4); // Replicate across columns to match p_foot_sx dimensions
    writeMatrixToFile(com_vector_expanded, basePath + "com_vector_expanded.txt");

    SX rd = p_foot_sx - com_vector_expanded;
    writeMatrixToFile(rd, basePath + "rd.txt");
    
    SX skew_rd = skewSym(rd);
    writeMatrixToFile(skew_rd, basePath + "skew_rd.txt");
    // std::cout << "Size of skew_rd: " << skew_rd.size1() << " x " << skew_rd.size2() << std::endl;
    SX skew_dd = skewSym(SX::mtimes(rot_mat, SX(params.dd))); 
    SX B_tmp_1 = SX::mtimes(SX::mtimes(rot_mat, solve(J_sx,rot_mat.T())), skew_rd);
    SX B_tmp_2 = SX::mtimes(SX::mtimes(rot_mat, solve(J_sx,rot_mat.T())), skew_dd);
    SX B_tmp_3 = SX::mtimes(rot_mat, solve(J_sx,rot_mat.T()));
    // writeMatrixToFile(B_tmp_1, basePath + "B_tmp.txt");
    // writeMatrixToFile(skew_rd, basePath + "skew_rd.txt");
    // writeMatrixToFile(rd, basePath + "rd.txt");

    SX B_SRB = SX::vertcat({
        SX::horzcat({z3,z3,z3,z3,z3,z3}),
        SX::horzcat({SX::repmat(1/params.mass * i3, 1, 5),z3}),
        SX::horzcat({z3,z3,z3,z3,z3,z3}),
        SX::horzcat({B_tmp_1, B_tmp_2, B_tmp_3})}); //SX::mtimes(SX::mtimes(Rz, J_sx), Rz.T()) * skew_rd

    // writeMatrixToFile(B, basePath + "B.txt");
    SX D_SRB = SX::zeros(12,1); // D is a 12x1 vector
    D_SRB(Slice(3,6)) = SX::vertcat({SX::zeros(2,1), -G}); // Fill the 4th to 6th rows

    // writeMatrixToFile(D, basePath + "D.txt");

    SX rhs_SRB = SX::mtimes(A_SRB, states_SRB) + SX::mtimes(B_SRB, SX::vertcat({controls_SRB, controls_lambda_int})) + D_SRB;

    SX rhs_SRB_reduced = SX::mtimes(A_SRB_reduced, states_SRB) + SX::mtimes(B_SRB, SX::vertcat({controls_SRB, controls_lambda_int})) + D_SRB;

    // writeMatrixToFile(rhs_SRB, basePath + "rhs_SRB.txt");

    Function f_SRB = Function("f_SRB", {states_SRB, controls_SRB, controls_lambda_int, p_foot_sx}, {rhs_SRB}, {"st", "con", "int","pf"}, {"rhs"});
    
    Function f_SRB_reduced = Function("f_SRB_reduced", {states_SRB, controls_SRB, controls_lambda_int, p_foot_sx}, {rhs_SRB_reduced}, {"st", "con", "int","pf"}, {"rhs"});
    /* std::map<std::string, SX> arg_SRB;
    arg_SRB["st"] = SX::vertcat({1,2,3,4,5,6,7,8,9,10,11,12});
    arg_SRB["con"] = SX::vertcat({13,14,15,16,17,18,19,20,21,22,23,24});
    arg_SRB["int"] = SX::vertcat({25,26,27,28,29,30});
    std::map<std::string, SX> result_SRB = f_SRB(arg_SRB);
    SX f_value_SRB = result_SRB.at("rhs");
    writeMatrixToFile(f_value_SRB, basePath + "f_value_SRB.txt"); */


    //=======================================Arm Dynamics=======================================//
    // %% define the nonlinear right-hand-side "f" for Kinova (4DOF arm's dynamics)
    SX rhs_arm_1  = SX::mtimes(SX::vertcat({SX::horzcat({SX::zeros(10,10),SX::eye(10)}),SX::horzcat({SX::zeros(10,10),SX::zeros(10,10)}) }), 
                        SX::vertcat({q_arm,dq_arm}));
    SX rhs_arm_2 =   SX::mtimes(SX::vertcat({SX::horzcat({SX::zeros(10,4),SX::zeros(10,6)}), SX::horzcat({solve(D_q_arm,B_arm),solve(-D_q_arm,J_arm.T())})}),
                        SX::vertcat({controls_arm,controls_lambda_int}));
    SX rhs_arm_3 =   SX::vertcat({SX::zeros(10,1), solve(-D_q_arm,Gravity_arm)});

    SX rhs_arm = rhs_arm_1 + rhs_arm_2 + rhs_arm_3;


    Function f_arm = Function("f_arm", {q_arm, dq_arm, controls_arm, controls_lambda_int}, {rhs_arm}, {"q", "dq", "con", "int"}, {"rhs"}); 

    SX rhs_arm_1_reduced  = SX::mtimes(SX::vertcat({SX::horzcat({SX::zeros(10,10),SX::eye(10)}),SX::horzcat({SX::zeros(10,10),SX::zeros(10,10)}) }), 
                        SX::vertcat({q_arm,dq_arm}));
    SX rhs_arm_2_reduced =   SX::mtimes(SX::vertcat({SX::horzcat({SX::zeros(10,4),SX::zeros(10,6)}), SX::horzcat({solve(D_q_arm_reduced,B_arm),solve(-D_q_arm_reduced,J_arm.T())})}),
                        SX::vertcat({controls_arm,controls_lambda_int}));
    SX rhs_arm_3_reduced =   SX::vertcat({SX::zeros(10,1), solve(-D_q_arm_reduced,Gravity_arm)});

    SX rhs_arm_reduced = rhs_arm_1_reduced + rhs_arm_2_reduced + rhs_arm_3_reduced;

    Function f_arm_reduced = Function("f_arm_reduced", {q_arm, dq_arm, controls_arm, controls_lambda_int}, {rhs_arm_reduced}, {"q", "dq", "con", "int"}, {"rhs"}); 

    //Passing the Mass inertia matrix from Outside
    SX rhs_arm_1_outside  = SX::mtimes(SX::vertcat({SX::horzcat({SX::zeros(10,10),SX::eye(10)}),SX::horzcat({SX::zeros(10,10),SX::zeros(10,10)}) }), 
                        SX::vertcat({q_arm,dq_arm}));
    SX rhs_arm_2_outside =   SX::mtimes(SX::vertcat({SX::horzcat({SX::zeros(10,4),SX::zeros(10,6)}), SX::horzcat({solve(D_q_arm_outside,B_arm),solve(-D_q_arm_outside,J_arm.T())})}),
                        SX::vertcat({controls_arm,controls_lambda_int}));
    SX rhs_arm_3_outside =   SX::vertcat({SX::zeros(10,1), solve(-D_q_arm_outside,Gravity_arm)});

    SX rhs_arm_outside = rhs_arm_1_outside + rhs_arm_2_outside + rhs_arm_3_outside;

    Function f_arm_outside = Function("f_arm_outside", {q_arm, dq_arm, controls_arm, controls_lambda_int, D_q_arm_outside}, {rhs_arm_outside}, {"q", "dq", "con", "int", "D_q"}, {"rhs"}); 
    
    Function D_q_arm_sym = Function("D_q_arm_sym", {q_arm, dq_arm}, {D_q_arm}, {"q", "dq"}, {"D_q"});
    /* writeMatrixToFile(rhs_arm, basePath + "rhs_arm.txt");
    std::map<std::string, SX> arg_arm;
    arg_arm["q"] = SX::vertcat({1,2,3,4,5,6,7,8,9,10});
    arg_arm["dq"] = SX::vertcat({11,12,13,14,15,16,17,18,19,20});
    arg_arm["con"] = SX::vertcat({21,22,23,24});
    arg_arm["int"] = SX::vertcat({25,26,27,28,29,30});
    std::map<std::string, SX> result_arm = f_arm(arg_arm);
    SX f_value_arm = result_arm.at("rhs");
    writeMatrixToFile(f_value_arm, basePath + "f_value_arm.txt"); */
  
    SX st_next_SRB = states_SRB + params.dt * rhs_SRB;
    SX st_next_arm = states_arm + params.dt * rhs_arm_1_outside; 

    SX p = states_SRB(Slice(0, 3));
    SX dp = states_SRB(Slice(3, 6));
    SX theta = states_SRB(Slice(6, 9));
    SX omega_hl = states_SRB(Slice(9, 12));
    SX R = RotMat(theta);
    SX A  = A_Theta(theta);

    SX ddp = rhs_SRB(Slice(3, 6));
    SX dtheta = rhs_SRB(Slice(6, 9));
    SX d_hl = rhs_SRB(Slice(9, 12));
    SX ddtheta =  SX::mtimes(SX::jacobian(SX::mtimes(A,omega_hl), theta),dtheta) + SX::mtimes(A,d_hl);

    SX dp_ab = rhs_arm_outside(Slice(0,3));
    SX ddp_ab = rhs_arm_outside(Slice(10,13));

    SX dtheta_ab = rhs_arm_outside(Slice(3,6));
    SX ddtheta_ab = rhs_arm_outside(Slice(13,16));

   // ----------------------1 st Holonomic Constraint (Position)----------------------//
    SX HolonomicPosition = p + SX::mtimes(R, SX(params.dd)) - states_arm(Slice(0, 3));
    SX HolonomicOrientation = theta - states_arm(Slice(3, 6));

    // ----------------------2 nd Holonomic Constraint (Velocity)----------------------//
    SX HolonomicVelocity = dp + SX::mtimes(SX::jacobian(SX::mtimes(R,SX(params.dd)),theta),dtheta) - dp_ab;
    SX HolonomicAngularVelocity = dtheta - dtheta_ab;

    // ----------------------3 rd Holonomic Constraint (Acceleration)----------------------//
    SX HolonomicAcceleration  = ddp
        + SX::mtimes(SX::jacobian(SX::mtimes(R,SX(params.dd)),theta),ddtheta)
        + SX::mtimes(SX::jacobian((SX::jacobian(SX::mtimes(R,SX(params.dd)),theta),dtheta),theta),dtheta)
        - ddp_ab;
    SX HolonomicAngularAcceleration = ddtheta - ddtheta_ab;

    SX bias_position = SX::vertcat({0, 0, 0.0});
    SX bias_orientation = SX::vertcat({0, 0, 0});

    // This is what you actually add:
    SX g_eq_hl = SX::vertcat({
        HolonomicAcceleration + params.Kp * (HolonomicPosition - bias_position)
                            + params.Kd * HolonomicVelocity,
        HolonomicAngularAcceleration + params.Kp * (HolonomicOrientation - bias_orientation)
                                    + params.Kd * HolonomicAngularVelocity
    });

    SX x_step  = SX::vertcat({states_SRB, states_arm, controls_lambda_int});
    SX Jacobian_holo = SX::jacobian(g_eq_hl, x_step);

    Function Jacobian_holonomic = Function("Jacobian_holonomic", {states_SRB, states_arm, controls_SRB, controls_arm,controls_lambda_int, footholds,D_q_arm_outside}, {Jacobian_holo}, {"st_SRB", "st_arm", "con_SRB", "con_arm", "lambda_int", "footholds", "D_q"}, {"Jacobian_holo"});


    


    SX obj = 0, g_eq; 
    // equalityconstraints(X, U, P, params, obj, g_eq, f_SRB_reduced, f_arm_reduced);
    // equalityconstraints(X, U, P, params, obj, g_eq, f_SRB, f_arm_reduced);
    equalityconstraints(X, U, P, params, obj, g_eq, f_SRB, f_arm_outside);
    // equalityconstraints(X, U, P, params, obj, g_eq, f_SRB, f_arm);
    writeMatrixToFile(obj, basePath + "obj.txt");
    writeMatrixToFile(g_eq, basePath + "g_eq.txt");
    // Declare variables to hold the outputs
    SX g_force, g_ub, g_lb;

    // Call the function
    force_inequality(U, params, g_force, g_ub, g_lb);

    // writeMatrixToFile(obj, basePath + "obj.txt");
    // writeMatrixToFile(g_eq, basePath + "g_eq.txt");

    SX g = vertcat(g_eq, g_force);

    // Dictionary to hold NLP problem definitions
    SXDict nlp_prob = {{"f", obj}, {"x", OPT_variables}, {"g", g}, {"p", P}};
    writeMatrixToFile(obj, basePath + "obj.txt");
    writeMatrixToFile(OPT_variables, basePath + "OPT_variables.txt");
    writeMatrixToFile(g, basePath + "g.txt");
    writeMatrixToFile(P, basePath + "P.txt");

    // Define options for the solver
    Dict opts;

    // For ipopt
    // Set Ipopt-specific options
    opts["ipopt.max_iter"] = 11;  // Replace Max_mpciter with its actual value
    opts["ipopt.print_level"] = 3;  // Can be changed to 0 or 3 based on verbosity required
    opts["print_time"] = 0;  // Disable printing solver time
    opts["ipopt.linear_solver"]="ma57";             // Use ma57 linear solver
    opts["ipopt.ma57_automatic_scaling"] = "no";    // Disable automatic scaling
    opts["ipopt.acceptable_tol"] = 1e-2;  // Tolerance for stopping criterion
    opts["ipopt.acceptable_obj_change_tol"] = 1e-2;  // Objective change tolerance for stopping

    /// for snopt
    // opts["snopt.Iterations limit"] = 10;
    // opts["snopt.Major Print level"] = 3;
    // opts["snopt.Minor Print level"] = 3;
    // opts["snopt.Derivative option"] = 0;
    // opts["snopt.Major feasibility tolerance"] = 1.0e-3;
    // opts["snopt.Minor feasibility tolerance"] = 1.0e-3;
    // opts["snopt.Major optimality tolerance"] = 1.0e-3;
    // opts["snopt."] = 10;
    // opts["print_level"] = 0;
    // // opts["print_file"] = "snopt.out";
    // opts["major_feasibility_tolerance"] = 1e-2;
    // opts["major_optimality_tolerance"] = 1e-2;

    // Create an NLP solver instance
    Function solver = nlpsol("solver", "ipopt", nlp_prob, opts);
    // Function solver = nlpsol("solver", "snopt", {{"f", obj}, {"x", OPT_variables}, {"g", g}, {"p", P}});

    // =========================== CODE GEN =========================== //
    // // Generate C code for the NLP functions
    if (code_gen == 1)
    {
        // // file name
        std::string file_name = "a1_kinova_nlp_code";
        // code predix
        std::string prefix_code = fs::current_path().string() + "/";

        casadi::Dict opts = casadi::Dict();
        opts["cpp"] = false;
        opts["with_header"] = false;


        solver.generate_dependencies(file_name + ".c");
        // f.generate_dependencies(file_name + "_f" + ".c");

        CodeGenerator codegen = CodeGenerator(file_name + "_f" + ".c", opts);
        codegen.add(f_SRB);
        codegen.add(f_arm);
        codegen.add(f_SRB_reduced);
        codegen.add(f_arm_reduced);
        codegen.add(f_arm_outside);
        codegen.add(D_q_arm_sym);
        codegen.add(Jacobian_holonomic);
        // codegen.add(solver);
        // codegen.with_header = true;
        // codegen.verbose = true;
        codegen.generate();

        // shared library prefix
        std::string prefix_lib = fs::current_path().string() + "/";    

        // compile c code to a shared library
        // std::string compile_command_solver = "gcc -fPIC -shared -O3 " + 
        //     prefix_code + file_name + ".c -o " +
        //     prefix_lib + file_name + ".so";

        std::string compile_command_solver = "gcc -fPIC -shared -O3 " + 
            prefix_code + file_name + ".c -o " +
            prefix_lib + file_name + ".so -L/usr/local/lib -lipopt";
        
        std::string compile_command_f = "gcc -fPIC -shared -O3 " + 
            prefix_code + file_name + "_f" + ".c -o " +
            prefix_lib + file_name + "_f" + ".so";

        std::cout << compile_command_solver << std::endl;
        std::cout << compile_command_f << std::endl;

        int compile_flag_f = std::system(compile_command_f.c_str());
        casadi_assert(compile_flag_f==0, "Compilation failed");
        std::cout << "Compilation succeeded!" << std::endl;

        int compile_flag_solver = std::system(compile_command_solver.c_str());
        casadi_assert(compile_flag_solver==0, "Compilation failed");
        std::cout << "Compilation succeeded!" << std::endl;

    

        code_gen = 0; // do not generate code again
    }

        // =========================== CODE GEN =========================== //

    else{
        // =========================== SIMULATION =========================== //}

        while(mpciter < sim_time / dt) {

            
            
            // DEBUG SESSION //
            params.current_time = current_time;
            params.mpc_iter = mpciter;

            
            
            // Updates the xd to obtain desired 12+10-state reference for next N domains
            planner(current_time, qk, xd, gait, params); // Assuming gait and other details are handled inside

            writeMatrixToFile(states, basePath + "states.txt");
            writeMatrixToFile(controls, basePath + "controls.txt");
            writeMatrixToFile(qk, basePath + "qk.txt");
            writeMatrixToFile(xd, basePath + "xd.txt"); 
            writeMatrixToFile(xx, basePath + "xx.txt");
            writeMatrixToFile(X, basePath + "X.txt");
            writeMatrixToFile(P, basePath + "P.txt");
            writeMatrixToFile(U, basePath + "U.txt");
            writeMatrixToFile(OPT_variables, basePath + "OPT_variables.txt");
            writeMatrixToFile(params, basePath + "params.txt");

            
          


        
            // Declare variables to hold the outputs
            SX g_force, g_ub, g_lb;

            // Call the function
            force_inequality(U, params, g_force, g_ub, g_lb);

            writeMatrixToFile(g_force, basePath + "g_force.txt");
            writeMatrixToFile(g_ub, basePath + "g_ub.txt");
            writeMatrixToFile(g_lb, basePath + "g_lb.txt");
            writeMatrixToFile(params, basePath + "params.txt");


            // args.lbg = DM::zeros(n_states * (N + 1) + 4*4*N, 1);
            // args.ubg = DM::zeros(n_states * (N + 1) + 4*4*N, 1);
            set_statebounds(mpciter, params, args);
            set_constraint_bounds(args, g_lb, g_ub, n_states, params.N, params.NumHolCon);

            writeMatrixToFile(args.lbx, basePath + "args_lbx.txt");
            writeMatrixToFile(args.ubx, basePath + "args_ubx.txt");
            writeMatrixToFile(args.lbg, basePath + "args_lbg.txt");
            writeMatrixToFile(args.ubg, basePath + "args_ubg.txt");

            

            // Reshape X0 and u0 transposed into column vectors
            DM X0_col = reshape(X0.T(), n_states * (params.N + 1), 1);
            DM u0_col = reshape(u0.T(), params.n_inputs * params.N, 1);

            // Concatenate reshaped vectors vertically
            DM x0u0= vertcat(X0_col, u0_col);
            args.x0 = x0u0;
            args.p = SX::vertcat({qk, xd, reshape(params.pf,12,1), params.Q_SRB, params.Q_arm, params.R_SRB, params.R_arm, params.R_lambda, params.P_SRB, params.P_arm});
            // writeMatrixToFile(args.p, basePath + "args_p.txt");
            // Solve the problem
            DMDict arg = {{"x0", args.x0}, {"lbx", args.lbx}, {"ubx", args.ubx}, {"lbg", args.lbg}, {"ubg", args.ubg}, {
                "p", args.p}};

            auto start = std::chrono::high_resolution_clock::now();
            DMDict res = solver(arg);
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            std::cout << "NMPC Solve Time: " << duration.count() << "ms" << std::endl;
        

            // writeMatrixToFile(res.at("x"), basePath + "solution.txt");
            // writeMatrixToFile(args.p, basePath + "args_p.txt");


            DM sol = DM::zeros(n_states * (params.N + 1) + n_controls * params.N, 1); // sol.x should be initialized with actual solution data
            sol = res.at("x");

    
            // Explicitly cast start and stop indices to casadi_int
            casadi_int start_index_u = static_cast<casadi_int>(n_states * (params.N + 1));
            casadi_int end_index = static_cast<casadi_int>(sol.numel());

            // Get controls from the solution
            DM u = reshape(sol(Slice(start_index_u, end_index)).T(), n_controls, params.N).T();

            // Get solution trajectory
            DM sol_x_N = reshape(sol(Slice(0, static_cast<casadi_int>(n_states * (params.N + 1)))).T(), n_states, params.N + 1).T();
            // writeMatrixToFile(sol_x_N, basePath + "x_sol_N.txt");

            // Update logs
            uout(Slice(), mpciter) = u(0, Slice());     // Store current control outputs

            // writeMatrixToFile(u(0, Slice()).T(), basePath + "u_sol.txt");
            // writeMatrixToFile(u.T(), basePath + "u_sol_N.txt");

            // Call the shift function
            // writeMatrixToFile(qk, basePath + "x_sol.txt");
            shift(t0, qk, u0, dt, u,f_SRB,f_arm, params);

            // Retrieve solution trajectory (assuming sol.x is available)
            // X0 = reshape(X0.T(), n_states, N + 1).T();
            //X0 = vertcat(sol_x_N(Slice(1, sol_x_N.size1()), Slice()), sol_x_N(Slice(X0.size1() - 1, sol_x_N.size1()), Slice()));
            X0 = DM::vertcat({sol_x_N(Slice(1, sol_x_N.size1()), Slice()), sol_x_N(Slice(sol_x_N.size1() - 1, sol_x_N.size1()), Slice())});
            // writeMatrixToFile(X0, basePath + "x_init_next_step.txt");

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

        writeMatrixToFile(xout.T(), basePath + "xout_test1.txt");
        writeMatrixToFile(feet.T(), basePath + "feet_test1.txt");
        writeMatrixToFile(tout.T(), basePath + "tout_test1.txt");
        writeMatrixToFile(uout.T(), basePath + "uout_test1.txt");
        writeMatrixToFile(NMPC_solve_time.T(), basePath + "NMPC_solve_time_test1.txt");

    }

    

    // =========================== EXAMPLE CODE =========================== //

    /*  Test problem
     *
     *    min x0^2 + x1^2
     *    s.t.    x0 + x1 - 10 = 0
     */

    // // Optimization variables
    // casadi::MX x = casadi::MX::sym("x", 2);

    // // Objective
    // casadi::MX f = x(0)*x(0) + x(1)*x(1);

    // // Constraints
    // casadi::MX g = x(0)+x(1)-10;

    // // Create an NLP solver instance
    // casadi::Function solver = casadi::nlpsol("solver", "ipopt", {{"x", x}, {"f", f}, {"g", g}});

    // // file name
    // std::string file_name = "nlp_code";
    // // code predix
    // std::string prefix_code = fs::current_path().string() + "/";

    // // Generate C code for the NLP functions
    // solver.generate_dependencies(file_name + ".c");

    // // shared library prefix
    // std::string prefix_lib = fs::current_path().string() + "/";

    // // compile c code to a shared library
    // std::string compile_command = "gcc -fPIC -shared -O3 " + 
    //     prefix_code + file_name + ".c -o " +
    //     prefix_lib + file_name + ".so";

    // std::cout << compile_command << std::endl;
    // int compile_flag = std::system(compile_command.c_str());
    // casadi_assert(compile_flag==0, "Compilation failed");
    // std::cout << "Compilation successed!" << std::endl;

    // return 0;
}


// int main(){
//     /*  Test problem
//      *
//      *    min x0^2 + x1^2
//      *    s.t.    x0 + x1 - 10 = 0
//      */

//     // Optimization variables
//     casadi::SX x = casadi::SX::sym("x", 2);

//     // Objective
//     casadi::SX f = x(0)*x(0) + x(1)*x(1);

//     // Constraints
//     casadi::SX g = x(0)+x(1)-10;

//     // Create an NLP solver instance
//     casadi::Function solver = casadi::nlpsol("solver", "ipopt", {{"x", x}, {"f", f}, {"g", g}});

//     // file name
//     std::string file_name = "nlp_code";
//     // code predix
//     std::string prefix_code = fs::current_path().string() + "/";

//     // Generate C code for the NLP functions
//     solver.generate_dependencies(file_name + ".c");

//     // shared library prefix
//     std::string prefix_lib = fs::current_path().string() + "/";

//     // compile c code to a shared library
//     std::string compile_command = "gcc -fPIC -shared -O3 " + 
//         prefix_code + file_name + ".c -o " +
//         prefix_lib + file_name + ".so";

//     std::cout << compile_command << std::endl;
//     int compile_flag = std::system(compile_command.c_str());
//     casadi_assert(compile_flag==0, "Compilation failed");
//     std::cout << "Compilation successed!" << std::endl;

//     return 0;
// }