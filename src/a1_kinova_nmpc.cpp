// Author: Basit Muhammad Imran
// Modified: Ruturaj Sudhir Sambhus
// Date: 2024-10-17

#include "a1_kinova_nmpc.hpp"
#include <yaml-cpp/yaml.h>

//Define when using standalone without old nmpc code
SX blkdiag(const std::vector<SX> &matrices)
{
    if (matrices.empty())
        return SX();

    // Start with the first matrix
    SX result = matrices[0];
    for (size_t i = 1; i < matrices.size(); ++i)
    {
        // Pad the existing result and the next matrix with zeros to make them block diagonal
        SX topRight = SX::zeros(result.size1(), matrices[i].size2());
        SX bottomLeft = SX::zeros(matrices[i].size1(), result.size2());

        // Concatenate to form the new result
        result = vertcat(horzcat(result, topRight),
                         horzcat(bottomLeft, matrices[i]));
    }

    return result;
}


/* NMPC_A1_KINOVA::NMPC_A1_KINOVA(int argc, char *argv[]) : Parameters(argc,argv){
    
}
 */
casadi::DM NMPC_A1_KINOVA::bezier(const DM &alpha, double s)
{
    int n = alpha.size1(); // Number of rows
    int m = alpha.size2(); // Number of columns
    DM value = DM::zeros(n);
    int M = m - 1;

    std::vector<int> k; // Binomial coefficients
    switch (M)
    {
    case 3:
        k = {1, 3, 3, 1};
        break;
    case 4:
        k = {1, 4, 6, 4, 1};
        break;
    case 5:
        k = {1, 5, 10, 10, 5, 1};
        break;
    case 6:
        k = {1, 6, 15, 20, 15, 6, 1};
        break;
    case 7:
        k = {1, 7, 21, 35, 35, 21, 7, 1};
        break;
    case 8:
        k = {1, 8, 28, 56, 70, 56, 28, 8, 1};
        break;
    case 9:
        k = {1, 9, 36, 84, 126, 126, 84, 36, 9, 1};
        break;
    case 10:
        k = {1, 10, 45, 120, 210, 252, 210, 120, 45, 10, 1};
        break;
    case 20:
        k = {1, 20, 190, 1140, 4845, 15504, 38760, 77520, 125970, 167960, 184756, 167960, 125970, 77520, 38760, 15504, 4845, 1140, 190, 20, 1};
        break;
    default:
        std::cerr << "Degree " << M << " not supported." << std::endl;
        return value; // Optionally handle other degrees dynamically
    }

    std::vector<double> x(M + 1, 1.0), y(M + 1, 1.0);
    for (int i = 1; i <= M; ++i)
    {
        x[i] = s * x[i - 1];
        y[i] = (1 - s) * y[i - 1];
    }

    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j <= M; ++j)
        {
            value(i) += alpha(i, j) * k[j] * x[j] * y[M - j];
        }
    }

    return value;
}

DM NMPC_A1_KINOVA::bezierd(const DM &alpha, double s)
{
    int n = alpha.size1(); // Number of rows
    int m = alpha.size2(); // Number of columns
    DM value = DM::zeros(n);
    int M = m - 1; // Degree of the Bezier curve

    std::vector<int> k; // Binomial coefficients for the derivative
    switch (M)
    {
    case 3:
        k = {3, 6, 3};
        break;
    case 4:
        k = {4, 12, 12, 4};
        break;
    case 5:
        k = {5, 20, 30, 20, 5};
        break;
    case 6:
        k = {6, 30, 60, 60, 30, 6};
        break;
    case 7:
        k = {7, 42, 105, 140, 105, 42, 7};
        break;
    case 8:
        k = {8, 56, 168, 280, 280, 168, 56, 8};
        break;
    case 9:
        k = {9, 72, 252, 504, 630, 504, 252, 72, 9};
        break;
    case 10:
        k = {10, 45, 120, 210, 252, 210, 120, 45, 10, 1};
        break;
    case 20:
        k = {20, 380, 3420, 19380, 77520, 232560, 542640, 1007760, 1511640, 1847560, 1847560, 1511640, 1007760, 542640, 232560, 77520, 19380, 3420, 380, 20};
        break;
    default:
        std::cerr << "Degree " << M << " not supported for derivative calculation." << std::endl;
        return value;
    }

    std::vector<double> x(M, 1.0), y(M, 1.0); // Adjust for derivative size
    for (int i = 1; i < M; ++i)
    {
        x[i] = s * x[i - 1];
        y[i] = (1 - s) * y[i - 1];
    }

    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < M; ++j)
        {
            value(i) += (alpha(i, j + 1) - alpha(i, j)) * k[j] * x[j] * y[M - 1 - j];
        }
    }

    return value;
}

// SX NMPC_A1_KINOVA::Rotz(const SX &t)
// {
//     // Ensure t is a vector of size 3
//     if (t.size1() != 3 || t.size2() != 1)
//     {
//         throw std::runtime_error("Input vector t must be of size 3.");
//     }

//     casadi::SX Rz = casadi::SX::zeros(3, 3);

//     // Note: In CasADi, indexing is zero-based, similar to C++ and Eigen but different from MATLAB.
//     Rz(0, 0) = cos(t(1)) * cos(t(2));
//     Rz(0, 1) = -sin(t(2));
//     Rz(0, 2) = 0;
//     Rz(1, 0) = cos(t(1)) * sin(t(2));
//     Rz(1, 1) = cos(t(2));
//     Rz(1, 2) = 0;
//     Rz(2, 0) = -sin(t(1));
//     Rz(2, 1) = 0;
//     Rz(2, 2) = 1;

//     return Rz;
// }

SX NMPC_A1_KINOVA::Rotz(const SX &t)
{
    // Ensure t is a vector of size 3
    if (t.size1() != 3 || t.size2() != 1)
    {
        throw std::runtime_error("Input vector t must be of size 3.");
    }

    casadi::SX Rz = casadi::SX::zeros(3, 3);

    // Note: In CasADi, indexing is zero-based, similar to C++ and Eigen but different from MATLAB.
    Rz(0, 0) = cos(t(2));
    Rz(0, 1) = -sin(t(2));
    Rz(0, 2) = 0;
    Rz(1, 0) = sin(t(2));
    Rz(1, 1) = cos(t(2));
    Rz(1, 2) = 0;
    Rz(2, 0) = 0;
    Rz(2, 1) = 0;
    Rz(2, 2) = 1;

    return Rz;
}

casadi::SX NMPC_A1_KINOVA::RotMat_XYZ(const casadi::SX& rpy) {
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
    R(0, 1) = -cos(rpy(1)) * sin(rpy(2));
    R(0, 2) = sin(rpy(1));
    R(1, 0) = cos(rpy(0)) * sin(rpy(2)) + sin(rpy(0)) * sin(rpy(1)) * cos(rpy(2));
    R(1, 1) = cos(rpy(0))*cos(rpy(2)) - sin(rpy(0))*sin(rpy(1))*sin(rpy(2));
    R(1, 2) = -cos(rpy(1))*sin(rpy(0));
    R(2, 0) = sin(rpy(0))*sin(rpy(2)) - cos(rpy(0))*sin(rpy(1))*cos(rpy(2));
    R(2, 1) = cos(rpy(2))*sin(rpy(0)) + cos(rpy(0))*sin(rpy(1))*sin(rpy(2));
    R(2, 2) = cos(rpy(1))*cos(rpy(0));

    return R;
}

casadi::SX NMPC_A1_KINOVA::rot_x(const casadi::SX& t) {
    // Ensure t is a vector of size 1
    if (t.size1() != 1 || t.size2() != 1) {
        throw std::runtime_error("Input vector t must be of size 1.");
    }
    casadi::SX Rx = casadi::SX::vertcat({casadi::SX::horzcat({1, 0, 0}), casadi::SX::horzcat({0, cos(t), -sin(t)}), casadi::SX::horzcat({0, sin(t), cos(t)})});
    return Rx;
}

casadi::SX NMPC_A1_KINOVA::rot_y(const casadi::SX& t){
    // Ensure t is a vector of size 1
    if (t.size1() != 1 || t.size2() != 1) {
        throw std::runtime_error("Input vector t must be of size 1.");
    }

    casadi::SX Ry =  casadi::SX::vertcat({casadi::SX::horzcat({cos(t), 0, sin(t)}), casadi::SX::horzcat({0, 1, 0}), casadi::SX::horzcat({-sin(t), 0, cos(t)})});
   
    return Ry;

}

casadi::SX NMPC_A1_KINOVA::rot_z(const casadi::SX& t){
    // Ensure t is a vector of size 1
    if (t.size1() != 1 || t.size2() != 1) {
        throw std::runtime_error("Input vector t must be of size 1.");
    }

    casadi::SX Rz =  casadi::SX::vertcat({casadi::SX::horzcat({cos(t), -sin(t), 0}), casadi::SX::horzcat({sin(t), cos(t), 0}), casadi::SX::horzcat({0, 0, 1})});
   
    return Rz;

}

casadi::SX NMPC_A1_KINOVA::RotMat_ZYX(const casadi::SX& rpy) {
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



casadi::DM NMPC_A1_KINOVA::RotMat_ZYX(const casadi::DM& rpy) {
    //ZYX convention Rz*Ry*Rx
    // Ensure rpy is a vector of size 3
    if (rpy.size1() != 3 || rpy.size2() != 1) {
        throw std::runtime_error("Input vector rpy must be of size 3.");
    }
    /* phi = rpy(0);
       theta = rpy(1);
       psi = rpy(2) */

    casadi::DM R = casadi::DM::zeros(3, 3);

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

casadi::DM NMPC_A1_KINOVA::RotMat_XYZ(const casadi::DM& rpy) {
    //ZYX convention Rz*Ry*Rx
    // Ensure rpy is a vector of size 3
    if (rpy.size1() != 3 || rpy.size2() != 1) {
        throw std::runtime_error("Input vector rpy must be of size 3.");
    }
    /* phi = rpy(0);
       theta = rpy(1);
       psi = rpy(2) */

    casadi::DM R = casadi::DM::zeros(3, 3);

    // Note: In CasADi, indexing is zero-based, similar to C++ and Eigen but different from MATLAB.
    R(0, 0) = cos(rpy(1)) * cos(rpy(2));
    R(0, 1) = -cos(rpy(1)) * sin(rpy(2));
    R(0, 2) = sin(rpy(1));
    R(1, 0) = cos(rpy(0)) * sin(rpy(2)) + sin(rpy(0)) * sin(rpy(1)) * cos(rpy(2));
    R(1, 1) = cos(rpy(0))*cos(rpy(2)) - sin(rpy(0))*sin(rpy(1))*sin(rpy(2));
    R(1, 2) = -cos(rpy(1))*sin(rpy(0));
    R(2, 0) = sin(rpy(0))*sin(rpy(2)) - cos(rpy(0))*sin(rpy(1))*cos(rpy(2));
    R(2, 1) = cos(rpy(2))*sin(rpy(0)) + cos(rpy(0))*sin(rpy(1))*sin(rpy(2));
    R(2, 2) = cos(rpy(1))*cos(rpy(0));

    return R;
}
//TODO: Use template to avoid code duplication
Eigen::Matrix<double,3,3> NMPC_A1_KINOVA::RotMat_XYZ(const Eigen::Matrix<double,3,1>& rpy) {
    //ZYX convention Rz*Ry*Rx
    // Ensure rpy is a vector of size 3
    if (rpy.rows() != 3 || rpy.cols() != 1) {
        throw std::runtime_error("Input vector rpy must be of size 3.");
    }
    /* phi = rpy(0);
       theta = rpy(1);
       psi = rpy(2) */

   Eigen::Matrix<double,3,3> R = Eigen::Matrix<double,3,3>::Zero();

    // Note: In CasADi, indexing is zero-based, similar to C++ and Eigen but different from MATLAB.
    R(0, 0) = cos(rpy(1)) * cos(rpy(2));
    R(0, 1) = -cos(rpy(1)) * sin(rpy(2));
    R(0, 2) = sin(rpy(1));
    R(1, 0) = cos(rpy(0)) * sin(rpy(2)) + sin(rpy(0)) * sin(rpy(1)) * cos(rpy(2));
    R(1, 1) = cos(rpy(0))*cos(rpy(2)) - sin(rpy(0))*sin(rpy(1))*sin(rpy(2));
    R(1, 2) = -cos(rpy(1))*sin(rpy(0));
    R(2, 0) = sin(rpy(0))*sin(rpy(2)) - cos(rpy(0))*sin(rpy(1))*cos(rpy(2));
    R(2, 1) = cos(rpy(2))*sin(rpy(0)) + cos(rpy(0))*sin(rpy(1))*sin(rpy(2));
    R(2, 2) = cos(rpy(1))*cos(rpy(0));

    return R;
}

void NMPC_A1_KINOVA::get_Euler_ZYX_from_XYZ(Eigen::Matrix<double,3,1>& rpy_zyx, const Eigen::Matrix<double,3,1>& rpy_xyz){
    Eigen::Matrix<double,3,3> R = RotMat_XYZ(rpy_xyz);

    rpy_zyx(1) = asin(-R(2,0));
    double cos_theta = cos(rpy_zyx(1));
    rpy_zyx(0)= atan2(R(2,1)/cos_theta, R(2,2)/cos_theta);
    rpy_zyx(2)= atan2(R(1,0)/cos_theta, R(0,0)/cos_theta);
}

void NMPC_A1_KINOVA::get_Euler_XYZ_from_ZYX(casadi::DM& rpy_xyz, const casadi::DM& rpy_zyx){
    casadi::DM R = RotMat_ZYX(rpy_zyx);

    rpy_xyz(1) = asin(R(0,2));
    casadi::DM cos_theta = cos(rpy_xyz(1));
    rpy_xyz(0)= atan2(-R(1,2)/cos_theta, R(2,2)/cos_theta);
    rpy_xyz(2)= atan2(-R(0,1)/cos_theta, R(0,0)/cos_theta);
}

void NMPC_A1_KINOVA::get_Omega_ZYX_world_from_XYZ_body(Eigen::Matrix<double,3,1>& omega_zyx_world, const Eigen::Matrix<double,3,1>& omega_xyz_body, const Eigen::Matrix<double,3,1>& rpy_xyz)
{
    Eigen::Matrix<double,3,3> R = RotMat_XYZ(rpy_xyz);
    omega_zyx_world = R*omega_xyz_body;

}

void NMPC_A1_KINOVA::get_Omega_ZYX_world_from_XYZ_world(Eigen::Matrix<double,3,1>& omega_zyx_world, const Eigen::Matrix<double,3,1>& omega_xyz_world, const Eigen::Matrix<double,3,1>& rpy_xyz)
{
    omega_zyx_world = omega_xyz_world;

}


void NMPC_A1_KINOVA::get_Omega_ZYX_world_from_Eulerrate_XYZ_world(Eigen::Matrix<double,3,1>& omega_zyx_world, const Eigen::Matrix<double,3,1>& omega_xyz_world, const Eigen::Matrix<double,3,1>& rpy_xyz)
{
    Eigen::Matrix<double,3,3> Ainv_theta = Ainv_Theta_XYZ(rpy_xyz);
    omega_zyx_world = Ainv_theta*omega_xyz_world;

}

void NMPC_A1_KINOVA::get_Omega_ZYX_world_from_Eulerrate_XYZ_body(Eigen::Matrix<double,3,1>& omega_zyx_world, const Eigen::Matrix<double,3,1>& omega_xyz_world, const Eigen::Matrix<double,3,1>& rpy_xyz)
{
    Eigen::Matrix<double,3,3> Ainv_theta = Ainv_Theta_XYZ(rpy_xyz);
    Eigen::Matrix<double,3,3> R = RotMat_XYZ(rpy_xyz);
    omega_zyx_world = Ainv_theta*R*omega_xyz_world;

}



void NMPC_A1_KINOVA::get_Omega_XYZ_body_from_ZYX_world(casadi::DM& omega_xyz_body, const casadi::DM& omega_zyx_world, const casadi::DM& rpy_zyx)
{
    casadi::DM R = RotMat_ZYX(rpy_zyx);
    omega_xyz_body = casadi::DM::mtimes(R.T(),omega_zyx_world);
    
}

void NMPC_A1_KINOVA::get_Omega_XYZ_world_from_ZYX_world(casadi::DM& omega_xyz_world, const casadi::DM& omega_zyx_world, const casadi::DM& rpy_zyx)
{
    casadi::DM R = RotMat_ZYX(rpy_zyx);
    omega_xyz_world = omega_zyx_world;

}

void NMPC_A1_KINOVA::get_Eulerrate_XYZ_world_from_ZYX_world(casadi::DM& omega_xyz_world, const casadi::DM& omega_zyx_world, const casadi::DM& rpy_xyz)
{
    casadi::DM A_theta = A_Theta_XYZ(rpy_xyz);
    omega_xyz_world = casadi::DM::mtimes(A_theta, omega_zyx_world);

}

void NMPC_A1_KINOVA::get_Eulerrate_XYZ_body_from_ZYX_world(casadi::DM& omega_xyz_world, const casadi::DM& omega_zyx_world, const casadi::DM& rpy_xyz)
{
    casadi::DM A_theta = A_Theta_XYZ(rpy_xyz);
    casadi::DM R = RotMat_XYZ(rpy_xyz);
    omega_xyz_world = casadi::DM::mtimes(R.T(), casadi::DM::mtimes(A_theta, omega_zyx_world));

}



casadi::SX NMPC_A1_KINOVA::A_Theta_ZYX(const casadi::SX& rpy){
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

Eigen::Matrix<double,3,3> NMPC_A1_KINOVA::A_Theta_ZYX(const Eigen::Matrix<double,3,1>& rpy){
    //ZYX convention Rz*Ry*Rx
    // Ensure rpy is a vector of size 3
    if (rpy.rows() != 3 || rpy.cols() != 1) {
        throw std::runtime_error("Input vector rpy must be of size 3.");
    }
     /* phi = rpy(0);
       theta = rpy(1);
       psi = rpy(2) */
    Eigen::Matrix<double,3,3> A = Eigen::Matrix<double,3,3>::Zero();
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

casadi::SX NMPC_A1_KINOVA::A_Theta_XYZ(const casadi::SX& rpy){
    //ZYX convention Rz*Ry*Rx
    // Ensure rpy is a vector of size 3
    if (rpy.size1() != 3 || rpy.size2() != 1) {
        throw std::runtime_error("Input vector rpy must be of size 3.");
    }
     /* phi = rpy(0);
       theta = rpy(1);
       psi = rpy(2) */
    casadi::SX A = casadi::SX::zeros(3, 3);
    A(0,0) = 1;
    A(0,1) = sin(rpy(0))*tan(rpy(1));
    A(0,2) = -cos(rpy(0))*tan(rpy(1));
    A(1,0) = 0;
    A(1,1) = cos(rpy(0));
    A(1,2) = sin(rpy(0));
    A(2,0) = 0;
    A(2,1) = -sin(rpy(0))/cos(rpy(1));
    A(2,2) = cos(rpy(0))/cos(rpy(1));

    return A;

}


casadi::DM NMPC_A1_KINOVA::A_Theta_XYZ(const casadi::DM& rpy){
    //ZYX convention Rz*Ry*Rx
    // Ensure rpy is a vector of size 3
    if (rpy.size1() != 3 || rpy.size2() != 1) {
        throw std::runtime_error("Input vector rpy must be of size 3.");
    }
     /* phi = rpy(0);
       theta = rpy(1);
       psi = rpy(2) */
    casadi::DM A = casadi::DM::zeros(3, 3);
    A(0,0) = 1;
    A(0,1) = sin(rpy(0))*tan(rpy(1));
    A(0,2) = -cos(rpy(0))*tan(rpy(1));
    A(1,0) = 0;
    A(1,1) = cos(rpy(0));
    A(1,2) = sin(rpy(0));
    A(2,0) = 0;
    A(2,1) = -sin(rpy(0))/cos(rpy(1));
    A(2,2) = cos(rpy(0))/cos(rpy(1));


    return A;

}


Eigen::Matrix<double,3,3> NMPC_A1_KINOVA::A_Theta_XYZ(const Eigen::Matrix<double,3,1>& rpy){
    //ZYX convention Rz*Ry*Rx
    // Ensure rpy is a vector of size 3
    if (rpy.rows() != 3 || rpy.cols() != 1) {
        throw std::runtime_error("Input vector rpy must be of size 3.");
    }
     /* phi = rpy(0);
       theta = rpy(1);
       psi = rpy(2) */
    Eigen::Matrix<double,3,3> A = Eigen::Matrix<double,3,3>::Zero();
    A(0,0) = 1;
    A(0,1) = sin(rpy(0))*tan(rpy(1));
    A(0,2) = -cos(rpy(0))*tan(rpy(1));
    A(1,0) = 0;
    A(1,1) = cos(rpy(0));
    A(1,2) = sin(rpy(0));
    A(2,0) = 0;
    A(2,1) = -sin(rpy(0))/cos(rpy(1));
    A(2,2) = cos(rpy(0))/cos(rpy(1));


    return A;

}

Eigen::Matrix<double,3,3> NMPC_A1_KINOVA::Ainv_Theta_XYZ(const Eigen::Matrix<double,3,1>& rpy)
{
    Eigen::Matrix<double,3,3> A = A_Theta_XYZ(rpy);
    return A.inverse();
}


casadi::SX NMPC_A1_KINOVA::skewSym(const casadi::SX &x)
{
    using namespace casadi;

    // Determine the size based on the input x
    int cols = x.size2();
    SX mat = SX::zeros(3, 3 * cols);

    // Loop through each column of x to create skew-symmetric matrices
    for (int n = 0; n < cols; ++n)
    {
        SX slice = x(Slice(), n); // Get the nth column of x
        SX skew = SX::zeros(3, 3);
        // Fill the skew-symmetric matrix
        skew(0, 1) = -slice(2);
        skew(0, 2) = slice(1);
        skew(1, 0) = slice(2);
        skew(1, 2) = -slice(0);
        skew(2, 0) = -slice(1);
        skew(2, 1) = slice(0);

        // Place the skew matrix into the correct position in mat
        mat(Slice(0, 3), Slice(3 * n, 3 * (n + 1))) = skew;
    }

    return mat;
}

Eigen::MatrixXd NMPC_A1_KINOVA::skewSym(const Eigen::MatrixXd& x)
{
    using namespace casadi;

    // Determine the size based on the input x
    int cols = x.cols();
    Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(3, 3 * cols);

    // Loop through each column of x to create skew-symmetric matrices
    for (int n = 0; n < cols; ++n)
    {
        Eigen::Matrix<double, 3, 1> slice = x.col(n); // Get the nth column of x
        Eigen::MatrixXd skew = Eigen::MatrixXd::Zero(3, 3);
        // Fill the skew-symmetric matrix
        skew(0, 1) = -slice(2);
        skew(0, 2) = slice(1);
        skew(1, 0) = slice(2);
        skew(1, 2) = -slice(0);
        skew(2, 0) = -slice(1);
        skew(2, 1) = slice(0);

        // Place the skew matrix into the correct position in mat
        mat.block(0, 3 * n, 3, 3) = skew;
    }

    return mat;
}

casadi::DM NMPC_A1_KINOVA::transform_ZYX_to_XYZ(const casadi::DM& zyx_input)
{
    
    // Determine the size based on the input x
    int rows = zyx_input.size1();
    int cols = zyx_input.size2();
    casadi::DM xyz_output = zyx_input;

    if (cols != params.n_SRB)
    {
        throw std::runtime_error("Number of columns of the input must be n_SRB ");
    }
    casadi::DM vel = zyx_input(Slice(), Slice(3, 6));


    casadi::DM eul = zyx_input(Slice(), Slice(6, 9));
    casadi::DM omega = zyx_input(Slice(), Slice(9, 12));
    // Loop through each column of x to create skew-symmetric matrices
    for (int n = 0; n < rows; ++n)
    {
        casadi::DM rpy_zyx = eul(Slice(n, n + 1), Slice()).T();
        casadi::DM omega_zyx = omega(Slice(n, n + 1), Slice()).T();
        casadi::DM rpy_xyz = casadi::DM::zeros(3,1);
        casadi::DM omega_xyz = casadi::DM::zeros(3,1);


        get_Euler_XYZ_from_ZYX(rpy_xyz,rpy_zyx);
        // get_Omega_XYZ_body_from_ZYX_world(omega_xyz, omega_zyx, rpy_zyx);
        // get_Omega_XYZ_world_from_ZYX_world(omega_xyz, omega_zyx, rpy_zyx);
        get_Eulerrate_XYZ_world_from_ZYX_world(omega_xyz, omega_zyx, rpy_xyz);
        // get_Eulerrate_XYZ_body_from_ZYX_world(omega_xyz, omega_zyx, rpy_xyz);

        xyz_output(Slice(n, n + 1), Slice(6, 9)) = rpy_xyz.T();
        xyz_output(Slice(n, n + 1), Slice(9, 12)) = omega_xyz.T();

        casadi::DM vel_world = vel(Slice(n, n + 1), Slice()).T();
        casadi::DM R_zyx = RotMat_ZYX(rpy_zyx);
        casadi::DM vel_body = casadi::DM::mtimes(R_zyx.T(), vel_world);
        xyz_output(Slice(n, n + 1), Slice(3, 6)) = vel_body.T();
        
    }
    
    return xyz_output;

 
}

void NMPC_A1_KINOVA::transform_XYZ_to_ZYX(double *q, double *dq){

    Eigen::Map<Eigen::Matrix<double, 3, 1>> eul_xyz(&q[3],3);
    Eigen::Map<Eigen::Matrix<double, 3, 1>> omega_xyz_body(&dq[3],3);
    Eigen::Matrix3d R_xyz = RotMat_XYZ(eul_xyz);
    Eigen::Matrix<double,3,1> eul_zyx, omega_zyx_world;
    eul_zyx.setZero();
    omega_zyx_world.setZero();
    get_Euler_ZYX_from_XYZ(eul_zyx, eul_xyz);
    get_Omega_ZYX_world_from_XYZ_body(omega_zyx_world, omega_xyz_body, eul_xyz);
    
    for (size_t i = 0; i < 3; ++i)
    {
        q[3+i] = eul_zyx(i);
        dq[3+i] = omega_zyx_world(i);
    }
}

/* void NMPC_A1_KINOVA::updateJoystickInput(Eigen::VectorXd& input){
    auto start = std::chrono::high_resolution_clock::now();
    JoystickEvent event;
    while (joystick.sample(&event))
    {
        if (event.isAxis())
        {
            if (event.number ==0 || event.number == 6 || event.number == 4 || event.number == 3 || event.number == 7 )
            {   
                int max_val = event.MAX_AXES_VALUE;
                int min_val = event.MIN_AXES_VALUE;
                double range = max_val - min_val;
    
            
            
                if (event.number == 4)
                {
                    input(0) = -((event.value)/range); //-0.5 to +0.5;
                }
                else if (event.number == 3)
                {
                    input(1) = -((event.value)/range); //-0.5 to +0.5;
                }
                else if (event.number == 0)
                {
                    input(2) = -((event.value)/range); //-0.5 to +0.5;
                }
                else if (event.number == 6)
                {
                    input(3) =-3*((event.value)/range); //-1.5 to +1.5;
                }
                else if (event.number == 7)
                {
                    input(4) = -3*((event.value)/range); //-1.5 to +1.5;
                }
            }
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout  <<" Joystick Time: " << duration.count() << " us" << std::endl;

}
 */
void NMPC_A1_KINOVA::updateReference(float *joystick_state){
    memcpy(this->joystick_state, joystick_state, sizeof(this->joystick_state));
};


void NMPC_A1_KINOVA::planner()
{
    const int n_states = params.n_states; // Example: Number of states
    const int n_u_SRB = params.n_u_SRB;   // Example: Number of control inputs
    // const int N = params.N;
    //JoyStick input
    if (joystick_gait_counter_==30)
    {
        joystick_toggle_gait_ = joystick_toggle_gait_ ^ static_cast<bool>(joystick_state[18]);
        joystick_gait_counter_ = 0;    
        joystick_sit_down_ =  !joystick_toggle_gait_ ?  joystick_sit_down_ || static_cast<bool>(joystick_state[12]):0;
    }
    joystick_gait_counter_++;
    std::cout << "Joystick toggle gait: " << joystick_toggle_gait_ << std::endl;
   
   

    DM xd = DM::zeros(n_states * params.N + params.n_u_SRB);
    const float stand_duration = 4; // Stand up duration
    const float sit_duration = 4; // Sit down duration
    double settlingTime= 8; // Time to settle
    double stand_settle = 1; // Time to settle
    double trot_settle_counter = 50; // Counter to settle for Trot to Stand - multiply by NMPC freq to get time in s

    if (params.current_time <= settlingTime)
    {
        x0 = static_cast<double>(qk_(0));
        y0 = static_cast<double>(qk_(1));
        z0 = static_cast<double>(qk_(2));
        xf = static_cast<double>(qk_(0));
        yf = static_cast<double>(qk_(1));
        zf = params.height; // Assuming the height is constant or predefined
        yaw0 = static_cast<double>(qk_(8));
    

    }   
    
    
    //Gait selection for 
    if (params.current_time <= settlingTime + stand_duration)
    { // Stand up duration
        params.gait = 1;
    }
    else if (params.current_time <= settlingTime + stand_duration + stand_settle)
    {
        params.gait = 9;
    }
    else
    {   
        if (joystick_sit_down_)
        {
            params.gait = 10;
        }
        else if (joystick_toggle_gait_)
        {
            params.gait = 8; //TROT
            reduce_swing_height_ = false; // Reset swing height for TROT
            joystick_send_gait_ = joystick_toggle_gait_; // TROT or STAND_MPC
            gait_switch_counter_= 0;
            first_stand_flag_ = false; // Reset first stand flag
        }
        else
        {    
            if (gait_switch_counter_ >= trot_settle_counter || first_stand_flag_)
            {
                params.gait = 9; // STAND_MPC 
                joystick_send_gait_ = joystick_toggle_gait_; // TROT or STAND_MPC
                gait_switch_counter_ = 0; // Reset counter after switching to STAND_MPC
                first_stand_flag_ = true; // Set first stand flag to true
            }
            else
            {
                params.gait = 8; // TROT
                reduce_swing_height_ = true; // Reduce swing height for TROT
                gait_switch_counter_++;
                first_stand_flag_ = false; // Set first stand flag to true
            }
            
            
            
        }
        
    }

    // writeMatrixToFile(q_des_, basePath + "q_des_.txt");
    // traj_ = q_des_(casadi::Slice(0, n_states), 0); // Second parameter 0 is to indicate column index for a vector

    switch (params.gait)
    {
    case 1:
    {
        
        double t0 = settlingTime;
        double tf = t0 + stand_duration;                                          // Start and end times for mode 1
        double s = std::min((params.current_time - t0) / (tf - t0), 1.0); // Ensure s is within [0, 1]
        s = std::max(s, 0.0); // Ensure s is non-negative      
        

        /* // Extract relevant parameters for easy access
        double xf = -0.03;           // Initial x position
        double yf = -0.02;           // Initial y position
        double x0 = -0.03;           // Initial x position
        double y0 = -0.02;           // Initial y position
        double zf = params.height; // Assuming the height is constant or predefined
        double z0 = 0.055;       // Initial z position */

         // Calculate new-footholds
        params.pf(0, 0) = toePos_(0, 0);
        params.pf(1, 0) = toePos_(1, 0);
        params.pf(2, 0) = toePos_(2, 0);
        params.pf(0, 1) = toePos_(0, 1);
        params.pf(1, 1) = toePos_(1, 1);
        params.pf(2, 1) = toePos_(2, 1);
        params.pf(0, 2) = toePos_(0, 2);
        params.pf(1, 2) = toePos_(1, 2);
        params.pf(2, 2) = toePos_(2, 2);
        params.pf(0, 3) = toePos_(0, 3);
        params.pf(1, 3) = toePos_(1, 3);
        params.pf(2, 3) = toePos_(2, 3);


        // Using DM::horzcat to create each row of the alpha matrix
        casadi::DM row1 = casadi::DM::horzcat({DM(x0), DM(x0), DM(x0), DM(xf / 4), DM(3 * xf / 4), DM(xf), DM(xf), DM(xf)});
        casadi::DM row2 = casadi::DM::horzcat({DM(y0), DM(y0), DM(y0), DM(yf / 4), DM(3 * yf / 4), DM(yf), DM(yf), DM(yf)});
        casadi::DM row3 = casadi::DM::horzcat({DM(z0), DM(z0), DM(z0), DM(zf / 4), DM(3 * zf / 4), DM(zf), DM(zf), DM(zf)});

        casadi::DM alpha = casadi::DM::vertcat({row1, row2, row3});

        // Update trajectory position
        casadi::DM position_update = bezier(alpha, s);

        for (int i = 0; i < 3; ++i)
        {
            traj_(i) = position_update(i);
            traj_(i + params.n_SRB) = position_update(i) + params.dd(i); // Update arm base position
        }
        traj_(8) = yaw0; // Keep yaw constant

        // Update trajectory velocity
        casadi::DM velocity_update = bezierd(alpha, s);

        for (int i = 0; i < 3; ++i)
        {
            traj_(i + 3) = velocity_update(i)/(tf-t0);
            traj_(i+params.n_SRB+ static_cast<int>(params.n_arm/2)) = velocity_update(i)/(tf-t0); // Update arm base velocity
        }
        break;
    }
    case 8:
    {
        
        double domLen = 200.0;                         // Domain length in milliseconds
        double domLenSec = domLen / 1000.0;            // Domain length in seconds
        DM R_ZYX = RotMat_ZYX(vertcat(qk_(6), qk_(7), qk_(8))); // Assuming q contains Euler angles in the last 3 elements
        // size_t currentStep = std::floor(t / params.dt) + 1;
        // std::cout << "What is NMPC domain?: " << domain_ << std::endl;
        // std::cout << "What is params.stpcnt?: " << params.stepcnt << std::endl;
        // std::cout << "What is params.current_time: " << params.current_time << std::endl;
        // std::cout << "What is control_tick_: " << control_tick_ * 0.001 << std::endl;
        // std::cout << "What is my toePos_: " << toePos_ << std::endl;

        // TODO-Integration : LL is responsible for handling the domain change and corresponding updates
        // Such as changing the contact sequence, increasing velocity, and updating step foot position
        // if (params.current_time >= (params.stepcnt * domLenSec + 1)) {  // Domain change condition // params.stepcnt previously defined

        // TODO-Integration : Enforce domain change through LL
        // if (params.current_time >= (domain_ * domLenSec + 1)) {

        if (isDomainChanged_)
        {
            // Update contact sequence
            if (static_cast<double>(params.contact(0)) == 1.0)
            {
                params.contact = {0, 1, 1, 0};
            }
            else
            {
                params.contact = {1, 0, 0, 1};
            }

            params.contact(0) = contactInd[0];
            params.contact(1) = contactInd[1];
            params.contact(2) = contactInd[2];
            params.contact(3) = contactInd[3];

            
            // Calculate new-footholds
            params.pf(0, 0) = toePos_(0, 0);
            params.pf(1, 0) = toePos_(1, 0);
            params.pf(2, 0) = toePos_(2, 0);
            params.pf(0, 1) = toePos_(0, 1);
            params.pf(1, 1) = toePos_(1, 1);
            params.pf(2, 1) = toePos_(2, 1);
            params.pf(0, 2) = toePos_(0, 2);
            params.pf(1, 2) = toePos_(1, 2);
            params.pf(2, 2) = toePos_(2, 2);
            params.pf(0, 3) = toePos_(0, 3);
            params.pf(1, 3) = toePos_(1, 3);
            params.pf(2, 3) = toePos_(2, 3);
/* 
            params.p_hip(0,0) = toePos_(0, 0);  params.p_hip(1,0) = toePos_(1, 0); params.p_hip(2,0) = toePos_(2, 0);
            params.p_hip(0,1) = toePos_(0, 1);  params.p_hip(1,1) = toePos_(1, 1); params.p_hip(2,1) = toePos_(2, 1);
            params.p_hip(0,2) = toePos_(0, 2);  params.p_hip(1,2) = toePos_(1, 2); params.p_hip(2,2) = toePos_(2, 2);
            params.p_hip(0,3) = toePos_(0, 3);  params.p_hip(1,3) = toePos_(1, 3); params.p_hip(2,3) = toePos_(2, 3);

            DM hipTmp = params.p_hip + stepLen;
            hipTmp = mtimes(R, hipTmp); // Rotate to the world frame
            params.pf = hipTmp + vertcat(qk_(Slice(0, 2)), 0.0);
 */
            // writeMatrixToFile(traj, basePath + "traj_planner.txt");
            // params.stepcnt++;
            // params.stepcnt++;
            params.stepcnt = domain_;
        }

        DM cmdBody = mtimes(R_ZYX.T(), traj_(Slice(3, 6))); // Convert to body frame
        traj_(Slice(3, 6)) = cmdBody;


        /* // Increase velocity in body frame
        traj_(3) += 0.1; // Increase command velocity in body X
        traj_(4) += 0.1; // Increase command velocity in body Y
        // traj_(11) += 0.05; // Increase command velocity in yaw

        // Saturate the command velocities
        traj_(3) = fmin(traj_(3), params.desVel(0));
        traj_(4) = fmin(traj_(4), params.desVel(1));
            */
        // Increase velocity in body frame
        traj_(3) +=((abs(joystick_state[3]-static_cast<double>(traj_(3)))<0.1) ? joystick_state[3]-traj_(3): 0.1*sign(joystick_state[3]-traj_(3))); // Increase command velocity in body X
        //Saturate the velocity to avoid exceeding the joystick input
        traj_(3) = sign(traj_(3))*fmin(abs(traj_(3)), 0.5);
        traj_(4) += ((abs(joystick_state[2]-static_cast<double>(traj_(4))) < 0.1 )? joystick_state[2]-traj_(4): 0.1*sign(joystick_state[2]-traj_(4))); // Increase command velocity in body Y
        traj_(4) = sign(traj_(4))*fmin(abs(traj_(4)), 0.5);
        traj_(10) += 0*((abs(joystick_state[1]-static_cast<double>(traj_(10))) < 0.1)? joystick_state[1]-traj_(10): 0.1*sign(joystick_state[1]-traj_(10))); // Increase command velocity in pitch
        // traj_(11) += ((abs(joystick_state[0]-static_cast<double>(traj_(11))) < 0.1)? joystick_state[0]-traj_(11): 0.1*sign(joystick_state[0]-traj_(11))); // Increase command velocity in yaw
        traj_(11) = 0; // Yaw rate command disabled - heading held constant
        // traj_(28) += 0*((abs(joystick_state[6]-static_cast<double>(traj_(28))) < 0.2)? joystick_state[6]-traj_(28): 0.2*sign(joystick_state[6]-traj_(28))); // Increase command velocity in arm yaw
        // traj_(29) += 0*((abs(joystick_state[4]-static_cast<double>(traj_(29))) < 0.2)? joystick_state[4]-traj_(29): 0.2*sign(joystick_state[4]-traj_(29))); // Increase command velocity in arm j2
        // traj_(30) += 0*((abs(joystick_state[8]-static_cast<double>(traj_(30))) < 0.2)? joystick_state[8]-traj_(30): 0.2*sign(joystick_state[8]-traj_(30))); // Increase command velocity in arm j3
        // traj_(31) += 0*((abs(joystick_state[10]-static_cast<double>(traj_(31))) < 1)? joystick_state[10]-traj_(31): 1*sign(joystick_state[10]-traj_(31))); // Increase command velocity in arm j4
        /* // Saturate the command velocities
        traj_(3) = sign(input(0))*fmin(abs(traj_(3)), abs(input(0)));
        traj_(4) = sign(input(1))*fmin(abs(traj_(4)), abs(input(1)));
        traj_(11) = sign(input(2))*fmin(abs(traj_(11)), abs(input(2)));
*/
        /* std::cout << "What is my input(0): " << input(0) << std::endl;
        std::cout << "What is my input(1): " << input(1) << std::endl;
        std::cout << "What is my input(2): " << input(2) << std::endl;
        std::cout << "What is my input(3): " << input(3) << std::endl;
        std::cout << "What is my input(4): " << input(4) << std::endl;
*/
        traj_(28) = joystick_state[6]; // Set arm yaw velocity directly from joystick input
        traj_(29) = joystick_state[4]; // Set arm joint 2 velocity directly from joystick input 
        traj_(30) = joystick_state[8]; // Set arm joint 3 velocity directly from joystick input
        traj_(31) = joystick_state[10]; // Set arm joint 4 velocity directly from joystick input


        /* Eigen::Matrix<double,3,1> input = updateJoystickInput();

        // Increase velocity in body frame
        traj_(3) += 0.1*sign(input(0)); // Increase command velocity in body X
        traj_(4) += 0.1*sign(input(1)); // Increase command velocity in body Y
        traj_(11) += 0.05*sign(input(2)); // Increase command velocity in ya1

        // Saturate the command velocities
        traj_(3) = sign(input(0))*fmin(abs(traj_(3)), abs(input(0)));
        traj_(4) = sign(input(1))*fmin(abs(traj_(4)), abs(input(1)));
        traj_(11) = sign(input(2))*fmin(abs(traj_(11)), abs(input(2))); */

        // traj_(3) = 0.5  ; //Prd_refined_(0, params.mpc_iter);
        // traj_(4) = Prd_refined_(1, params.mpc_iter);

        // Calculate step length in body frame and update step foot position
        // DM stepLen = vertcat(traj_(3) * domLenSec / 2, 0.0, 0.0);
            // vertcat(traj_(3) * domLenSec / 2, traj_(4) * domLenSec / 2, 0.0);

        // Set velocity in world frame
        DM cmdWrld = mtimes(R_ZYX, traj_(Slice(3, 6))); // Convert to world frame
        traj_(Slice(3, 6)) = cmdWrld;
        DM A_theta_ZYX  = A_Theta_ZYX(vertcat(qk_(6), qk_(7), qk_(8)));
        // DM cmdWrld_omega = mtimes(A_theta_ZYX, traj(Slice(9, 12))); // Convert to world frame
        //MIght be unstable due to this
        DM cmdWrld_omega = casadi::DM::solve(A_theta_ZYX, traj_(Slice(9, 12))); // Convert to world frame
        traj_(Slice(9, 12)) = cmdWrld_omega;



        float heading = 0;
        float yaw_lim = 3.14;
        float pitch_lim = 0.35;

        float pitch_vel_lim = 0.1;
        float arm_joint_vel_lim = 0.6; // Limit for arm joint velocities
        float yaw_vel_lim = 0.1; // Limit for yaw velocity
        // Update position and height based on velocity
        traj_(0) = qk_(0) + params.dt * traj_(3);
        traj_(1) = qk_(1) + params.dt * traj_(4);
        traj_(2) = params.height; // Assuming height is constant or predefined
        traj_(5) = 0;             // Assuming zero vertical velocity
        traj_(6) = 0;             // Roll or phi
        traj_(7) = 0;// params.dt * traj_(10);             // Pitch or theta
        traj_(8) += params.dt * traj_(11); //qk_(8) + params.dt * traj_(11); // Update yaw angle
    // saturate yaw angle to 1.2 
        traj_(7) = fmin(pitch_lim, fmax(-pitch_lim, traj_(7)));
        traj_(8) = fmin(yaw_lim, fmax(-yaw_lim, traj_(8)));
        traj_(9) = 0;             // Angular velocities (assuming zeros)//Saturate the command velocities
        // traj_(10) = sign(traj_(10))*fmin(abs(traj_(10)), pitch_vel_lim);
        traj_(28) = sign(traj_(28))*fmin(abs(traj_(28)), arm_joint_vel_lim);
        traj_(29) = sign(traj_(29))*fmin(abs(traj_(29)), arm_joint_vel_lim);
        traj_(30) = sign(traj_(30))*fmin(abs(traj_(30)), arm_joint_vel_lim);
        traj_(31) = sign(traj_(31))*fmin(abs(traj_(31)), 10*arm_joint_vel_lim);            
        // Update arm shape variables
        traj_(18) += params.dt * traj_(28); //joint 1 angle
        traj_(19) += params.dt * traj_(29); //joint 2 angle
        traj_(20) += params.dt * traj_(30); //joint 3 angle
        traj_(21) += params.dt * traj_(31); //joint 4 angle
        for (int n = 1; n <= (int)params.N; ++n)
        {
            // Compute the predicted state for each step in the horizon
            DM state = DM::zeros(n_states, 1);

            // Index to access the right column in Prd_refined_
            int index = params.mpc_iter + n - 1; // Adjust index for zero-based indexing if necessary
/* 
            // Velocity components from Prd_refined_
            double vx = Prd_refined_(0, index); // Access the velocity in x-direction
            double vy = Prd_refined_(1, index); // Access the velocity in y-direction */

            // Updating the state based on velocities from Prd_refined_
            state(0) = qk_(0) + n * params.dt * traj_(3); // pos X
            state(1) = qk_(1) + n * params.dt * traj_(4); // pos Y
            state(2) = params.height;               // pos Z (constant height assumption)
            state(3) = traj_(3); // vx;                          // vel X (updated from Prd_refined_)
            state(4) = traj_(4); //vy;                          // vel Y (updated from Prd_refined_)
            state(5) = 0.0;                         // vel Z (assuming zero vertical velocity)
            // Assuming Euler angles (heading) and angular velocity are zero
            state(6) = 0.0;     // Roll or phi
            state(7) = 0.0; //traj_(7) + (n-1)*params.dt * traj_(10);     // Pitch or theta
            state(8) = traj_(8) + (n-1)*params.dt * traj_(11) ;//qk_(8) + n * params.dt * traj_(11); // Yaw or psi
            // state(8) = 0;
            state(7) = fmin(pitch_lim, fmax(-pitch_lim, state(7))); //saturation of pitch angle
            //saturation of yaw angle
            state(8) = fmin(yaw_lim, fmax(-yaw_lim, state(8)));
            state(9) = 0.0;     // Angular velocities (assuming zeros)
            state(10) = 0.0;
            state(11) = traj_(11); // Angular velocity in yaw
            
            DM rot_mat = RotMat_ZYX(vertcat(state(6), state(7), state(8)));
            DM rot_mat_times_dd = DM::mtimes(rot_mat, vertcat(params.dd(0), params.dd(1), params.dd(2)));
            DM rot_mat_times_traj = DM::mtimes(rot_mat, vertcat(traj_(3), traj_(4), 0.0)); //Need to fix this to correct velocity

            //Arm states
            state(12) = qk_(0) + n * params.dt * traj_(3) + rot_mat_times_dd(0); // pos X
            state(13) = qk_(1) + n * params.dt * traj_(4) + rot_mat_times_dd(1); // pos Y
            state(14) = params.height + rot_mat_times_dd(2);                  // pos Z 

            state(15) = 0; //roll or phi
            state(16) = traj_(7) + (n-1) * params.dt * traj_(10); //pitch or theta
            state(17) = traj_(8) + (n-1) * params.dt * traj_(11); //yaw or psi

            state(18) = traj_(18) + (n-1)*params.dt* traj_(28); //joint 1 angle
            state(19) =  traj_(19) + (n-1)*params.dt* traj_(29); //joint 2 angle
            state(20) = traj_(20) + (n-1)*params.dt* traj_(30); //joint 3 angle
            state(21) = traj_(21) + (n-1)*params.dt* traj_(31); //joint 4 angle

            state(22) = rot_mat_times_traj(0); //vel X
            state(23) = rot_mat_times_traj(1); //vel Y
            state(24) = rot_mat_times_traj(2); //vel Z

            state(25) = 0; //roll rate
            state(26) = 0; //pitch rate
            state(27) = traj_(11); //yaw rate

            state(28) = traj_(28); //joint 1 rate
            state(29) = traj_(29); //joint 2 rate
            state(30) = traj_(30); //joint 3 rate
            state(31) = traj_(31); //joint 4 rate


            // Place the computed state into the appropriate segment of xd
            // Adjust indexing if your container xd is one-dimensional but structured like a two-dimensional array
            casadi::DM grf_des_leg = DM::vertcat({0.0, 0.0, 0.5* (params.mass + params.mass_arm) * params.G}); // Assuming the mass is constant
            casadi::DM grf_des = DM::repmat(grf_des_leg, 4, 1); // Repeat the leg GRF for all legs     
            // writwMatrixToFile(grf_des, "tmp/grf_des.txt");   
            xd(Slice(static_cast<int>((n-1)*(n_states)), static_cast<int>(n*(n_states)+n_u_SRB))) = DM::vertcat({state, grf_des});
            
    }
            // appendMatrixToFile(xd.T(), basePath + "planner_xd.txt");

        break;
    }

    case 9:
    {
        double domLen = 200.0;                         // Domain length in milliseconds
        double domLenSec = domLen / 1000.0;            // Domain length in seconds
        DM R_ZYX = RotMat_ZYX(vertcat(qk_(6), qk_(7), qk_(8))); // Assuming q contains Euler angles in the last 3 elements
        // size_t currentStep = std::floor(t / params.dt) + 1;
        /* std::cout << "What is NMPC domain?: " << domain_ << std::endl;
        std::cout << "What is params.stpcnt?: " << params.stepcnt << std::endl;
        std::cout << "What is params.current_time: " << params.current_time << std::endl;
        std::cout << "What is control_tick_: " << control_tick_ * 0.001 << std::endl;
        std::cout << "What is my toePos_: " << toePos_ << std::endl; */

        // TODO-Integration : LL is responsible for handling the domain change and corresponding updates
        // Such as changing the contact sequence, increasing velocity, and updating step foot position
        // if (params.current_time >= (params.stepcnt * domLenSec + 1)) {  // Domain change condition // params.stepcnt previously defined

        // TODO-Integration : Enforce domain change through LL
        // if (params.current_time >= (domain_ * domLenSec + 1)) {
        //Enforce domain change to 4 feet contact immediately
        params.contact = {1, 1, 1, 1}; // All feet in contact
        if (isDomainChanged_)
        {
            // Calculate new-footholds
            params.pf(0, 0) = toePos_(0, 0);
            params.pf(1, 0) = toePos_(1, 0);
            params.pf(2, 0) = toePos_(2, 0);
            params.pf(0, 1) = toePos_(0, 1);
            params.pf(1, 1) = toePos_(1, 1);
            params.pf(2, 1) = toePos_(2, 1);
            params.pf(0, 2) = toePos_(0, 2);
            params.pf(1, 2) = toePos_(1, 2);
            params.pf(2, 2) = toePos_(2, 2);
            params.pf(0, 3) = toePos_(0, 3);
            params.pf(1, 3) = toePos_(1, 3);
            params.pf(2, 3) = toePos_(2, 3);
/* 
            params.p_hip(0,0) = toePos_(0, 0);  params.p_hip(1,0) = toePos_(1, 0); params.p_hip(2,0) = toePos_(2, 0);
            params.p_hip(0,1) = toePos_(0, 1);  params.p_hip(1,1) = toePos_(1, 1); params.p_hip(2,1) = toePos_(2, 1);
            params.p_hip(0,2) = toePos_(0, 2);  params.p_hip(1,2) = toePos_(1, 2); params.p_hip(2,2) = toePos_(2, 2);
            params.p_hip(0,3) = toePos_(0, 3);  params.p_hip(1,3) = toePos_(1, 3); params.p_hip(2,3) = toePos_(2, 3);

            DM hipTmp = params.p_hip + stepLen;
            hipTmp = mtimes(R, hipTmp); // Rotate to the world frame
            params.pf = hipTmp + vertcat(qk_(Slice(0, 2)), 0.0);
 */
            // Set velocity in world frame
            DM cmdWrld = mtimes(R_ZYX, traj_(Slice(3, 6))); // Convert to world frame
            traj_(Slice(3, 6)) = cmdWrld;
            // params.stepcnt++;
            params.stepcnt = domain_;
            x0 = static_cast<double>(qk_(0));
            y0 = static_cast<double>(qk_(1));
        }
        
        float heading = 0;
        float yaw_lim = 0.8;
        float pitch_lim = 0.35;
        float pitch_vel_lim = 0.1;
        float arm_joint_vel_lim = 0.6; // Limit for arm joint velocities
        float yaw_vel_lim = 0.1; // Limit for yaw velocity
        // Increase velocity in body frame
        traj_(3) +=0*((abs(joystick_state[3]-static_cast<double>(traj_(3)))<0.1) ? joystick_state[3]-traj_(3): 0.1*sign(joystick_state[3]-traj_(3))); // Increase command velocity in body X
        //Saturate the velocity to avoid exceeding the joystick input
        traj_(3) = sign(traj_(3))*fmin(abs(traj_(3)), 0.5);
        traj_(4) += 0*((abs(joystick_state[2]-static_cast<double>(traj_(4))) < 0.1 )? joystick_state[2]-traj_(4): 0.1*sign(joystick_state[2]-traj_(4))); // Increase command velocity in body Y
        traj_(4) = sign(traj_(4))*fmin(abs(traj_(4)), 0.5);
        traj_(10) = (abs(joystick_state[1])>=0.4)?0.1*joystick_state[1]:0; // Set command velocity in pitch directly from joystick input
        // traj_(11) = (abs(joystick_state[0])>=0.4)?0.1*joystick_state[0]:0; // Set command velocity in yaw directly from joystick input
        traj_(11) = 0; // Yaw rate command disabled - heading held constant
        /* traj_(10) += ((abs(joystick_state[1]-static_cast<double>(traj_(10))) < 0.1)? joystick_state[1]-traj_(10): 0.05*sign(joystick_state[1]-traj_(10))); // Increase command velocity in pitch
        traj_(11) += ((abs(joystick_state[0]-static_cast<double>(traj_(11))) < 0.1)? joystick_state[0]-traj_(11): 0.05*sign(joystick_state[0]-traj_(11))); // Increase command velocity in yaw */
        /* 
        traj_(28) += ((abs(joystick_state[6]-static_cast<double>(traj_(28))) < 0.05)? joystick_state[6]-traj_(28): 0.05*sign(joystick_state[6]-traj_(28))); // Increase command velocity in arm yaw
        traj_(29) += ((abs(joystick_state[4]-static_cast<double>(traj_(29))) < 0.05)? joystick_state[4]-traj_(29): 0.05*sign(joystick_state[4]-traj_(29))); // Increase command velocity in arm j2
        traj_(30) += ((abs(joystick_state[8]-static_cast<double>(traj_(30))) < 0.05)? joystick_state[8]-traj_(30): 0.05*sign(joystick_state[8]-traj_(30))); // Increase command velocity in arm j3
        traj_(31) += ((abs(joystick_state[10]-static_cast<double>(traj_(31))) < 1)? joystick_state[10]-traj_(31): 1*sign(joystick_state[10]-traj_(31))); // Increase command velocity in arm j4
         */
        traj_(28) = joystick_state[6]; // Set arm yaw velocity directly from joystick input
        traj_(29) = joystick_state[4]; // Set arm joint 2 velocity directly from joystick input 
        traj_(30) = joystick_state[8]; // Set arm joint 3 velocity directly from joystick input
        traj_(31) = joystick_state[10]; // Set arm joint 4 velocity directly from joystick input

        //Saturate the command velocities
        traj_(10) = sign(traj_(10))*fmin(abs(traj_(10)), pitch_vel_lim);
        traj_(28) = sign(traj_(28))*fmin(abs(traj_(28)), arm_joint_vel_lim);
        traj_(29) = sign(traj_(29))*fmin(abs(traj_(29)), arm_joint_vel_lim);
        traj_(30) = sign(traj_(30))*fmin(abs(traj_(30)), arm_joint_vel_lim);
        traj_(31) = sign(traj_(31))*fmin(abs(traj_(31)), 10*arm_joint_vel_lim);
  
        traj_(0) = x0;//qk_(0);//qk_(0) ;//+ params.dt * traj_(3);
        traj_(1) = y0;//qk_(1);//qk_(1) ;//+ params.dt * traj_(4);
        traj_(2) = params.height; // Assuming height is constant or predefined
        traj_(5) = 0;             // Assuming zero vertical velocity
        /* traj_(7) += params.dt * traj_(10);             // Pitch or theta
        traj_(8) += params.dt * traj_(11); //qk_(8) + params.dt * traj_(11); // Update yaw angle */
        // traj_(7) = 0*0.1*joystick_state[1];
        // traj_(8) = 0*0.1*joystick_state[0]; // Keep yaw constant
        traj_(7) += params.dt * traj_(10);             // Pitch or theta
        // traj_(8) = yaw0; // Keep yaw constant
        traj_(8) += params.dt * traj_(11); //qk_(8) + params.dt * traj_(11); // Update yaw angle
    // saturate yaw angle to 1.2 
        traj_(7) = fmin(pitch_lim, fmax(-pitch_lim, traj_(7)));
        // traj_(8) = fmin(yaw_lim, fmax(-yaw_lim, traj_(8)));
        // traj_(8) = yaw0; // Keep yaw constant
        // Update arm shape variables
        traj_(18) += params.dt * traj_(28); //joint 1 angle
        traj_(19) += params.dt * traj_(29); //joint 2 angle
        traj_(20) += params.dt * traj_(30); //joint 3 angle
        traj_(21) += params.dt * traj_(31); //joint 4 angle

        for (int n = 1; n <= (int)params.N; ++n)
        {
            // Compute the predicted state for each step in the horizon
            DM state = DM::zeros(n_states, 1);

            // Index to access the right column in Prd_refined_
            int index = params.mpc_iter + n - 1; // Adjust index for zero-based indexing if necessary
/* 
            // Velocity components from Prd_refined_
            double vx = Prd_refined_(0, index); // Access the velocity in x-direction
            double vy = Prd_refined_(1, index); // Access the velocity in y-direction */

            // Updating the state based on velocities from Prd_refined_
            state(0) = x0;// qk_(0);//qk_(0) ; //n * params.dt * traj_(3); // pos X
            state(1) = y0;//qk_(1);//qk_(1) ;// n * params.dt * traj_(4); // pos Y
            state(2) = params.height;               // pos Z (constant height assumption)
            state(3) = 0; // vx;                          // vel X (updated from Prd_refined_)
            state(4) = 0; //vy;                          // vel Y (updated from Prd_refined_)
            state(5) = 0.0;                         // vel Z (assuming zero vertical velocity)
            // Assuming Euler angles (heading) and angular velocity are zero
            state(6) = 0.0;     // Roll or phi
            /* state(7) = 0.0;     // Pitch or theta
            state(8) = yaw0 + heading; // Yaw or psi
            state(9) = 0.0;     // Angular velocities (assuming zeros)
            state(10) = 0.0;
            state(11) = 0.0; */
            state(7) = traj_(7) + (n-1)*params.dt * traj_(10);     // Pitch or theta
            state(8) = traj_(8) + (n-1)*params.dt * traj_(11) ;//qk_(8) + n * params.dt * traj_(11); // Yaw or psi
            // state(8) = 0;
            state(7) = fmin(pitch_lim, fmax(-pitch_lim, state(7))); //saturation of pitch angle
            //saturation of yaw angle
            // state(8) = fmin(yaw_lim, fmax(-yaw_lim, state(8)));
            state(9) = 0.0;     // Angular velocities (assuming zeros)
            state(10) = 0.0;
            state(11) = traj_(11); // Angular velocity in yaw
            
            DM rot_mat = RotMat_ZYX(vertcat(state(6), state(7), state(8)));
            DM rot_mat_times_dd = DM::mtimes(rot_mat, vertcat(params.dd(0), params.dd(1), params.dd(2)));
            DM rot_mat_times_traj = DM::mtimes(rot_mat, vertcat(traj_(3), traj_(4), 0.0));

            //Arm states
            state(12) = x0 + rot_mat_times_dd(0); // pos X
            state(13) = y0 + rot_mat_times_dd(1); // pos Y
            state(14) = params.height + rot_mat_times_dd(2);                  // pos Z 

            state(15) = 0; //roll or phi
            /* state(16) = 0; //pitch or theta
            state(17) = 0; //yaw or psi

            state(18) = q_des_(18); //joint 1 angle
            state(19) = q_des_(19); //joint 2 angle
            state(20) = q_des_(20); //joint 3 angle
            state(21) = q_des_(21); //joint 4 angle

            state(22) = rot_mat_times_traj(0); //vel X
            state(23) = rot_mat_times_traj(1); //vel Y
            state(24) = rot_mat_times_traj(2); //vel Z

            state(25) = 0; //roll rate
            state(26) = 0; //pitch rate
            state(27) = 0; //yaw rate

            state(28) = q_des_(28); //joint 1 rate
            state(29) = q_des_(29); //joint 2 rate
            state(30) = q_des_(30); //joint 3 rate
            state(31) = q_des_(31); //joint 4 rate
            */         
            state(16) = traj_(7) + (n-1) * params.dt * traj_(10); //pitch or theta
            state(17) = traj_(8) + (n-1) * params.dt * traj_(11); //yaw or psi

            state(18) = traj_(18) + (n-1)*params.dt* traj_(28); //joint 1 angle
            state(19) =  traj_(19) + (n-1)*params.dt* traj_(29); //joint 2 angle
            state(20) = traj_(20) + (n-1)*params.dt* traj_(30); //joint 3 angle
            state(21) = traj_(21) + (n-1)*params.dt* traj_(31); //joint 4 angle

            state(22) = rot_mat_times_traj(0); //vel X
            state(23) = rot_mat_times_traj(1); //vel Y
            state(24) = rot_mat_times_traj(2); //vel Z

            state(25) = 0; //roll rate
            state(26) = 0; //pitch rate
            state(27) = traj_(11); //yaw rate

            state(28) = traj_(28); //joint 1 rate
            state(29) = traj_(29); //joint 2 rate
            state(30) = traj_(30); //joint 3 rate
            state(31) = traj_(31); //joint 4 rate

            // Place the computed state into the appropriate segment of xd
            // Adjust indexing if your container xd is one-dimensional but structured like a two-dimensional array
            // casadi::DM grf_des_leg_FR = DM::vertcat({0.0, 0.0, 0.25* (params.mass + params.mass_arm) * params.G}); // Assuming the mass is constant
            casadi::DM grf_des_leg_FR = DM::vertcat({0.0, 0.0, (2.0/6.0) * (params.mass + params.mass_arm) * params.G}); // Assuming the mass is constant
            casadi::DM grf_des_leg_FL = DM::vertcat({0.0, 0.0, (2.0/6.0) * (params.mass + params.mass_arm) * params.G}); // Assuming the mass is constant
            casadi::DM grf_des_leg_RR = DM::vertcat({0.0, 0.0, (1.0/6.0) * (params.mass + params.mass_arm) * params.G}); // Assuming the mass is constant
            casadi::DM grf_des_leg_RL = DM::vertcat({0.0, 0.0, (1.0/6.0) * (params.mass + params.mass_arm) * params.G}); // Assuming the mass is constant

            // casadi::DM grf_des = DM::repmat(grf_des_leg, 4, 1); // Repeat the leg GRF for all legs     
            casadi::DM grf_des = DM::vertcat({grf_des_leg_FR,grf_des_leg_FL,grf_des_leg_RR,grf_des_leg_RL}); // Repeat the leg GRF for all legs     
            // writwMatrixToFile(grf_des, "tmp/grf_des.txt");   
            xd(Slice(static_cast<int>((n-1)*(n_states)), static_cast<int>(n*(n_states)+n_u_SRB))) = DM::vertcat({state, grf_des});
    }

        break;
    }
    case 10:
    {
        if (sit_down_flag_)
        {
            tsit0 = params.current_time; // Start time for mode sit
            tsitf = tsit0 + sit_duration;                                          // Start and end times for mode 1
            x0 = static_cast<double>(qk_(0));
            y0 = static_cast<double>(qk_(1));
            z0 = static_cast<double>(qk_(2));
            xf = static_cast<double>(qk_(0));
            yf = static_cast<double>(qk_(1));
            zf = 0.04; //Get down to zero.
            yaw0 = static_cast<double>(qk_(8));
            sit_down_flag_ = false; // Reset the flag after setting the initial conditions
    
        }
        
        double s = std::min((params.current_time - tsit0) / (sit_duration), 1.0); // Ensure s is within [0, 1]
        s = std::max(s, 0.0); // Ensure s is non-negative      
        

        /* // Extract relevant parameters for easy access
        double xf = -0.03;           // Initial x position
        double yf = -0.02;           // Initial y position
        double x0 = -0.03;           // Initial x position
        double y0 = -0.02;           // Initial y position
        double zf = params.height; // Assuming the height is constant or predefined
        double z0 = 0.055;       // Initial z position */

         // Calculate new-footholds
        params.pf(0, 0) = toePos_(0, 0);
        params.pf(1, 0) = toePos_(1, 0);
        params.pf(2, 0) = toePos_(2, 0);
        params.pf(0, 1) = toePos_(0, 1);
        params.pf(1, 1) = toePos_(1, 1);
        params.pf(2, 1) = toePos_(2, 1);
        params.pf(0, 2) = toePos_(0, 2);
        params.pf(1, 2) = toePos_(1, 2);
        params.pf(2, 2) = toePos_(2, 2);
        params.pf(0, 3) = toePos_(0, 3);
        params.pf(1, 3) = toePos_(1, 3);
        params.pf(2, 3) = toePos_(2, 3);


        // Using DM::horzcat to create each row of the alpha matrix
        casadi::DM row1 = casadi::DM::horzcat({DM(x0), DM(x0), DM(x0), DM(x0), DM(xf), DM(xf), DM(xf), DM(xf)});
        casadi::DM row2 = casadi::DM::horzcat({DM(y0), DM(y0), DM(y0), DM(y0), DM(yf), DM(yf), DM(yf), DM(yf)});
        casadi::DM row3 = casadi::DM::horzcat({DM(z0), DM(z0), DM(z0), DM(zf / 4), DM(3 * zf / 4), DM(zf), DM(zf), DM(zf)});

        casadi::DM alpha = casadi::DM::vertcat({row1, row2, row3});

        // Update trajectory position
        casadi::DM position_update = bezier(alpha, s);

        for (int i = 0; i < 3; ++i)
        {
            traj_(i) = position_update(i);
            traj_(i + params.n_SRB) = position_update(i) + params.dd(i); // Update arm base position
        }
        traj_(8) = yaw0; // Keep yaw constant

        // Update trajectory velocity
        casadi::DM velocity_update = bezierd(alpha, s);

        for (int i = 0; i < 3; ++i)
        {
            traj_(i + 3) = velocity_update(i)/(sit_duration);
            traj_(i+params.n_SRB+ static_cast<int>(params.n_arm/2)) = velocity_update(i)/(sit_duration); // Update arm base velocity
        }
        break;
    }

    // Handle other cases as necessary
    default:
        std::cerr << "Mode '" << params.gait << "' is not defined in the motion planner.\n";
    }

    if (params.gait != 8 && params.gait != 9)
    {
        casadi::DM grf_des_leg = DM::vertcat({0.0, 0.0, 0.25* (params.mass + params.mass_arm) * params.G}); // Assuming the mass is constant
        casadi::DM grf_des = DM::repmat(grf_des_leg, 4, 1); // Repeat the leg GRF for all legs
        xd = DM::vertcat({repmat(traj_, params.N, 1), grf_des});
    }

    q_des_ = xd; // Only xd needs to be returned
}

void NMPC_A1_KINOVA::force_inequality(const casadi::SX& U, casadi::SX& g_force) {
    // Use SX for symbolic constraint expressions
    casadi::SX g_force_sx = casadi::SX::vertcat(std::vector<casadi::SX>{}); //empty vertcat SX vectors so that appending works in future

    // g_ub and g_lb as symbolic vectors with appropriate bounds
    casadi::DM g_ub_dm = casadi::DM::vertcat(std::vector<casadi::DM>{});
    casadi::DM g_lb_dm = casadi::DM::vertcat(std::vector<casadi::DM>{});


    for (size_t k = 0; k < params.N; ++k) {
        for (size_t leg = 0; leg < 4; ++leg) {
            size_t base_idx = leg * 3;
            // Directly construct and append symbolic expressions
            g_force_sx = casadi::SX::vertcat({g_force_sx,
                U(base_idx, k) - params.mu * U(base_idx + 2, k),
                U(base_idx + 1, k) - params.mu * U(base_idx + 2, k),
                U(base_idx, k) + params.mu * U(base_idx + 2, k),
                U(base_idx + 1, k) + params.mu * U(base_idx + 2, k)
            });
            g_ub_dm = casadi::DM::vertcat({g_ub_dm, 0.0, 0.0, casadi::inf, casadi::inf}); 
            g_lb_dm = casadi::DM::vertcat({g_lb_dm, -casadi::inf, -casadi::inf, 0, 0});
        }

        //Add arm joint torque expression % Add arm joint limits based on the urdf file --> max torque = 60 N.m
        g_force_sx = casadi::SX::vertcat({g_force_sx,
        U(params.n_u_SRB,k),
        U(params.n_u_SRB + 1,k),
        U(params.n_u_SRB + 2,k),
        U(params.n_u_SRB + 3,k)
         });

        g_ub_dm = casadi::DM::vertcat({g_ub_dm, 60, 60, 60, 60});
        g_lb_dm = casadi::DM::vertcat({g_lb_dm, -60, -60, -60, -60});

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

        g_ub_dm = casadi::DM::vertcat({g_ub_dm, casadi::inf, casadi::inf, casadi::inf, casadi::inf, casadi::inf, casadi::inf});
        g_lb_dm = casadi::DM::vertcat({g_lb_dm, -casadi::inf, -casadi::inf, -casadi::inf, -casadi::inf, -casadi::inf, -casadi::inf});      
       

    }

    // Convert symbolic SX expressions to DM for g_ub and g_lb if necessary
    // Note: This might not be needed if keeping them symbolic is preferable
    g_force = g_force_sx; // g_force is already SX and can stay symbolic
    g_ub_ = g_ub_dm;
    g_lb_ = g_lb_dm;
}


void NMPC_A1_KINOVA::force_inequality_bounds()
{

     
    // g_ub and g_lb as symbolic vectors with appropriate bounds
    casadi::DM g_ub_dm = casadi::DM::vertcat(std::vector<casadi::DM>{});
    casadi::DM g_lb_dm = casadi::DM::vertcat(std::vector<casadi::DM>{});


    for (size_t k = 0; k < params.N; ++k) {
        for (size_t leg = 0; leg < 4; ++leg) {
            size_t base_idx = leg * 3;
            // Directly construct and append symbolic expressions
            g_ub_dm = casadi::DM::vertcat({g_ub_dm, 0.0, 0.0, casadi::inf, casadi::inf}); 
            g_lb_dm = casadi::DM::vertcat({g_lb_dm, -casadi::inf, -casadi::inf, 0, 0});
        }

        //Add arm joint torque expression % Add arm joint limits based on the urdf file --> max torque = 60 N.m
        g_ub_dm = casadi::DM::vertcat({g_ub_dm, 60, 60, 60, 60});
        g_lb_dm = casadi::DM::vertcat({g_lb_dm, -60, -60, -60, -60});

        //Add interaction wrench expressions
        // % Add interaction wrench limits -- considered free for now--but we can
    // % limit them in future to avoid enormously big interaction force on the
    // % base to avoid braking the base stand

        g_ub_dm = casadi::DM::vertcat({g_ub_dm, casadi::inf, casadi::inf, casadi::inf, casadi::inf, casadi::inf, casadi::inf});
        g_lb_dm = casadi::DM::vertcat({g_lb_dm, -casadi::inf, -casadi::inf, -casadi::inf, -casadi::inf, -casadi::inf, -casadi::inf});      
       

    }

    // Convert symbolic SX expressions to DM for g_ub and g_lb if necessary
    // Note: This might not be needed if keeping them symbolic is preferable
    g_ub_ = g_ub_dm;
    g_lb_ = g_lb_dm;
}

void NMPC_A1_KINOVA::writeMatrixToFile(const casadi::SX &matrix, const std::string &filename)
{
    std::ofstream file(filename);
    if (file.is_open())
    {
        // Print matrix dimensions
        // file << "Matrix size: " << matrix.size1() << "x" << matrix.size2() << std::endl;
        // Print matrix content preserving original shape
        for (int i = 0; i < matrix.size1(); ++i)
        {
            for (int j = 0; j < matrix.size2(); ++j)
            {
                file << matrix(i, j);
                if (j < matrix.size2() - 1)
                    file << ", ";
            }
            file << std::endl;
        }
        file.close();
    }
    else
    {
        std::cerr << "Unable to open file" << std::endl;
    }
}
void NMPC_A1_KINOVA::appendMatrixToFile(const casadi::DM& matrix, const std::string& filename) {
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
        file << std::endl;
        file.close();
    } else {
        std::cerr << "Unable to open file" << std::endl;
    }
}
void NMPC_A1_KINOVA::writeMatrixToFile(const casadi::DM &matrix, const std::string &filename)
{
    std::ofstream file(filename);
    if (file.is_open())
    {
        // Print matrix dimensions
        // file << "Matrix size: " << matrix.size1() << "x" << matrix.size2() << std::endl;
        // Print matrix content preserving original shape
        for (int i = 0; i < matrix.size1(); ++i)
        {
            for (int j = 0; j < matrix.size2(); ++j)
            {
                file << matrix(i, j);
                if (j < matrix.size2() - 1)
                    file << ", ";
            }
            file << std::endl;
        }
        file.close();
    }
    else
    {
        std::cerr << "Unable to open file" << std::endl;
    }
}

void NMPC_A1_KINOVA::writeMatrixToFile(const std::string &filename)
{
    std::ofstream file(filename, std::ios::out | std::ios::trunc); // Open for output and truncate to clear existing content
    if (file.is_open())
    {
        // Helper lambda to write a single matrix
        auto writeMatrix = [&](const casadi::DM &matrix)
        {
            // file << "Matrix size: " << matrix.size1() << "x" << matrix.size2() << "\n" << std::endl;
            for (int i = 0; i < matrix.size1(); ++i)
            {
                for (int j = 0; j < matrix.size2(); ++j)
                {
                    file << matrix(i, j);
                    if (j < matrix.size2() - 1)
                        file << ", ";
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
    }
    else
    {
        std::cerr << "Unable to open file" << std::endl;
    }
}

void NMPC_A1_KINOVA::writeMatrixToFile(const Eigen::MatrixXd &matrix, const std::string &filename)
{
    std::ofstream file(filename);
    if (file.is_open())
    {
        // Print matrix dimensions
        // file << "Matrix size: " << matrix.size1() << "x" << matrix.size2() << std::endl;
        // Print matrix content preserving original shape
        for (int i = 0; i < matrix.rows(); ++i)
        {
            for (int j = 0; j < matrix.cols(); ++j)
            {
                file << matrix(i, j);
                if (j < matrix.cols() - 1)
                    file << ", ";
            }
            file << std::endl;
        }
        file.close();
    }
    else
    {
        std::cerr << "Unable to open file" << std::endl;
    }
}

void NMPC_A1_KINOVA::equalityconstraints(const SX &X, const SX &U, const SX &P, SX &obj, SX &g_eq, const Function &f_SRB, const Function &f_arm)
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
        SX stage_cost_SRB = SX::mtimes(SX::mtimes(st_err_SRB.T(), Q_SRB), st_err_SRB) + SX::mtimes(SX::mtimes(grf_err.T(), R_SRB), grf_err);
        
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
        SX stage_cost_arm = SX::mtimes(SX::mtimes(st_err_arm.T(), Q_arm), st_err_arm) + SX::mtimes(SX::mtimes(con_arm.T(), R_arm), con_arm);

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
        arg_arm["q"] = st_arm(Slice(0, static_cast<int>(n_arm/2)));
        arg_arm["dq"] = st_arm(Slice(static_cast<int>(n_arm/2), static_cast<int>(n_arm)));
        arg_arm["con"] = con_arm;
        arg_arm["int"] = con_lambda;

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
        SX R = RotMat_ZYX(theta);
        SX A = A_Theta_ZYX(theta);

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
        SX HolonomicAngularAcceleration = ddtheta - ddtheta_ab;

        // Add holonomic constraints
        // g_eq = SX::vertcat({g_eq, HolonomicPosition, HolonomicOrientation, HolonomicVelocity, HolonomicAngularVelocity, HolonomicAcceleration, HolonomicAngularAcceleration});
        g_eq = SX::vertcat({g_eq, HolonomicAcceleration+ params.Kp * HolonomicPosition + params.Kd * HolonomicVelocity, HolonomicAngularAcceleration + params.Kp * HolonomicOrientation + params.Kd * HolonomicAngularVelocity});


        // // writeMatrixToFile(st_next, basePath + "st_next.txt");
        // // writeMatrixToFile(f_value, basePath + "f_value.txt");

        // // Compute constraints
        // SX st_next_euler = st + dt * f_value;
        // g_eq = SX::vertcat({g_eq, st_next - st_next_euler});

        // // writeMatrixToFile(st_next_euler, basePath + "st_next_euler.txt");
        // writeMatrixToFile(g_eq, basePath + "g_eq.txt");
     
    }
}


void NMPC_A1_KINOVA::set_constraint_bounds()
{
    // Calculate the start index for setting lower and upper bounds

    int startIndex = params.n_states * (params.N + 1) + params.NumHolCon * params.N ; // Adjust for zero-based indexing
    // int startIndex = params.n_states * (params.N + 1);

    // Resize the lbg and ubg if necessary
    if (args.lbg.is_empty() || args.lbg.size1() < startIndex + g_lb_.size1())
    {
        args.lbg = DM::zeros(startIndex + g_lb_.size1(), 1); // Adjust total size if needed
    }
    if (args.ubg.is_empty() || args.ubg.size1() < startIndex + g_ub_.size1())
    {
        args.ubg = DM::zeros(startIndex + g_ub_.size1(), 1); // Adjust total size if needed
    }

    /* for (size_t i = 0; i < params.N; ++i)  
    {
        args.lbg(Slice(static_cast<int>(params.n_states + (i+1)*params.n_states),static_cast<int>(params.n_states + (i+1)*params.n_states+params.NumHolCon))) = 0.1*DM::vertcat({-0.05,-0.05,-0.05,-0.1,-0.1,-0.1});
        args.ubg(Slice(static_cast<int>(params.n_states + (i+1)*params.n_states),static_cast<int>(params.n_states + (i+1)*params.n_states+params.NumHolCon))) = 0.1*DM::vertcat({0.05,0.05,0.05,0.1,0.1,0.1});           
    } */



    // Set values for lower bounds
    for (int i = 0; i < g_lb_.size1(); ++i)
    {
        args.lbg(startIndex + i) = g_lb_(i);
    }

    // Set values for upper bounds
    for (int i = 0; i < g_ub_.size1(); ++i)
    {
        args.ubg(startIndex + i) = g_ub_(i);
    }
}



void NMPC_A1_KINOVA::set_statebounds() 
{
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
    size_t mpc_iter = params.mpc_iter;
    


    // Initialize LB and UB
    args.lbx = DM::zeros(n_states*(N+1) + n_inputs*N, 1);
    args.ubx = DM::zeros(n_states*(N+1) + n_inputs*N, 1); //%state x lower bound % new - adapt the bound
    args.lbx(Slice()) = -DM::inf();
    args.ubx(Slice()) = DM::inf();

    //%% Introducing lower and upper bounds of arm joints q_s shape variables
    // %%%% Arm joints q_s = [q1 q2 q3 q4] in (rad) from the URDF file
    DM q_s_LB = DM::vertcat({-6.28318530718, 0.820304748437, 0.331612557879, -6.28318530718});
    DM q_s_UB = DM::vertcat({6.28318530718, 5.46288055874, 5.9515727493, 6.28318530718});
    //%%%% Arm joints rates dq_s = [dq1 dq2 dq3 dq4] in (rad/sec) from the URDF file
    DM dq_s_LB = -1;
    DM dq_s_UB = 1;

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
    args.lbx(Slice(n_SRB + 7+4+5+1-1, n_SRB + n_arm)) = DM::vertcat({dq_s_LB, dq_s_LB, dq_s_LB, dq_s_LB});
    args.ubx(Slice(n_SRB + 7+4+5+1-1, n_SRB + n_arm)) = DM::vertcat({dq_s_UB, dq_s_UB, dq_s_UB, dq_s_UB});

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
        args.lbx(Slice(n_states*(k+1) + n_SRB + 7+4+5+1-1, n_states*(k+1) + n_SRB + n_arm)) = DM::vertcat({dq_s_LB, dq_s_LB, dq_s_LB, dq_s_LB});
        args.ubx(Slice(n_states*(k+1) + n_SRB + 7+4+5+1-1, n_states*(k+1) + n_SRB + n_arm)) = DM::vertcat({dq_s_UB, dq_s_UB, dq_s_UB, dq_s_UB});

        // %% This part will assign lower and upper bound for the vertical component of the GRFs: "f_z"
        // %%% SRB
        // int contact_index = (mpc_iter + k) % 40; // Adjusting MATLAB 1-based to C++ 0-based index
        for (int leg = 0; leg < 4; ++leg) {
            int index = n_states*(N+1) + n_inputs*k + 3*leg + 3-1; // Adjusted for 0-based indexing
            args.lbx(index) = 0;
            args.ubx(index) = params.contact(leg) * zmax; // Assuming contact is std::vector<double>
        }
    }
}


void NMPC_A1_KINOVA::update_statebounds()
{
    int n_states = params.n_states;  // Number of states
    int n_inputs = params.n_inputs;  // Number of inputs
    int N = params.N;
    double zmax = params.zmax;

    for (int k = 0; k < N; ++k) {

        for (int leg = 0; leg < 4; ++leg) {
            int index = n_states*(N+1) + n_inputs*k + 3*leg + 3-1; // Adjusted for 0-based indexing
            args.lbx(index) = 0;
            args.ubx(index) = params.contact(leg) * zmax; // Assuming contact is std::vector<double>
        }
    }
}


void NMPC_A1_KINOVA::shift()
{

    //Shift inputs
    DM prev_ukAll = ukAll_;

    // Shift control inputs
    DM u_new = vertcat(prev_ukAll(Slice(1, prev_ukAll.size1()), Slice()),                       // Skip the first control set
                       prev_ukAll(Slice(prev_ukAll.size1() - 1, prev_ukAll.size1()), Slice())); // Duplicate the last control set
    ukAll_ = u_new;
}

void NMPC_A1_KINOVA::code_gen()
{
    // Note: This function is not used in the current implementation
    // It is a placeholder for generating code for the NMPC problem
    //Go to SRB_NMPC_Casadi_Formulation to generate the code

    const double dt = params.dt;               // Sample time
    const double sim_time = params.sim_length; // Simulation time
    const float sim_length = sim_time / dt;    // Simulation length
    const int n_states = params.n_states;      // Number of states
    const int n_inputs = params.n_inputs;      // Number of inputs
    const int n_controls = params.n_controls;  // Number of inputs
    const int n_SRB = params.n_SRB;            // Number of SRB states
    const int n_arm = params.n_arm;            // Number of arm states
    const int gait = params.gait;              // Gait type
    const double G = params.G;                 // Gravity
    const double mass = params.mass;           // Mass
    DM dd = params.dd;                         // Position vector of the arm's base w.r.t the A1 body frame
    DM arm_pos = params.arm_pos;               // Position vector of the arm's base w.r.t the A1 body frame

    init(); // Initialize the variables


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

    SX mass_B = 0.46784;
    SX comvector_B = SX::horzcat({0.0, 0.0, 0.1255});
    SX inertia_B = SX::diag(SX::vertcat({0.000951270861568, 0.000951270861568, 0.000374272}));

    SX mass_L1 = 0.7477;
    SX comvector_L1 = SX::horzcat({0, -0.002, -0.0605});
    SX inertia_L1 = SX::diag(SX::vertcat({0.00152031725204, 0.00152031725204, 0.00059816}));

    SX mass_L2 = 0.99;
    SX comvector_L2 = SX::horzcat({0 ,-0.2065, -0.01});
    SX inertia_L2 = SX::diag(SX::vertcat({0.010502207991, 0.000792, 0.010502207991}));

    SX mass_L3 = 0.6763;
    SX comvector_L3 = SX::horzcat({0, 0.081, -0.0086});
    SX inertia_L3 = SX::diag(SX::vertcat({0.00142022431908, 0.000304335, 0.00142022431908}));

    SX mass_L4 = 0.99;
    SX comvector_L4 = SX::horzcat({0 ,0 ,-0.06});
    SX inertia_L4 = SX::diag(SX::vertcat({0.0003453236187, 0.0003453236187, 0.0005816}));

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

    // %% COM Jacobian Jv   (diff --> jacobian  in CaSaDi)
    SX J_v_comB = SX::horzcat({SX::jacobian(I_comBpos, x_b), SX::jacobian(I_comBpos, y_b), SX::jacobian(I_comBpos, z_b),
                               SX::jacobian(I_comBpos, phi_b), SX::jacobian(I_comBpos, theta_b), SX::jacobian(I_comBpos, psi_b),
                               SX::jacobian(I_comBpos, q1), SX::jacobian(I_comBpos, q2), SX::jacobian(I_comBpos, q3), SX::jacobian(I_comBpos, q4),
                               SX::jacobian(I_comBpos, q10), SX::jacobian(I_comBpos, q11),
                               SX::jacobian(I_comBpos, q12), SX::jacobian(I_comBpos, q13),
                               SX::jacobian(I_comBpos, q14), SX::jacobian(I_comBpos, q15)});

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
    SX w_bB = SX::solve(A_Theta_ZYX(SX::vertcat({phi_b, theta_b, psi_b})), SX::vertcat({dphi_b, dtheta_b, dpsi_b}));
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

    // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Nonlinearity vector%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//

    // %%%% gravity vector  
    SX G_sx = 9.81;
    SX long_Gravity = - SX::mtimes(J_v_comB.T(),SX::vertcat({0,0,-mass_B*G_sx}))
                        - SX::mtimes(J_v_comL1.T(),SX::vertcat({0,0,-mass_L1*G_sx}))     
                        - SX::mtimes(J_v_comL2.T(),SX::vertcat({0,0,-mass_L2*G_sx}))
                        - SX::mtimes(J_v_comL3.T(),SX::vertcat({0,0,-mass_L3*G_sx}))
                        - SX::mtimes(J_v_comL4.T(),SX::vertcat({0,0,-mass_L4*G_sx}));

    Gravity_arm = long_Gravity(Slice(0,10),0); // % 10x1 gravity vector      % For now, disregard fingers' dynamics
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
    B_arm  = SX::vertcat({SX::zeros(6,4),B_bar}); // % gear ratio matrix

    J_arm = SX::vertcat({SX::horzcat({SX::eye(3),SX::zeros(3,3),SX::zeros(3,4)}),SX::horzcat({SX::zeros(3,3),SX::eye(3),SX::zeros(3,4)})});
 
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
    // const int n_states = states.size1();

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
    // size_t n_controls = controls.size1();



    SX U = SX::sym("U", n_controls, params.N); //% Decision variables (controls) for all horizons
    SX P1 = SX::sym("P1", n_states + n_states * params.N); //% a parameter to store the current state and the desired future (for all horizons)
    SX P = SX::vertcat({P1, reshape(footholds,12,1)}); //% a parameter to store the current state and the desired future (for all horizons)
// % This is a multipl-shooting in which we strore both states and inputs
    SX X = SX::sym("X", n_states, params.N + 1); //% A Matrix that represents the states over the optimization problem.
// % N+1: Because we store the current states in the 1st column
    SX OPT_variables = vertcat(reshape(X, n_states * (params.N + 1), 1), reshape(U, params.n_inputs * params.N, 1)); //% make the decision variable one column  vector ((32*11)+(22*10))


     // %%--SRB ICs--%%

    DM qk = DM::zeros(n_states);
    qk(2) = 0.08; // Assuming SRB z position is constant or predefined

    // %%%%--arm ICs--%%%%
    // % The initial condition for the arm's base should match those of the
    // % holonomically fixed to the A1
    qk(Slice(12,15)) = qk(Slice(0,3)) + DM::mtimes(RotMat_ZYX(qk(Slice(6,9))),SX(params.dd));

    // % Arm's base orientation
    qk(Slice(15,18)) = qk(Slice(6,9));

    //     % Shape variable ICs: for now, consider the arm with fixed joint angles
    // % x0(19:22) = [ZZero; 2; 5;ZZero];  % q_s = [q1 q2 q3 q4] arm's joints (rad)
    // qk(Slice(18,22)) = DM::vertcat({DM(arm_pos(0)),DM(arm_pos(1)),DM(arm_pos(2)),DM(arm_pos(3))});//% q_s = [q1 q2 q3 q4] arm's joints (rad)
    qk(Slice(18,22)) = arm_pos;//% q_s = [q1 q2 q3 q4] arm's joints (rad)
    // % Arm's base velocity
    qk(Slice(22,25)) = qk(Slice(3,6)) + DM::mtimes(skewSym(qk(Slice(9,12))),SX::mtimes(RotMat_ZYX(qk(Slice(6,9))),SX(params.dd)));

    //% Arm's base orientation rate
    qk(Slice(25,28)) = DM::mtimes(A_Theta_ZYX(qk(Slice(6,9))),qk(Slice(9,12)));

//     % x0(26:28) = [ZZero; ZZero; ZZero];
// x0(29:32) = [ZZero;ZZero;ZZero;ZZero];  % dq_s = [dq1 dq2 dq3 dq4] arm's joints rates (rad/s)


    double t0 = 0.0;
    DM xx(n_states, sim_length);
    xx(Slice(0, static_cast<int>(n_states)), 0) = qk;  

    DM u0 = DM::zeros(params.N, params.n_inputs); // Control inputs for N horizon steps, not sim_steps

    // States decision variables initialization
    DM X0 = repmat(qk, 1, params.N + 1);
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


    DM uout = DM::zeros(n_controls, sim_length); // Vector to store control outputs
    DM xout = DM::zeros(n_states, sim_length);
    DM feet = DM::zeros(12, sim_length); // Vector to store foot positions
    DM traj = DM::zeros(n_states, sim_length); // Vector to store desired trajectory
    DM tout = DM::zeros(1, sim_length);
    DM NMPC_solve_time = DM::zeros(1, sim_length);  // Vector to store NMPC solve time
    // DM xsolout = DM::zeros(n_states * (params.N + 1), sim_length); // Vector to store optimal states

    // params.current_time = current_time;
    params.current_time = control_tick_ * 0.001;
    params.mpc_iter = mpciter;
    planner(); // Assuming gait and other details are handled inside
    args.p = vertcat(qk, xd);

        //=======================================SRB Dynamics=======================================//
    //  define the nonlinear right-hand-side "f"  for A1 (SRB dynamics)
    
    SX t_rot = casadi::SX::vertcat({states(6), states(7), states(8)});  // Extract phi, theta, and psi into a column vector for Rotz
    SX rot_mat = RotMat_ZYX(t_rot);    // Call RotMat with the column vector
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
    A_SRB(Slice(6, 9), Slice(9, 12)) = A_Theta_ZYX(states(Slice(6,9)));

    A_SRB(Slice(9, 12), Slice(0, 3)) = z3;
    A_SRB(Slice(9, 12), Slice(3, 6)) = z3;
    A_SRB(Slice(9, 12), Slice(6, 9)) = z3;
    A_SRB(Slice(9, 12), Slice(9, 12)) = -IwI; //-IwI

    SX A_SRB_reduced = A_SRB;
    A_SRB_reduced(Slice(9, 12), Slice(9, 12)) = z3;

    // writeMatrixToFile(A, basePath + "A.txt");
    // SX p_foot_sx = SX(params.pf);
    SX p_foot_sx = reshape(footholds.T(), 3, 4); // Reshape the footholds into a 3x4 matrix

    SX com_vector = SX::vertcat({states(0), states(1), states(2)});
    SX com_vector_expanded = repmat(com_vector, 1, 4); // Replicate across columns to match p_foot_sx dimensions

    SX rd = p_foot_sx - com_vector_expanded;
    
    SX skew_rd = skewSym(rd);
    // writeMatrixToFile(skew_rd, basePath + "skew_rd.txt");
    // std::cout << "Size of skew_rd: " << skew_rd.size1() << " x " << skew_rd.size2() << std::endl;
    SX skew_dd = skewSym(SX(params.dd));
    SX B_tmp_1 = SX::mtimes(SX::mtimes(rot_mat, solve(J_sx,rot_mat.T())), skew_rd);
    SX B_tmp_2 = SX::mtimes(SX::mtimes(rot_mat, solve(J_sx,rot_mat.T())), skew_dd);
    SX B_tmp_3 = SX::mtimes(rot_mat, solve(J_sx,rot_mat.T()));
    // writeMatrixToFile(B_tmp_1, basePath + "B_tmp.txt");
    // writeMatrixToFile(skew_rd, basePath + "skew_rd.txt");
    // writeMatrixToFile(rd, basePath + "rd.txt");

    SX B_SRB = SX::vertcat({
        SX::horzcat({z3,z3,z3,z3,z3,z3}),
        SX::horzcat({SX::repmat(1/mass * i3, 1, 5),z3}),
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

    /* writeMatrixToFile(rhs_arm, basePath + "rhs_arm.txt");
    std::map<std::string, SX> arg_arm;
    arg_arm["q"] = SX::vertcat({1,2,3,4,5,6,7,8,9,10});
    arg_arm["dq"] = SX::vertcat({11,12,13,14,15,16,17,18,19,20});
    arg_arm["con"] = SX::vertcat({21,22,23,24});
    arg_arm["int"] = SX::vertcat({25,26,27,28,29,30});
    std::map<std::string, SX> result_arm = f_arm(arg_arm);
    SX f_value_arm = result_arm.at("rhs");
    writeMatrixToFile(f_value_arm, basePath + "f_value_arm.txt"); */

    // Declare variables to hold the outputs
    SX g_force;

    // Call the function
    force_inequality(U, g_force);

    SX obj = 0, g_eq;
    equalityconstraints(X, U, P, obj, g_eq, f_SRB_reduced, f_arm_reduced);
    SX g = vertcat(g_eq, g_force);
    set_statebounds();
    set_constraint_bounds();
    SXDict nlp_prob = {{"f", obj}, {"x", OPT_variables}, {"g", g}, {"p", P}};

    // Define options for the solver
    Dict opts;

    // For ipopt
    opts["ipopt.max_iter"] = params.solver_iter;                    // Replace Max_mpciter with its actual value
    opts["ipopt.print_level"] = 3;                  // Can be changed to 0 or 3 based on verbosity required
    opts["print_time"] = 0;                         // Disable printing solver time
    opts["ipopt.linear_solver"]= "ma57";             // Use ma57 linear solver
    opts["ipopt.ma57_automatic_scaling"] = "no";    // Disable automatic scaling
    opts["ipopt.acceptable_tol"] = 1e-2;            // Tolerance for stopping criterion
    opts["ipopt.acceptable_obj_change_tol"] = 1e-2; // Objective change tolerance for stopping
   
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

  /*   // Reshape X0 and u0 transposed into column vectors
    DM X0_col = reshape(X0.T(), n_states * (params.N + 1), 1);
    DM u0_col = reshape(u0.T(), n_inputs * params.N, 1);

    // Concatenate reshaped vectors vertically
    DM x0u0 = vertcat(X0_col, u0_col);
    args.x0 = x0u0;
    args.p = vertcat(qk, xd);
    // Solve the problem
    DMDict arg = {{"x0", args.x0}, {"lbx", args.lbx}, {"ubx", args.ubx}, {"lbg", args.lbg}, {"ubg", args.ubg}, {"p", args.p}};
    DMDict res = solver(arg);

    DM sol = DM::zeros(n_states * (params.N + 1) + n_controls * params.N, 1); // sol.x should be initialized with actual solution data
    sol = res.at("x"); */

    // =========================== CODE GEN =========================== //

// // file name
    
    std::string file_name = "a1_kinova_nlp_code";
    // code predix
    std::string prefix_code = fs::current_path().string() + "/";

    casadi::Dict opts_compilation = casadi::Dict();
    opts_compilation["cpp"] = false;
    opts_compilation["with_header"] = false;


    solver.generate_dependencies(file_name + ".c");
    // f.generate_dependencies(file_name + "_f" + ".c");

    CodeGenerator codegen = CodeGenerator(file_name + "_f" + ".c", opts_compilation);
    codegen.add(f_SRB);
    codegen.add(f_arm);
    codegen.add(f_SRB_reduced);
    codegen.add(f_arm_reduced);
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
    std::cout << "Compilation successed!" << std::endl;

    int compile_flag_solver = std::system(compile_command_solver.c_str());
    casadi_assert(compile_flag_solver==0, "Compilation failed");
    std::cout << "Compilation successed!" << std::endl;

}

void NMPC_A1_KINOVA::init()
{
    DM arm_pos = {0.0,3.09066,5.67232,0.0};//{0.0,3.49066,5.67232,0.0};//{0.0,3.14,4.71,0.0};//{0.0,4.2411,5.81195,0.0};//{0.0, 2.68, 1.051, 0.0};//{0.0,3.75066,5.67232,0.0};// {0.0, 1.13, 2.57, 0.0}; // Position vector of the arm's base w.r.t the A1 body frame

    //Wrapping the arm_pos to be in -pi to pi as our formulation has it this way
    /* for (size_t i = 0; i < 4; ++i)
    {
        arm_pos(i) = (static_cast<double>(arm_pos(i)) >= M_PI)? static_cast<double>(arm_pos(i)) - 2*M_PI : (static_cast<double>(arm_pos(i)) <= -M_PI)?  static_cast<double>(arm_pos(i)) + 2*M_PI :static_cast<double>(arm_pos(i)) ;
    } */
    D_q_outside_ = {
        3.87184,    0.0,        0.0,        0.0,       1.60385,    0.0044475,  -0.0044475,  -0.650884,   -0.183204,   0.0,
        0.0,        3.87184,    0.0,       -1.60385,   0.0,       -0.0419411,  0.0419411,  -7.71421e-17,-3.8457e-17,  0.0,
        0.0,        0.0,        3.87184,   -0.0044475, 0.0419411,  0.0,        -5.44662e-19,-0.0419411,  -0.261643,   0.0,
        0.0,       -1.60385,   -0.0044475,  0.784366,  0.00122519, 0.0275056,  -0.0275056,  -0.00122519,  0.00243846, -0.000476419,
        1.60385,    0.0,        0.0419411,  0.00122519, 0.800594,   0.00276418, -0.00276418, -0.374532,   -0.0808089, -8.40116e-20,
        0.0044475, -0.0419411,  0.0,        0.0275056,  0.00276418, 0.019407,   -0.0190327,  -0.001626,   -0.00170742,  0.000333591,
       -0.0044475,  0.0419411, -5.44662e-19,-0.0275056, -0.00276418,-0.0190327,   0.0190327,   0.001626,    0.00170742, -0.000333591,
       -0.650884,  -7.71421e-17,-0.0419411, -0.00122519,-0.374532,  -0.001626,   0.001626,    0.195213,    0.0303361,  5.24052e-21,
       -0.183204,  -3.8457e-17, -0.261643,   0.00243846,-0.0808089, -0.00170742,  0.00170742,   0.0303361,   0.0769375,  3.56127e-20,
        0.0,        0.0,        0.0,       -0.000476419,-8.40116e-20, 0.000333591,-0.000333591,  5.24052e-21,  3.56127e-20, 0.0005816
    };
    




    qk_ = DM::zeros(params.n_states);
    qk_(2) = 0.03; // sitting com height
    // %%%%--arm ICs--%%%%
    // % The initial condition for the arm's base should match those of the
    // % holonomically fixed to the A1
    qk_(Slice(12,15)) = qk_(Slice(0,3)) + DM::mtimes(RotMat_ZYX(qk_(Slice(6,9))),SX(params.dd));

    // % Arm's base orientation
    qk_(Slice(15,18)) = qk_(Slice(6,9));

    //     % Shape variable ICs: for now, consider the arm with fixed joint angles
    // % x0(19:22) = [ZZero; 2; 5;ZZero];  % q_s = [q1 q2 q3 q4] arm's joints (rad)
    // qk(Slice(18,22)) = DM::vertcat({DM(arm_pos(0)),DM(arm_pos(1)),DM(arm_pos(2)),DM(arm_pos(3))});//% q_s = [q1 q2 q3 q4] arm's joints (rad)
    qk_(Slice(18,22)) = arm_pos;//% q_s = [q1 q2 q3 q4] arm's joints (rad)
    // % Arm's base velocity
    qk_(Slice(22,25)) = qk_(Slice(3,6)) + DM::mtimes(skewSym(qk_(Slice(9,12))),SX::mtimes(RotMat_ZYX(qk_(Slice(6,9))),SX(params.dd)));

    //% Arm's base orientation rate
    qk_(Slice(25,28)) = DM::mtimes(A_Theta_ZYX(qk_(Slice(6,9))),qk_(Slice(9,12)));

    q_des_ = DM::zeros(params.n_states, 1);
    ukAll_ = DM::zeros(params.N, params.n_inputs);

    qkAll_ = repmat(qk_, 1, params.N + 1); // contains initial state, and solution for all N horizon steps
    q_des_ = repmat(qk_, params.N, 1);     // contains desired state for all N horizon steps
    traj_ = q_des_(casadi::Slice(0, static_cast<int>(params.n_states)), 0); // Second parameter 0 is to indicate column index for a vector

    args.lbg = DM::zeros(params.n_states * (params.N + 1) + params.NumHolCon * params.N + 4*4*params.N + (4 + 6) * params.N, 1);
    args.ubg = DM::zeros(params.n_states * (params.N + 1) + params.NumHolCon * params.N + 4*4*params.N + (4 + 6) * params.N, 1);

    uout = DM::zeros(params.n_controls, params.sim_length * 1000);
    xout = DM::zeros(params.n_states, params.sim_length * 1000);
    feet = DM::zeros(12, params.sim_length * 1000); // Vector to store foot positions
    traj = DM::zeros(params.n_states, params.sim_length * 1000);
    tout = DM::zeros(1, params.sim_length * 1000);
    NMPC_solve_time = DM::zeros(1, params.sim_length * 1000);
    // ukArm_ = {0.0354537, 1.52263, -1.46063, 0};
    ukArm_ = {0};

    std::string file_name = "a1_kinova_nlp_code";

    std::string prefix_lib = fs::current_path().string() + "/"; // run from build/, where runCodegenA1K.sh copies the solver

    // std::cout << "prefix_lib: " << prefix_lib << std::endl; std::cin.get();

    std::string lib_name = prefix_lib + file_name + ".so";

    // std::cout << "lib_name: " << lib_name << std::endl; std::cin.get();
    // Load the nlp solver
    // Define options for the solver
    Dict opts;

    /* if (params.current_time < 1)
    {
        iter_lim = 13;
    }
    else
        iter_lim = 8; // 12 also works */

    opts["ipopt.max_iter"] = params.solver_iter;       // Replace Max_mpciter with its actual value
    opts["ipopt.print_level"] = 0;                  // Can be changed to 0 or 3 based on verbosity required
    // opts["ipopt.mu_strategy"] = "adaptive";
    opts["print_time"] = 0;                         // Disable printing solver time
    opts["ipopt.acceptable_tol"] = 1e-2;            // Tolerance for stopping criterion
    opts["ipopt.acceptable_obj_change_tol"] = 1e-2; // Objective change tolerance for stopping
    opts["ipopt.linear_solver"] = "ma57";            // Use ma57 linear solver
    // opts["ipopt.warm_start_init_point"] = "yes";
    // opts["ipopt.warm_start_bound_push"] = 1e-9;
    // opts["ipopt.warm_start_mult_bound_push"] = 1e-8;
    // opts["ipopt.mu_init"] = 1e-4;
    // opts["ipopt.hessian_approximation"]="limited-memory";
    // opts["ipopt.bound_relax_factor"] = 1e-2;

    // opts["ipopt.acceptable_constr_viol_tol"] = 1e0;
    solver_ = casadi::nlpsol("solver", "ipopt", lib_name, opts);

    // Load nonlinear dynamics function
    lib_name = prefix_lib + file_name + "_f"
                                        ".so";
    f_SRB_ = external("f_SRB", lib_name);
    f_arm_ = external("f_arm", lib_name);
    f_SRB_reduced_ = external("f_SRB_reduced", lib_name);
    f_arm_reduced_ = external("f_arm_reduced", lib_name);
    f_arm_outside_ = external("f_arm_outside", lib_name);
    D_q_arm_sym_ = external("D_q_arm_sym", lib_name);
    Jacobian_holonomic_ = external("Jacobian_holonomic", lib_name);

    //Logging
    std::string filename = prefix_lib + "./tmp/A1_KINOVA_NMPC_x_log.txt";
    if(!filename.empty()){
        fid = std::fstream(filename, std::ios::out);
    }
    success = (fid.is_open()) ? 1 : 0;

    //Set bounds
    force_inequality_bounds();
    set_statebounds();
    set_constraint_bounds();

    
}

void NMPC_A1_KINOVA::run_NMPC()
{

    // call the planner function to update q_des_ for N horizon steps
    planner();
    update_statebounds(); // Update state bounds based on current state
    // writeMatrixToFile(q_des_, basePath + "q_des_.txt");
    // writeMatrixToFile(basePath + "params.txt");
    /* force_inequality_bounds(); // Updates g_ub_ and g_lb_ with appropriate bounds
    set_statebounds();         // Updates args.lbx and args.ubx with appropriate bounds
    set_constraint_bounds();   // Updates args.lbg and args.ubg with g_lb_ and g_ub_ */
    // writeMatrixToFile(g_ub, basePath + "g_ub.txt");
    // writeMatrixToFile(g_lb, basePath + "g_lb.txt");
    // writeMatrixToFile(params, basePath + "params.txt");

    solve_nlp(); // Solve the NMPC problem
}

void NMPC_A1_KINOVA::solve_nlp()
{

    DM q0_col = reshape(qkAll_.T(), params.n_states * (params.N + 1), 1);//DM::zeros(params.n_states * (params.N + 1), 1); // Initial states for N horizon steps
    DM u0_col =  reshape(ukAll_.T(), params.n_inputs * params.N, 1); //DM::zeros(params.n_inputs * params.N, 1); // Control inputs for N horizon steps, not sim_steps

    // writeMatrixToFile(q0_col, basePath + "q0_col.txt");
    // writeMatrixToFile(u0_col, basePath + "u0_col.txt");
    //Get D_q_arm from the function
    
    std::map<std::string, DM> arg_D_q_arm;
    DM arg_D_arm_input_q = qk_(Slice(12, 22));
    DM arg_D_arm_input_dq = qk_(Slice(22, 32));
    // arg_D_arm_input_q(4) = 0;
    // arg_D_arm_input_dq(4) = 0;
    // arg_D_arm_input_q(5) = 0;
    // arg_D_arm_input_dq(5) = 0;
    arg_D_q_arm["q"] = arg_D_arm_input_q;
    arg_D_q_arm["dq"] = arg_D_arm_input_dq;

    std::map<std::string, DM> result_D_q_arm= D_q_arm_sym_(arg_D_q_arm);
    D_q_outside_ = reshape(result_D_q_arm.at("D_q"),100,1);
    // writeMatrixToFile(D_q_outside_, basePath + "D_q_outside.txt");
   /*  
    DM D_q_outside_debug = result_D_q_arm.at("D_q");
    casadi_int n = D_q_outside_debug.size1();
    casadi_int m = D_q_outside_debug.size2();

    Eigen::MatrixXd D_q_outside_eigen(n, m);
    for (casadi_int i = 0; i < n; ++i) {
        for (casadi_int j = 0; j < m; ++j) {
            D_q_outside_eigen(i,j) = static_cast<double>(D_q_outside_debug(i,j));
        }
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(D_q_outside_eigen,
    Eigen::ComputeThinU | Eigen::ComputeThinV);

    const auto& s = svd.singularValues();  // sorted descending
    double cond_number = s(0) / s(s.size() - 1);
    std::cout << "Singular values of D_q_arm: " << s.transpose() << std::endl; */
    // std::cout << "Condition number of D_q_arm: " << cond_number << std::endl;
    // After getting D_q_arm from Pinocchio:
    // DM D_q_debug = reshape(D_q_outside_, 10,10);
    // DM reg_diag = DM::vertcat({
    //     0.01, 0.01, 0.01,           // translation (small reg)
    //     0.1, 0.1, 0.1,              // rotation (larger reg for numerical stability)
    //     0.01, 0.01, 0.01, 0.01      // joints
    // });
    // DM D_q_arm_reg = D_q_debug + DM::diag(reg_diag);
    // D_q_outside_ = reshape(D_q_arm_reg,100,1);
    
    std::map<std::string, DM> arg_Jacobian_holonomic;
    arg_Jacobian_holonomic["st_SRB"] = qk_(Slice(0, 12));
    arg_Jacobian_holonomic["st_arm"] = qk_(Slice(12, 32));
    arg_Jacobian_holonomic["con_SRB"] = ukAll_(Slice(0, 12));
    arg_Jacobian_holonomic["con_arm"] = ukAll_(Slice(12, 16));
    arg_Jacobian_holonomic["lambda_int"] = ukAll_(Slice(16, 22));
    arg_Jacobian_holonomic["footholds"] = reshape(params.pf, 12, 1);
    arg_Jacobian_holonomic["D_q"] = reshape(D_q_outside_, 10, 10);

   /*  std::map<std::string, DM> result_Jacobian_holonomic = Jacobian_holonomic_(arg_Jacobian_holonomic);
    DM J_dm = result_Jacobian_holonomic.at("Jacobian_holo");

        // Map to Eigen
    Eigen::MatrixXd J = Eigen::Map<Eigen::MatrixXd>(
        J_dm.nonzeros().data(),
        J_dm.size1(),
        J_dm.size2()
    );

    // SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svdJ(J);
    auto singJ = svdJ.singularValues();
    double condJ = singJ(0) / singJ(singJ.size()-1);
    std::cout << "J_holo singular values: " << singJ.transpose() << std::endl;
    std::cout << "cond(J_holo): " << condJ << std::endl; */


    // Concatenate reshaped vectors vertically
    DM x0u0 = DM::vertcat({q0_col, u0_col});
    args.x0 = x0u0;
    args.p = DM::vertcat({qk_, q_des_, reshape(params.pf, 12, 1), params.Q_SRB, params.Q_arm, params.R_SRB, params.R_arm, params.R_lambda, params.P_SRB, params.P_arm, D_q_outside_});


    std::map<std::string, casadi::DM> arg, res;
    arg["lbx"] = args.lbx;
    arg["ubx"] = args.ubx;
    arg["lbg"] = args.lbg;
    arg["ubg"] = args.ubg;
    arg["x0"] = args.x0;
    arg["p"] = args.p;

    // writeMatrixToFile(args.lbx, basePath + "args_lbx.txt");
    // writeMatrixToFile(args.ubx, basePath + "args_ubx.txt");
    // writeMatrixToFile(args.lbg, basePath + "args_lbg.txt");
    // writeMatrixToFile(args.ubg, basePath + "args_ubg.txt");
    // writeMatrixToFile(args.x0, basePath + "args_x0.txt");
    // writeMatrixToFile(args.p, basePath + "args_p.txt");

    auto start = std::chrono::high_resolution_clock::now();
    res = solver_(arg);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    DM sol = DM::zeros(params.n_states * (params.N + 1) + params.n_controls * params.N, 1); // sol.x should be initialized with actual solution data
    sol = res.at("x");
    // writeMatrixToFile(sol, basePath + "sol.txt");

    casadi_int start_index_u = static_cast<casadi_int>(params.n_states * (params.N + 1));
    casadi_int end_index = static_cast<casadi_int>(sol.numel());

    ukAll_ = reshape(sol(Slice(start_index_u, end_index)).T(), params.n_controls, params.N).T();
    DM sol_x_N = reshape(sol(Slice(0, static_cast<casadi_int>(params.n_states * (params.N + 1)))).T(), params.n_states, params.N + 1).T();
    // Assuming sol_x_N is a previously defined CasADi DM or MX matrix
    // appendMatrixToFile(ukAll_sol, basePath + "ukAll_sol_append.txt");
    // appendMatrixToFile(sol_x_N, basePath + "sol_x_N_append.txt");
    DM mpc_sol_wo_IC = sol_x_N(Slice(1, sol_x_N.size1()), Slice()); // Slices all columns
    // writeMatrixToFile(mpc_sol_wo_IC, basePath + "mpc_sol_wo_IC.txt");
    // DM mpc_sol_wo_IC = sol_x_N(Slice(1, static_cast<casadi_int>(params.n_SRB)), Slice()); // Slices all columns

    x_sol_N_ = mpc_sol_wo_IC; // assign class variable
    DM x_sol_N_SRB_zyx = mpc_sol_wo_IC(Slice(),Slice(0, static_cast<casadi_int>(params.n_SRB))); // assign class variable
    //Change from xyz to zyx
    //We don't care about other states
    DM x_sol_N_SRB_xyz = transform_ZYX_to_XYZ(x_sol_N_SRB_zyx); 
    x_sol_N_SRB_ = x_sol_N_SRB_xyz;

    /* if (static_cast<double> (casadi::DM::abs(x_sol_N_SRB_(1,8) - qk_(8)))> M_PI)
    {
        std::cout << "Yaw angle difference is greater than pi, adjusting..." << std::endl;
        for (int i = 0; i < params.N; ++i) {
            x_sol_N_SRB_(i, 8) = -x_sol_N_SRB_(i, 8);
        }
    } */


    ukAllSRB_ = ukAll_(Slice(),Slice(0, static_cast<casadi_int>(params.n_u_SRB)));
    ukAllArm_ = ukAll_(Slice(), Slice(static_cast<casadi_int>(params.n_u_SRB), static_cast<casadi_int>(params.n_u_SRB+params.n_u_arm)));
    ukAllLambda_ = ukAll_(Slice(), Slice(static_cast<casadi_int>(params.n_u_SRB+params.n_u_arm), static_cast<casadi_int>(params.n_u_SRB+ params.n_u_arm+params.n_u_lambda_int)));
    

    // writeMatrixToFile(ukAll_, basePath + "ukAll_.txt");

    // writeMatrixToFile(ukAll_sol, basePath + "ukAll_.txt");
    // writeMatrixToFile(sol_x_N, basePath + "sol_x_N.txt");
    params.mpc_iter++;

    DM uk = ukAll_(0, Slice());
    uk_ = uk;
    uout(Slice(), params.mpc_iter) = uk; // Store current control outputs
    ukSRB_ = uk(Slice(0, static_cast<casadi_int>(params.n_u_SRB))); //This will be input torquEs for SRB
    ukArm_ = uk(Slice(static_cast<casadi_int>(params.n_u_SRB), static_cast<casadi_int>(params.n_u_arm + params.n_u_SRB))); //This will be input torques for Arm
    ukLambda_ = uk(Slice(static_cast<casadi_int>(params.n_u_arm + params.n_u_SRB), static_cast<casadi_int>(params.n_u_arm + params.n_u_SRB + params.n_u_lambda_int)));
    
    // writeMatrixToFile(ukArm_, basePath + "ukSRB_.txt");

    // Enable the state-feedback here
    shift();
    qkAll_ = vertcat(sol_x_N(Slice(1, sol_x_N.size1()), Slice()), sol_x_N(Slice(sol_x_N.size1() - 1, sol_x_N.size1()), Slice()));
    // qk_ = qk_ + DM(params.dt) * f_value;
    // Looks like updatubg the state from low level, this should happen before
    //TODO add rotation feedback
    // for (size_t i = 0; i < 3; i++)
    // {
    //     qk_(i) = q[i];
    //     qk_(i + 3) = dq[i];
      
    // }
    ukArmPD_ = ukArm_;
    ukArmPD_(3) = ukArm_(3) + 10*(q_des_(21)-qk_(21)) + 1*(q_des_(31)-qk_(31)); //PD control for 4 th joint only

    x_sol_ = reshape(x_sol_N_SRB_.T(), x_sol_N_SRB_.T().size1() * x_sol_N_SRB_.T().size2(), 1);

    // qkAll_ = reshape(sol_x_N, params.N+1*params.n_states, 1);
 

    // writeMatrixToFile(x_sol_, basePath + "x_sol_.txt");

    // Data Logging
    // std::cout << "MPC Iter: " << params.mpc_iter << std::endl;
    xout(Slice(), params.mpc_iter) = qk_.T();                             // Store current states, transposed
    traj(Slice(), params.mpc_iter) = q_des_(casadi::Slice(0, static_cast<casadi_int>(params.n_states)), 0).T(); // Store reshaped desired trajectory
    feet(Slice(), params.mpc_iter) = reshape(params.pf, 1, 12);           // Store reshaped foot positions
    tout(0, params.mpc_iter) = params.current_time - params.dt;           // Store current time
    NMPC_solve_time(0, params.mpc_iter) = DM(duration.count());           // Store NMPC solve time
    NMPC_solve_time_ = DM(duration.count()); // Store NMPC solve time for the current iteration
    // fitBezierTrajectory();
    
    // xsolout(Slice(), params.mpc_iter) = x_sol_N_(Slice()).T();                      // Store current solution
    // writeMatrixToFile(qk_, basePath + "qk_.txt");
    // writeMatrixToFile(uk.T(), basePath + "uk_.txt");
    // writeMatrixToFile(x_sol_N_SRB_, basePath + "x_sol_N_SRB_.txt");
    // writeMatrixToFile(x_sol_N_SRB_, basePath + "x_sol_N_SRB_.txt");
    // writeMatrixToFile(bez_alpha_, basePath + "bez_alpha.txt");
    casadiToEigen(x_sol_ , x_sol_eigen_);
    // casadiToEigen(bez_alpha_, bez_alpha_eigen_);
    // casadiToEigen(bez_alpha_GRF_, bez_alpha_GRF_eigen_);
    casadiToEigen(ukAllSRB_, ukAllSRB_eigen_);
    // writeMatrixToFile(x_sol_eigen_, basePath + "x_sol_eigen.txt");
    // writeMatrixToFile(bez_alpha_eigen_, basePath + "bez_alpha_eigen.txt");

    // std::cout << "Current time: " << params.current_time << std::endl;

    params.current_time = control_tick_ * 0.001;
    writeData();

    // params.current_time += params.dt; is already done in shift()
}


void NMPC_A1_KINOVA::logData()
{
    std::string basePath = "./tmp/";
    writeMatrixToFile(xout.T(), basePath + "xout.txt");
    writeMatrixToFile(traj.T(), basePath + "traj.txt");
    writeMatrixToFile(uout.T(), basePath + "uout.txt");
    writeMatrixToFile(feet.T(), basePath + "feet.txt");
    writeMatrixToFile(tout.T(), basePath + "tout.txt");
    writeMatrixToFile(NMPC_solve_time, basePath + "NMPC_solve_time.txt");
}
void NMPC_A1_KINOVA::writeData()
{
if (success==1){
        fid << params.current_time - params.dt << "," 
            << x_sol_eigen_(0)<< "," 
            << x_sol_eigen_(1)<< ","
            << x_sol_eigen_(2)<< ","
            << x_sol_eigen_(3)<< ","
            << x_sol_eigen_(4)<< ","
            << x_sol_eigen_(5)<< ","
            << x_sol_eigen_(6)<< ","
            << x_sol_eigen_(7)<< ","
            << x_sol_eigen_(8)<< ","
            << x_sol_eigen_(9)<< ","
            << x_sol_eigen_(10)<< ","
            << x_sol_eigen_(11)<< ","
            <<qk_(0)<< ","
            <<qk_(1)<< ","
            <<qk_(2)<< ","
            <<qk_(3)<< ","
            <<qk_(4)<< ","
            <<qk_(5)<< ","
            <<qk_(6)<< ","
            <<qk_(7)<< ","
            <<qk_(8)<< ","
            <<qk_(9)<< ","
            <<qk_(10)<< ","
            <<qk_(11)<< ","
            <<qk_(12)<< ","
            <<qk_(13)<< ","
            <<qk_(14)<< ","
            <<qk_(15)<< ","
            <<qk_(16)<< ","
            <<qk_(17)<< ","
            <<qk_(18)<< ","
            <<qk_(19)<< ","
            <<qk_(20)<< ","
            <<qk_(21)<< ","
            <<qk_(22)<< ","
            <<qk_(23)<< ","
            <<qk_(24)<< ","
            <<qk_(25)<< ","
            <<qk_(26)<< ","
            <<qk_(27)<< ","
            <<qk_(28)<< ","
            <<qk_(29)<< ","
            <<qk_(30)<< ","
            <<qk_(31)<< ","
            <<ukSRB_(0)<< ","
            <<ukSRB_(1)<< ","
            <<ukSRB_(2)<< ","
            <<ukSRB_(3)<< ","
            <<ukSRB_(4)<< ","
            <<ukSRB_(5)<< ","
            <<ukSRB_(6)<< ","
            <<ukSRB_(7)<< ","
            <<ukSRB_(8)<< ","
            <<ukSRB_(9)<< ","
            <<ukSRB_(10)<< ","
            <<ukSRB_(11)<< ","
            <<ukArm_(0)<< ","
            <<ukArm_(1)<< ","
            <<ukArm_(2)<< ","
            <<ukArm_(3)<< ","
            <<ukLambda_(0)<< ","
            <<ukLambda_(1)<< ","
            <<ukLambda_(2)<< ","
            <<ukLambda_(3)<< ","
            <<ukLambda_(4)<< ","
            <<ukLambda_(5)<< ","
            << traj_(0)<< ","
            << traj_(1)<< ","
            << traj_(2)<< ","
            << traj_(3)<< ","
            << traj_(4)<< ","
            << traj_(5)<< ","
            << traj_(6)<< ","
            << traj_(7)<< ","
            << traj_(8)<< ","
            << traj_(9)<< ","
            << traj_(10)<< ","
            << traj_(11)<< ","
            << traj_(12)<< ","
            << traj_(13)<< ","
            << traj_(14)<< ","  
            << traj_(15)<< ","
            << traj_(16)<< ","  
            << traj_(17)<< ","
            << traj_(18)<< ","
            << traj_(19)<< ","
            << traj_(20)<< ","
            << traj_(21)<< ","
            << traj_(22)<< ","
            << traj_(23)<< ","
            << traj_(24)<< ","
            << traj_(25)<< ","
            << traj_(26)<< ","
            << traj_(27)<< ","
            << traj_(28)<< ","
            << traj_(29)<< ","
            << traj_(30)<< ","
            << traj_(31)<< ","
            << params.Q_SRB(0)<< ","
            << params.Q_SRB(1)<< ","
            << params.Q_SRB(2)<< ","
            << params.Q_SRB(3)<< ","
            << params.Q_SRB(4)<< ","
            << params.Q_SRB(5)<< ","
            << params.Q_SRB(6)<< ","
            << params.Q_SRB(7)<< ","
            << params.Q_SRB(8)<< ","
            << params.Q_SRB(9)<< ","
            << params.Q_SRB(10)<< ","
            << params.Q_SRB(11)<< ","
            << params.P_SRB(0)<< ","
            << params.P_SRB(1)<< ","
            << params.P_SRB(2)<< ","
            << params.P_SRB(3)<< ","
            << params.R_SRB(0)<< ","
            << params.R_SRB(1)<< ","
            << params.R_SRB(2)<< ","
            << params.R_SRB(3)<< ","
            << params.Q_arm(0)<< ","
            << params.Q_arm(1)<< ","
            << params.Q_arm(2)<< ","
            << params.Q_arm(3)<< ","
            << params.Q_arm(4)<< ","
            << params.Q_arm(5)<< ","
            << params.Q_arm(6)<< ","
            << params.Q_arm(7)<< ","
            << params.Q_arm(8)<< ","
            << params.Q_arm(9)<< ","
            << params.Q_arm(10)<< ","
            << params.Q_arm(11)<< ","
            << params.Q_arm(12)<< ","
            << params.Q_arm(13)<< ","
            << params.Q_arm(14)<< ","
            << params.Q_arm(15)<< ","
            << params.Q_arm(16)<< ","
            << params.Q_arm(17)<< ","
            << params.Q_arm(18)<< ","
            << params.Q_arm(19)<< ","
            << params.R_arm(0)<< ","
            << params.R_arm(1)<< ","
            << params.R_arm(2)<< ","
            << params.R_arm(3)<< ","
            << params.R_lambda(0)<< ","
            << params.R_lambda(1)<< ","
            << params.R_lambda(2)<< ","
            << params.R_lambda(3)<< ","
            << params.R_lambda(4)<< ","
            << params.R_lambda(5)<< ","
            << params.P_arm(0)<< ","
            << params.P_arm(1)<< ","
            << params.P_arm(2)<< ","
            << params.P_arm(3)<< ","
            << params.P_arm(4)<< ","
            << params.P_arm(5)<< ","
            << NMPC_solve_time_<< ","
            << std::endl;
            
    }
}


int NMPC_A1_KINOVA::binomial(int n, int k)
{
    int result = 1;
    if (k > n - k)
        k = n - k;
    for (int i = 0; i < k; ++i)
    {
        result *= (n - i);
        result /= (i + 1);
    }
    return result;
}

int NMPC_A1_KINOVA::binomialCoeff(int n, int k)
{
    if (k > n)
        return 0;
    if (k == 0 || k == n)
        return 1;
    return binomialCoeff(n - 1, k - 1) + binomialCoeff(n - 1, k);
}

void NMPC_A1_KINOVA::fitBezierTrajectory()
{
    int degree = 4; // Degree of the Bezier curve
    int num_points = params.N;
    int num_states = params.n_states;
    int num_inputs_SRB = params.n_u_SRB;
    int num_states_SRB = params.n_SRB;
    int num_states_arm = params.n_arm;

    // Initialize matrix A for Bezier basis coefficients
    DM A = DM::zeros(num_points, degree + 1);

    // Populate matrix A with Bezier basis values
    for (int i = 0; i < num_points; ++i)
    {
        double t = static_cast<double>(i) / (num_points - 1); // Normalized time for this index
        for (int j = 0; j <= degree; ++j)
        {
            A(i, j) = binomialCoeff(degree, j) * pow(t, j) * pow(1 - t, degree - j);
        }
    }

    // Bezier coefficients matrix (one row per state)
    DM bez_alpha = DM::zeros(num_states_SRB, degree + 1);
    DM bez_alpha_GRF = DM::zeros(num_inputs_SRB, degree + 1);

    // Check if x_sol_ is empty
    // writeMatrixToFile(x_sol_N_, basePath + "x_sol_N_.txt");

    // Perform matrix operations for all states
    DM ATA = mtimes(transpose(A), A);
    // DM ATx = mtimes(transpose(A), x_sol_N_); // Transpose x_sol for correct dimension alignment
    //Need to do this for only SRB portion of the states
    // DM ATx = mtimes(transpose(A), x_sol_N_SRB_); // Transpose x_sol for correct dimension alignment
    // DM x_trial = DM::zeros(num_points, num_states_SRB);

    // x_trial(0,2) = 0.28;
    // x_trial(1,2) = 0.28;
    // x_trial(2,2) = 0.28;
    // x_trial(3,2) = 0.28;
    // x_trial(4,2) = 0.28;
    // for (int i = 0; i < num_points; ++i)
    // {
    //     x_trial(i,3) = 0.28;
    // }
    // x_sol_N_SRB_ = x_trial;
    DM ATx = mtimes(transpose(A), x_sol_N_SRB_); // Transpose x_sol for correct dimension alignment

    // Solve the system to find Bezier coefficients
    bez_alpha = solve(ATA, ATx);

    bez_alpha_ = bez_alpha.T();

    DM ATx_GRF = mtimes(transpose(A),ukAllSRB_);
    bez_alpha_GRF = solve(ATA, ATx_GRF);
    bez_alpha_GRF_ = bez_alpha_GRF.T();
    // Output the Bezier Coefficients Matrix
    // writeMatrixToFile(bez_alpha_, basePath + "bez_alpha.txt");
    // writeMatrixToFile(bez_alpha_GRF_, basePath + "bez_alpha_GRF.txt");
    // std::cout << "Bezier Coefficients Matrix:" << std::endl;
    // std::cout << bez_alpha_ << std::endl;
}

void NMPC_A1_KINOVA::casadiToEigen(const casadi::DM &mat1, Eigen::MatrixXd &mat2)
{
    // Resize the Eigen matrix to match the dimensions of the CasADi matrix
    mat2.resize(mat1.rows(), mat1.columns());

    // Iterate over all elements to copy them
    for (int i = 0; i < mat1.rows(); ++i)
    {
        for (int j = 0; j < mat1.columns(); ++j)
        {
            mat2(i, j) = static_cast<double>(mat1(i, j)); // Cast CasADi double to Eigen double
        }
    }
}


void NMPC_A1_KINOVA::updateState(double *qin, double *dqin, double *qin_arm, double *dqin_arm , int *ind, Eigen::Matrix<double, 3, 4> toePos, Eigen::Vector4d state_vec, const int domain, const int control_tick)
{
    memcpy(q, qin, 18 * sizeof(double));
    memcpy(dq, dqin, 18 * sizeof(double));
    memcpy(contactInd, ind, 4 * sizeof(int));
    memcpy(q_arm, qin_arm, 4 * sizeof(double));
    memcpy(dq_arm, dqin_arm, 4 * sizeof(double));
    toePos_ = toePos;
    state_other = state_vec;
    if (domain > domain_)
    {
        isDomainChanged_ = true;
    }
    else
    {
        isDomainChanged_ = false;
    }
    domain_ = domain;
    control_tick_ = control_tick;
    // cout << "OTHER AGENT's STATE (" << log_ID << "):   " << state_other.transpose() << std::endl;
    //get rotation matrix
    Eigen::Map<Eigen::Matrix<double, 3, 1>> eul_xyz(&q[3],3);
    Eigen::Map<Eigen::Matrix<double, 3, 1>> omega_xyz_body(&dq[3],3);
    Eigen::Matrix<double,3,3> R_xyz = RotMat_XYZ(eul_xyz);
    // writeMatrixToFile(R_xyz, basePath + "R_xyz.txt");

    
    Eigen::MatrixXd dd;
    casadiToEigen(params.dd, dd);
    Eigen::Matrix<double,3,1> R_dd = R_xyz * dd;
    Eigen::Matrix<double,3,1> dotR_dd = R_xyz * skewSym(omega_xyz_body) * dd;
    //convert eul and omega to ZYX
    transform_XYZ_to_ZYX(q,dq);
    
    //For Arm Base theta dot calculation
    Eigen::Map<Eigen::Matrix<double, 3, 1>> eul_zyx(&q[3],3);
    Eigen::Map<Eigen::Matrix<double, 3, 1>> omega_zyx_world(&dq[3],3);
    Eigen::Matrix<double,3,1> dtheta_zyx = A_Theta_ZYX(eul_zyx)*omega_zyx_world;
    // writeMatrixToFile(eul_zyx, basePath + "eul_zyx.txt");
    // writeMatrixToFile(omega_zyx_world, basePath + "omega_zyx_world.txt");
    

    //Update the qk_ for SRB and arm base
    for (size_t i = 0; i < 3; i++)
    {
        qk_(i) = q[i]; //SRB COM position
        qk_(i + 3) = dq[i]; //SRB COM velocity
        qk_(i + 6) = q[i + 3]; //SRB orientation
        qk_(i + 9) = dq[i + 3]; // SRB omega
        qk_(i+12) = (q[i] + R_dd(i)); //Arm base position
        qk_(i+15) = q[i+3]; //Arm base orientation same as SRB
        qk_(i+22) = (dq[i] + dotR_dd(i)); //Arm base velocity same as SRB COM? No
        qk_(i+25) = dtheta_zyx(i); //Arm base theta dot

        /* if (params.gait==8){
            qk_(i + 3) = 0*q_des_(i+3); //SRB COM velocity
            qk_(i+22) = 0*(dq[i] + dotR_dd(i)); //Arm base velocity same as SRB COM? No

        } */

    }
    // qk_(5) = q[2]; //Z Velocity of SRB COM
    // qk_(24) = dq[2] + dotR_dd(2); //Z Velocity of Arm base
    //Update qk_ shape variable (joint pos/vel) for arm
    for (size_t i = 0; i < 4; ++i)
    {
        qk_(i + 18) = q_arm[i];//(q_arm[i] >= M_PI)? q_arm[i] - 2*M_PI : (q_arm[i] <= -M_PI)?  q_arm[i] + 2*M_PI :q_arm[i] ;
        qk_(i + 28) = dq_arm[i];
    }
    // qk_(18) = 6.28; 
    // qk_(18) = (static_cast<double>(qk_(18)) >= M_PI) ? qk_(18) - 2*M_PI : (static_cast<double>(qk_(18)) <= -M_PI)? qk_(18) + 2*M_PI: qk_(18);
    // writeMatrixToFile(qk_, basePath + "qk_.txt");
    

}

void NMPC_A1_KINOVA::generateReferenceTrajectory()
{   
    /* This would be the high level QP based trajectory generator */
    // New way to load parameters from a YAML file, avoids compiling each time a parameter is changed
    // YAML::Node config = YAML::LoadFile("../src/config.yaml");
    YAML::Node config = YAML::LoadFile("/home/trec/WorkRaj/raisim_legged/Basit_NMPC/SRBNMPC_A1_wLL/src/config.yaml");
    double c = config["parameters"]["c"].as<double>();
    double m = config["parameters"]["m"].as<double>();
    double epsilon = config["parameters"]["epsilon"].as<double>();
    double sigma = config["parameters"]["sigma"].as<double>();
    double dth = config["parameters"]["dth"].as<double>();
    double alpha = config["parameters"]["alpha"].as<double>();
    double eta = config["parameters"]["eta"].as<double>();
    double dmin = config["parameters"]["dmin"].as<double>();

    // Old way
    // double c = 300.0, m = 12.45, epsilon = 60, sigma = 1.0;
    // double dth = 0.8, alpha = 150.0, eta = 400.0, dmin = 1.0;

    int loopSize = SIM_TIME / params.dt;
    size_t num_obs = Pobs.cols();
    const size_t ramp_up_iterations = 10;

    double Ts = 0.01; // 43*0.001/10;

    Eigen::MatrixXd Ae;
    Ae.setZero(4 * NUMBER_OF_AGENTS, 4 * NUMBER_OF_AGENTS);

    Eigen::MatrixXd A(4, 4);
    Eigen::MatrixXd B(4, 2);
    A << 0, 0, 1, 0,
        0, 0, 0, 1,
        0, 0, -c / m, 0,
        0, 0, 0, -c / m;
    B << 0, 0,
        0, 0,
        1 / m, 0,
        0, 1 / m;
    // Discretize
    Eigen::MatrixXd Ad = Eigen::MatrixXd::Identity(4, 4) + Ts * A;
    Eigen::MatrixXd Bd = Ts * B;

    std::cout << "Ad: " << Ad << std::endl;
    std::cout << "Bd: " << Bd << std::endl;

    // Eigen::MatrixXd Ad; Ad.setZero(4, 4);
    // Eigen::MatrixXd Bd; Bd.setZero(4, 2);

    // Ad <<   1,                   0,   0.001025061239872,                   0,
    //         0,                   1,                   0,   0.001025061239872,
    //         0,                   0,   0.929527039758809,                   0,
    //         0,                   0,                   0,   0.929527039758809;

    // Bd <<   0.000000068070472960,                   0,
    //         0,                   0.000000068070472960,
    //         0.000128132654983983,                   0,
    //         0,                   0.000128132654983983;

    for (int agent = 0; agent < NUMBER_OF_AGENTS; ++agent)
    {
        int blockStartIndex = agent * 4; // Calculate the start index for both row and column
        Ae.block(blockStartIndex, blockStartIndex, 4, 4) = Ad;
    }

    Eigen::MatrixXd Be;
    Be.setZero(4 * NUMBER_OF_AGENTS, 2 * NUMBER_OF_AGENTS);

    for (int agent = 0; agent < NUMBER_OF_AGENTS; ++agent)
    {
        int blockRowStartIndex = agent * 4; // Calculate the start index for the row
        int blockColStartIndex = agent * 2; // Calculate the start index for the column
        Be.block(blockRowStartIndex, blockColStartIndex, 4, 2) = Bd;
    }

    // Reference Loop
    Eigen::MatrixXd q;
    q.setZero(4 * NUMBER_OF_AGENTS, loopSize + 1);

    // Generic initialization of starting positions and velocities
    for (int k = 0; k < NUMBER_OF_AGENTS; ++k)
    {
        q.block(4 * k, 0, 2, 1) = Pstart_.segment(2 * k, 2); // Set starting positions for each agent
        q.block(4 * k + 2, 0, 2, 1).setZero();               // Set initial velocities to zero
    }

    // Forget number of agents for now 
    // Set target velocities
    double vd_x = 0.5;
    double vd_y = 0.0;
    for (int k = 0; k < q.cols(); ++k)
    {
        q.block( 2, k, 2, 1) << vd_x, vd_y;
    }

    /*     for (size_t i = 0; i < loopSize; i++)
        {
            Eigen::MatrixXd F; F.setZero(8, 1);
            // for each agent
            for (size_t k = 0; k < NUMBER_OF_AGENTS; k++)
            {
                // Initialized required variables
                Eigen::Vector2d qk_p, qk_p_other, p_g, F_att, F_rep, F_agent; qk_p.setZero(); qk_p_other.setZero(); p_g.setZero(); F_att.setZero(); F_rep.setZero(); F_agent.setZero();
                Eigen::VectorXd d_obs; d_obs.setZero(NUMBER_OF_OBS);
                double d_goal = 0.0, d_agent = 0.0;

                // Index current and other agent positions
                qk_p = q.block(0+k*4, i, 2, 1);

                // Goal point
                p_g << GOAL_X, GOAL_Y;

                // Distance from goal
                d_goal = (qk_p - p_g).norm();

                // Attractive Force
                if (d_goal > 0.1)
                    F_att = -alpha*((qk_p - p_g)/(qk_p - p_g).norm());
                else
                    F_att = -0*((qk_p - p_g)/(qk_p - p_g).norm());
                // Distance from Obstacle
                for (size_t j = 0; j < num_obs; j++)
                {
                    d_obs(j) = (qk_p - Pobs.col(j)).norm();
                }

                // Repulsive Force due to obstacles
                Eigen::Vector2d F_tmp; F_tmp.setZero();
                for (size_t j = 0; j < num_obs; j++)
                {
                    if (d_obs(j) < dmin)
                    {
                        F_tmp += eta*(1/d_obs(j) - 1/dmin)*(1/pow(d_obs(j), 2))*((qk_p - Pobs.col(j))/(qk_p - Pobs.col(j)).norm());
                    }
                }
                F_rep = F_tmp;

                for (size_t j = 0; j < NUMBER_OF_AGENTS; j++)
                {
                    if ( k != j)
                    {
                        // Distance form other agent
                        qk_p_other = q.block(0+(j)*4, i, 2, 1);
                        d_agent = (qk_p - qk_p_other).norm();
                        // Lennard Jones Potential
                        F_agent = F_agent - 4*epsilon*( (6*pow(sigma, 6))/pow(d_agent, 7) - (12*pow(sigma, 12))/pow(d_agent, 13))*( (qk_p - qk_p_other)/(qk_p - qk_p_other).norm() );
                    }
                }

                F.block(0 + k*2, 0, 2, 1) = (F_att + F_rep + F_agent);

                if (d_goal < 0.001)
                {
                    F.block(0 + k*2, 0, 2, 1) = 0*F_att;
                }
            }
            // Scale the force vector during the ramp-up phase
            double scale_factor = (i < ramp_up_iterations) ? (static_cast<double>(i) / ramp_up_iterations) : 1.0;

            q.col(i+1) = Ae*q.col(i) + Be*F*scale_factor;
        } */

    size_t resample_freq = 1; // 40

    for (size_t i = 0; i < loopSize / resample_freq; i++)
    {
        q.col(i) = q.col(resample_freq * i);
    }

    Eigen::MatrixXd Pr;
    Pr.setZero(2 * NUMBER_OF_AGENTS, loopSize / resample_freq);
    Eigen::MatrixXd Prd;
    Prd.setZero(2 * NUMBER_OF_AGENTS, loopSize / resample_freq);

    for (int agent = 0; agent < NUMBER_OF_AGENTS; ++agent)
    {
        int baseRowPr = agent * 2;  // Base row for Pr operations
        int baseRowPrd = agent * 2; // Base row for Prd operations
        int baseRowQ = agent * 4;   // Base row for q.block operations

        Pr.row(baseRowPr) = q.block(baseRowQ, 2, 1, loopSize / resample_freq);
        Pr.row(baseRowPr + 1) = q.block(baseRowQ + 1, 2, 1, loopSize / resample_freq);
        Prd.row(baseRowPrd) = q.block(baseRowQ + 2, 2, 1, loopSize / resample_freq);
        Prd.row(baseRowPrd + 1) = q.block(baseRowQ + 3, 2, 1, loopSize / resample_freq);
    }

    Pr_refined_ = Pr;
    Prd_refined_ = Prd;

    // =========================================================================//
    // ================================= LOGGING ===============================//
    // =========================================================================//

    // Hardcoded base directory
    std::string baseDir = "/home/trec/WorkRaj/raisim_legged/Basit_NMPC/SRBNMPC_A1_wLL";
    std::string outputsDir = baseDir + "/Sim_Outputs";

    // No directory existence check or creation

    // Log HL Path
    std::string filePath = outputsDir + "/HLPath.txt";
    std::fstream myfile5;
    myfile5.open(filePath, std::fstream::out);
    if (!myfile5.is_open())
    {
        std::cerr << "Failed to open file: " << filePath << std::endl;
        // Handle error, possibly exit the function or program
    }
    else
    {
        myfile5 << Pr_refined_;
        myfile5.close();
    }

    // Log HL Velocity
    filePath = outputsDir + "/HLVelocity.txt";
    std::fstream myfile6;
    myfile6.open(filePath, std::fstream::out);
    if (!myfile6.is_open())
    {
        std::cerr << "Failed to open file: " << filePath << std::endl;
        // Handle error, possibly exit the function or program
    }
    else
    {
        myfile6 << Prd_refined_;
        myfile6.close();
    }

    // Log Pstart
    filePath = outputsDir + "/Pstart.txt";
    std::fstream Pstartf;
    Pstartf.open(filePath, std::fstream::out);
    if (!Pstartf.is_open())
    {
        std::cerr << "Failed to open file: " << filePath << std::endl;
        // Handle error, possibly exit the function or program
    }
    else
    {
        Pstartf << Pstart_;
        Pstartf.close();
    }

    // Log Pobs
    filePath = outputsDir + "/Pobs.txt";
    std::fstream Pobsf;
    Pobsf.open(filePath, std::fstream::out);
    if (!Pobsf.is_open())
    {
        std::cerr << "Failed to open file: " << filePath << std::endl;
        // Handle error, possibly exit the function or program
    }
    else
    {
        Pobsf << Pobs;
        Pobsf.close();
    }

    // Log Pobs_real
    filePath = outputsDir + "/Pobs_real.txt";
    std::fstream Pobs_realf;
    Pobs_realf.open(filePath, std::fstream::out);
    if (!Pobs_realf.is_open())
    {
        std::cerr << "Failed to open file: " << filePath << std::endl;
        // Handle error, possibly exit the function or program
    }
    else
    {
        Pobs_realf << Pobs_real;
        Pobs_realf.close();
    }
}

void NMPC_A1_KINOVA::setPstart(Eigen::Matrix<double, 2 * NUMBER_OF_AGENTS, 1> &Pstart)
{
    Pstart_ << Pstart;
    agent_Initial_(0) = Pstart_(2 * agent_id_);
    agent_Initial_(1) = Pstart_(2 * agent_id_ + 1);
    // mpc_state_alpha_buffer_<< agent_Initial_(0), 0, agent_Initial_(1), 0;
}

void NMPC_A1_KINOVA::setPobs(Eigen::Matrix<double, 2, NUMBER_OF_OBS> &Pobs_in)
{
    Pobs << Pobs_in;
}

void NMPC_A1_KINOVA::setPobs_real(Eigen::Matrix<double, 2, NUMBER_OF_OBS> &Pobs_in)
{
    Pobs_real << Pobs_in;
}

void NMPC_A1_KINOVA::setAgentID(size_t agent_id)
{
    agent_id_ = agent_id;
}



void NMPC_A1_KINOVA::jointevolution(double twave){

    //Joint evolution for the arm only
    // Joint 1
    double A1 = M_PI / 9; // Amplitude
    double f1 = 0.3;      // Frequency
    double phi1 = 0;      // Phase
    // q_des_(18) = arm0_pos_(0) + A1 * sin(2 * M_PI * f1 * t + phi1) * (t_ramp / 1.0); // Ramp up over 1 second
    q_des_(28) = -A1 * 2 * M_PI * f1 * sin(2 * M_PI * f1 * twave + phi1);// * (t_ramp / 1.0); // Velocity
    // q_des_(28) = 0.0; // Velocity

    // Joint 2
    double A2 = M_PI / 9;
    double f2 = 0.3;
    double phi2 = 0*M_PI / 4;
    // q_des_(19) = arm0_pos_(1) + A2 * sin(2 * M_PI * f2 * twave + phi2);
    q_des_(29) = -A2 * 2 * M_PI * f2 * sin(2 * M_PI * f2 * twave + phi2);

    // Joint 3
    double A3 = M_PI / 8;
    double f3 = 0.3;
    double phi3 = 0*M_PI / 2;
    // q_des_(20) = arm0_pos_(2) + A3 * sin(2 * M_PI * f3 * twave + phi3);
    q_des_(30) = -A3 * 2 * M_PI * f3 * sin(2 * M_PI * f3 * twave + phi3);

    // Joint 4
    // double A4 = M_PI / 4;
    // double f4 = 0.1;
    // double phi4 = M_PI;
    // q_des_(21) = arm0_pos_(3) + A4 * sin(2 * M_PI * f4 * t + phi4) * (t_ramp / 1.0);
    // q_des_(31) = A4 * 2 * M_PI * f4 * cos(2 * M_PI * f4 * t + phi4) * (t_ramp / 1.0);
    // q_des_(31) = 0.0;
}
