// Author: Basit Muhammad Imran
// Date: 2024-04-17

#include "nmpc.hpp"
#include <yaml-cpp/yaml.h>

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

casadi::DM NMPC::bezier(const DM &alpha, double s)
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

DM NMPC::bezierd(const DM &alpha, double s)
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

/* SX NMPC::Rotz(const SX &t)
{
    // Ensure t is a vector of size 3
    if (t.size1() != 3 || t.size2() != 1)
    {
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
} */

SX NMPC::Rotz(const SX &t)
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

casadi::SX NMPC::RotMat_XYZ(const casadi::SX& rpy) {
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

casadi::SX NMPC::rot_x(const casadi::SX& t) {
    // Ensure t is a vector of size 1
    if (t.size1() != 1 || t.size2() != 1) {
        throw std::runtime_error("Input vector t must be of size 1.");
    }
    casadi::SX Rx = casadi::SX::vertcat({casadi::SX::horzcat({1, 0, 0}), casadi::SX::horzcat({0, cos(t), -sin(t)}), casadi::SX::horzcat({0, sin(t), cos(t)})});
    return Rx;
}

casadi::SX NMPC::rot_y(const casadi::SX& t){
    // Ensure t is a vector of size 1
    if (t.size1() != 1 || t.size2() != 1) {
        throw std::runtime_error("Input vector t must be of size 1.");
    }

    casadi::SX Ry =  casadi::SX::vertcat({casadi::SX::horzcat({cos(t), 0, sin(t)}), casadi::SX::horzcat({0, 1, 0}), casadi::SX::horzcat({-sin(t), 0, cos(t)})});
   
    return Ry;

}

casadi::SX NMPC::rot_z(const casadi::SX& t){
    // Ensure t is a vector of size 1
    if (t.size1() != 1 || t.size2() != 1) {
        throw std::runtime_error("Input vector t must be of size 1.");
    }

    casadi::SX Rz =  casadi::SX::vertcat({casadi::SX::horzcat({cos(t), -sin(t), 0}), casadi::SX::horzcat({sin(t), cos(t), 0}), casadi::SX::horzcat({0, 0, 1})});
   
    return Rz;

}

casadi::SX NMPC::RotMat_ZYX(const casadi::SX& rpy) {
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



casadi::DM NMPC::RotMat_ZYX(const casadi::DM& rpy) {
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

casadi::DM NMPC::RotMat_XYZ(const casadi::DM& rpy) {
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
Eigen::Matrix<double,3,3> NMPC::RotMat_XYZ(const Eigen::Matrix<double,3,1>& rpy) {
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

void NMPC::get_Euler_ZYX_from_XYZ(Eigen::Matrix<double,3,1>& rpy_zyx, const Eigen::Matrix<double,3,1>& rpy_xyz){
    Eigen::Matrix<double,3,3> R = RotMat_XYZ(rpy_xyz);
    rpy_zyx(1) = asin(-R(2,0));
    double cos_theta = cos(rpy_zyx(1));
    rpy_zyx(0)= atan2(R(2,1)/cos_theta, R(2,2)/cos_theta);
    rpy_zyx(2)= atan2(R(1,0)/cos_theta, R(0,0)/cos_theta);
}

void NMPC::get_Euler_XYZ_from_ZYX(casadi::DM& rpy_xyz, const casadi::DM& rpy_zyx){
    casadi::DM R = RotMat_ZYX(rpy_zyx);
    rpy_xyz(1) = asin(R(0,2));
    casadi::DM cos_theta = cos(rpy_xyz(1));
    rpy_xyz(0)= atan2(-R(1,2)/cos_theta, R(2,2)/cos_theta);
    rpy_xyz(2)= atan2(-R(0,1)/cos_theta, R(0,0)/cos_theta);

}

void NMPC::get_Omega_ZYX_world_from_XYZ_body(Eigen::Matrix<double,3,1>& omega_zyx_world, const Eigen::Matrix<double,3,1>& omega_xyz_body, const Eigen::Matrix<double,3,1>& rpy_xyz)
{
    Eigen::Matrix<double,3,3> R = RotMat_XYZ(rpy_xyz);
    omega_zyx_world = R*omega_xyz_body;

}

void NMPC::get_Omega_ZYX_world_from_XYZ_world(Eigen::Matrix<double,3,1>& omega_zyx_world, const Eigen::Matrix<double,3,1>& omega_xyz_world, const Eigen::Matrix<double,3,1>& rpy_xyz)
{
    omega_zyx_world = omega_xyz_world;

}

void NMPC::get_Omega_ZYX_world_from_Eulerrate_XYZ_world(Eigen::Matrix<double,3,1>& omega_zyx_world, const Eigen::Matrix<double,3,1>& omega_xyz_world, const Eigen::Matrix<double,3,1>& rpy_xyz)
{
    Eigen::Matrix<double,3,3> Ainv_theta = Ainv_Theta_XYZ(rpy_xyz);
    omega_zyx_world = Ainv_theta*omega_xyz_world;

}

void NMPC::get_Omega_ZYX_world_from_Eulerrate_XYZ_body(Eigen::Matrix<double,3,1>& omega_zyx_world, const Eigen::Matrix<double,3,1>& omega_xyz_world, const Eigen::Matrix<double,3,1>& rpy_xyz)
{
    Eigen::Matrix<double,3,3> Ainv_theta = Ainv_Theta_XYZ(rpy_xyz);
    Eigen::Matrix<double,3,3> R = RotMat_XYZ(rpy_xyz);
    omega_zyx_world = Ainv_theta*R*omega_xyz_world;

}

void NMPC::get_Omega_XYZ_body_from_ZYX_world(casadi::DM& omega_xyz_body, const casadi::DM& omega_zyx_world, const casadi::DM& rpy_zyx)
{
    casadi::DM R = RotMat_ZYX(rpy_zyx);
    omega_xyz_body = casadi::DM::mtimes(R.T(),omega_zyx_world);
    
}


void NMPC::get_Omega_XYZ_world_from_ZYX_world(casadi::DM& omega_xyz_world, const casadi::DM& omega_zyx_world, const casadi::DM& rpy_zyx)
{
    casadi::DM R = RotMat_ZYX(rpy_zyx);
    omega_xyz_world = omega_zyx_world;

}

void NMPC::get_Eulerrate_XYZ_world_from_ZYX_world(casadi::DM& omega_xyz_world, const casadi::DM& omega_zyx_world, const casadi::DM& rpy_xyz)
{
    casadi::DM A_theta = A_Theta_XYZ(rpy_xyz);
    omega_xyz_world = casadi::DM::mtimes(A_theta, omega_zyx_world);

}

void NMPC::get_Eulerrate_XYZ_body_from_ZYX_world(casadi::DM& omega_xyz_world, const casadi::DM& omega_zyx_world, const casadi::DM& rpy_xyz)
{
    casadi::DM A_theta = A_Theta_XYZ(rpy_xyz);
    casadi::DM R = RotMat_XYZ(rpy_xyz);
    omega_xyz_world = casadi::DM::mtimes(R.T(), casadi::DM::mtimes(A_theta, omega_zyx_world));

}

casadi::SX NMPC::A_Theta_ZYX(const casadi::SX& rpy){
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


casadi::DM NMPC::A_Theta_ZYX(const casadi::DM& rpy){
    //ZYX convention Rz*Ry*Rx
    // Ensure rpy is a vector of size 3
    if (rpy.size1() != 3 || rpy.size2() != 1) {
        throw std::runtime_error("Input vector rpy must be of size 3.");
    }
     /* phi = rpy(0);
       theta = rpy(1);
       psi = rpy(2) */
    casadi::DM A = casadi::DM::zeros(3, 3);
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


casadi::SX NMPC::A_Theta_XYZ(const casadi::SX& rpy){
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


casadi::DM NMPC::A_Theta_XYZ(const casadi::DM& rpy){
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


Eigen::Matrix<double,3,3> NMPC::A_Theta_XYZ(const Eigen::Matrix<double,3,1>& rpy){
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

Eigen::Matrix<double,3,3> NMPC::Ainv_Theta_XYZ(const Eigen::Matrix<double,3,1>& rpy)
{
    Eigen::Matrix<double,3,3> A = A_Theta_XYZ(rpy);
    return A.inverse();
}


casadi::SX NMPC::skewSym(const casadi::SX &x)
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

casadi::DM NMPC::transform_ZYX_to_XYZ(const casadi::DM& zyx_input)
{
    
    // Determine the size based on the input x
    int rows = zyx_input.size1();
    int cols = zyx_input.size2();
    casadi::DM xyz_output = zyx_input;

    if (cols != params.n_states)
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

void NMPC::transform_XYZ_to_ZYX(double *q, double *dq){
    Eigen::Map<Eigen::Matrix<double, 3, 1>> eul_xyz(&q[3],3);
    Eigen::Map<Eigen::Matrix<double, 3, 1>> omega_xyz_body(&dq[3],3);
    Eigen::Matrix3d R_xyz = RotMat_XYZ(eul_xyz);
    Eigen::Matrix<double,3,1> eul_zyx, omega_zyx_world;
    eul_zyx.setZero();
    omega_zyx_world.setZero();
    get_Euler_ZYX_from_XYZ(eul_zyx, eul_xyz);
    get_Omega_ZYX_world_from_XYZ_body(omega_zyx_world, omega_xyz_body, eul_xyz);
    // get_Omega_ZYX_world_from_XYZ_world(omega_zyx_world, omega_xyz_body, eul_xyz);
    // get_Omega_ZYX_world_from_Eulerrate_XYZ_world(omega_zyx_world, omega_xyz_body, eul_xyz);
    // get_Omega_ZYX_world_from_Eulerrate_XYZ_body(omega_zyx_world, omega_xyz_body, eul_xyz);

    for (size_t i = 0; i < 3; ++i)
    {
        q[3+i] = eul_zyx(i);
        dq[3+i] = omega_zyx_world(i);
    }
}

void NMPC::planner()
{
    const int n_states = params.n_states; // Example: Number of states
    // const int N = params.N;
    const float stand_duration = 10.0; // Stand up duration
    DM xd = DM::zeros(n_states * params.N);
    double settlingTime= 8.0;
    if (params.current_time <= settlingTime + stand_duration)
    { // Stand up duration
        params.gait = 1;
    }
    else
    {
        params.gait = 8;
    }

    // writeMatrixToFile(q_des_, basePath + "q_des_.txt");

    DM traj = q_des_(casadi::Slice(0, n_states), 0); // Second parameter 0 is to indicate column index for a vector

    switch (params.gait)
    {
    case 1:
    {
        // std::cout << "[NMPC STANDING] " << std::endl;
        double t0 = settlingTime;
        double tf = t0 + stand_duration;                                          // Start and end times for mode 1
        double s = std::min((params.current_time - t0) / (tf - t0), 1.0); // Ensure s is within [0, 1]

        // Extract relevant parameters for easy access
        double xf = 0.0;           // Initial x position
        double yf = 0.0;           // Initial y position
        double zf = params.height; // Assuming the height is constant or predefined
        double z0 = 0.08;
        double yaw = 2.79;//-1.57;//  // Assuming the yaw is constant or predefined

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
        casadi::DM row1 = casadi::DM::horzcat({DM(0), DM(0), DM(0), DM(xf / 4), DM(3 * xf / 4), DM(xf), DM(xf), DM(xf)});
        casadi::DM row2 = casadi::DM::horzcat({DM(0), DM(0), DM(0), DM(yf / 4), DM(3 * yf / 4), DM(yf), DM(yf), DM(yf)});
        casadi::DM row3 = casadi::DM::horzcat({DM(z0), DM(z0), DM(z0), DM(zf / 4), DM(3 * zf / 4), DM(zf), DM(zf), DM(zf)});

        casadi::DM alpha = casadi::DM::vertcat({row1, row2, row3});

        // Update trajectory position
        casadi::DM position_update = bezier(alpha, s);

        for (int i = 0; i < 3; ++i)
        {
            traj(i) = position_update(i);
        }
        traj(8) = yaw; // Update yaw

        // Update trajectory velocity
        casadi::DM velocity_update = bezierd(alpha, s);

        for (int i = 0; i < 3; ++i)
        {
            traj(i + 3) = velocity_update(i)/(tf-t0);
        }
        break;
    }
    case 8:
    {
        double domLen = 200.0;                         // Domain length in milliseconds
        double domLenSec = domLen / 1000.0;            // Domain length in seconds
        DM R_ZYX = RotMat_ZYX(vertcat(qk_(6), qk_(7), qk_(8))); // Assuming q contains Euler angles in the last 3 elements
        // std::cout << "[NMPC TROTTING] " << std::endl;

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
            // if (static_cast<double>(params.contact(0)) == 1.0)
            // {
            //     params.contact = {0, 1, 1, 0};
            // }
            // else
            // {
            //     params.contact = {1, 0, 0, 1};
            // }
            params.contact(0) = contactInd[0];
            params.contact(1) = contactInd[1];
            params.contact(2) = contactInd[2];
            params.contact(3) = contactInd[3];

            DM cmdBody = mtimes(R_ZYX.T(), traj(Slice(3, 6))); // Convert to body frame
            traj(Slice(3, 6)) = cmdBody;

            // Increase velocity in body frame
            traj(3) += 0.1*sign(params.desVel(0)); // Increase command velocity in body X
            traj(4) += 0.1*sign(params.desVel(1)); // Increase command velocity in body Y
            traj(11) += 0.05*sign(params.yawSpeed); // Increase command velocity in yaw

            // Saturate the command velocities
            traj(3) = sign(params.desVel(0))*fmin(abs(traj(3)), abs(params.desVel(0)));
            traj(4) = sign(params.desVel(1))*fmin(abs(traj(4)), abs(params.desVel(1)));
            traj(9) = 0;
            traj(10) = 0;
            traj(11) = sign(params.yawSpeed)*fmin(abs(traj(11)), abs(params.yawSpeed));

           /*  if ((params.current_time >= 7.0 && params.current_time <= 12.0) || (params.current_time >= 16.0 && params.current_time <= 24.0) || params.current_time >=26.0)
            {
                traj(11) = 0.0;
            } */
            
            

            // traj(3) = 0.0  ; //Prd_refined_(0, params.mpc_iter);
            // traj(4) = 0.0;
            // // Prd_refined_(1, params.mpc_iter);

            // Calculate step length in body frame and update step foot position
            DM stepLen = vertcat(traj(3) * domLenSec / 2, 0.0, 0.0);
             // vertcat(traj(3) * domLenSec / 2, traj(4) * domLenSec / 2, 0.0);

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

            // params.p_hip(0,0) = toePos_(0, 0);  params.p_hip(1,0) = toePos_(1, 0); params.p_hip(2,0) = toePos_(2, 0);
            // params.p_hip(0,1) = toePos_(0, 1);  params.p_hip(1,1) = toePos_(1, 1); params.p_hip(2,1) = toePos_(2, 1);
            // params.p_hip(0,2) = toePos_(0, 2);  params.p_hip(1,2) = toePos_(1, 2); params.p_hip(2,2) = toePos_(2, 2);
            // params.p_hip(0,3) = toePos_(0, 3);  params.p_hip(1,3) = toePos_(1, 3); params.p_hip(2,3) = toePos_(2, 3);

            // DM hipTmp = params.p_hip + stepLen;
            // hipTmp = mtimes(Rz, hipTmp); // Rotate to the world frame
            // params.pf = hipTmp + vertcat(qk_(Slice(0, 2)), 0.0);

            // Set velocity in world frame
            DM cmdWrld = mtimes(R_ZYX, traj(Slice(3, 6))); // Convert to world frame
            traj(Slice(3, 6)) = cmdWrld;
            DM A_theta_ZYX  = A_Theta_ZYX(vertcat(qk_(6), qk_(7), qk_(8)));
            // DM cmdWrld_omega = mtimes(A_theta_ZYX, traj(Slice(9, 12))); // Convert to world frame
            //MIght be unstable due to this
            DM cmdWrld_omega = casadi::DM::solve(A_theta_ZYX, traj(Slice(9, 12))); // Convert to world frame
            traj(Slice(9, 12)) = cmdWrld_omega;
            // writeMatrixToFile(traj, basePath + "traj_planner.txt");
            // params.stepcnt++;
            params.stepcnt = domain_;
        }

        float heading = 0;

        // Update position and height based on velocity
        traj(0) = qk_(0) + params.dt * traj(3);
        traj(1) = qk_(1) + params.dt * traj(4);
        traj(2) = params.height; // Assuming height is constant or predefined
        traj(5) = 0;             // Assuming zero vertical velocity
        // traj(6) = qk_(6) + params.dt * traj(9); // Update roll
        // traj(7) = qk_(7) + params.dt * traj(10); // Update pitch
        traj(8) = qk_(8) + params.dt * traj(11);//qk_(8) + params.dt * traj(11); // Update yaw
        // traj(8) = params.current_time * traj(11); // Update yaw

        // std::cout<<"speed command: "<< traj(3) << " " << traj(4) <<std::endl;


        for (int n = 1; n <= (int)params.N; ++n)
        {
            // Compute the predicted state for each step in the horizon
            DM state = DM::zeros(n_states, 1);

            // Index to access the right column in Prd_refined_
            int index = params.mpc_iter + n - 1; // Adjust index for zero-based indexing if necessary

            // Make sure we don't exceed the bounds of Prd_refined_
           /*  if (index >= Prd_refined_.cols())
            {
                std::cerr << "Index exceeds the size of Prd_refined_" << std::endl;
                break; // or handle overflow more gracefully
            } */

            // Velocity components from Prd_refined_
            double vx = static_cast<double>(traj(3)); //Prd_refined_(0, index); // Access the velocity in x-direction
            double vy = static_cast<double>(traj(4)); //Prd_refined_(1, index); // Access the velocity in y-direction

            // Updating the state based on velocities from Prd_refined_
            state(0) = (qk_(0) + n * params.dt * vx); // pos X
            state(1) = (qk_(1) + n * params.dt * vy); // pos Y
            state(2) = params.height;               // pos Z (constant height assumption)
            state(3) = vx;//-0.01*(0-qk_(3)) ; //vx;                          // vel X (updated from Prd_refined_)
            state(4) =  vy;//-0.01*(0-qk_(4));//vy;                          // vel Y (updated from Prd_refined_)
            state(5) = 0.0;                         // vel Z (assuming zero vertical velocity)
            // Assuming Euler angles (heading) and angular velocity are zero
            state(6) = 0;//qk_(6) + n * params.dt * traj(9); // Roll or phi
            state(7) = 0;//qk_(7) + n * params.dt * traj(10); // Pitch or theta
            state(8) = (qk_(8) + n * params.dt * traj(11)); // Yaw or psi 

            state(9) = 0;//traj(9); // Angular velocity in roll
            state(10) = 0;//traj(10); // Angular velocity in pitch
            state(11) = traj(11);//traj(11); // Angular velocity in yaw

            // Place the computed state into the appropriate segment of xd
            // Adjust indexing if your container xd is one-dimensional but structured like a two-dimensional array
            xd(Slice((n - 1) * n_states, n * n_states)) = state;
        }



        break;
    }

        case 9:
    {
        double domLen = 200.0;                         // Domain length in milliseconds
        double domLenSec = domLen / 1000.0;            // Domain length in seconds
        DM Rz = Rotz(vertcat(qk_(6), qk_(7), qk_(8))); // Assuming q contains Euler angles in the last 3 elements
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
                params.contact = {1, 1, 1, 1};
            }
            else
            {
                params.contact = {1, 1, 1, 1};
            }

            // Saturate the command velocities
            traj(3) = 0;//fmin(traj(3), params.desVel(0));
            traj(4) = 0; //fmin(traj(4), params.desVel(1));


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


            params.stepcnt = domain_;
        }

        float heading = 0;

        // Update position and height based on velocity
        traj(0) = 0;//qk_(0) ;//+ params.dt * traj(3);
        traj(1) = 0;//qk_(1) ;//+ params.dt * traj(4);
        traj(2) = params.height; // Assuming height is constant or predefined
        traj(5) = 0;             // Assuming zero vertical velocity

        for (int n = 1; n <= (int)params.N; ++n)
        {
            // Compute the predicted state for each step in the horizon
            DM state = DM::zeros(n_states, 1);

            // Index to access the right column in Prd_refined_
            int index = params.mpc_iter + n - 1; // Adjust index for zero-based indexing if necessary

            // Make sure we don't exceed the bounds of Prd_refined_
           /*  if (index >= Prd_refined_.cols())
            {
                std::cerr << "Index exceeds the size of Prd_refined_" << std::endl;
                break; // or handle overflow more gracefully
            } */

            // Velocity components from Prd_refined_
            double vx = static_cast<double>(traj(3)); //Prd_refined_(0, index); // Access the velocity in x-direction
            double vy = static_cast<double>(traj(4)); //Prd_refined_(1, index); // Access the velocity in y-direction

            // Updating the state based on velocities from Prd_refined_
            state(0) = 0;//qk_(0) ;//+ n * params.dt * vx; // pos X
            state(1) = 0;//qk_(1) ;//+ n * params.dt * vy; // pos Y
            state(2) = params.height;               // pos Z (constant height assumption)
            state(3) = 0.0;//vx;                          // vel X (updated from Prd_refined_)
            state(4) = 0.0;//vy;                          // vel Y (updated from Prd_refined_)
            state(5) = 0.0;                         // vel Z (assuming zero vertical velocity)
            // Assuming Euler angles (heading) and angular velocity are zero
            state(6) = 0.0;     // Roll or phi
            state(7) = 0.0;     // Pitch or theta
            state(8) = heading; // Yaw or psi
            state(9) = 0.0;     // Angular velocities (assuming zeros)
            state(10) = 0.0;
            state(11) = 0.0;

            // Place the computed state into the appropriate segment of xd
            // Adjust indexing if your container xd is one-dimensional but structured like a two-dimensional array
            xd(Slice((n - 1) * n_states, n * n_states)) = state;
        }

        break;
    }
    // Handle other cases as necessary
    default:
        std::cerr << "Mode '" << params.gait << "' is not defined in the motion planner.\n";
    }

    if (params.gait != 8 && params.gait != 9)
    {
        xd = repmat(traj, params.N, 1);
    }

    q_des_ = xd; // Only xd needs to be returned
    // std::cout<< std::endl << "[MPC_Toe]"<<std::endl;
    // std::cout << params.pf << std::endl;
}

void NMPC::force_inequality(const casadi::SX &U, casadi::SX &g_force)
{
    // Use SX for symbolic constraint expressions
    casadi::SX g_force_sx = casadi::SX::vertcat(std::vector<casadi::SX>{});

    // g_ub and g_lb as symbolic vectors with appropriate bounds
    casadi::DM g_ub_DM = casadi::DM::vertcat(std::vector<casadi::DM>{});
    casadi::DM g_lb_DM = casadi::DM::vertcat(std::vector<casadi::DM>{});

    for (size_t k = 0; k < params.N; ++k)
    {
        for (size_t leg = 0; leg < 4; ++leg)
        {
            size_t base_idx = leg * 3;
            // Directly construct and append symbolic expressions
            g_force_sx = casadi::SX::vertcat({g_force_sx,
                                              U(base_idx, k) - params.mu * U(base_idx + 2, k),
                                              U(base_idx + 1, k) - params.mu * U(base_idx + 2, k),
                                              U(base_idx, k) + params.mu * U(base_idx + 2, k),
                                              U(base_idx + 1, k) + params.mu * U(base_idx + 2, k)});
            g_ub_DM = casadi::SX::vertcat({g_ub_DM, 0.0, 0.0, casadi::inf, casadi::inf});
            g_lb_DM = casadi::SX::vertcat({g_lb_DM, -casadi::inf, -casadi::inf, 0, 0});
        }
    }

    // Convert symbolic SX expressions to DM for g_ub and g_lb if necessary
    // Note: This might not be needed if keeping them symbolic is preferable
    g_force = g_force_sx; // g_force is already SX and can stay symbolic
    g_ub_ = g_ub_DM;
    g_lb_ = g_lb_DM;
}

void NMPC::force_inequality_bounds()
{

    // g_ub and g_lb as DM vectors with appropriate bounds
    casadi::DM g_ub_sx;
    casadi::DM g_lb_sx;

    for (size_t k = 0; k < params.N; ++k)
    {
        for (size_t leg = 0; leg < 4; ++leg)
        {
            // Directly construct and append symbolic expressions
            g_ub_sx = casadi::SX::vertcat({g_ub_sx, 0.0, 0.0, casadi::inf, casadi::inf});
            g_lb_sx = casadi::SX::vertcat({g_lb_sx, -casadi::inf, -casadi::inf, 0, 0});
        }
    }
    g_ub_ = g_ub_sx;
    g_lb_ = g_lb_sx;
}

void NMPC::writeMatrixToFile(const casadi::SX &matrix, const std::string &filename)
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
        std::cerr << "Unable to open file " << filename << std::endl;
    }
}

void NMPC::writeMatrixToFile(const casadi::DM &matrix, const std::string &filename)
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
        std::cerr << "Unable to open file " << filename << std::endl;
    }
}

void NMPC::writeMatrixToFile(const std::string &filename)
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
        std::cerr << "Unable to open file " << filename << std::endl;
    }
}

void NMPC::appendMatrixToFile(const casadi::DM& matrix, const std::string& filename) {
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

void NMPC::equalityconstraints(const SX &X, const SX &U, const SX &P, SX &obj, SX &g_eq, const Function &f)
{

    std::string basePath = "./tmp/";
    int n_states = params.n_states;
    double dt = params.dt;

    // Initial condition constraints
    SX st = X(Slice(0, n_states), 0);
    g_eq = st - P(Slice(0, n_states));

    for (int k = 0; k < (int)params.N; ++k)
    {
        SX st = X(Slice(0, n_states), k);
        SX con = U(Slice(0, n_states), k);

        SX des_st = P(Slice(k * n_states, (k + 1) * n_states)); // Desired state

        // writeMatrixToFile(st, basePath + "st.txt");
        // writeMatrixToFile(des_st, basePath + "des_st.txt");
        // writeMatrixToFile(params.Q, basePath + "params_Q.txt");
        // writeMatrixToFile(con, basePath + "con.txt");
        // writeMatrixToFile(params.R, basePath + "params_R.txt");
        // Calculate objective
        SX st_err = st - des_st;
        // writeMatrixToFile(st_err, basePath + "st_err.txt");
        SX stage_cost = mtimes(mtimes(st_err.T(), params.Q), st_err);
        SX terminal_cost = mtimes(mtimes(con.T(), params.R), con);

        // writeMatrixToFile(stage_cost, basePath + "stage_cost.txt");
        // writeMatrixToFile(terminal_cost, basePath + "terminal_cost.txt");

        obj = obj + stage_cost + terminal_cost;

        // Calculate next state prediction
        SX st_next = X(Slice(0, n_states), k + 1);

        std::map<std::string, SX> arg;
        arg["st"] = st; // Example random inputs
        arg["con"] = con;

        std::map<std::string, SX> result = f(arg);
        SX f_value = result.at("rhs");

        // writeMatrixToFile(st_next, basePath + "st_next.txt");
        // writeMatrixToFile(f_value, basePath + "f_value.txt");

        // Compute constraints
        SX st_next_euler = st + dt * f_value;
        g_eq = SX::vertcat({g_eq, st_next - st_next_euler});

        // writeMatrixToFile(st_next_euler, basePath + "st_next_euler.txt");
        // writeMatrixToFile(g_eq, basePath + "g_eq.txt");
    }
}

void NMPC::set_constraint_bounds()
{
    // Calculate the start index for setting lower and upper bounds

    int startIndex = params.n_states * (params.N + 1); // Adjust for zero-based indexing

    // Resize the lbg and ubg if necessary
    if (args.lbg.is_empty() || args.lbg.size1() < startIndex + g_lb_.size1())
    {
        args.lbg = DM::zeros(startIndex + g_lb_.size1(), 1); // Adjust total size if needed
    }
    if (args.ubg.is_empty() || args.ubg.size1() < startIndex + g_ub_.size1())
    {
        args.ubg = DM::zeros(startIndex + g_ub_.size1(), 1); // Adjust total size if needed
    }

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

void NMPC::set_statebounds()
{
    int n_states = params.n_states; // Number of states
    int n_inputs = params.n_inputs; // Number of inputs
    int N = params.N;
    double zmax = 150;

    // Initialize LB and UB
    args.lbx = DM::zeros(n_states * (N + 1) + n_inputs * N, 1);
    args.ubx = DM::zeros(n_states * (N + 1) + n_inputs * N, 1);
    args.lbx(Slice()) = -DM::inf();
    args.ubx(Slice()) = DM::inf();

    for (int k = 0; k < N; ++k)
    {
        for (int leg = 0; leg < 4; ++leg)
        {
            int index = n_states * (N + 1) + n_inputs * k + 3 * leg + 2; // Adjusted for 0-based indexing
            args.lbx(index) = 0;
            args.ubx(index) = params.contact(leg) * zmax; // Assuming contact is std::vector<double>
        }
    }
}

void NMPC::shift()
{
    // // Prepare input for the function f
    // std::map<std::string, DM> arg;
    // arg["st"] = qk_;                     // Current state
    // arg["con"] = ukAll_(0, Slice()).T(); // First control input, transposed to match MATLAB

    // // Call the function f
    // std::map<std::string, DM> result = f_dyn_(arg);
    // DM f_value = result["rhs"]; // Assuming the output is named 'rhs'

    // Update state
    // qk_ = qk_ + DM(params.dt) * f_value;

    // Increment time
    // params.current_time += params.dt;
    //TODO: Shift States as well

    DM prev_ukAll = ukAll_;

    // Shift control inputs
    DM u_new = vertcat(prev_ukAll(Slice(1, prev_ukAll.size1()), Slice()),                       // Skip the first control set
                       prev_ukAll(Slice(prev_ukAll.size1() - 1, prev_ukAll.size1()), Slice())); // Duplicate the last control set
    ukAll_ = u_new;
}

void NMPC::code_gen()
{

    const double dt = params.dt;               // Sample time
    const double sim_time = params.sim_length; // Simulation time
    const float sim_length = sim_time / dt;    // Simulation length
    const int n_states = params.n_states;      // Number of states
    const int n_inputs = params.n_inputs;      // Number of inputs
    const int n_controls = params.n_controls;  // Number of inputs
    const int gait = params.gait;              // Gait type
    const double G = params.G;                 // Gravity
    const double mass = params.mass;           // Mass

    init(); // Initialize the variables

    // Symbolic variables
    casadi::SXVector states_vec = {
        SX::sym("Xx"), SX::sym("y"), SX::sym("z"),
        SX::sym("dx"), SX::sym("dy"), SX::sym("dz"),
        SX::sym("phi"), SX::sym("theta"), SX::sym("psi"),
        SX::sym("dphi"), SX::sym("dtheta"), SX::sym("dpsi")};

    // Define controls similarly
    casadi::SXVector controls_vec = {
        SX::sym("u1x"), SX::sym("u1y"), SX::sym("u1z"),
        SX::sym("u2x"), SX::sym("u2y"), SX::sym("u2z"),
        SX::sym("u3x"), SX::sym("u3y"), SX::sym("u3z"),
        SX::sym("u4x"), SX::sym("u4y"), SX::sym("u4z")};

    casadi::SX states = casadi::SX::vertcat(states_vec);
    casadi::SX controls = casadi::SX::vertcat(controls_vec);

    SX U = SX::sym("U", n_controls, params.N);
    SX P = SX::sym("P", n_states + n_states * params.N);
    SX X = SX::sym("X", n_states, params.N + 1);
    SX OPT_variables = vertcat(reshape(X, n_states * (params.N + 1), 1), reshape(U, n_inputs * params.N, 1));

    DM qk = DM::zeros(n_states);
    qk(2) = 0.08; // Assuming z position is constant or predefined

    double t0 = 0.0;
    DM xx(n_states, sim_length);
    xx(Slice(0, n_states), 0) = qk;

    DM u0 = DM::zeros(params.N, params.n_inputs); // Control inputs for N horizon steps, not sim_steps

    // States decision variables initialization
    DM X0 = repmat(qk, 1, params.N + 1);
    DM xd = repmat(qk, params.N, 1);

    double current_time = 0.0;
    size_t mpciter = 0;

    Args args;
    args.lbg = DM::zeros(n_states * (params.N + 1) + 4 * 4 * params.N, 1);
    args.ubg = DM::zeros(n_states * (params.N + 1) + 4 * 4 * params.N, 1);

    DM uout = DM::zeros(n_controls, sim_length); // Vector to store control outputs
    DM xout = DM::zeros(n_states, sim_length);
    DM feet = DM::zeros(12, sim_length); // Vector to store foot positions
    DM traj = DM::zeros(12, sim_length); // Vector to store desired trajectory
    DM tout = DM::zeros(1, sim_length);

    // params.current_time = current_time;
    params.current_time = control_tick_ * 0.001;
    params.mpc_iter = mpciter;
    planner(); // Assuming gait and other details are handled inside
    args.p = vertcat(qk, xd);

    SX t_rot = casadi::SX::vertcat({states(6), states(7), states(8)}); // Extract phi, theta, and psi into a column vector for Rotz
    SX Rz = Rotz(t_rot);                                               // Call Rotz with the column vector
    SX z3 = casadi::SX::zeros(3, 3);                                   // Create a 3x3 zero matrix
    SX i3 = casadi::SX::eye(3);                                        // Create a 3x3 identity matrix

    DM J = DM::zeros(3, 3);                                              // Assuming a paramse-5; J(2,2) = 0.064713601;
    SX omega = casadi::SX::vertcat({states(9), states(10), states(11)}); // Extracting angular velocities (dphi, dtheta, dpsi)
    SX skew_omega = skewSym(omega);
    SX J_sx = SX(J); // Convert J from DM to SX
    SX IwI = mtimes(mtimes(mtimes(Rz, J_sx), Rz.T()), skew_omega) * mtimes(mtimes(Rz, J_sx), Rz.T());

    SX A = SX::zeros(12, 12); // A is a 12x12 matrix given your block structure
    // Fill in the blocks of A
    A(Slice(0, 3), Slice(0, 3)) = z3;
    A(Slice(0, 3), Slice(3, 6)) = i3;
    A(Slice(0, 3), Slice(6, 9)) = z3;
    A(Slice(0, 3), Slice(9, 12)) = z3;

    A(Slice(3, 6), Slice(0, 3)) = z3;
    A(Slice(3, 6), Slice(3, 6)) = z3;
    A(Slice(3, 6), Slice(6, 9)) = z3;
    A(Slice(3, 6), Slice(9, 12)) = z3;

    A(Slice(6, 9), Slice(0, 3)) = z3;
    A(Slice(6, 9), Slice(3, 6)) = z3;
    A(Slice(6, 9), Slice(6, 9)) = z3;
    A(Slice(6, 9), Slice(9, 12)) = Rz.T();

    A(Slice(9, 12), Slice(0, 3)) = z3;
    A(Slice(9, 12), Slice(3, 6)) = z3;
    A(Slice(9, 12), Slice(6, 9)) = z3;
    A(Slice(9, 12), Slice(9, 12)) = z3; //-IwI

    SX D = SX::zeros(12, 1);                             // D is a 12x1 vector
    D(Slice(3, 6)) = SX::vertcat({SX::zeros(2, 1), -G}); // Fill the 4th to 6th rows

    SX p_foot_sx = SX(params.pf);
    SX com_vector = SX::vertcat({states(0), states(1), states(2)});
    SX com_vector_expanded = repmat(com_vector, 1, 4); // Replicate across columns to match p_foot_sx dimensions

    SX rd = p_foot_sx - com_vector_expanded;

    SX skew_rd = skewSym(rd);
    SX B_tmp = mtimes(mtimes(Rz, J_sx), Rz.T()) * skew_rd;

    SX B = SX::vertcat({SX::zeros(3, 12),
                        SX::repmat(1 / mass * i3, 1, 4),
                        SX::zeros(3, 12),
                        mtimes(mtimes(Rz, J_sx), Rz.T()) * skew_rd}); // mtimes(mtimes(Rz, J_sx), Rz.T()) * skew_rd
    SX rhs = mtimes(A, states) + mtimes(B, controls) + D;

    // writeMatrixToFile(rhs, basePath + "rhs.txt");

    Function f_dyn = Function("f_dyn", {states, controls}, {rhs}, {"st", "con"}, {"rhs"});

    // Declare variables to hold the outputs
    SX g_force;

    // Call the function
    force_inequality(U, g_force);

    SX obj = 0, g_eq;
    equalityconstraints(X, U, P, obj, g_eq, f_dyn);
    SX g = vertcat(g_eq, g_force);
    set_statebounds();
    set_constraint_bounds();
    SXDict nlp_prob = {{"f", obj}, {"x", OPT_variables}, {"g", g}, {"p", P}};


    // Define options for the solver
    Dict opts;

    // For ipopt
    opts["ipopt.max_iter"] = 10;                    // Replace Max_mpciter with its actual value
    opts["ipopt.print_level"] = 3;                  // Can be changed to 0 or 3 based on verbosity required
    opts["print_time"] = 0;                         // Disable printing solver time
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

    // Reshape X0 and u0 transposed into column vectors
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
    sol = res.at("x");

    // =========================== CODE GEN =========================== //

    std::string file_name = "nlp_code";
    // code predix
    std::string prefix_code = fs::current_path().string() + "/";
    casadi::Dict code_gen_opts = casadi::Dict();
    code_gen_opts["cpp"] = false;
    code_gen_opts["with_header"] = false;
    solver.generate_dependencies(file_name + ".c");

    CodeGenerator codegen = CodeGenerator(file_name + "_f" + ".c", code_gen_opts);
    codegen.add(f_dyn);
    codegen.generate();
    std::string prefix_lib = fs::current_path().string() + "/";
    std::string compile_command_solver = "gcc -fPIC -shared -O3 " +
                                         prefix_code + file_name + ".c -o " +
                                         prefix_lib + file_name + ".so -L/usr/local/lib -lipopt";

    std::string compile_command_f = "gcc -fPIC -shared -O3 " +
                                    prefix_code + file_name + "_f" + ".c -o " +
                                    prefix_lib + file_name + "_f" + ".so";
    std::cout << compile_command_solver << std::endl;
    std::cout << compile_command_f << std::endl;

    int compile_flag_solver = std::system(compile_command_solver.c_str());
    casadi_assert(compile_flag_solver == 0, "Compilation failed");
    std::cout << "Compilation successed!" << std::endl;

    int compile_flag_f = std::system(compile_command_f.c_str());
    casadi_assert(compile_flag_f == 0, "Compilation failed");
    std::cout << "Compilation successed!" << std::endl;
}

void NMPC::init()
{
    qk_ = DM::zeros(params.n_states);
    qk_(2) = 0.08; // sitting com height
    q_des_ = DM::zeros(params.n_states, 1);
    ukAll_ = DM::zeros(params.N, params.n_inputs);

    qkAll_ = repmat(qk_, 1, params.N + 1); // contains initial state, and solution for all N horizon steps
    q_des_ = repmat(qk_, params.N, 1);     // contains desired state for all N horizon steps

    args.lbg = DM::zeros(params.n_states * (params.N + 1) + 4 * 4 * params.N, 1);
    args.ubg = DM::zeros(params.n_states * (params.N + 1) + 4 * 4 * params.N, 1);

    uout = DM::zeros(params.n_controls, params.sim_length * 1000);
    xout = DM::zeros(params.n_states, params.sim_length * 1000);
    feet = DM::zeros(12, params.sim_length * 1000); // Vector to store foot positions
    traj = DM::zeros(12, params.sim_length * 1000);
    tout = DM::zeros(1, params.sim_length * 1000);
    NMPC_solve_time = DM::zeros(1, params.sim_length * 1000);

    std::string file_name = "nlp_code";

    std::string prefix_lib = "/home/trec/WorkRaj/raisim_legged/Basit_NMPC/SRBNMPC_A1_wLL/build/";

    // std::cout << "prefix_lib: " << prefix_lib << std::endl; std::cin.get();

    std::string lib_name = prefix_lib + file_name + ".so";

    // std::cout << "lib_name: " << lib_name << std::endl; std::cin.get();
    // Load the nlp solver
    // Define options for the solver
    Dict opts;

    int iter_lim = 10;
    if (params.current_time < 1)
    {
        iter_lim = 13;
    }
    else
    {    iter_lim = 10; // 12 also works
        }

    opts["ipopt.max_iter"] = iter_lim;              // Replace Max_mpciter with its actual value
    opts["ipopt.print_level"] = 0;                  // Can be changed to 0 or 3 based on verbosity required
    opts["print_time"] = 0;                         // Disable printing solver time
    opts["ipopt.acceptable_tol"] = 1e-3;            // Tolerance for stopping criterion
    opts["ipopt.acceptable_obj_change_tol"] = 1e-3; // Objective change tolerance for stopping
    solver_ = casadi::nlpsol("solver", "ipopt", lib_name, opts);

    // Load nonlinear dynamics function
    lib_name = prefix_lib + file_name + "_f"
                                        ".so";
    f_dyn_ = external("f_dyn", lib_name);


    //Logging
    std::string filename = prefix_lib + "./tmp/NMPC_x_log.txt";
    if(!filename.empty()){
        fid = std::fstream(filename, std::ios::out);
    }
    success = (fid.is_open()) ? 1 : 0;
}

void NMPC::run_NMPC()
{

    // call the planner function to update q_des_ for N horizon steps
    planner();

    // writeMatrixToFile(q_des_, basePath + "q_des_.txt");
    // writeMatrixToFile(basePath + "params.txt");

    force_inequality_bounds(); // Updates g_ub_ and g_lb_ with appropriate bounds
    set_statebounds();         // Updates args.lbx and args.ubx with appropriate bounds
    set_constraint_bounds();   // Updates args.lbg and args.ubg with g_lb_ and g_ub_
    // writeMatrixToFile(g_ub, basePath + "g_ub.txt");
    // writeMatrixToFile(g_lb, basePath + "g_lb.txt");
    // writeMatrixToFile(params, basePath + "params.txt");

    solve_nlp(); // Solve the NMPC problem
}

void NMPC::solve_nlp()
{   //This q0_col and u0_col should come from Raisim
    curr_time+= 0.01;
    DM q0_col = reshape(qkAll_.T(), params.n_states * (params.N + 1), 1);
    DM u0_col = reshape(ukAll_.T(), params.n_inputs * params.N, 1);

    // Concatenate reshaped vectors vertically
    DM x0u0 = vertcat(q0_col, u0_col);
    args.x0 = x0u0;
    // qk_ = DM::vertcat({-0.22262, 0.435923, 0.281147, 0.0689813, 5.13423e-09, 0.0169365, 4.19124e-13, 1.16626e-13, -4.61192e-14, -0.000373239, -2.75582e-05, 0.000223812});
    // args.p = vertcat(qk_, q_des_);
    args.p = DM::vertcat({qk_, q_des_, reshape(params.pf, 12, 1),params.Q,params.R, params.P}); 

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
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    DM sol = DM::zeros(params.n_states * (params.N + 1) + params.n_controls * params.N, 1); // sol.x should be initialized with actual solution data
    sol = res.at("x");  

    casadi_int start_index_u = static_cast<casadi_int>(params.n_states * (params.N + 1));
    casadi_int end_index = static_cast<casadi_int>(sol.numel());
    
    DM ukAll_sol = reshape(sol(Slice(start_index_u, end_index)).T(), params.n_controls, params.N).T();
    DM sol_x_N = reshape(sol(Slice(0, static_cast<casadi_int>(params.n_states * (params.N + 1)))).T(), params.n_states, params.N + 1).T();
    // Assuming sol_x_N is a previously defined CasADi DM or MX matrix
    // writeMatrixToFile(sol_x_N, basePath + "sol_x_N.txt");
    // appendMatrixToFile(sol_x_N, basePath + "sol_x_N_append.txt");
    DM mpc_sol_wo_IC = sol_x_N(Slice(1, sol_x_N.size1()), Slice()); // Slices all columns

    x_sol_N_ = mpc_sol_wo_IC; // assign class variable
    //Need to make sure yaw is according to feedback - 
   
    /* 
    for (int i = 0; i < params.N; ++i) {
        x_sol_N_(i, 8) = static_cast<double>(n)>0?-x_sol_N_(i, 8) + n * M_PI:x_sol_N_(i, 8);
    } */
    DM x_sol_N_SRB_xyz = transform_ZYX_to_XYZ(x_sol_N_); // Transform from ZYX to XYZ
    // x_sol_N_ = sol_x_N;
    x_sol_N_ = x_sol_N_SRB_xyz;

    if (static_cast<double> (casadi::DM::abs(x_sol_N_(1,8) - qk_(8)))> M_PI)
    {
        std::cout << "Yaw angle difference is greater than pi, adjusting..." << std::endl;
        for (int i = 0; i < params.N; ++i) {
            x_sol_N_(i, 8) = -x_sol_N_(i, 8);
        }
    }

    ukAll_ = ukAll_sol;

    // writeMatrixToFile(ukAll_, basePath + "ukAll_.txt");
    // appendMatrixToFile(ukAll_, basePath + "ukAll_append.txt");
    // writeMatrixToFile(ukAll_sol, basePath + "ukAll_.txt");
    // writeMatrixToFile(sol_x_N, basePath + "sol_x_N.txt");
    params.mpc_iter++;

    DM uk = ukAll_sol(0, Slice());
    uk_ = uk;
    uout(Slice(), params.mpc_iter) = uk; // Store current control outputs

    // Enable the state-feedback here
    shift();
    qkAll_ = vertcat(sol_x_N(Slice(1, sol_x_N.size1()), Slice()), sol_x_N(Slice(sol_x_N.size1() - 1, sol_x_N.size1()), Slice()));
/* 
    DM sol_vector = reshape(x_sol_N_.T(), x_sol_N_.T().size1() * x_sol_N_.T().size2(), 1);

    // qkAll_ = reshape(sol_x_N, params.N+1*params.n_states, 1);
    x_sol_ = sol_vector;


    writeMatrixToFile(x_sol_, basePath + "x_sol_.txt"); */

    
    

    // Data Logging
    xout(Slice(), params.mpc_iter) = qk_.T();                             // Store current states, transposed
    traj(Slice(), params.mpc_iter) = q_des_(casadi::Slice(0, 12), 0).T(); // Store reshaped desired trajectory
    feet(Slice(), params.mpc_iter) = reshape(params.pf, 1, 12);           // Store reshaped foot positions
    tout(0, params.mpc_iter) = params.current_time - params.dt;           // Store current time
    NMPC_solve_time(0, params.mpc_iter) = DM(duration.count());           // Store NMPC solve time

    // writeMatrixToFile(qk_, basePath + "qk_.txt");
    // writeMatrixToFile(uk.T(), basePath + "uk_.txt");
    // writeMatrixToFile(x_sol_N_, basePath + "x_sol_N_.txt");
    fitBezierTrajectory();

    DM sol_vector = reshape(x_sol_N_.T(), x_sol_N_.T().size1() * x_sol_N_.T().size2(), 1);
    // writeMatrixToFile(sol_vector, basePath + "sol_vector.txt");

    // qkAll_ = reshape(sol_x_N, params.N+1*params.n_states, 1);
    x_sol_ = sol_vector;


    // writeMatrixToFile(x_sol_, basePath + "x_sol_.txt");
    

    casadiToEigen(x_sol_, x_sol_eigen_);
    casadiToEigen(bez_alpha_, bez_alpha_eigen_);
    // casadiToEigen(bez_alpha_GRF_, bez_alpha_GRF_eigen_);
    casadiToEigen(ukAll_, ukAllSRB_eigen_);
    
    // std::cout << "MPC Iter: " << params.mpc_iter << std::endl;
    // std::cout << "Current time: " << params.current_time << std::endl;

    params.current_time = control_tick_ * 0.001;
    writeData();

    // params.current_time += params.dt; is already done in shift()
}

void NMPC::logData()
{
    std::string basePath = "./tmp/";
    writeMatrixToFile(xout.T(), basePath + "xout.txt");
    writeMatrixToFile(traj.T(), basePath + "traj.txt");
    writeMatrixToFile(uout.T(), basePath + "uout.txt");
    writeMatrixToFile(feet.T(), basePath + "feet.txt");
    writeMatrixToFile(tout.T(), basePath + "tout.txt");
    writeMatrixToFile(NMPC_solve_time, basePath + "NMPC_solve_time.txt");

}

void NMPC::writeData()
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
            <<uk_(0)<< ","
            <<uk_(1)<< ","
            <<uk_(2)<< ","
            <<uk_(3)<< ","
            <<uk_(4)<< ","
            <<uk_(5)<< ","
            <<uk_(6)<< ","
            <<uk_(7)<< ","
            <<uk_(8)<< ","
            <<uk_(9)<< ","
            <<uk_(10)<< ","
            <<uk_(11)<< ","
            <<q_des_(0)<< ","
            <<q_des_(1)<< ","
            <<q_des_(2)<< ","
            <<q_des_(3)<< ","
            <<q_des_(4)<< ","
            <<q_des_(5)<< ","
            <<q_des_(6)<< ","
            <<q_des_(7)<< ","
            <<q_des_(8)<< ","
            <<q_des_(9)<< ","
            <<q_des_(10)<< ","
            <<q_des_(11)<< ","
            << std::endl;
            
    }
}

int NMPC::binomial(int n, int k)
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

int NMPC::binomialCoeff(int n, int k)
{
    if (k > n)
        return 0;
    if (k == 0 || k == n)
        return 1;
    return binomialCoeff(n - 1, k - 1) + binomialCoeff(n - 1, k);
}

void NMPC::fitBezierTrajectory()
{
    int degree = 4; // Degree of the Bezier curve
    int num_points = params.N ; // Number of points in the trajectory
    int num_states = params.n_states;

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
    DM bez_alpha = DM::zeros(num_states, degree + 1);
    /* double x_vel = 0.0;
    double y_vel = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    // Check if x_sol_ is empty
    // writeMatrixToFile(x_sol_N_, basePath + "x_sol_N_.txt");
    // DM x_trial = DM::zeros(num_points, num_states);
    for (int i = 0; i < num_points; ++i)
    {
        x_trial(i,0) = (curr_time >= 0.8)?x_trial(i,0)+0.01*x_trial(i,3):0.0;
        x_trial(i,1) = (curr_time >= 0.8)?x_trial(i,1)+0.01*x_trial(i,4):0.0;
        x_trial(i,2) = 0.28;
        x_trial(i,6) = (curr_time >= 0.8)?x_trial(i,6)+0.01*x_trial(i,9):0.0;
        x_trial(i,7) = (curr_time >= 0.8)?x_trial(i,7)+0.01*x_trial(i,10):0.0;
        DM tmp_yaw = x_trial(i,8)+0.01*x_trial(i,11);
        x_trial(i,8) = (curr_time >= 0.8)? tmp_yaw:0.0;
        // account for sign flip only one time
        if (static_cast<double>(qk_(8))<0 && static_cast<double>(tmp_yaw)>pi){
             x_trial(i,8) = -(2*pi-tmp_yaw);
        }
     
        
        

        std::cout << std::setprecision(6) << "Des_Yaw:" << x_trial(i,8) << " Curr_Yaw: " << qk_(8)<<std::endl;
        
        // if(curr_time >0){x_vel = 0.0; y_vel = 0.0;roll = 0.0;pitch=0.0;yaw = 0.8;}
        if(curr_time <4){x_vel = 0.0; y_vel = 0.0;roll = 0.0;pitch=0.0;yaw = 0.0;}
        if(curr_time >=4 && curr_time <7){x_vel = 0.0; y_vel = 0.0;roll = 0.00;pitch=-0.0;yaw = 0.8;}
        if(curr_time >=7 && curr_time <11){x_vel = 0.0; y_vel = 0.0; roll = 0.0;pitch=0.0;yaw = 0.8;}
        if(curr_time >=11 && curr_time <17){x_vel = 0.0; y_vel = 0.0;roll = 0.1;pitch=0.00;yaw = -0.8;}
        if(curr_time >=17 && curr_time <21){x_vel = 0.0; y_vel = 0.0;roll = 0.0;pitch=0.0;yaw = -0.8;}
        if(curr_time >=21){x_vel = -0.5; y_vel = 0.0;}

        x_trial(i,3) = (curr_time >=0.8)?
                            ((abs(static_cast<double>(x_trial(i,3))-x_vel)>0.003)
                                ?(x_trial(i,3)+0.01*(sign(x_vel-x_trial(i,3)))*1.5)
                                :x_vel)
                            :0.0;
        x_trial(i,4) = (curr_time >= 0.8)?
                            ((abs(static_cast<double>(x_trial(i,4))-y_vel)>0.003)?
                                (x_trial(i,4)+0.01*(sign(y_vel-x_trial(i,4)))*1.5)
                                :y_vel )
                            :0.0;

        // x_trial(i,6) = (curr_time >= 0.8)?
        //                         ((abs(static_cast<double>(x_trial(i,6))-roll)>0.02)?
        //                             (x_trial(i,6)+0.01*(roll-x_trial(i,6)))
        //                             :roll )
        //                         :0.0;

        // x_trial(i,7) = (curr_time >= 0.8)?
        //                         ((abs(static_cast<double>(x_trial(i,7))-pitch)>0.02)?
        //                             (x_trial(i,7)+0.01*(pitch-x_trial(i,7)))
        //                             :pitch )
        //                         :0.0;

        // x_trial(i,8) = (curr_time >= 0.8)?
        //                 ((abs(static_cast<double>(x_trial(i,8))-yaw)>0.02)?
        //                     (x_trial(i,7)+0.01*(yaw-x_trial(i,8)))
        //                     :yaw )
        //                 :0.0;

        x_trial(i,9) = (curr_time >= 0.8)?
                            ((abs(static_cast<double>(x_trial(i,9))-roll)>0.03)?
                                (x_trial(i,9)+0.01*0.1*sign(roll-x_trial(i,9)))
                                :roll )
                            :0.0;
        x_trial(i,10) = (curr_time >= 0.8)?
                            ((abs(static_cast<double>(x_trial(i,10))-pitch)>0.03)?
                                (x_trial(i,10)+0.01*sign(pitch-x_trial(i,10))*0.1)
                                :pitch )
                            :0.0;
        x_trial(i,11) = (curr_time >= 0.8)?
                            ((abs(static_cast<double>(x_trial(i,11))-yaw)>0.03)?
                                (x_trial(i,11)+0.01*0.5*sign(yaw-x_trial(i,11)))
                                :yaw )
                            :0.0;

    }
    // std::cout << std::setprecision(6) << "Des_Roll_rate:" << x_trial(0,9) << " Curr_roll_rate: " << qk_(9)<<std::endl;
    x_sol_N_ = x_trial; */
    // writeMatrixToFile(x_sol_N_, basePath + "x_sol_N_.txt");

    // Perform matrix operations for all states
    DM ATA = mtimes(transpose(A), A);
    DM ATx = mtimes(transpose(A), x_sol_N_); // Transpose x_sol for correct dimension alignment

    // Solve the system to find Bezier coefficients
    bez_alpha = solve(ATA, ATx);

    bez_alpha_ = bez_alpha.T();

    // Output the Bezier Coefficients Matrix
    // writeMatrixToFile(bez_alpha_, basePath + "bez_alpha.txt");
    // std::cout << "Bezier Coefficients Matrix:" << std::endl;
    // std::cout << bez_alpha_ << std::endl;
}

void NMPC::casadiToEigen(const casadi::DM &mat1, Eigen::MatrixXd &mat2)
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

void NMPC::updateState(double *qin, double *dqin, int *ind, Eigen::Matrix<double, 3, 4> toePos, Eigen::Vector4d state_vec, const int domain, const int control_tick)
{   
    memcpy(q, qin, 18 * sizeof(double));
    memcpy(dq, dqin, 18 * sizeof(double));
    memcpy(contactInd, ind, 4 * sizeof(int));
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
    //-----------------Transform XYZ to ZYX-----------------//
    //convert eul and omega to ZYX
    transform_XYZ_to_ZYX(q,dq);
    // std::cout << " [State update to inside NMPC class] " <<q[0] <<','<< q[1] << ','<<q[2]<<"\n";

    for (size_t i = 0; i < 3; i++)
    {
        qk_(i) = q[i]; //SRB COM position
        qk_(i + 3) = dq[i];//q_des_(i+3);//dq[i];//q_des_(i+3);//dq[i];//q_des_(i+3);//casadi::abs(qk_(i+3)-q_des_(i+3))>0.1 ? (qk_(i+3)+0.05*(qk_(i+3)- q_des_(i+3))) :q_des_(i+3); //SRB COM velocity
        qk_(i + 6) = q[i + 3]; //SRB orientation
        qk_(i + 9) = dq[i + 3]; // SRB omega
        
    }

    // qk_(5) = q_des_(5); // SRB COM heighSRBt
    // qk_(5) = q_des_(5); // SRB COM height
    // qk_(2) = q_des_(2); // SRB COM height
    DM z_vel_lim = 0.1;
    // qk_(5) = fmax(-z_vel_lim, fmin(qk_(5), z_vel_lim));

    // qk_(8) = q[5]; //yaw
    // qk_(11) = dq[5]; //yaw rate
    // cout << "OTHER AGENT's STATE (" << log_ID << "):   " << state_other.transpose() << std::endl;
    // vector to contain feedback for states and toePos
    // casadi::DM fbdk(1,24);
    // fbdk(Slice(),Slice(0,12)) = qk_;
    // fbdk(12) = toePos_(0,0);
    // fbdk(13) = toePos_(0,1);
    // fbdk(14) = toePos_(0,2);
    // fbdk(15) = toePos_(0,3);
    // fbdk(16) = toePos_(1,0);
    // fbdk(17) = toePos_(1,1);
    // fbdk(18) = toePos_(1,2);
    // fbdk(19) = toePos_(1,3);
    // fbdk(20) = toePos_(2,0);
    // fbdk(21) = toePos_(2,1);
    // fbdk(22) = toePos_(2,2);
    // fbdk(23) = toePos_(2,3);

    // appendMatrixToFile(fbdk, basePath + "fbdk_append.txt");

}

void NMPC::generateReferenceTrajectory()
{
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
    double vd_y = 0.5;
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

void NMPC::setPstart(Eigen::Matrix<double, 2 * NUMBER_OF_AGENTS, 1> &Pstart)
{
    Pstart_ << Pstart;
    agent_Initial_(0) = Pstart_(2 * agent_id_);
    agent_Initial_(1) = Pstart_(2 * agent_id_ + 1);
    // mpc_state_alpha_buffer_<< agent_Initial_(0), 0, agent_Initial_(1), 0;
}

void NMPC::setPobs(Eigen::Matrix<double, 2, NUMBER_OF_OBS> &Pobs_in)
{
    Pobs << Pobs_in;
}

void NMPC::setPobs_real(Eigen::Matrix<double, 2, NUMBER_OF_OBS> &Pobs_in)
{
    Pobs_real << Pobs_in;
}

void NMPC::setAgentID(size_t agent_id)
{
    agent_id_ = agent_id;
}