#include <gtest/gtest.h>
#include <Eigen/Dense>


class EulerConversionTest : public ::testing::Test {

protected:
    void get_Euler_ZYX_from_XYZ(Eigen::Matrix<double,3,1>& rpy_zyx, const Eigen::Matrix<double,3,1>& rpy_xyz){
    Eigen::Matrix<double,3,3> R = RotMat_XYZ(rpy_xyz);

    rpy_zyx(1) = asin(-R(2,0));
    double cos_theta = cos(rpy_zyx(1));
    rpy_zyx(0)= atan2(R(2,1)/cos_theta, R(2,2)/cos_theta);
    rpy_zyx(2)= atan2(R(1,0)/cos_theta, R(0,0)/cos_theta);
    };
    Eigen::Matrix<double, 3, 1> rpy_zyx;
    Eigen::Matrix<double, 3, 1> rpy_xyz;
};

// Test 1: Zero Rotation
TEST_F(EulerConversionTest, ZeroRotation) {
    rpy_xyz << 0.0, 0.0, 0.0;
    get_Euler_ZYX_from_XYZ(rpy_zyx, rpy_xyz);
    EXPECT_NEAR(rpy_zyx(0), 0.0, 1e-6);
    EXPECT_NEAR(rpy_zyx(1), 0.0, 1e-6);
    EXPECT_NEAR(rpy_zyx(2), 0.0, 1e-6);
}

// Test 2: 90-degree Rotation about X-axis
TEST_F(EulerConversionTest, NinetyDegreeRotationX) {
    rpy_xyz << M_PI / 2, 0.0, 0.0;
    get_Euler_ZYX_from_XYZ(rpy_zyx, rpy_xyz);
    EXPECT_NEAR(rpy_zyx(0), 0.0, 1e-6);
    EXPECT_NEAR(rpy_zyx(1), 0.0, 1e-6);
    EXPECT_NEAR(rpy_zyx(2), M_PI / 2, 1e-6);
}

// Test 3: 90-degree Rotation about Y-axis
TEST_F(EulerConversionTest, NinetyDegreeRotationY) {
    rpy_xyz << 0.0, M_PI / 2, 0.0;
    get_Euler_ZYX_from_XYZ(rpy_zyx, rpy_xyz);
    EXPECT_NEAR(rpy_zyx(0), 0.0, 1e-6);
    EXPECT_NEAR(rpy_zyx(1), M_PI / 2, 1e-6);
    EXPECT_NEAR(rpy_zyx(2), 0.0, 1e-6);
}

// Test 4: 90-degree Rotation about Z-axis
TEST_F(EulerConversionTest, NinetyDegreeRotationZ) {
    rpy_xyz << 0.0, 0.0, M_PI / 2;
    get_Euler_ZYX_from_XYZ(rpy_zyx, rpy_xyz);
    EXPECT_NEAR(rpy_zyx(0), M_PI / 2, 1e-6);
    EXPECT_NEAR(rpy_zyx(1), 0.0, 1e-6);
    EXPECT_NEAR(rpy_zyx(2), 0.0, 1e-6);
}

// Test 5: Gimbal Lock Scenario (Pitch = ±90 degrees)
TEST_F(EulerConversionTest, GimbalLockPositive) {
    rpy_xyz << 0.0, M_PI / 2, 0.0;
    get_Euler_ZYX_from_XYZ(rpy_zyx, rpy_xyz);
    EXPECT_NEAR(rpy_zyx(1), M_PI / 2, 1e-6);
}

TEST_F(EulerConversionTest, GimbalLockNegative) {
    rpy_xyz << 0.0, -M_PI / 2, 0.0;
    get_Euler_ZYX_from_XYZ(rpy_zyx, rpy_xyz);
    EXPECT_NEAR(rpy_zyx(1), -M_PI / 2, 1e-6);
}

