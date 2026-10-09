#pragma once
#include <Eigen/Dense>
#include <cmath>

namespace gim_kinematics {

// Helper to create a rotation matrix from fixed-axis Roll-Pitch-Yaw
inline Eigen::Matrix3d get_rotation_matrix_from_rpy(double r, double p, double y) {
    Eigen::Matrix3d Rx;
    Rx << 1, 0, 0,
          0, std::cos(r), -std::sin(r),
          0, std::sin(r), std::cos(r);
          
    Eigen::Matrix3d Ry;
    Ry << std::cos(p), 0, std::sin(p),
          0, 1, 0,
          -std::sin(p), 0, std::cos(p);
          
    Eigen::Matrix3d Rz;
    Rz << std::cos(y), -std::sin(y), 0,
          std::sin(y), std::cos(y), 0,
          0, 0, 1;
          
    return Rz * Ry * Rx;
}

// Joint constants extracted from URDF
const Eigen::Vector3d j1_xyz(0.031381, -0.48621, 0.64846);
const Eigen::Vector3d j1_rpy(1.4637, 1.364, 1.4753);
const Eigen::Vector3d j1_axis(0.99932, -0.031099, -0.019999);

const Eigen::Vector3d j2_xyz(-0.049966, 0.001555, 0.00099995);
const Eigen::Vector3d j2_rpy(-1.376255, 0.017904, 1.358142);
const Eigen::Vector3d j2_axis(0, -1, 0);

const Eigen::Vector3d j3_xyz(-0.014232, 0, -0.21358);
const Eigen::Vector3d j3_rpy(-1.570821, 1.51637, -0.000021);
const Eigen::Vector3d j3_axis(0, 0, -1);

} // namespace gim_kinematics
