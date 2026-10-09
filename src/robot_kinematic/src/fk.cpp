#include "fk.hpp"
#include "utils.hpp"
#include <cmath>

namespace gim_kinematics {

// Helper to compute transformation matrix of a single joint
Eigen::Matrix4d get_joint_transform(double theta, const Eigen::Vector3d& axis, const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy) {
    Eigen::Matrix3d R_orig = get_rotation_matrix_from_rpy(rpy(0), rpy(1), rpy(2));
    Eigen::Matrix4d T_orig = Eigen::Matrix4d::Identity();
    T_orig.block<3,3>(0,0) = R_orig;
    T_orig.block<3,1>(0,3) = xyz;
    
    // Rodrigues formula for joint rotation
    Eigen::Matrix3d K;
    K << 0, -axis(2), axis(1),
         axis(2), 0, -axis(0),
         -axis(1), axis(0), 0;
         
    Eigen::Matrix3d R_joint = Eigen::Matrix3d::Identity() + std::sin(theta) * K + (1.0 - std::cos(theta)) * (K * K);
    Eigen::Matrix4d T_joint = Eigen::Matrix4d::Identity();
    T_joint.block<3,3>(0,0) = R_joint;
    
    return T_orig * T_joint;
}

Eigen::Matrix4d forward_kinematics(const std::vector<double>& q) {
    if(q.size() < 3) return Eigen::Matrix4d::Identity();
    
    Eigen::Matrix4d T1 = get_joint_transform(q[0], j1_axis, j1_xyz, j1_rpy);
    Eigen::Matrix4d T2 = get_joint_transform(q[1], j2_axis, j2_xyz, j2_rpy);
    Eigen::Matrix4d T3 = get_joint_transform(q[2], j3_axis, j3_xyz, j3_rpy);
    
    return T1 * T2 * T3;
}

} // namespace gim_kinematics
