#include "jacobian.hpp"
#include "fk.hpp"

namespace gim_kinematics {

Eigen::MatrixXd jacobian(const std::vector<double>& q) {
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 3);
    if(q.size() < 3) return J;
    
    double delta = 1e-5;
    
    Eigen::Matrix4d T_curr = forward_kinematics(q);
    Eigen::Vector3d pos_curr = T_curr.block<3,1>(0,3);
    Eigen::Matrix3d R_curr = T_curr.block<3,3>(0,0);
    
    for (int i = 0; i < 3; ++i) {
        std::vector<double> q_next = q;
        q_next[i] += delta;
        
        Eigen::Matrix4d T_next = forward_kinematics(q_next);
        Eigen::Vector3d pos_next = T_next.block<3,1>(0,3);
        Eigen::Matrix3d R_next = T_next.block<3,3>(0,0);
        
        // Linear velocity Jacobian
        Eigen::Vector3d linear_vel = (pos_next - pos_curr) / delta;
        J.block<3,1>(0, i) = linear_vel;
        
        // Angular velocity Jacobian
        Eigen::Matrix3d R_delta = R_next * R_curr.transpose();
        Eigen::Matrix3d omega_skew = (R_delta - Eigen::Matrix3d::Identity()) / delta;
        Eigen::Vector3d angular_vel(omega_skew(2,1), omega_skew(0,2), omega_skew(1,0));
        
        J.block<3,1>(3, i) = angular_vel;
    }
    
    return J;
}

} // namespace gim_kinematics
