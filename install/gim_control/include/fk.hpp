#pragma once
#include <Eigen/Dense>
#include <vector>

namespace gim_kinematics {
    
    /**
     * @brief Compute the forward kinematics for the gim_arm.
     * @param q A vector containing the 3 joint angles (q1, q2, q3).
     * @return Eigen::Matrix4d The 4x4 homogeneous transformation matrix of the end-effector.
     */
    Eigen::Matrix4d forward_kinematics(const std::vector<double>& q);

} // namespace gim_kinematics
