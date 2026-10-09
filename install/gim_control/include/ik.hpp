#pragma once
#include <Eigen/Dense>
#include <vector>

namespace gim_kinematics {
    
    /**
     * @brief Compute the inverse kinematics for the gim_arm to reach a target position.
     * @param target_pos The target x, y, z position.
     * @param q_guess The initial guess for the joint angles (q1, q2, q3).
     * @return std::vector<double> The computed joint angles.
     */
    std::vector<double> inverse_kinematics(const Eigen::Vector3d& target_pos, const std::vector<double>& q_guess);

} // namespace gim_kinematics
