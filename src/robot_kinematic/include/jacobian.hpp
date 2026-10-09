#pragma once
#include <Eigen/Dense>
#include <vector>

namespace gim_kinematics {
    
    /**
     * @brief Compute the numeric Jacobian for the gim_arm.
     * @param q A vector containing the 3 joint angles (q1, q2, q3).
     * @return Eigen::MatrixXd The 6x3 Jacobian matrix mapping joint velocities to end-effector Cartesian velocities.
     */
    Eigen::MatrixXd jacobian(const std::vector<double>& q);

} // namespace gim_kinematics
