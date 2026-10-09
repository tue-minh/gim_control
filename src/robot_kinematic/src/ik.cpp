#include "ik.hpp"
#include "fk.hpp"
#include "jacobian.hpp"
#include <algorithm>
#include <iostream>

namespace gim_kinematics {

std::vector<double> inverse_kinematics(const Eigen::Vector3d& target_pos, const std::vector<double>& q_guess) {
    std::vector<double> q = q_guess;
    if (q.size() < 3) q = {0.0, 0.0, 0.0};
    
    int max_iterations = 500;
    double tolerance = 1e-4;
    double lambda = 0.1; // Damping factor for DLS
    double alpha = 0.5;  // Step size multiplier
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        Eigen::Matrix4d T = forward_kinematics(q);
        Eigen::Vector3d current_pos = T.block<3,1>(0,3);
        
        Eigen::Vector3d error = target_pos - current_pos;
        if (error.norm() < tolerance) {
            break;
        }
        
        Eigen::MatrixXd J = jacobian(q);
        Eigen::MatrixXd J_v = J.block(0, 0, 3, 3); // Position Jacobian (first 3 rows)
        
        // Damped Least Squares (DLS): dq = J^T * (J * J^T + lambda^2 * I)^-1 * error
        Eigen::MatrixXd J_v_T = J_v.transpose();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
        
        Eigen::MatrixXd to_invert = J_v * J_v_T + (lambda * lambda) * I;
        Eigen::Vector3d dq = J_v_T * to_invert.inverse() * error;
        
        for (int i = 0; i < 3; ++i) {
            q[i] += alpha * dq(i);
        }
        
        // Enforce joint limits from URDF
        q[0] = std::max(-0.7121, std::min(1.0226, q[0]));
        q[1] = std::max(-0.2366, std::min(1.3435, q[1]));
        q[2] = std::max(-0.2336, std::min(1.7762, q[2]));
    }
    
    return q;
}

} // namespace gim_kinematics
