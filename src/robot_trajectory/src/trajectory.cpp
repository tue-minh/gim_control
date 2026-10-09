#include "trajectory.hpp"
#include <cmath>

namespace gim_trajectory {

CircleTrajectory::CircleTrajectory(const Eigen::Vector3d& center, double radius, double omega)
    : center_(center), radius_(radius), omega_(omega) {}

Eigen::Vector3d CircleTrajectory::get_position(double t) const {
    // Generate a circle in the X-Y plane (or X-Z / Y-Z depending on how you map it)
    // Here we generate it in the X-Z plane as an example for an arm facing forward
    
    double current_angle = omega_ * t;
    
    Eigen::Vector3d position;
    // Assuming X is forward, Y is left, Z is up for the base frame
    // Let's draw the circle in the X-Z plane:
    position.x() = center_.x() + radius_ * std::cos(current_angle);
    position.y() = center_.y(); // Constant Y
    position.z() = center_.z() + radius_ * std::sin(current_angle);
    
    return position;
}

} // namespace gim_trajectory
