#pragma once
#include <Eigen/Dense>

namespace gim_trajectory {

class CircleTrajectory {
public:
    /**
     * @brief Construct a new Circle Trajectory
     * 
     * @param center The 3D center point of the circle
     * @param radius The radius of the circle
     * @param omega The angular velocity (rad/s). Controls how fast it draws the circle.
     */
    CircleTrajectory(const Eigen::Vector3d& center, double radius, double omega);

    /**
     * @brief Get the Cartesian position at a given time t
     * 
     * @param t time in seconds
     * @return Eigen::Vector3d Cartesian position [x, y, z]
     */
    Eigen::Vector3d get_position(double t) const;

private:
    Eigen::Vector3d center_;
    double radius_;
    double omega_;
};

} // namespace gim_trajectory
