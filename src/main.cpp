#include "gim_control_interface.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ros2_gim_control::GimControlInterface>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}