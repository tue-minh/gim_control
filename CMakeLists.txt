cmake_minimum_required(VERSION 3.8)
project(gim_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(gim_kinematics
  src/robot_kinematic/src/fk.cpp
  src/robot_kinematic/src/ik.cpp
  src/robot_kinematic/src/jacobian.cpp
)
target_include_directories(gim_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/robot_kinematic/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gim_kinematics PUBLIC Eigen3::Eigen)

add_library(gim_trajectory
  src/robot_trajectory/src/trajectory.cpp
)
target_include_directories(gim_trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/robot_trajectory/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gim_trajectory PUBLIC Eigen3::Eigen)

add_library(gim_control_interface
  src/robot_hardware/include/gim_control_interface.hpp
  src/robot_hardware/src/gim_control_interface.cpp
)

target_include_directories(gim_control_interface
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/robot_hardware/include>
    $<INSTALL_INTERFACE:src/robot_hardware/include>
)

ament_target_dependencies(gim_control_interface
  rclcpp
  std_msgs
  sensor_msgs
)

ament_export_dependencies(rclcpp std_msgs sensor_msgs)

add_executable(gim_control_node
  src/main.cpp
)
target_include_directories(gim_control_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_hardware/include
)
ament_target_dependencies(gim_control_node rclcpp std_msgs sensor_msgs)
target_link_libraries(gim_control_node gim_control_interface)

add_executable(trajectory_simulator_node
  src/trajectory_simulator_node.cpp
)
ament_target_dependencies(trajectory_simulator_node rclcpp sensor_msgs)
target_link_libraries(trajectory_simulator_node gim_kinematics gim_trajectory)

add_executable(impedance_simulator_node
  src/impedance_simulator_node.cpp
)
ament_target_dependencies(impedance_simulator_node rclcpp sensor_msgs)
target_link_libraries(impedance_simulator_node gim_kinematics gim_trajectory)

ament_export_libraries(gim_control_interface gim_kinematics gim_trajectory)

install(TARGETS gim_control_interface gim_kinematics gim_trajectory gim_control_node trajectory_simulator_node impedance_simulator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY src/robot_hardware/include/ src/robot_kinematic/include/ src/robot_trajectory/include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)

# Robot description (URDF + meshes) + visualization
# Source layout: src/robot_description/urdf/, src/robot_description/meshes/
# Installed as share/gim_control/urdf|meshes so URDF can use
#   package://gim_control/meshes/*.STL
install(DIRECTORY src/robot_description/urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY src/robot_description/meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(DIRECTORY src/robot_description/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY src/robot_description/rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
