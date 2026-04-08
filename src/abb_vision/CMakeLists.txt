cmake_minimum_required(VERSION 3.8)
project(abb_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Generate messages FIRST - before any executable that uses them
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(abb_vision
  "msg/CscanStatus.msg"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

# Export the generated interfaces and their dependencies
ament_export_targets(abb_vision__rosidl_typesupport_cpp)
ament_export_dependencies(rosidl_typesupport_cpp)

# Battery detection node
add_executable(battery_detector src/battery_detector.cpp)
ament_target_dependencies(battery_detector
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  image_transport
  cv_bridge
)
target_link_libraries(battery_detector ${OpenCV_LIBS})

# Hand-eye calibration node
add_executable(hand_eye_calibrator src/hand_eye_calibrator.cpp)
ament_target_dependencies(hand_eye_calibrator
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  std_srvs
  image_transport
  cv_bridge
)
target_link_libraries(hand_eye_calibrator ${OpenCV_LIBS})

# Battery pose estimation node
add_executable(battery_pose_estimator src/battery_pose_estimator.cpp)
ament_target_dependencies(battery_pose_estimator
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  cv_bridge
)
target_link_libraries(battery_pose_estimator ${OpenCV_LIBS})

# Battery alignment controller node (PBVS visual servoing)
add_executable(battery_alignment_controller src/battery_alignment_controller.cpp)
ament_target_dependencies(battery_alignment_controller
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  moveit_msgs
)
ament_target_dependencies(battery_alignment_controller
  tf2
  tf2_ros
)

# C-scan trajectory generator node
add_executable(cscan_trajectory_generator src/cscan_trajectory_generator.cpp)
target_include_directories(cscan_trajectory_generator PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
# Manually link the generated type support library (plain form)
target_link_libraries(cscan_trajectory_generator
  ${CMAKE_CURRENT_BINARY_DIR}/libabb_vision__rosidl_typesupport_cpp.so
)
ament_target_dependencies(cscan_trajectory_generator
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  moveit_msgs
  std_srvs
  tf2
  tf2_ros
)

# C-scan UDP bridge node (NI ↔ ROS2)
add_executable(cscan_udp_bridge src/cscan_udp_bridge.cpp)
ament_target_dependencies(cscan_udp_bridge
  rclcpp
  std_msgs
)
target_link_libraries(cscan_udp_bridge Boost::system)

# C-scan real-time visualizer node
add_executable(cscan_visualizer src/cscan_visualizer.cpp)
target_include_directories(cscan_visualizer PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
# Manually link the generated type support library
target_link_libraries(cscan_visualizer
  ${CMAKE_CURRENT_BINARY_DIR}/libabb_vision__rosidl_typesupport_cpp.so
  ${OpenCV_LIBS}
)
ament_target_dependencies(cscan_visualizer
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  image_transport
  cv_bridge
)

# Install targets
install(TARGETS
  battery_detector
  hand_eye_calibrator
  battery_pose_estimator
  battery_alignment_controller
  cscan_trajectory_generator
  cscan_udp_bridge
  cscan_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

# Install mock data node scripts
install(PROGRAMS
  scripts/mock_data_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

ament_package()
