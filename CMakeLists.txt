cmake_minimum_required(VERSION 3.8)
project(can_test_motor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Tìm các package phụ thuộc
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# 2. Build plugin ros2_control Hardware Interface (Shared Library)
add_library(zlac_hardware_interface SHARED
  src/zlac_hardware_interface.cpp
  src/zlac_udp_driver.cpp
)
target_include_directories(zlac_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(zlac_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
# Đăng ký file plugin XML với pluginlib
pluginlib_export_plugin_description_file(hardware_interface zlac_interfaces.xml)

# 3. Build các node executable truyền thống (Cũ)
add_executable(zlac_udp_odom_node
  src/zlac_udp_odom_node.cpp)
ament_target_dependencies(zlac_udp_odom_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs)
target_include_directories(zlac_udp_odom_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(teleop_joy
  src/teleop_joy.cpp)
ament_target_dependencies(teleop_joy
  rclcpp
  sensor_msgs
  geometry_msgs
  std_srvs)
target_include_directories(teleop_joy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 4. Cài đặt các target
install(TARGETS
  zlac_hardware_interface
  zlac_udp_odom_node
  teleop_joy
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(PROGRAMS
  scripts/zlac_udp_odom_node.py
  scripts/teleop_joy.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  urdf
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()