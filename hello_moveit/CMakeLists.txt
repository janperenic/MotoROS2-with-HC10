cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)

# Add hello_moveit executable
add_executable(hello_moveit_node src/hello_moveit.cpp)
target_include_directories(hello_moveit_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hello_moveit_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Add linear_move executable
add_executable(linear_move_node src/linear_move.cpp)
target_include_directories(linear_move_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(linear_move_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Add linear_move executable
add_executable(ptp_move_node src/ptp_move.cpp)
target_include_directories(ptp_move_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ptp_move_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Dependencies for hello_moveit_node
ament_target_dependencies(
  hello_moveit_node
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

# Dependencies for linear_move_node
ament_target_dependencies(
  linear_move_node
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

# Dependencies for linear_move_node
ament_target_dependencies(
  ptp_move_node
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

# Install hello_moveit and linear_move executables
install(TARGETS hello_moveit_node linear_move_node ptp_move_node
  DESTINATION lib/${PROJECT_NAME})

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
