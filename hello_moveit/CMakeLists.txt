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
find_package(eigen_stl_containers REQUIRED)

add_executable(hello_moveit_node src/hello_moveit.cpp)
add_executable(linear_move_node src/linear_move.cpp)
add_executable(ptp_move_node src/ptp_move.cpp)

target_include_directories(hello_moveit_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hello_moveit_node PUBLIC c_std_99 cxx_std_17)
target_link_libraries(hello_moveit_node eigen_stl_containers::eigen_stl_containers)
ament_target_dependencies(hello_moveit_node moveit_ros_planning_interface moveit_visual_tools rclcpp)


# Add linear_move executable

target_include_directories(linear_move_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(linear_move_node PUBLIC c_std_99 cxx_std_17)
target_link_libraries(linear_move_node eigen_stl_containers::eigen_stl_containers)
ament_target_dependencies(linear_move_node moveit_ros_planning_interface moveit_visual_tools rclcpp)


# Add ptp_move executable

target_include_directories(ptp_move_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ptp_move_node PUBLIC c_std_99 cxx_std_17)
target_link_libraries(ptp_move_node eigen_stl_containers::eigen_stl_containers)
ament_target_dependencies(ptp_move_node moveit_ros_planning_interface moveit_visual_tools rclcpp)

# Install executables
install(TARGETS hello_moveit_node linear_move_node ptp_move_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
