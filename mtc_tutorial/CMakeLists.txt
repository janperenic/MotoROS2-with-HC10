cmake_minimum_required(VERSION 3.8)
project(mtc_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)

# Add executable for mtc_node
add_executable(mtc_node src/mtc_node.cpp)
ament_target_dependencies(mtc_node
  "moveit_task_constructor_core"
  "rclcpp"
)

# Add executable for mtc_y_node
add_executable(mtc_y_node src/mtc_y_node.cpp)
ament_target_dependencies(mtc_y_node
  "moveit_task_constructor_core"
  "rclcpp"
)

# Add executable for minimal_mtc_tutorial
add_executable(minimal_mtc_tutorial src/minimal.cpp)
ament_target_dependencies(minimal_mtc_tutorial
  "moveit_task_constructor_core"
  "rclcpp"
)

# Include directories
target_include_directories(mtc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Include directories for mtc_y_node
target_include_directories(mtc_y_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(mtc_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(mtc_y_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Install the executables
install(TARGETS mtc_node minimal_mtc_tutorial mtc_y_node
  DESTINATION lib/${PROJECT_NAME})

# Install the launch directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Add testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
