cmake_minimum_required(VERSION 3.8)
project(motion_planning_python_api)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)  # Add this to avoid warnings about missing rclpy

# Install the Python scripts
install(PROGRAMS
  scripts/motion_planning_python_api_tutorial.py
  scripts/motion_planning_python_api_planning_scene.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch files
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Mark the package for ament
ament_package()
