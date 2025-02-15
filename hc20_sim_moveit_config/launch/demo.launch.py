# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# Simulacija launch


from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("motoman_hc20_sim", package_name="hc20_sim_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)


# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# Uradmno dela !!!!!!!!! 

# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# from moveit_configs_utils import MoveItConfigsBuilder
# import xacro

# def generate_launch_description():
#     db_arg = DeclareLaunchArgument("warehouse_host", default_value="", description="Database connection path")

#     package_share_path = get_package_share_directory("hc20_sim_moveit_config")

#     robot_description_file_path = os.path.join(package_share_path, "config", "motoman_hc20_sim.urdf.xacro")
#     robot_description_semantic_file_path = os.path.join(package_share_path, "config", "motoman_hc20_sim.srdf")
#     trajectory_execution_file_path = os.path.join(package_share_path, "config", "moveit_controllers.yaml")
#     rviz_config_file_path = os.path.join(package_share_path, "config", "moveit.rviz")
#     kinematics_file_path = os.path.join(package_share_path, "config", "kinematics.yaml")
#     joint_limits_file_path = os.path.join(package_share_path, "config", "joint_limits.yaml")

#     robot_description_contents = xacro.process_file(robot_description_file_path).toxml()

#     warehouse_ros_config = {
#         "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
#         "warehouse_host": LaunchConfiguration("warehouse_host")
#     }

#     moveit_config = (MoveItConfigsBuilder("motoman_hc20_sim", package_name="hc20_sim_moveit_config")
#                      .robot_description(file_path=robot_description_file_path)
#                      .robot_description_semantic(file_path=robot_description_semantic_file_path)
#                      .trajectory_execution(file_path=trajectory_execution_file_path)
#                      .robot_description_kinematics(file_path=kinematics_file_path)
#                      .joint_limits(file_path=joint_limits_file_path)
#                      .planning_scene_monitor(publish_robot_description_semantic=True)
#                      .to_moveit_configs())

#     controller_manager_node = Node(
#         package="controller_manager",
#         executable="ros2_control_node",
#         parameters=[os.path.join(package_share_path, "config", "ros2_controllers.yaml")],
#         output="screen",
#     )

#     joint_state_broadcaster_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["joint_state_broadcaster"],
#         output="screen"
#     )

#     manipulator_controller_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["manipulator_controller"],
#         output="screen"
#     )

#     move_group_node = Node(
#         package="moveit_ros_move_group",
#         executable="move_group",
#         output="screen",
#         parameters=[moveit_config.to_dict(), warehouse_ros_config, {"use_sim_time": True, "use_octomap": False}],
#         arguments=["--ros-args", "--log-level", "info"],
#     )

#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         output="log",
#         arguments=["-d", rviz_config_file_path],
#         parameters=[
#             moveit_config.robot_description,
#             moveit_config.robot_description_semantic,
#             moveit_config.robot_description_kinematics,
#             moveit_config.joint_limits,
#             moveit_config.planning_pipelines,
#             warehouse_ros_config,
#         ],
#     )

#     robot_state_publisher_node = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         name="robot_state_publisher",
#         output="both",
#         parameters=[{'robot_description': robot_description_contents}]
#     )

#     return LaunchDescription([
#         db_arg,
#         controller_manager_node,
#         joint_state_broadcaster_spawner,
#         manipulator_controller_spawner,
#         robot_state_publisher_node,
#         move_group_node,
#         rviz_node,
#     ])
