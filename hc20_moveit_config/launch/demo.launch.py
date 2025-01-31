# import os

# from launch import LaunchDescription
# from launch.actions import (
#     DeclareLaunchArgument,
#     IncludeLaunchDescription,
# )
# from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration

# from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterValue

# from srdfdom.srdf import SRDF

# from moveit_configs_utils.launch_utils import (
#     add_debuggable_node,
#     DeclareBooleanLaunchArg,
# )


# def generate_demo_launch(moveit_config, launch_package_path=None):
#     """
#     Launches a self-contained demo

#     launch_package_path is optional to use different launch and config packages

#     Includes:
#     * static_virtual_joint_tfs
#     * robot_state_publisher
#     * move_group
#     * moveit_rviz
#     * warehouse_db (optional)
#     * ros2_control_node + controller spawners
#     """
#     if launch_package_path is None:
#         launch_package_path = moveit_config.package_path

#     ld = LaunchDescription()
#     ld.add_action(
#         DeclareBooleanLaunchArg(
#             "db",
#             default_value=False,
#             description="By default, we do not start a database (it can be large)",
#         )
#     )
#     ld.add_action(
#         DeclareBooleanLaunchArg(
#             "debug",
#             default_value=False,
#             description="By default, we are not in debug mode",
#         )
#     )
#     ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

#     # If there are virtual joints, broadcast static tf by including virtual_joints launch
#     virtual_joints_launch = (
#         launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
#     )

#     if virtual_joints_launch.exists():
#         ld.add_action(
#             IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource(str(virtual_joints_launch)),
#             )
#         )

#     # Given the published joint states, publish tf for the robot links
#     ld.add_action(
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 str(launch_package_path / "launch/rsp.launch.py")
#             ),
#         )
#     )

#     ld.add_action(
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 str(launch_package_path / "launch/move_group.launch.py")
#             ),
#         )
#     )

#     # Run Rviz and load the default config to see the state of the move_group node
#     ld.add_action(
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 str(launch_package_path / "launch/moveit_rviz.launch.py")
#             ),
#             condition=IfCondition(LaunchConfiguration("use_rviz")),
#         )
#     )

#     # If database loading was enabled, start mongodb as well
#     ld.add_action(
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 str(launch_package_path / "launch/warehouse_db.launch.py")
#             ),
#             condition=IfCondition(LaunchConfiguration("db")),
#         )
#     )

#     # Fake joint driver
#     ld.add_action(
#         Node(
#             package="controller_manager",
#             executable="ros2_control_node",
#             parameters=[
#                 moveit_config.robot_description,
#                 str(moveit_config.package_path / "config/ros2_controllers.yaml"),
#             ],
#         )
#     )

#     ld.add_action(
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 str(launch_package_path / "launch/spawn_controllers.launch.py")
#             ),
#         )
#     )

#     return ld


# def generate_launch_description():
#     # Create MoveIt configuration using MoveItConfigsBuilder
#     from moveit_configs_utils import MoveItConfigsBuilder

#     moveit_config = MoveItConfigsBuilder("motoman_hc10dt", package_name="hc10dt_moveit_config").to_moveit_configs()

#     # Return the launch description using generate_demo_launch()
#     return generate_demo_launch(moveit_config)



# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_demo_launch


# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("motoman_hc10dt", package_name="hc10dt_moveit_config").to_moveit_configs()
#     return generate_demo_launch(moveit_config)











# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#  pilz ne dela




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

#     package_share_path = get_package_share_directory("hc10dt_moveit_config")

#     robot_description_file_path = os.path.join(package_share_path, "config", "motoman_hc10dt.urdf.xacro")

#     robot_description_semantic_file_path = os.path.join(package_share_path, "config", "motoman_hc10dt.srdf")

#     trajectory_execution_file_path = os.path.join(package_share_path, "config", "moveit_controllers.yaml")

#     rviz_config_file_path = os.path.join(package_share_path, "config", "moveit.rviz")

#     kinematics_file_path = os.path.join(package_share_path, "config", "kinematics.yaml")

#     pilz_planner_file_path = os.path.join(package_share_path, "config", "pilz_industrial_motion_planner_planning.yaml")

#     robot_description_contents = xacro.process_file(robot_description_file_path).toxml()

#     warehouse_ros_config = {
#         "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
#         "warehouse_host": LaunchConfiguration("warehouse_host")
#     }

#     moveit_config = (MoveItConfigsBuilder("motoman_hc10dt", package_name="hc10dt_moveit_config")
#                      .robot_description(file_path="config/motoman_hc10dt.urdf.xacro")
#                      .robot_description_semantic(file_path="config/motoman_hc10dt.srdf")
#                      .trajectory_execution(file_path="config/moveit_controllers.yaml")
#                      .robot_description_kinematics(file_path="config/kinematics.yaml")
#                      .planning_scene_monitor(publish_robot_description_semantic=True)
#                      .planning_pipelines(pipelines=["pilz_industrial_motion_planner", "ompl"], additional_configs=[pilz_planner_file_path])
#                      .to_moveit_configs()    
#     )

#     # Start the actual move_group node/action server
#     move_group_node = Node(
#         package="moveit_ros_move_group",
#         executable="move_group",
#         output="screen",
#         parameters=[moveit_config.to_dict(), warehouse_ros_config, {"use_octomap": False}],
#         arguments=["--ros-args", "--log-level", "info"],
#     )

#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         output="log",
#         arguments=["-d", rviz_config_file_path],
#         parameters=[
#             moveit_config.robot_description_kinematics,
#             moveit_config.planning_pipelines,
#             moveit_config.joint_limits,
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

#     return LaunchDescription(
#         [
#             db_arg,
#             move_group_node,
#             rviz_node,
#             robot_state_publisher_node,
#         ]
#     )









# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# Uradmno dela !!!!!!!!! 

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import xacro


def generate_launch_description():
    db_arg = DeclareLaunchArgument("warehouse_host", default_value="", description="Database connection path")

    package_share_path = get_package_share_directory("hc20_moveit_config")

    robot_description_file_path = os.path.join(package_share_path, "config", "motoman_hc20.urdf.xacro")

    robot_description_semantic_file_path = os.path.join(package_share_path, "config", "motoman_hc20.srdf")

    trajectory_execution_file_path = os.path.join(package_share_path, "config", "moveit_controllers.yaml")

    rviz_config_file_path = os.path.join(package_share_path, "config", "moveit.rviz")

    kinematics_file_path = os.path.join(package_share_path, "config", "kinematics.yaml")

    robot_description_contents = xacro.process_file(robot_description_file_path).toxml()

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": LaunchConfiguration("warehouse_host")
    }

    # moveit_config = (
    #     MoveItConfigsBuilder("motoman_hc10dt", package_name="hc10dt_moveit_config")
    #     .robot_description(file_path=robot_description_file_path)
    #     .robot_description_semantic(file_path=robot_description_semantic_file_path)
    #     .trajectory_execution(file_path=trajectory_execution_file_path)
    #     .robot_description_kinematics(file_path=kinematics_file_path)
    #     .to_moveit_configs()
    # )


    # .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"]) # pilz_industrial_motion_planner, ompl

    #                      .planning_pipelines(
    #                     pipelines=["ompl", "pilz_industrial_motion_planner"],
    #                     default_planning_pipeline="pilz_industrial_motion_planner",
    # )

    moveit_config = (MoveItConfigsBuilder("motoman_hc20", package_name="hc20_moveit_config")
                     .robot_description(file_path="config/motoman_hc20.urdf.xacro")
                     .robot_description_semantic(file_path="config/motoman_hc20.srdf")
                     .trajectory_execution(file_path="config/moveit_controllers.yaml")
                     .robot_description_kinematics(file_path="config/kinematics.yaml")
                     .planning_scene_monitor(publish_robot_description_semantic=True)
                    #  .planning_pipelines(pipelines=["pilz_industrial_motion_planner"]) # pilz_industrial_motion_planner, ompl
                     .to_moveit_configs()    
    )
    # Starts Pilz Industrial Motion Planner MoveGroupSequenceAction and MoveGroupSequenceService servers
    # move_group_capabilities = {
    #     "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService"
    #     }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), warehouse_ros_config, {"use_octomap": False}], #, move_group_capabilities
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file_path],
        parameters=[
            # moveit_config.robot_description,
            # moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{'robot_description': robot_description_contents}]
    )

    return LaunchDescription(
        [
            db_arg,
            move_group_node,
            rviz_node,
            robot_state_publisher_node,
        ]
    )










# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# przge se samo mas simulacijo in real robota v rvizu. Skripta dela samo do prvega giba oz
# se tudi prvega giba ne nardi ce misli da je simulacija  ??




# import os
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# from moveit_configs_utils import MoveItConfigsBuilder
# import xacro


# def generate_launch_description():
#     # Path to the package directory
#     package_share_path = get_package_share_directory("hc10dt_moveit_config")

#     # URDF/Xacro
#     robot_description_file_path = os.path.join(package_share_path, "config", "motoman_hc10dt.urdf.xacro")
#     robot_description_config = xacro.process_file(robot_description_file_path)
#     robot_description = {"robot_description": robot_description_config.toxml()}

#     # MoveIt configuration
#     moveit_config = (
#         MoveItConfigsBuilder("motoman_hc10dt", package_name="hc10dt_moveit_config")
#         .robot_description(file_path=robot_description_file_path)
#         .robot_description_semantic(file_path=os.path.join(package_share_path, "config", "motoman_hc10dt.srdf"))
#         .trajectory_execution(file_path=os.path.join(package_share_path, "config", "moveit_controllers.yaml"))
#         .robot_description_kinematics(file_path=os.path.join(package_share_path, "config", "kinematics.yaml"))
#         .to_moveit_configs()
#     )

#     # Planning Scene Monitor parameters
#     planning_scene_monitor_parameters = {
#         "publish_planning_scene": True,
#         "publish_geometry_updates": True,
#         "publish_state_updates": True,
#         "publish_transforms_updates": True,
#         "publish_robot_description": True,
#         "publish_robot_description_semantic": True,
#     }

#     # Move Group Node
#     run_move_group_node = Node(
#         package="moveit_ros_move_group",
#         executable="move_group",
#         output="screen",
#         parameters=[
#             moveit_config.to_dict(),
#             planning_scene_monitor_parameters,
#         ],
#     )

#     # RViz Node with MoveIt configuration
#     rviz_config_file_path = os.path.join(package_share_path, "config", "moveit.rviz")
#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         output="log",
#         arguments=["-d", rviz_config_file_path],
#         parameters=[
#             moveit_config.robot_description,
#             moveit_config.robot_description_semantic,
#             moveit_config.planning_pipelines,
#             moveit_config.robot_description_kinematics,
#         ],
#     )

#     # Robot State Publisher Node
#     robot_state_publisher_node = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         name="robot_state_publisher",
#         output="both",
#         parameters=[robot_description],
#     )

#     # ros2_control Node using FakeSystem as hardware
#     ros2_controllers_path = os.path.join(package_share_path, "config", "ros2_controllers.yaml")
#     ros2_control_node = Node(
#         package="controller_manager",
#         executable="ros2_control_node",
#         parameters=[robot_description, ros2_controllers_path],  # Ensure ros_control tag in xacro file
#         output="both",
#     )

#     # Joint State Publisher Node
#     joint_state_publisher_node = Node(
#         package="joint_state_publisher",
#         executable="joint_state_publisher",
#         name="joint_state_publisher",
#         parameters=[
#             {
#                 "robot_description": robot_description_config.toxml(),
#                 "rate": 43,
#             }
#         ],
#     )

#     return LaunchDescription(
#         [
#             rviz_node,
#             robot_state_publisher_node,
#             run_move_group_node,
#             ros2_control_node,
#             joint_state_publisher_node,
#         ]
#     )

















#  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#  narjena po temu :https://github.com/moveit/moveit_resources/blob/ros2/panda_moveit_config/launch/demo.launch.py samo
#  da je js ne morm usposobt


# import os
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterValue
# from launch.substitutions import Command
# from ament_index_python.packages import get_package_share_directory


# def generate_launch_description():
#     # Define paths to URDF, SRDF, and RViz config
#     package_share_path = get_package_share_directory('hc10dt_moveit_config')
#     urdf_path = os.path.join(package_share_path, 'urdf', 'hc10dt.urdf.xacro')
#     srdf_path = os.path.join(package_share_path, 'config', 'motoman_hc10dt.srdf')
#     rviz_config_file_path = os.path.join(package_share_path, 'config', 'moveit.rviz')

#     # Load robot description (URDF) using xacro
#     robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

#     # Load SRDF explicitly
#     with open(srdf_path, 'r') as srdf_file:
#         semantic_content = srdf_file.read()

#     # Define the Move Group Node
#     move_group_node = Node(
#         package='moveit_ros_move_group',
#         executable='move_group',
#         output='screen',
#         parameters=[{
#             'robot_description': robot_description,
#             'robot_description_semantic': semantic_content,
#             'publish_robot_description': True,
#             'publish_robot_description_semantic': True,
#             # Add more parameters here if needed
#         }]
#     )

#     # Define the Robot State Publisher Node
#     rsp_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         respawn=True,
#         output='screen',
#         parameters=[{
#             'robot_description': robot_description,
#             'publish_frequency': 15.0
#         }]
#     )

#     # Define the RViz Node
#     rviz_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         output='log',
#         arguments=['-d', rviz_config_file_path]
#     )

#     # Return the Launch Description with all nodes
#     return LaunchDescription([
#         move_group_node,
#         rsp_node,
#         rviz_node,
#     ])










# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

