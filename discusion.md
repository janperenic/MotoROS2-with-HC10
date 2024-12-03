i Have a problem with moveing the robot in rviz2. I get joint position from real robot and also in simulation. The planing stage works but when I try to move the robot I cant. Does anybody know why this is happening?

[move_group-3] [WARN] [1731251918.411273694] [moveit_ros.trajectory_execution_manager]: Failed to validate trajectory: couldn't receive full current joint state within 1s
[move_group-3] [INFO] [1731251918.411468533] [moveit_move_group_default_capabilities.move_action_capability]: Solution found but controller failed during execution
[rviz2-4] [INFO] [1731251918.412526932] [move_group_interface]: Plan and Execute request aborted
[rviz2-4] [ERROR] [1731251918.413016384] [move_group_interface]: MoveGroupInterface::move() failed or timeout reached

I also have a problem that i am geting this message all the time:

[spawner-5] [WARN] [1731251924.933122738] [spawner_follow_joint_trajectory]: Could not contact service /controller_manager/list_controllers
[spawner-5] [INFO] [1731251924.933988833] [spawner_follow_joint_trajectory]: waiting for service /controller_manager/list_controllers to become available...

I made launch files with setup assiente. I also tried ros2-starter-for-yaskawa-robots but i have the same error.
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!(dodaj link za github od tega ros2-starter)!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

My launch file:
!!!!!!!!!!!!!!!!!!!!!!!!!!!!

ros2_controllers.yaml:
!!!!!!!!!!!!!!!!!!!!!!

moveit_controllers.yaml:
!!!!!!!!!!!!!!!!!!!!!!!

Full debug log from control start to the finish when the error occurs:

1731251903.5633657 [INFO] [launch]: All log files can be found below /home/janperenic/.ros/log/2024-11-10-16-18-23-562586-janperenic-OMEN-282650
1731251903.5635130 [INFO] [launch]: Default logging verbosity is set to INFO
1731251904.5322800 [INFO] [static_transform_publisher-1]: process started with pid [282681]
1731251904.5324986 [INFO] [robot_state_publisher-2]: process started with pid [282683]
1731251904.5326040 [INFO] [move_group-3]: process started with pid [282685]
1731251904.5327003 [INFO] [rviz2-4]: process started with pid [282687]
1731251904.5327928 [INFO] [spawner-5]: process started with pid [282689]
1731251904.5328832 [INFO] [spawner-6]: process started with pid [282691]
1731251904.5632803 [static_transform_publisher-1] [INFO] [1731251904.562899378] [static_transform_publisher0]: Spinning until stopped - publishing transform
1731251904.5635054 [static_transform_publisher-1] translation: ('0.000000', '0.000000', '0.000000')
1731251904.5635886 [static_transform_publisher-1] rotation: ('0.000000', '0.000000', '0.000000', '1.000000')
1731251904.5636656 [static_transform_publisher-1] from 'world' to 'base_link'
1731251904.9662881 [rviz2-4] [INFO] [1731251904.965944074] [rviz2]: Stereo is NOT SUPPORTED
1731251904.9667206 [rviz2-4] [INFO] [1731251904.966049549] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
1731251904.9903302 [rviz2-4] [INFO] [1731251904.990052671] [rviz2]: Stereo is NOT SUPPORTED
1731251905.1007957 [rviz2-4] Warning: class_loader.impl: SEVERE WARNING!!! A namespace collision has occurred with plugin factory for class rviz_default_plugins::displays::InteractiveMarkerDisplay. New factory will OVERWRITE existing one. This situation occurs when libraries containing plugins are directly linked against an executable (the one running right now generating this message). Please separate plugins out into their own library or just don't link against the library and use either class_loader::ClassLoader/MultiLibraryClassLoader to open.
1731251905.1009841 [rviz2-4]          at line 253 in /opt/ros/humble/include/class_loader/class_loader/class_loader_core.hpp
1731251908.1244886 [rviz2-4] [ERROR] [1731251908.124006391] [moveit_ros_visualization.motion_planning_frame]: Action server: /recognize_objects not available
1731251908.1366892 [rviz2-4] [INFO] [1731251908.136411907] [moveit_ros_visualization.motion_planning_frame]: MoveGroup namespace changed: / -> . Reloading params.
1731251908.2493341 [rviz2-4] [INFO] [1731251908.249007651] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.0640194 seconds
1731251908.2495289 [rviz2-4] [INFO] [1731251908.249047815] [moveit_robot_model.robot_model]: Loading robot model 'motoman_hc10dt'...
1731251908.3047600 [rviz2-4] [INFO] [1731251908.304463392] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
1731251908.3053455 [rviz2-4] [INFO] [1731251908.305066473] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/monitored_planning_scene'
1731251908.4979236 [rviz2-4] [INFO] [1731251908.497561975] [interactive_marker_display_110660297430144]: Connected on namespace: /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic
1731251908.5051236 [rviz2-4] [INFO] [1731251908.504862921] [moveit_ros_visualization.motion_planning_frame]: group manipulator
1731251908.5053177 [rviz2-4] [INFO] [1731251908.504884070] [moveit_ros_visualization.motion_planning_frame]: Constructing new MoveGroup connection for group 'manipulator' in namespace ''
1731251908.5141654 [rviz2-4] [INFO] [1731251908.513806983] [interactive_marker_display_110660297430144]: Sending request for interactive markers
1731251908.5146472 [rviz2-4] [INFO] [1731251908.514240601] [move_group_interface]: Ready to take commands for planning group manipulator.
1731251908.5153017 [rviz2-4] [INFO] [1731251908.515129899] [moveit_ros_visualization.motion_planning_frame]: group manipulator
1731251908.5487952 [rviz2-4] [INFO] [1731251908.548215642] [interactive_marker_display_110660297430144]: Service response received for initialization
1731251916.3498521 [rviz2-4] [INFO] [1731251916.349024935] [move_group_interface]: Plan and Execute request accepted
1731251918.4129479 [rviz2-4] [INFO] [1731251918.412526932] [move_group_interface]: Plan and Execute request aborted
1731251918.4133170 [rviz2-4] [ERROR] [1731251918.413016384] [move_group_interface]: MoveGroupInterface::move() failed or timeout reached

Motors2 version & ROS2 distribution:
0.1.2
Humble

Robot Controller & Robot:
YRC1000
HC10





















se prav mam tezave z unim grafom. 
robot je prvo splaniru in pol je robot se izvedu ampak mam nakonc se nek error.









janperenic@janperenic-OMEN:~/moveit2$ ros2 launch hc10dt_moveit_config demo.launch.py 
[INFO] [launch]: All log files can be found below /home/janperenic/.ros/log/2024-11-20-15-56-09-358892-janperenic-OMEN-114392
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [move_group-1]: process started with pid [114393]
[INFO] [rviz2-2]: process started with pid [114395]
[INFO] [robot_state_publisher-3]: process started with pid [114397]
[robot_state_publisher-3] [INFO] [1732114569.594887459] [robot_state_publisher]: got segment base
[robot_state_publisher-3] [INFO] [1732114569.595091440] [robot_state_publisher]: got segment base_link
[robot_state_publisher-3] [INFO] [1732114569.595099283] [robot_state_publisher]: got segment link_1
[robot_state_publisher-3] [INFO] [1732114569.595104090] [robot_state_publisher]: got segment link_2
[robot_state_publisher-3] [INFO] [1732114569.595108898] [robot_state_publisher]: got segment link_3
[robot_state_publisher-3] [INFO] [1732114569.595113155] [robot_state_publisher]: got segment link_4
[robot_state_publisher-3] [INFO] [1732114569.595117222] [robot_state_publisher]: got segment link_5
[robot_state_publisher-3] [INFO] [1732114569.595121959] [robot_state_publisher]: got segment link_6
[robot_state_publisher-3] [INFO] [1732114569.595126407] [robot_state_publisher]: got segment tool0
[move_group-1] [INFO] [1732114569.604070435] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.0235261 seconds
[move_group-1] [INFO] [1732114569.604134729] [moveit_robot_model.robot_model]: Loading robot model 'motoman_hc10dt'...
[move_group-1] [INFO] [1732114569.640956340] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Publishing maintained planning scene on 'monitored_planning_scene'
[move_group-1] [INFO] [1732114569.641143103] [moveit.ros_planning_interface.moveit_cpp]: Listening to 'joint_states' for joint states
[move_group-1] [INFO] [1732114569.641875791] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[move_group-1] [INFO] [1732114569.642119497] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/attached_collision_object' for attached collision objects
[move_group-1] [INFO] [1732114569.642132187] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[move_group-1] [INFO] [1732114569.642328175] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[move_group-1] [INFO] [1732114569.642339453] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[move_group-1] [INFO] [1732114569.642559090] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[move_group-1] [INFO] [1732114569.642757281] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[move_group-1] [WARN] [1732114569.642944936] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[move_group-1] [ERROR] [1732114569.642956114] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[move_group-1] [INFO] [1732114569.644864830] [moveit.ros_planning_interface.moveit_cpp]: Loading planning pipeline 'ompl'
[move_group-1] [INFO] [1732114569.654970593] [moveit.ros_planning.planning_pipeline]: Using planning interface 'OMPL'
[move_group-1] [INFO] [1732114569.657236799] [moveit_ros.add_time_optimal_parameterization]: Param 'ompl.path_tolerance' was not set. Using default value: 0.100000
[move_group-1] [INFO] [1732114569.657271205] [moveit_ros.add_time_optimal_parameterization]: Param 'ompl.resample_dt' was not set. Using default value: 0.100000
[move_group-1] [INFO] [1732114569.657276173] [moveit_ros.add_time_optimal_parameterization]: Param 'ompl.min_angle_change' was not set. Using default value: 0.001000
[move_group-1] [INFO] [1732114569.657292950] [moveit_ros.fix_workspace_bounds]: Param 'ompl.default_workspace_bounds' was not set. Using default value: 10.000000
[move_group-1] [INFO] [1732114569.657306572] [moveit_ros.fix_start_state_bounds]: Param 'ompl.start_state_max_bounds_error' was set to 0.100000
[move_group-1] [INFO] [1732114569.657312652] [moveit_ros.fix_start_state_bounds]: Param 'ompl.start_state_max_dt' was not set. Using default value: 0.500000
[move_group-1] [INFO] [1732114569.657321727] [moveit_ros.fix_start_state_collision]: Param 'ompl.start_state_max_dt' was not set. Using default value: 0.500000
[move_group-1] [INFO] [1732114569.657327526] [moveit_ros.fix_start_state_collision]: Param 'ompl.jiggle_fraction' was set to 0.050000
[move_group-1] [INFO] [1732114569.657332394] [moveit_ros.fix_start_state_collision]: Param 'ompl.max_sampling_attempts' was not set. Using default value: 100
[move_group-1] [INFO] [1732114569.657439127] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Add Time Optimal Parameterization'
[move_group-1] [INFO] [1732114569.657444296] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Resolve constraint frames to robot links'
[move_group-1] [INFO] [1732114569.657447501] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Workspace Bounds'
[move_group-1] [INFO] [1732114569.657451137] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Bounds'
[move_group-1] [INFO] [1732114569.657454322] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State In Collision'
[move_group-1] [INFO] [1732114569.657459160] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Path Constraints'
[move_group-1] [INFO] [1732114569.659263697] [moveit.ros_planning_interface.moveit_cpp]: Loading planning pipeline 'pilz_industrial_motion_planner'
[move_group-1] [INFO] [1732114569.661265254] [moveit.pilz_industrial_motion_planner.joint_limits_aggregator]: Reading limits from namespace robot_description_planning
[move_group-1] [INFO] [1732114569.663076943] [moveit.pilz_industrial_motion_planner]: Available plugins: pilz_industrial_motion_planner/PlanningContextLoaderCIRC pilz_industrial_motion_planner/PlanningContextLoaderLIN pilz_industrial_motion_planner/PlanningContextLoaderPTP 
[move_group-1] [INFO] [1732114569.663088582] [moveit.pilz_industrial_motion_planner]: About to load: pilz_industrial_motion_planner/PlanningContextLoaderCIRC
[move_group-1] [INFO] [1732114569.663935716] [moveit.pilz_industrial_motion_planner]: Registered Algorithm [CIRC]
[move_group-1] [INFO] [1732114569.663945221] [moveit.pilz_industrial_motion_planner]: About to load: pilz_industrial_motion_planner/PlanningContextLoaderLIN
[move_group-1] [INFO] [1732114569.664508265] [moveit.pilz_industrial_motion_planner]: Registered Algorithm [LIN]
[move_group-1] [INFO] [1732114569.664516919] [moveit.pilz_industrial_motion_planner]: About to load: pilz_industrial_motion_planner/PlanningContextLoaderPTP
[move_group-1] [INFO] [1732114569.665046498] [moveit.pilz_industrial_motion_planner]: Registered Algorithm [PTP]
[move_group-1] [INFO] [1732114569.665056344] [moveit.ros_planning.planning_pipeline]: Using planning interface 'Pilz Industrial Motion Planner'
[move_group-1] [INFO] [1732114569.665527117] [moveit.ros_planning_interface.moveit_cpp]: Loading planning pipeline 'chomp'
[move_group-1] [ERROR] [1732114569.666464348] [moveit.ros_planning.planning_pipeline]: Exception while loading planner 'chomp_interface/CHOMPPlanner': According to the loaded plugin descriptions the class chomp_interface/CHOMPPlanner with base class type planning_interface::PlannerManager does not exist. Declared types are  ompl_interface/OMPLPlanner pilz_industrial_motion_planner/CommandPlannerAvailable plugins: ompl_interface/OMPLPlanner, pilz_industrial_motion_planner/CommandPlanner
[move_group-1] [INFO] [1732114569.667323061] [moveit_ros.add_time_optimal_parameterization]: Param 'chomp.path_tolerance' was not set. Using default value: 0.100000
[move_group-1] [INFO] [1732114569.667331254] [moveit_ros.add_time_optimal_parameterization]: Param 'chomp.resample_dt' was not set. Using default value: 0.100000
[move_group-1] [INFO] [1732114569.667334840] [moveit_ros.add_time_optimal_parameterization]: Param 'chomp.min_angle_change' was not set. Using default value: 0.001000
[move_group-1] [INFO] [1732114569.667354882] [moveit_ros.fix_workspace_bounds]: Param 'chomp.default_workspace_bounds' was not set. Using default value: 10.000000
[move_group-1] [INFO] [1732114569.667365630] [moveit_ros.fix_start_state_bounds]: Param 'chomp.start_state_max_bounds_error' was set to 0.100000
[move_group-1] [INFO] [1732114569.667369456] [moveit_ros.fix_start_state_bounds]: Param 'chomp.start_state_max_dt' was not set. Using default value: 0.500000
[move_group-1] [INFO] [1732114569.667377699] [moveit_ros.fix_start_state_collision]: Param 'chomp.start_state_max_dt' was not set. Using default value: 0.500000
[move_group-1] [INFO] [1732114569.667381766] [moveit_ros.fix_start_state_collision]: Param 'chomp.jiggle_fraction' was set to 0.050000
[move_group-1] [INFO] [1732114569.667385702] [moveit_ros.fix_start_state_collision]: Param 'chomp.max_sampling_attempts' was not set. Using default value: 100
[move_group-1] [INFO] [1732114569.667395799] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Add Time Optimal Parameterization'
[move_group-1] [INFO] [1732114569.667398794] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Resolve constraint frames to robot links'
[move_group-1] [INFO] [1732114569.667401638] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Workspace Bounds'
[move_group-1] [INFO] [1732114569.667405074] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Bounds'
[move_group-1] [INFO] [1732114569.667407478] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State In Collision'
[move_group-1] [INFO] [1732114569.667411043] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Path Constraints'
[move_group-1] [ERROR] [1732114569.667740027] [moveit.ros_planning_interface.moveit_cpp]: Failed to initialize planning pipeline 'chomp'.
[move_group-1] [INFO] [1732114569.689489708] [moveit.plugins.moveit_simple_controller_manager]: Added FollowJointTrajectory controller for follow_joint_trajectory
[move_group-1] [INFO] [1732114569.689712630] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-1] [INFO] [1732114569.689732662] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-1] [INFO] [1732114569.690088370] [moveit_ros.trajectory_execution_manager]: Trajectory execution is managing controllers
[move_group-1] [INFO] [1732114569.690103154] [move_group.move_group]: MoveGroup debug mode is ON
[move_group-1] [INFO] [1732114569.702036772] [move_group.move_group]: 
[move_group-1] 
[move_group-1] ********************************************************
[move_group-1] * MoveGroup using: 
[move_group-1] *     - ApplyPlanningSceneService
[move_group-1] *     - ClearOctomapService
[move_group-1] *     - CartesianPathService
[move_group-1] *     - ExecuteTrajectoryAction
[move_group-1] *     - GetPlanningSceneService
[move_group-1] *     - KinematicsService
[move_group-1] *     - MoveAction
[move_group-1] *     - MotionPlanService
[move_group-1] *     - QueryPlannersService
[move_group-1] *     - StateValidationService
[move_group-1] ********************************************************
[move_group-1] 
[move_group-1] [INFO] [1732114569.702096869] [moveit_move_group_capabilities_base.move_group_context]: MoveGroup context using planning plugin ompl_interface/OMPLPlanner
[move_group-1] [INFO] [1732114569.702105944] [moveit_move_group_capabilities_base.move_group_context]: MoveGroup context initialization complete
[move_group-1] Loading 'move_group/ApplyPlanningSceneService'...
[move_group-1] Loading 'move_group/ClearOctomapService'...
[move_group-1] Loading 'move_group/MoveGroupCartesianPathService'...
[move_group-1] Loading 'move_group/MoveGroupExecuteTrajectoryAction'...
[move_group-1] Loading 'move_group/MoveGroupGetPlanningSceneService'...
[move_group-1] Loading 'move_group/MoveGroupKinematicsService'...
[move_group-1] Loading 'move_group/MoveGroupMoveAction'...
[move_group-1] Loading 'move_group/MoveGroupPlanService'...
[move_group-1] Loading 'move_group/MoveGroupQueryPlannersService'...
[move_group-1] Loading 'move_group/MoveGroupStateValidationService'...
[move_group-1] 
[move_group-1] You can start planning now!
[move_group-1] 
[rviz2-2] [INFO] [1732114569.850640350] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-2] [INFO] [1732114569.850825941] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
[rviz2-2] [INFO] [1732114569.868951032] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-2] Warning: class_loader.impl: SEVERE WARNING!!! A namespace collision has occurred with plugin factory for class rviz_default_plugins::displays::InteractiveMarkerDisplay. New factory will OVERWRITE existing one. This situation occurs when libraries containing plugins are directly linked against an executable (the one running right now generating this message). Please separate plugins out into their own library or just don't link against the library and use either class_loader::ClassLoader/MultiLibraryClassLoader to open.
[rviz2-2]          at line 253 in /opt/ros/humble/include/class_loader/class_loader/class_loader_core.hpp
[rviz2-2] [ERROR] [1732114572.956672772] [moveit_ros_visualization.motion_planning_frame]: Action server: /recognize_objects not available
[rviz2-2] [INFO] [1732114572.968170806] [moveit_ros_visualization.motion_planning_frame]: MoveGroup namespace changed: / -> . Reloading params.
[rviz2-2] [INFO] [1732114573.040175715] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.0195664 seconds
[rviz2-2] [INFO] [1732114573.040245438] [moveit_robot_model.robot_model]: Loading robot model 'motoman_hc10dt'...
[rviz2-2] [INFO] [1732114573.100962354] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[rviz2-2] [INFO] [1732114573.101653741] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/monitored_planning_scene'
[rviz2-2] [INFO] [1732114573.329440664] [interactive_marker_display_101501286873456]: Connected on namespace: /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic
[move_group-1] [WARN] [1732114573.334236718] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Unable to transform object from frame 'r1/base' to planning frame'base_link' (Could not find a connection between 'base_link' and 'r1/base' because they are not part of the same tree.Tf has two or more unconnected trees.)
[move_group-1] [WARN] [1732114573.334282422] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Unable to transform object from frame 'world' to planning frame'base_link' (Could not find a connection between 'base_link' and 'world' because they are not part of the same tree.Tf has two or more unconnected trees.)
[move_group-1] [WARN] [1732114573.334292779] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Unable to transform object from frame 'r1/flange' to planning frame'base_link' (Could not find a connection between 'base_link' and 'r1/flange' because they are not part of the same tree.Tf has two or more unconnected trees.)
[move_group-1] [WARN] [1732114573.334300231] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Unable to transform object from frame 'r1/tool0' to planning frame'base_link' (Could not find a connection between 'base_link' and 'r1/tool0' because they are not part of the same tree.Tf has two or more unconnected trees.)
[move_group-1] [WARN] [1732114573.334307453] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Unable to transform object from frame 'r1/tcp_15' to planning frame'base_link' (Could not find a connection between 'base_link' and 'r1/tcp_15' because they are not part of the same tree.Tf has two or more unconnected trees.)
[rviz2-2] [INFO] [1732114573.335085801] [moveit_ros_visualization.motion_planning_frame]: group manipulator
[rviz2-2] [INFO] [1732114573.335103420] [moveit_ros_visualization.motion_planning_frame]: Constructing new MoveGroup connection for group 'manipulator' in namespace ''
[rviz2-2] [INFO] [1732114573.342973943] [move_group_interface]: Ready to take commands for planning group manipulator.
[rviz2-2] [INFO] [1732114573.344037997] [moveit_ros_visualization.motion_planning_frame]: group manipulator
[rviz2-2] [INFO] [1732114573.344630643] [interactive_marker_display_101501286873456]: Sending request for interactive markers
[rviz2-2] [INFO] [1732114573.377712283] [interactive_marker_display_101501286873456]: Service response received for initialization
[rviz2-2] [INFO] [1732114584.785945070] [move_group_interface]: MoveGroup action client/server ready
[move_group-1] [INFO] [1732114584.786967816] [moveit_move_group_default_capabilities.move_action_capability]: Received request
[rviz2-2] [INFO] [1732114584.787224318] [move_group_interface]: Planning request accepted
[move_group-1] [INFO] [1732114584.787236448] [moveit_move_group_default_capabilities.move_action_capability]: executing..
[move_group-1] [WARN] [1732114584.804238224] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Unable to transform object from frame 'r1/base' to planning frame'base_link' (Could not find a connection between 'base_link' and 'r1/base' because they are not part of the same tree.Tf has two or more unconnected trees.)
[move_group-1] [WARN] [1732114584.804295459] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Unable to transform object from frame 'world' to planning frame'base_link' (Could not find a connection between 'base_link' and 'world' because they are not part of the same tree.Tf has two or more unconnected trees.)
[move_group-1] [WARN] [1732114584.804307719] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Unable to transform object from frame 'r1/flange' to planning frame'base_link' (Could not find a connection between 'base_link' and 'r1/flange' because they are not part of the same tree.Tf has two or more unconnected trees.)
[move_group-1] [WARN] [1732114584.804388011] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Unable to transform object from frame 'r1/tool0' to planning frame'base_link' (Could not find a connection between 'base_link' and 'r1/tool0' because they are not part of the same tree.Tf has two or more unconnected trees.)
[move_group-1] [WARN] [1732114584.804396495] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Unable to transform object from frame 'r1/tcp_15' to planning frame'base_link' (Could not find a connection between 'base_link' and 'r1/tcp_15' because they are not part of the same tree.Tf has two or more unconnected trees.)
[move_group-1] [INFO] [1732114584.804498043] [moveit_move_group_default_capabilities.move_action_capability]: Planning request received for MoveGroup action. Forwarding to planning pipeline.
[move_group-1] [INFO] [1732114584.804580899] [moveit_move_group_capabilities_base.move_group_capability]: Using planning pipeline 'pilz_industrial_motion_planner'
[move_group-1] [INFO] [1732114584.804673472] [moveit.pilz_industrial_motion_planner.trajectory_generator_ptp]: Initialized Point-to-Point Trajectory Generator.
[move_group-1] [INFO] [1732114584.804724676] [moveit.pilz_industrial_motion_planner.trajectory_generator]: Generating PTP trajectory...
[move_group-1] [INFO] [1732114584.805673770] [moveit_move_group_default_capabilities.move_action_capability]: Motion plan was computed successfully.
[rviz2-2] [INFO] [1732114584.806191643] [move_group_interface]: Planning request complete!
[rviz2-2] [INFO] [1732114584.806876762] [move_group_interface]: time taken to generate plan: 0.000292091 seconds
[move_group-1] [INFO] [1732114586.534683611] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Received goal request
[move_group-1] [INFO] [1732114586.534827228] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Execution request received
[move_group-1] [INFO] [1732114586.534887057] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-1] [INFO] [1732114586.534907300] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[rviz2-2] [INFO] [1732114586.534912308] [move_group_interface]: Execute request accepted
[move_group-1] [INFO] [1732114586.534998451] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 0.01
[move_group-1] [INFO] [1732114586.540882605] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-1] [INFO] [1732114586.540939990] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-1] [INFO] [1732114586.540957439] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-1] [INFO] [1732114586.541080562] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to follow_joint_trajectory
[move_group-1] [INFO] [1732114586.570922692] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: follow_joint_trajectory started execution
[move_group-1] [INFO] [1732114586.570979556] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[move_group-1] [WARN] [1732114589.076368877] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'follow_joint_trajectory' failed with error unknown error: Final position was outside tolerance. Check robot safety-limits that could be inhibiting motion. [joint_3: 0.0000 deviation] [joint_5: 0.0000 deviation]
[move_group-1] [WARN] [1732114589.076444883] [moveit_ros.trajectory_execution_manager]: Controller handle follow_joint_trajectory reports status ABORTED
[move_group-1] [INFO] [1732114589.076457794] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status ABORTED ...
[move_group-1] [INFO] [1732114589.076523513] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Execution completed: ABORTED
[rviz2-2] [INFO] [1732114589.076854281] [move_group_interface]: Execute request aborted
[rviz2-2] [ERROR] [1732114589.076949709] [move_group_interface]: MoveGroupInterface::execute() failed or timeout reached












errror od scrite:
janperenic@janperenic-OMEN:~/moveit2$ ros2 run hello_moveit ptp_move_node
[ERROR] [1732114716.351273585] [hello_moveit]: Could not find parameter robot_description_semantic and did not receive robot_description_semantic via std_msgs::msg::String subscription within 10.000000 seconds.
Error:   Could not parse the SRDF XML File. Error=XML_ERROR_EMPTY_DOCUMENT ErrorID=13 (0xd) Line number=0
         at line 715 in ./src/model.cpp
[ERROR] [1732114716.369310078] [moveit_rdf_loader.rdf_loader]: Unable to parse SRDF
[FATAL] [1732114716.369419763] [move_group_interface]: Unable to construct robot model. Please make sure all needed information is on the parameter server.
terminate called after throwing an instance of 'std::runtime_error'
  what():  Unable to construct robot model. Please make sure all needed information is on the parameter server.
[ros2run]: Aborted



















janperenic@janperenic-OMEN:~/moveit2$ ros2 run hello_moveit hello_moveit_node
[INFO] [1732790183.976530666] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 1.02152 seconds
[INFO] [1732790183.976582603] [moveit_robot_model.robot_model]: Loading robot model 'motoman_hc10dt'...
[WARN] [1732790183.983306583] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1732790183.994349193] [move_group_interface]: Ready to take commands for planning group manipulator.
[INFO] [1732790183.994958971] [hello_moveit.remote_control]: RemoteControl Ready.
[INFO] [1732790184.007419256] [hello_moveit]: Visualizing plan (Cartesian path) (100.00% achieved)
[INFO] [1732790184.009501803] [hello_moveit.remote_control]: Waiting to continue: Press 'next' in the RvizVisualToolsGui window to execute Cartesian Path
[INFO] [1732790186.167874829] [hello_moveit.remote_control]: ... continuing
[INFO] [1732790186.168376635] [move_group_interface]: Execute request accepted
[INFO] [1732790188.931120564] [move_group_interface]: Execute request aborted
[ERROR] [1732790188.931348820] [move_group_interface]: MoveGroupInterface::execute() failed or timeout reached
janperenic@janperenic-OMEN:~/moveit2$ 







janperenic@janperenic-OMEN:~/moveit2$ ros2 launch hc10dt_moveit_config demo.launch.py 
[INFO] [launch]: All log files can be found below /home/janperenic/.ros/log/2024-11-28-11-36-03-533121-janperenic-OMEN-192367
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [static_transform_publisher-1]: process started with pid [192370]
[INFO] [robot_state_publisher-2]: process started with pid [192372]
[INFO] [move_group-3]: process started with pid [192374]
[INFO] [rviz2-4]: process started with pid [192376]
[INFO] [ros2_control_node-5]: process started with pid [192378]
[INFO] [spawner-6]: process started with pid [192380]
[INFO] [spawner-7]: process started with pid [192382]
[static_transform_publisher-1] [INFO] [1732790164.179473082] [static_transform_publisher0]: Spinning until stopped - publishing transform
[static_transform_publisher-1] translation: ('0.000000', '0.000000', '0.000000')
[static_transform_publisher-1] rotation: ('0.000000', '0.000000', '0.000000', '1.000000')
[static_transform_publisher-1] from 'world' to 'base_link'
[ros2_control_node-5] [WARN] [1732790164.186142109] [controller_manager]: [Deprecated] Passing the robot description parameter directly to the control_manager node is deprecated. Use '~/robot_description' topic from 'robot_state_publisher' instead.
[ros2_control_node-5] [INFO] [1732790164.186337384] [resource_manager]: Loading hardware 'motoman_hc10dt_control' 
[ros2_control_node-5] terminate called after throwing an instance of 'pluginlib::LibraryLoadException'
[ros2_control_node-5]   what():  According to the loaded plugin descriptions the class fake_components/GenericSystem with base class type hardware_interface::ActuatorInterface does not exist. Declared types are  test_hardware_components/TestSingleJointActuator
[move_group-3] [INFO] [1732790164.224671159] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.0239031 seconds
[move_group-3] [INFO] [1732790164.224738465] [moveit_robot_model.robot_model]: Loading robot model 'motoman_hc10dt'...
[robot_state_publisher-2] [INFO] [1732790164.231023475] [robot_state_publisher]: got segment base
[robot_state_publisher-2] [INFO] [1732790164.231128120] [robot_state_publisher]: got segment base_link
[robot_state_publisher-2] [INFO] [1732790164.231138800] [robot_state_publisher]: got segment link_1
[robot_state_publisher-2] [INFO] [1732790164.231146434] [robot_state_publisher]: got segment link_2
[robot_state_publisher-2] [INFO] [1732790164.231153137] [robot_state_publisher]: got segment link_3
[robot_state_publisher-2] [INFO] [1732790164.231160150] [robot_state_publisher]: got segment link_4
[robot_state_publisher-2] [INFO] [1732790164.231167333] [robot_state_publisher]: got segment link_5
[robot_state_publisher-2] [INFO] [1732790164.231174577] [robot_state_publisher]: got segment link_6
[robot_state_publisher-2] [INFO] [1732790164.231181830] [robot_state_publisher]: got segment tool0
[robot_state_publisher-2] [INFO] [1732790164.231188914] [robot_state_publisher]: got segment world
[move_group-3] [INFO] [1732790164.262940484] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Publishing maintained planning scene on 'monitored_planning_scene'
[move_group-3] [INFO] [1732790164.263086006] [moveit.ros_planning_interface.moveit_cpp]: Listening to 'joint_states' for joint states
[move_group-3] [INFO] [1732790164.263654687] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[move_group-3] [INFO] [1732790164.264081464] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/attached_collision_object' for attached collision objects
[move_group-3] [INFO] [1732790164.264100269] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[move_group-3] [INFO] [1732790164.264402453] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[move_group-3] [INFO] [1732790164.264417361] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[move_group-3] [INFO] [1732790164.264731236] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[move_group-3] [INFO] [1732790164.265028902] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[move_group-3] [WARN] [1732790164.265438356] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[move_group-3] [ERROR] [1732790164.265455458] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[move_group-3] [INFO] [1732790164.268095956] [moveit.ros_planning_interface.moveit_cpp]: Loading planning pipeline 'ompl'
[move_group-3] [INFO] [1732790164.276217574] [moveit.ros_planning.planning_pipeline]: Using planning interface 'OMPL'
[move_group-3] [INFO] [1732790164.277646732] [moveit_ros.add_time_optimal_parameterization]: Param 'ompl.path_tolerance' was not set. Using default value: 0.100000
[move_group-3] [INFO] [1732790164.277659866] [moveit_ros.add_time_optimal_parameterization]: Param 'ompl.resample_dt' was not set. Using default value: 0.100000
[move_group-3] [INFO] [1732790164.277664004] [moveit_ros.add_time_optimal_parameterization]: Param 'ompl.min_angle_change' was not set. Using default value: 0.001000
[move_group-3] [INFO] [1732790164.277678411] [moveit_ros.fix_workspace_bounds]: Param 'ompl.default_workspace_bounds' was not set. Using default value: 10.000000
[move_group-3] [INFO] [1732790164.277689511] [moveit_ros.fix_start_state_bounds]: Param 'ompl.start_state_max_bounds_error' was set to 0.100000
[move_group-3] [INFO] [1732790164.277694691] [moveit_ros.fix_start_state_bounds]: Param 'ompl.start_state_max_dt' was not set. Using default value: 0.500000
[move_group-3] [INFO] [1732790164.277702476] [moveit_ros.fix_start_state_collision]: Param 'ompl.start_state_max_dt' was not set. Using default value: 0.500000
[move_group-3] [INFO] [1732790164.277706844] [moveit_ros.fix_start_state_collision]: Param 'ompl.jiggle_fraction' was set to 0.050000
[move_group-3] [INFO] [1732790164.277710751] [moveit_ros.fix_start_state_collision]: Param 'ompl.max_sampling_attempts' was not set. Using default value: 100
[move_group-3] [INFO] [1732790164.277718536] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Add Time Optimal Parameterization'
[move_group-3] [INFO] [1732790164.277721882] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Resolve constraint frames to robot links'
[move_group-3] [INFO] [1732790164.277724437] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Workspace Bounds'
[move_group-3] [INFO] [1732790164.277727031] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Bounds'
[move_group-3] [INFO] [1732790164.277729416] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State In Collision'
[move_group-3] [INFO] [1732790164.277731830] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Path Constraints'
[move_group-3] [INFO] [1732790164.279432004] [moveit.ros_planning_interface.moveit_cpp]: Loading planning pipeline 'pilz_industrial_motion_planner'
[move_group-3] [INFO] [1732790164.281022191] [moveit.pilz_industrial_motion_planner.joint_limits_aggregator]: Reading limits from namespace robot_description_planning
[move_group-3] [INFO] [1732790164.283756014] [moveit.pilz_industrial_motion_planner]: Available plugins: pilz_industrial_motion_planner/PlanningContextLoaderCIRC pilz_industrial_motion_planner/PlanningContextLoaderLIN pilz_industrial_motion_planner/PlanningContextLoaderPTP 
[move_group-3] [INFO] [1732790164.283775330] [moveit.pilz_industrial_motion_planner]: About to load: pilz_industrial_motion_planner/PlanningContextLoaderCIRC
[move_group-3] [INFO] [1732790164.284657446] [moveit.pilz_industrial_motion_planner]: Registered Algorithm [CIRC]
[move_group-3] [INFO] [1732790164.284668938] [moveit.pilz_industrial_motion_planner]: About to load: pilz_industrial_motion_planner/PlanningContextLoaderLIN
[move_group-3] [INFO] [1732790164.285175183] [moveit.pilz_industrial_motion_planner]: Registered Algorithm [LIN]
[move_group-3] [INFO] [1732790164.285183769] [moveit.pilz_industrial_motion_planner]: About to load: pilz_industrial_motion_planner/PlanningContextLoaderPTP
[move_group-3] [INFO] [1732790164.285674675] [moveit.pilz_industrial_motion_planner]: Registered Algorithm [PTP]
[move_group-3] [INFO] [1732790164.285685215] [moveit.ros_planning.planning_pipeline]: Using planning interface 'Pilz Industrial Motion Planner'
[move_group-3] [INFO] [1732790164.286420036] [moveit.ros_planning_interface.moveit_cpp]: Loading planning pipeline 'chomp'
[move_group-3] [ERROR] [1732790164.287233745] [moveit.ros_planning.planning_pipeline]: Exception while loading planner 'chomp_interface/CHOMPPlanner': According to the loaded plugin descriptions the class chomp_interface/CHOMPPlanner with base class type planning_interface::PlannerManager does not exist. Declared types are  ompl_interface/OMPLPlanner pilz_industrial_motion_planner/CommandPlannerAvailable plugins: ompl_interface/OMPLPlanner, pilz_industrial_motion_planner/CommandPlanner
[move_group-3] [INFO] [1732790164.288024631] [moveit_ros.add_time_optimal_parameterization]: Param 'chomp.path_tolerance' was not set. Using default value: 0.100000
[move_group-3] [INFO] [1732790164.288033748] [moveit_ros.add_time_optimal_parameterization]: Param 'chomp.resample_dt' was not set. Using default value: 0.100000
[move_group-3] [INFO] [1732790164.288037835] [moveit_ros.add_time_optimal_parameterization]: Param 'chomp.min_angle_change' was not set. Using default value: 0.001000
[move_group-3] [INFO] [1732790164.288052883] [moveit_ros.fix_workspace_bounds]: Param 'chomp.default_workspace_bounds' was not set. Using default value: 10.000000
[move_group-3] [INFO] [1732790164.288062682] [moveit_ros.fix_start_state_bounds]: Param 'chomp.start_state_max_bounds_error' was set to 0.100000
[move_group-3] [INFO] [1732790164.288066719] [moveit_ros.fix_start_state_bounds]: Param 'chomp.start_state_max_dt' was not set. Using default value: 0.500000
[move_group-3] [INFO] [1732790164.288074935] [moveit_ros.fix_start_state_collision]: Param 'chomp.start_state_max_dt' was not set. Using default value: 0.500000
[move_group-3] [INFO] [1732790164.288078912] [moveit_ros.fix_start_state_collision]: Param 'chomp.jiggle_fraction' was set to 0.050000
[move_group-3] [INFO] [1732790164.288082158] [moveit_ros.fix_start_state_collision]: Param 'chomp.max_sampling_attempts' was not set. Using default value: 100
[move_group-3] [INFO] [1732790164.288090153] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Add Time Optimal Parameterization'
[move_group-3] [INFO] [1732790164.288093289] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Resolve constraint frames to robot links'
[move_group-3] [INFO] [1732790164.288096024] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Workspace Bounds'
[move_group-3] [INFO] [1732790164.288098649] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Bounds'
[move_group-3] [INFO] [1732790164.288101144] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State In Collision'
[move_group-3] [INFO] [1732790164.288103728] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Path Constraints'
[move_group-3] [ERROR] [1732790164.288683691] [moveit.ros_planning_interface.moveit_cpp]: Failed to initialize planning pipeline 'chomp'.
[move_group-3] [INFO] [1732790164.308566890] [moveit.plugins.moveit_simple_controller_manager]: Added FollowJointTrajectory controller for follow_joint_trajectory
[move_group-3] [INFO] [1732790164.308672277] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-3] [INFO] [1732790164.308686654] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-3] [INFO] [1732790164.308949665] [moveit_ros.trajectory_execution_manager]: Trajectory execution is managing controllers
[move_group-3] [INFO] [1732790164.308961908] [move_group.move_group]: MoveGroup debug mode is ON
[move_group-3] [INFO] [1732790164.320814709] [move_group.move_group]: 
[move_group-3] 
[move_group-3] ********************************************************
[move_group-3] * MoveGroup using: 
[move_group-3] *     - ApplyPlanningSceneService
[move_group-3] *     - ClearOctomapService
[move_group-3] *     - CartesianPathService
[move_group-3] *     - ExecuteTrajectoryAction
[move_group-3] *     - GetPlanningSceneService
[move_group-3] *     - KinematicsService
[move_group-3] *     - MoveAction
[move_group-3] *     - MotionPlanService
[move_group-3] *     - QueryPlannersService
[move_group-3] *     - StateValidationService
[move_group-3] ********************************************************
[move_group-3] 
[move_group-3] [INFO] [1732790164.320862549] [moveit_move_group_capabilities_base.move_group_context]: MoveGroup context using planning plugin ompl_interface/OMPLPlanner
[move_group-3] [INFO] [1732790164.320871425] [moveit_move_group_capabilities_base.move_group_context]: MoveGroup context initialization complete
[move_group-3] Loading 'move_group/ApplyPlanningSceneService'...
[move_group-3] Loading 'move_group/ClearOctomapService'...
[move_group-3] Loading 'move_group/MoveGroupCartesianPathService'...
[move_group-3] Loading 'move_group/MoveGroupExecuteTrajectoryAction'...
[move_group-3] Loading 'move_group/MoveGroupGetPlanningSceneService'...
[move_group-3] Loading 'move_group/MoveGroupKinematicsService'...
[move_group-3] Loading 'move_group/MoveGroupMoveAction'...
[move_group-3] Loading 'move_group/MoveGroupPlanService'...
[move_group-3] Loading 'move_group/MoveGroupQueryPlannersService'...
[move_group-3] Loading 'move_group/MoveGroupStateValidationService'...
[move_group-3] 
[move_group-3] You can start planning now!
[move_group-3] 
[spawner-7] [INFO] [1732790164.441689137] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-6] [INFO] [1732790164.457784337] [spawner_follow_joint_trajectory]: waiting for service /controller_manager/list_controllers to become available...
[rviz2-4] [INFO] [1732790164.489715934] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-4] [INFO] [1732790164.489779643] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
[rviz2-4] [INFO] [1732790164.503335353] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-4] Warning: class_loader.impl: SEVERE WARNING!!! A namespace collision has occurred with plugin factory for class rviz_default_plugins::displays::InteractiveMarkerDisplay. New factory will OVERWRITE existing one. This situation occurs when libraries containing plugins are directly linked against an executable (the one running right now generating this message). Please separate plugins out into their own library or just don't link against the library and use either class_loader::ClassLoader/MultiLibraryClassLoader to open.
[rviz2-4]          at line 253 in /opt/ros/humble/include/class_loader/class_loader/class_loader_core.hpp
[ERROR] [ros2_control_node-5]: process has died [pid 192378, exit code -6, cmd '/opt/ros/humble/lib/controller_manager/ros2_control_node --ros-args --params-file /tmp/launch_params_xqfqn6_x --params-file /home/janperenic/moveit2/install/hc10dt_moveit_config/share/hc10dt_moveit_config/config/ros2_controllers.yaml'].
[rviz2-4] [ERROR] [1732790167.588698961] [moveit_ros_visualization.motion_planning_frame]: Action server: /recognize_objects not available
[rviz2-4] [INFO] [1732790167.598038392] [moveit_ros_visualization.motion_planning_frame]: MoveGroup namespace changed: / -> . Reloading params.
[rviz2-4] [INFO] [1732790167.709033452] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.0626061 seconds
[rviz2-4] [INFO] [1732790167.709066123] [moveit_robot_model.robot_model]: Loading robot model 'motoman_hc10dt'...
[rviz2-4] [INFO] [1732790167.763409969] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[rviz2-4] [INFO] [1732790167.763888192] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/monitored_planning_scene'
[rviz2-4] [INFO] [1732790167.896328476] [interactive_marker_display_99891111094240]: Connected on namespace: /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic
[rviz2-4] [INFO] [1732790167.900454827] [moveit_ros_visualization.motion_planning_frame]: group manipulator
[rviz2-4] [INFO] [1732790167.900469765] [moveit_ros_visualization.motion_planning_frame]: Constructing new MoveGroup connection for group 'manipulator' in namespace ''
[rviz2-4] [INFO] [1732790167.907149402] [interactive_marker_display_99891111094240]: Sending request for interactive markers
[rviz2-4] [INFO] [1732790167.908272378] [move_group_interface]: Ready to take commands for planning group manipulator.
[rviz2-4] [INFO] [1732790167.939152472] [interactive_marker_display_99891111094240]: Service response received for initialization
[rviz2-4] [INFO] [1732790172.500244471] [move_group_interface]: MoveGroup action client/server ready
[move_group-3] [INFO] [1732790172.500930472] [moveit_move_group_default_capabilities.move_action_capability]: Received request
[move_group-3] [INFO] [1732790172.501176761] [moveit_move_group_default_capabilities.move_action_capability]: executing..
[rviz2-4] [INFO] [1732790172.501201628] [move_group_interface]: Planning request accepted
[move_group-3] [INFO] [1732790172.509040639] [moveit_move_group_default_capabilities.move_action_capability]: Planning request received for MoveGroup action. Forwarding to planning pipeline.
[move_group-3] [INFO] [1732790172.509114136] [moveit_move_group_capabilities_base.move_group_capability]: Using planning pipeline 'ompl'
[move_group-3] [INFO] [1732790172.509985482] [moveit.ompl_planning.model_based_planning_context]: Planner configuration 'manipulator' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[move_group-3] [INFO] [1732790172.544416914] [moveit_move_group_default_capabilities.move_action_capability]: Motion plan was computed successfully.
[rviz2-4] [INFO] [1732790172.544751749] [move_group_interface]: Planning request complete!
[rviz2-4] [INFO] [1732790172.544974325] [move_group_interface]: time taken to generate plan: 0.00211097 seconds
[spawner-7] [WARN] [1732790174.458927409] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-7] [INFO] [1732790174.459306797] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-6] [WARN] [1732790174.475001529] [spawner_follow_joint_trajectory]: Could not contact service /controller_manager/list_controllers
[spawner-6] [INFO] [1732790174.475438996] [spawner_follow_joint_trajectory]: waiting for service /controller_manager/list_controllers to become available...
[move_group-3] [INFO] [1732790174.514122754] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Received goal request
[move_group-3] [INFO] [1732790174.514293753] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Execution request received
[move_group-3] [INFO] [1732790174.514346602] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-3] [INFO] [1732790174.514410612] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-3] [INFO] [1732790174.514558828] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 1.5
[rviz2-4] [INFO] [1732790174.514365888] [move_group_interface]: Execute request accepted
[move_group-3] [INFO] [1732790174.524061083] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-3] [INFO] [1732790174.524113230] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-3] [INFO] [1732790174.524137806] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-3] [INFO] [1732790174.524292726] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to follow_joint_trajectory
[move_group-3] [INFO] [1732790174.543008987] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: follow_joint_trajectory started execution
[move_group-3] [INFO] [1732790174.543035938] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[move_group-3] [WARN] [1732790175.867314692] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: waitForExecution timed out
[move_group-3] [ERROR] [1732790175.867353394] [moveit_ros.trajectory_execution_manager]: Controller is taking too long to execute trajectory (the expected upper bound for the trajectory execution was 1.324100 seconds). Stopping trajectory.
[move_group-3] [INFO] [1732790175.867375185] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Cancelling execution for follow_joint_trajectory
[move_group-3] [INFO] [1732790175.934226615] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status TIMED_OUT ...
[move_group-3] [INFO] [1732790175.934343434] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Execution completed: TIMED_OUT
[rviz2-4] [INFO] [1732790175.934872070] [move_group_interface]: Execute request aborted
[rviz2-4] [ERROR] [1732790175.935568410] [move_group_interface]: MoveGroupInterface::execute() failed or timeout reached
[move_group-3] [WARN] [1732790175.956057570] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'follow_joint_trajectory' failed with error unknown error: Goal was cancelled by the user.
[move_group-3] [INFO] [1732790183.995170556] [moveit_move_group_default_capabilities.cartersian_path_service_capability]: Received request to compute Cartesian path
[move_group-3] [INFO] [1732790183.995266004] [moveit_move_group_default_capabilities.cartersian_path_service_capability]: Attempting to follow 7 waypoints for link 'link_6' using a step of 0.010000 m and jump threshold 0.000000 (in global reference frame)
[move_group-3] [INFO] [1732790184.007041592] [moveit_move_group_default_capabilities.cartersian_path_service_capability]: Computed Cartesian path with 133 points (followed 100.000000% of requested trajectory)
[spawner-7] [WARN] [1732790184.474273087] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-7] [INFO] [1732790184.474645753] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-6] [WARN] [1732790184.490817676] [spawner_follow_joint_trajectory]: Could not contact service /controller_manager/list_controllers
[spawner-6] [INFO] [1732790184.491390635] [spawner_follow_joint_trajectory]: waiting for service /controller_manager/list_controllers to become available...
[move_group-3] [INFO] [1732790186.168216336] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Received goal request
[move_group-3] [INFO] [1732790186.168298629] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Execution request received
[move_group-3] [INFO] [1732790186.168325029] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-3] [INFO] [1732790186.168343533] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-3] [INFO] [1732790186.168401552] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 1.5
[move_group-3] [INFO] [1732790186.191772982] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-3] [INFO] [1732790186.191829137] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-3] [INFO] [1732790186.191853783] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-3] [INFO] [1732790186.191944131] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to follow_joint_trajectory
[move_group-3] [INFO] [1732790186.234562063] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: follow_joint_trajectory started execution
[move_group-3] [INFO] [1732790186.234594213] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[move_group-3] [WARN] [1732790188.846583294] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: waitForExecution timed out
[move_group-3] [ERROR] [1732790188.846606107] [moveit_ros.trajectory_execution_manager]: Controller is taking too long to execute trajectory (the expected upper bound for the trajectory execution was 2.611836 seconds). Stopping trajectory.
[move_group-3] [INFO] [1732790188.846618991] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Cancelling execution for follow_joint_trajectory
[move_group-3] [INFO] [1732790188.930818640] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status TIMED_OUT ...
[move_group-3] [INFO] [1732790188.930885325] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Execution completed: TIMED_OUT
[move_group-3] [WARN] [1732790188.953613595] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'follow_joint_trajectory' failed with error unknown error: Goal was cancelled by the user.
[spawner-7] [WARN] [1732790194.490332099] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-7] [INFO] [1732790194.490843824] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-6] [WARN] [1732790194.505528972] [spawner_follow_joint_trajectory]: Could not contact service /controller_manager/list_controllers
[spawner-6] [INFO] [1732790194.505849721] [spawner_follow_joint_trajectory]: waiting for service /controller_manager/list_controllers to become available...
[spawner-7] [WARN] [1732790204.505017541] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-7] [INFO] [1732790204.505542692] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-6] [WARN] [1732790204.519892735] [spawner_follow_joint_trajectory]: Could not contact service /controller_manager/list_controllers
[spawner-6] [INFO] [1732790204.520178979] [spawner_follow_joint_trajectory]: waiting for service /controller_manager/list_controllers to become available...
[spawner-7] [WARN] [1732790214.519938637] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-7] [INFO] [1732790214.520265427] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-6] [WARN] [1732790214.534014518] [spawner_follow_joint_trajectory]: Could not contact service /controller_manager/list_controllers
[spawner-6] [INFO] [1732790214.534343362] [spawner_follow_joint_trajectory]: waiting for service /controller_manager/list_controllers to become available...
[spawner-7] [WARN] [1732790224.535372660] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-7] [INFO] [1732790224.536197309] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-6] [WARN] [1732790224.549028918] [spawner_follow_joint_trajectory]: Could not contact service /controller_manager/list_controllers
[spawner-6] [INFO] [1732790224.549458820] [spawner_follow_joint_trajectory]: waiting for service /controller_manager/list_controllers to become available...
[spawner-7] [WARN] [1732790234.552221816] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-7] [INFO] [1732790234.552735335] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-6] [WARN] [1732790234.565188989] [spawner_follow_joint_trajectory]: Could not contact service /controller_manager/list_controllers
[spawner-6] [INFO] [1732790234.565518824] [spawner_follow_joint_trajectory]: waiting for service /controller_manager/list_controllers to become available...


















[move_group-1] You can start planning now!
[move_group-1] 
[rviz2-2] [INFO] [1732885409.969282453] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-2] [INFO] [1732885409.969504337] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
[rviz2-2] [INFO] [1732885409.992574436] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-2] Warning: class_loader.impl: SEVERE WARNING!!! A namespace collision has occurred with plugin factory for class rviz_default_plugins::displays::InteractiveMarkerDisplay. New factory will OVERWRITE existing one. This situation occurs when libraries containing plugins are directly linked against an executable (the one running right now generating this message). Please separate plugins out into their own library or just don't link against the library and use either class_loader::ClassLoader/MultiLibraryClassLoader to open.
[rviz2-2]          at line 253 in /opt/ros/humble/include/class_loader/class_loader/class_loader_core.hpp
[rviz2-2] [ERROR] [1732885413.123598507] [moveit_ros_visualization.motion_planning_frame]: Action server: /recognize_objects not available
[rviz2-2] [INFO] [1732885413.138249059] [moveit_ros_visualization.motion_planning_frame]: MoveGroup namespace changed: / -> . Reloading params.
[rviz2-2] [WARN] [1732885413.158079024] [rcl.logging_rosout]: Publisher already registered for provided node name. If this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[rviz2-2] [WARN] [1732885413.195864094] [rcl.logging_rosout]: Publisher already registered for provided node name. If this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[rviz2-2] [INFO] [1732885413.275826238] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.0804161 seconds
[rviz2-2] [INFO] [1732885413.275885794] [moveit_robot_model.robot_model]: Loading robot model 'motoman_hc10dt'...
[rviz2-2] [INFO] [1732885413.340687699] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[rviz2-2] [INFO] [1732885413.341770233] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/monitored_planning_scene'
[rviz2-2] [INFO] [1732885413.619712649] [interactive_marker_display_109510055937840]: Connected on namespace: /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic
[rviz2-2] [INFO] [1732885413.627430193] [moveit_ros_visualization.motion_planning_frame]: group manipulator
[rviz2-2] [INFO] [1732885413.627480206] [moveit_ros_visualization.motion_planning_frame]: Constructing new MoveGroup connection for group 'manipulator' in namespace ''
[rviz2-2] [INFO] [1732885413.639605644] [move_group_interface]: Ready to take commands for planning group manipulator.
[rviz2-2] [INFO] [1732885413.642370242] [interactive_marker_display_109510055937840]: Sending request for interactive markers
[rviz2-2] [INFO] [1732885413.675280386] [interactive_marker_display_109510055937840]: Service response received for initialization
[move_group-1] [INFO] [1732885436.396878600] [moveit_move_group_default_capabilities.move_action_capability]: Received request
[move_group-1] [INFO] [1732885436.397111918] [moveit_move_group_default_capabilities.move_action_capability]: executing..
[move_group-1] [INFO] [1732885436.418338803] [moveit_move_group_default_capabilities.move_action_capability]: Planning request received for MoveGroup action. Forwarding to planning pipeline.
[move_group-1] [WARN] [1732885436.418528305] [moveit_move_group_capabilities_base.move_group_capability]: Couldn't find requested planning pipeline 'pilz_industrial_motion_planner'
[move_group-1] [INFO] [1732885436.418633020] [moveit_move_group_default_capabilities.move_action_capability]: Catastrophic failure

















janperenic@janperenic-OMEN:~/moveit2$ ros2 run hello_moveit ptp_move_node
2024-11-29 14:03:50.323 [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7415: open_and_lock_file failed -> Function open_port_internal
[INFO] [1732885431.369550822] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 1.02907 seconds
[INFO] [1732885431.369615070] [moveit_robot_model.robot_model]: Loading robot model 'motoman_hc10dt'...
[WARN] [1732885431.379997012] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1732885431.394308838] [move_group_interface]: Ready to take commands for planning group manipulator.
[INFO] [1732885431.394959487] [hello_moveit.remote_control]: RemoteControl Ready.
[INFO] [1732885431.394982191] [hello_moveit]: Planning pipeline: pilz_industrial_motion_planner
[INFO] [1732885431.395089281] [hello_moveit]: Planner ID: PTP
[INFO] [1732885431.395107287] [hello_moveit]: Planning time: 20.00
[INFO] [1732885436.395573177] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1732885436.397459498] [move_group_interface]: Planning request accepted
[INFO] [1732885436.419777853] [move_group_interface]: Planning request aborted
[ERROR] [1732885436.420395230] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[ERROR] [1732885436.420434614] [hello_moveit]: Planning failed for waypoint 1!
janperenic@janperenic-OMEN:~/moveit2$ 






























!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!




z hello world

janperenic@janperenic-OMEN:~/moveit2$ ros2 run hello_moveit hello_moveit_node
[INFO] [1733216601.631269845] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 1.02154 seconds
[INFO] [1733216601.631341082] [moveit_robot_model.robot_model]: Loading robot model 'motoman_hc10dt'...
[WARN] [1733216601.638266903] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1733216601.649259457] [move_group_interface]: Ready to take commands for planning group manipulator.
[INFO] [1733216601.649869010] [hello_moveit.remote_control]: RemoteControl Ready.
[INFO] [1733216601.650214804] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[INFO] [1733216601.680373170] [hello_moveit]: Visualizing plan (Cartesian path) (100.00% achieved)
[INFO] [1733216601.682864866] [hello_moveit.remote_control]: Waiting to continue: Press 'next' in the RvizVisualToolsGui window to execute Cartesian Path
[INFO] [1733216603.429581367] [hello_moveit.remote_control]: ... continuing
[INFO] [1733216603.430420036] [move_group_interface]: Execute request accepted
[INFO] [1733216605.467173771] [move_group_interface]: Execute request aborted
[ERROR] [1733216605.467729468] [move_group_interface]: MoveGroupInterface::execute() failed or timeout reached
[ERROR] [1733216605.467751507] [hello_moveit]: Failed to execute planned trajectory
janperenic@janperenic-OMEN:~/moveit2$ 







janperenic@janperenic-OMEN:~/moveit2$ ros2 launch hc10dt_moveit_config demo.launch.py 
[INFO] [launch]: All log files can be found below /home/janperenic/.ros/log/2024-12-03-10-03-09-839350-janperenic-OMEN-169322
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [move_group-1]: process started with pid [169323]
[INFO] [rviz2-2]: process started with pid [169325]
[INFO] [robot_state_publisher-3]: process started with pid [169327]
[robot_state_publisher-3] [INFO] [1733216590.389162002] [robot_state_publisher]: got segment base
[robot_state_publisher-3] [INFO] [1733216590.389285140] [robot_state_publisher]: got segment base_link
[robot_state_publisher-3] [INFO] [1733216590.389291682] [robot_state_publisher]: got segment link_1
[robot_state_publisher-3] [INFO] [1733216590.389295809] [robot_state_publisher]: got segment link_2
[robot_state_publisher-3] [INFO] [1733216590.389299867] [robot_state_publisher]: got segment link_3
[robot_state_publisher-3] [INFO] [1733216590.389303663] [robot_state_publisher]: got segment link_4
[robot_state_publisher-3] [INFO] [1733216590.389307691] [robot_state_publisher]: got segment link_5
[robot_state_publisher-3] [INFO] [1733216590.389311487] [robot_state_publisher]: got segment link_6
[robot_state_publisher-3] [INFO] [1733216590.389315615] [robot_state_publisher]: got segment tool0
[robot_state_publisher-3] [INFO] [1733216590.389320012] [robot_state_publisher]: got segment world
[move_group-1] [INFO] [1733216590.395980000] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.0241458 seconds
[move_group-1] [INFO] [1733216590.396047951] [moveit_robot_model.robot_model]: Loading robot model 'motoman_hc10dt'...
[move_group-1] [INFO] [1733216590.433331574] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Publishing maintained planning scene on 'monitored_planning_scene'
[move_group-1] [INFO] [1733216590.433523976] [moveit.ros_planning_interface.moveit_cpp]: Listening to 'joint_states' for joint states
[move_group-1] [INFO] [1733216590.433989703] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[move_group-1] [INFO] [1733216590.434253562] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/attached_collision_object' for attached collision objects
[move_group-1] [INFO] [1733216590.434265834] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[move_group-1] [INFO] [1733216590.434503125] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[move_group-1] [INFO] [1733216590.434527548] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[move_group-1] [INFO] [1733216590.434746307] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[move_group-1] [INFO] [1733216590.434989278] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[move_group-1] [WARN] [1733216590.435377107] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[move_group-1] [ERROR] [1733216590.435387937] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[move_group-1] [INFO] [1733216590.437342217] [moveit.ros_planning_interface.moveit_cpp]: Loading planning pipeline 'ompl'
[move_group-1] [INFO] [1733216590.446238186] [moveit.ros_planning.planning_pipeline]: Using planning interface 'OMPL'
[move_group-1] [INFO] [1733216590.447831345] [moveit_ros.add_time_optimal_parameterization]: Param 'ompl.path_tolerance' was not set. Using default value: 0.100000
[move_group-1] [INFO] [1733216590.447853204] [moveit_ros.add_time_optimal_parameterization]: Param 'ompl.resample_dt' was not set. Using default value: 0.100000
[move_group-1] [INFO] [1733216590.447859205] [moveit_ros.add_time_optimal_parameterization]: Param 'ompl.min_angle_change' was not set. Using default value: 0.001000
[move_group-1] [INFO] [1733216590.447873600] [moveit_ros.fix_workspace_bounds]: Param 'ompl.default_workspace_bounds' was not set. Using default value: 10.000000
[move_group-1] [INFO] [1733216590.447884209] [moveit_ros.fix_start_state_bounds]: Param 'ompl.start_state_max_bounds_error' was set to 0.100000
[move_group-1] [INFO] [1733216590.447889118] [moveit_ros.fix_start_state_bounds]: Param 'ompl.start_state_max_dt' was not set. Using default value: 0.500000
[move_group-1] [INFO] [1733216590.447896601] [moveit_ros.fix_start_state_collision]: Param 'ompl.start_state_max_dt' was not set. Using default value: 0.500000
[move_group-1] [INFO] [1733216590.447901881] [moveit_ros.fix_start_state_collision]: Param 'ompl.jiggle_fraction' was set to 0.050000
[move_group-1] [INFO] [1733216590.447905928] [moveit_ros.fix_start_state_collision]: Param 'ompl.max_sampling_attempts' was not set. Using default value: 100
[move_group-1] [INFO] [1733216590.447914132] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Add Time Optimal Parameterization'
[move_group-1] [INFO] [1733216590.447917458] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Resolve constraint frames to robot links'
[move_group-1] [INFO] [1733216590.447920303] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Workspace Bounds'
[move_group-1] [INFO] [1733216590.447923018] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Bounds'
[move_group-1] [INFO] [1733216590.447925543] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State In Collision'
[move_group-1] [INFO] [1733216590.447928478] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Path Constraints'
[move_group-1] [INFO] [1733216590.465290943] [moveit.plugins.moveit_simple_controller_manager]: Added FollowJointTrajectory controller for follow_joint_trajectory
[move_group-1] [INFO] [1733216590.465500565] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-1] [INFO] [1733216590.465517485] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-1] [INFO] [1733216590.465726516] [moveit_ros.trajectory_execution_manager]: Trajectory execution is managing controllers
[move_group-1] [INFO] [1733216590.465741563] [move_group.move_group]: MoveGroup debug mode is ON
[move_group-1] [INFO] [1733216590.478248184] [move_group.move_group]: 
[move_group-1] 
[move_group-1] ********************************************************
[move_group-1] * MoveGroup using: 
[move_group-1] *     - ApplyPlanningSceneService
[move_group-1] *     - ClearOctomapService
[move_group-1] *     - CartesianPathService
[move_group-1] *     - ExecuteTrajectoryAction
[move_group-1] *     - GetPlanningSceneService
[move_group-1] *     - KinematicsService
[move_group-1] *     - MoveAction
[move_group-1] *     - MotionPlanService
[move_group-1] *     - QueryPlannersService
[move_group-1] *     - StateValidationService
[move_group-1] ********************************************************
[move_group-1] 
[move_group-1] [INFO] [1733216590.478310244] [moveit_move_group_capabilities_base.move_group_context]: MoveGroup context using planning plugin ompl_interface/OMPLPlanner
[move_group-1] [INFO] [1733216590.478323728] [moveit_move_group_capabilities_base.move_group_context]: MoveGroup context initialization complete
[move_group-1] Loading 'move_group/ApplyPlanningSceneService'...
[move_group-1] Loading 'move_group/ClearOctomapService'...
[move_group-1] Loading 'move_group/MoveGroupCartesianPathService'...
[move_group-1] Loading 'move_group/MoveGroupExecuteTrajectoryAction'...
[move_group-1] Loading 'move_group/MoveGroupGetPlanningSceneService'...
[move_group-1] Loading 'move_group/MoveGroupKinematicsService'...
[move_group-1] Loading 'move_group/MoveGroupMoveAction'...
[move_group-1] Loading 'move_group/MoveGroupPlanService'...
[move_group-1] Loading 'move_group/MoveGroupQueryPlannersService'...
[move_group-1] Loading 'move_group/MoveGroupStateValidationService'...
[move_group-1] 
[move_group-1] You can start planning now!
[move_group-1] 
[rviz2-2] [INFO] [1733216590.565595889] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-2] [INFO] [1733216590.565795553] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
[rviz2-2] [INFO] [1733216590.584264784] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-2] Warning: class_loader.impl: SEVERE WARNING!!! A namespace collision has occurred with plugin factory for class rviz_default_plugins::displays::InteractiveMarkerDisplay. New factory will OVERWRITE existing one. This situation occurs when libraries containing plugins are directly linked against an executable (the one running right now generating this message). Please separate plugins out into their own library or just don't link against the library and use either class_loader::ClassLoader/MultiLibraryClassLoader to open.
[rviz2-2]          at line 253 in /opt/ros/humble/include/class_loader/class_loader/class_loader_core.hpp
[rviz2-2] [ERROR] [1733216593.676297729] [moveit_ros_visualization.motion_planning_frame]: Action server: /recognize_objects not available
[rviz2-2] [INFO] [1733216593.686994647] [moveit_ros_visualization.motion_planning_frame]: MoveGroup namespace changed: / -> . Reloading params.
[rviz2-2] [WARN] [1733216593.699340623] [rcl.logging_rosout]: Publisher already registered for provided node name. If this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[rviz2-2] [WARN] [1733216593.727671288] [rcl.logging_rosout]: Publisher already registered for provided node name. If this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[rviz2-2] [INFO] [1733216593.782453502] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.0551447 seconds
[rviz2-2] [INFO] [1733216593.782504242] [moveit_robot_model.robot_model]: Loading robot model 'motoman_hc10dt'...
[rviz2-2] [INFO] [1733216593.821668668] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[rviz2-2] [INFO] [1733216593.822314685] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/monitored_planning_scene'
[rviz2-2] [INFO] [1733216594.040709306] [interactive_marker_display_97360093519408]: Connected on namespace: /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic
[rviz2-2] [INFO] [1733216594.045559830] [moveit_ros_visualization.motion_planning_frame]: group manipulator
[rviz2-2] [INFO] [1733216594.045583082] [moveit_ros_visualization.motion_planning_frame]: Constructing new MoveGroup connection for group 'manipulator' in namespace ''
[rviz2-2] [INFO] [1733216594.053486298] [move_group_interface]: Ready to take commands for planning group manipulator.
[rviz2-2] [INFO] [1733216594.055131229] [interactive_marker_display_97360093519408]: Sending request for interactive markers
[rviz2-2] [INFO] [1733216594.089306335] [interactive_marker_display_97360093519408]: Service response received for initialization
[move_group-1] [INFO] [1733216601.673707621] [moveit_move_group_default_capabilities.cartersian_path_service_capability]: Received request to compute Cartesian path
[move_group-1] [INFO] [1733216601.673911824] [moveit_move_group_default_capabilities.cartersian_path_service_capability]: Attempting to follow 8 waypoints for link 'link_6' using a step of 0.010000 m and jump threshold 0.000000 (in global reference frame)
[move_group-1] [INFO] [1733216601.679897615] [moveit_move_group_default_capabilities.cartersian_path_service_capability]: Computed Cartesian path with 75 points (followed 100.000000% of requested trajectory)
[move_group-1] [INFO] [1733216603.430218178] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Received goal request
[move_group-1] [INFO] [1733216603.430412753] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Execution request received
[move_group-1] [INFO] [1733216603.430450761] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-1] [INFO] [1733216603.430475855] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-1] [INFO] [1733216603.430569842] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 1.5
[move_group-1] [INFO] [1733216603.460792914] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-1] [INFO] [1733216603.460886671] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-1] [INFO] [1733216603.460907458] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-1] [INFO] [1733216603.461062363] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to follow_joint_trajectory
[move_group-1] [INFO] [1733216603.493487997] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: follow_joint_trajectory started execution
[move_group-1] [INFO] [1733216603.493542033] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[move_group-1] [WARN] [1733216605.391273055] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: waitForExecution timed out
[move_group-1] [ERROR] [1733216605.391328103] [moveit_ros.trajectory_execution_manager]: Controller is taking too long to execute trajectory (the expected upper bound for the trajectory execution was 1.897549 seconds). Stopping trajectory.
[move_group-1] [INFO] [1733216605.391356593] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Cancelling execution for follow_joint_trajectory
[move_group-1] [INFO] [1733216605.465989758] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status TIMED_OUT ...
[move_group-1] [INFO] [1733216605.466239311] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Execution completed: TIMED_OUT
[move_group-1] [WARN] [1733216605.493525828] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'follow_joint_trajectory' failed with error unknown error: Goal was cancelled by the user.






















!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

z ptp




janperenic@janperenic-OMEN:~/moveit2$ ros2 run hello_moveit ptp_move_node
[INFO] [1733216666.000115223] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 1.02395 seconds
[INFO] [1733216666.000166825] [moveit_robot_model.robot_model]: Loading robot model 'motoman_hc10dt'...
[WARN] [1733216666.007099865] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1733216666.016907925] [move_group_interface]: Ready to take commands for planning group manipulator.
[INFO] [1733216666.017421758] [hello_moveit.remote_control]: RemoteControl Ready.
[INFO] [1733216666.017437225] [hello_moveit]: Planning pipeline: pilz_industrial_motion_planner
[INFO] [1733216666.017541992] [hello_moveit]: Planner ID: PTP
[INFO] [1733216666.017552300] [hello_moveit]: Planning time: 20.00
[INFO] [1733216671.017892741] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1733216671.019231721] [move_group_interface]: Planning request accepted
[INFO] [1733216671.036376259] [move_group_interface]: Planning request aborted
[ERROR] [1733216671.036879423] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[ERROR] [1733216671.036912582] [hello_moveit]: Planning failed for waypoint 1!
janperenic@janperenic-OMEN:~/moveit2$ 








janperenic@janperenic-OMEN:~/moveit2$ ros2 launch hc10dt_moveit_config demo.launch.py 
[INFO] [launch]: All log files can be found below /home/janperenic/.ros/log/2024-12-03-10-04-13-699028-janperenic-OMEN-170350
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [move_group-1]: process started with pid [170351]
[INFO] [rviz2-2]: process started with pid [170353]
[INFO] [robot_state_publisher-3]: process started with pid [170355]
[robot_state_publisher-3] [INFO] [1733216653.918405248] [robot_state_publisher]: got segment base
[robot_state_publisher-3] [INFO] [1733216653.918546258] [robot_state_publisher]: got segment base_link
[robot_state_publisher-3] [INFO] [1733216653.918554282] [robot_state_publisher]: got segment link_1
[robot_state_publisher-3] [INFO] [1733216653.918559612] [robot_state_publisher]: got segment link_2
[robot_state_publisher-3] [INFO] [1733216653.918564200] [robot_state_publisher]: got segment link_3
[robot_state_publisher-3] [INFO] [1733216653.918568247] [robot_state_publisher]: got segment link_4
[robot_state_publisher-3] [INFO] [1733216653.918573637] [robot_state_publisher]: got segment link_5
[robot_state_publisher-3] [INFO] [1733216653.918577604] [robot_state_publisher]: got segment link_6
[robot_state_publisher-3] [INFO] [1733216653.918581761] [robot_state_publisher]: got segment tool0
[robot_state_publisher-3] [INFO] [1733216653.918585588] [robot_state_publisher]: got segment world
[move_group-1] [INFO] [1733216653.940213830] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.034457 seconds
[move_group-1] [INFO] [1733216653.940317093] [moveit_robot_model.robot_model]: Loading robot model 'motoman_hc10dt'...
[move_group-1] [INFO] [1733216653.983673403] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Publishing maintained planning scene on 'monitored_planning_scene'
[move_group-1] [INFO] [1733216653.983851420] [moveit.ros_planning_interface.moveit_cpp]: Listening to 'joint_states' for joint states
[move_group-1] [INFO] [1733216653.984494542] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[move_group-1] [INFO] [1733216653.984801277] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/attached_collision_object' for attached collision objects
[move_group-1] [INFO] [1733216653.984817256] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[move_group-1] [INFO] [1733216653.985039010] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[move_group-1] [INFO] [1733216653.985053345] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[move_group-1] [INFO] [1733216653.985276883] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[move_group-1] [INFO] [1733216653.985503094] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[move_group-1] [WARN] [1733216653.985996181] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[move_group-1] [ERROR] [1733216653.986015475] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[move_group-1] [INFO] [1733216653.988364248] [moveit.ros_planning_interface.moveit_cpp]: Loading planning pipeline 'ompl'
[move_group-1] [INFO] [1733216653.998646749] [moveit.ros_planning.planning_pipeline]: Using planning interface 'OMPL'
[move_group-1] [INFO] [1733216654.000567513] [moveit_ros.add_time_optimal_parameterization]: Param 'ompl.path_tolerance' was not set. Using default value: 0.100000
[move_group-1] [INFO] [1733216654.000584753] [moveit_ros.add_time_optimal_parameterization]: Param 'ompl.resample_dt' was not set. Using default value: 0.100000
[move_group-1] [INFO] [1733216654.000591455] [moveit_ros.add_time_optimal_parameterization]: Param 'ompl.min_angle_change' was not set. Using default value: 0.001000
[move_group-1] [INFO] [1733216654.000615728] [moveit_ros.fix_workspace_bounds]: Param 'ompl.default_workspace_bounds' was not set. Using default value: 10.000000
[move_group-1] [INFO] [1733216654.000634862] [moveit_ros.fix_start_state_bounds]: Param 'ompl.start_state_max_bounds_error' was set to 0.100000
[move_group-1] [INFO] [1733216654.000643077] [moveit_ros.fix_start_state_bounds]: Param 'ompl.start_state_max_dt' was not set. Using default value: 0.500000
[move_group-1] [INFO] [1733216654.000658114] [moveit_ros.fix_start_state_collision]: Param 'ompl.start_state_max_dt' was not set. Using default value: 0.500000
[move_group-1] [INFO] [1733216654.000667270] [moveit_ros.fix_start_state_collision]: Param 'ompl.jiggle_fraction' was set to 0.050000
[move_group-1] [INFO] [1733216654.000674483] [moveit_ros.fix_start_state_collision]: Param 'ompl.max_sampling_attempts' was not set. Using default value: 100
[move_group-1] [INFO] [1733216654.000688848] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Add Time Optimal Parameterization'
[move_group-1] [INFO] [1733216654.000694749] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Resolve constraint frames to robot links'
[move_group-1] [INFO] [1733216654.000699828] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Workspace Bounds'
[move_group-1] [INFO] [1733216654.000704336] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Bounds'
[move_group-1] [INFO] [1733216654.000708974] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State In Collision'
[move_group-1] [INFO] [1733216654.000722618] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Path Constraints'
[move_group-1] [INFO] [1733216654.018464845] [moveit.plugins.moveit_simple_controller_manager]: Added FollowJointTrajectory controller for follow_joint_trajectory
[move_group-1] [INFO] [1733216654.018794210] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-1] [INFO] [1733216654.018829322] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-1] [INFO] [1733216654.019197777] [moveit_ros.trajectory_execution_manager]: Trajectory execution is managing controllers
[move_group-1] [INFO] [1733216654.019244831] [move_group.move_group]: MoveGroup debug mode is ON
[move_group-1] [INFO] [1733216654.031860708] [move_group.move_group]: 
[move_group-1] 
[move_group-1] ********************************************************
[move_group-1] * MoveGroup using: 
[move_group-1] *     - ApplyPlanningSceneService
[move_group-1] *     - ClearOctomapService
[move_group-1] *     - CartesianPathService
[move_group-1] *     - ExecuteTrajectoryAction
[move_group-1] *     - GetPlanningSceneService
[move_group-1] *     - KinematicsService
[move_group-1] *     - MoveAction
[move_group-1] *     - MotionPlanService
[move_group-1] *     - QueryPlannersService
[move_group-1] *     - StateValidationService
[move_group-1] ********************************************************
[move_group-1] 
[move_group-1] [INFO] [1733216654.031906069] [moveit_move_group_capabilities_base.move_group_context]: MoveGroup context using planning plugin ompl_interface/OMPLPlanner
[move_group-1] [INFO] [1733216654.031915676] [moveit_move_group_capabilities_base.move_group_context]: MoveGroup context initialization complete
[move_group-1] Loading 'move_group/ApplyPlanningSceneService'...
[move_group-1] Loading 'move_group/ClearOctomapService'...
[move_group-1] Loading 'move_group/MoveGroupCartesianPathService'...
[move_group-1] Loading 'move_group/MoveGroupExecuteTrajectoryAction'...
[move_group-1] Loading 'move_group/MoveGroupGetPlanningSceneService'...
[move_group-1] Loading 'move_group/MoveGroupKinematicsService'...
[move_group-1] Loading 'move_group/MoveGroupMoveAction'...
[move_group-1] Loading 'move_group/MoveGroupPlanService'...
[move_group-1] Loading 'move_group/MoveGroupQueryPlannersService'...
[move_group-1] Loading 'move_group/MoveGroupStateValidationService'...
[move_group-1] 
[move_group-1] You can start planning now!
[move_group-1] 
[rviz2-2] [INFO] [1733216654.192606441] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-2] [INFO] [1733216654.192818057] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
[rviz2-2] [INFO] [1733216654.211889285] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-2] Warning: class_loader.impl: SEVERE WARNING!!! A namespace collision has occurred with plugin factory for class rviz_default_plugins::displays::InteractiveMarkerDisplay. New factory will OVERWRITE existing one. This situation occurs when libraries containing plugins are directly linked against an executable (the one running right now generating this message). Please separate plugins out into their own library or just don't link against the library and use either class_loader::ClassLoader/MultiLibraryClassLoader to open.
[rviz2-2]          at line 253 in /opt/ros/humble/include/class_loader/class_loader/class_loader_core.hpp
[rviz2-2] [ERROR] [1733216657.301862236] [moveit_ros_visualization.motion_planning_frame]: Action server: /recognize_objects not available
[rviz2-2] [INFO] [1733216657.313020674] [moveit_ros_visualization.motion_planning_frame]: MoveGroup namespace changed: / -> . Reloading params.
[rviz2-2] [WARN] [1733216657.326420495] [rcl.logging_rosout]: Publisher already registered for provided node name. If this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[rviz2-2] [WARN] [1733216657.359316970] [rcl.logging_rosout]: Publisher already registered for provided node name. If this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[rviz2-2] [INFO] [1733216657.413025147] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.0542049 seconds
[rviz2-2] [INFO] [1733216657.413068935] [moveit_robot_model.robot_model]: Loading robot model 'motoman_hc10dt'...
[rviz2-2] [INFO] [1733216657.449930887] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[rviz2-2] [INFO] [1733216657.450603903] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/monitored_planning_scene'
[rviz2-2] [INFO] [1733216657.681065894] [interactive_marker_display_106099536119232]: Connected on namespace: /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic
[rviz2-2] [INFO] [1733216657.687596689] [moveit_ros_visualization.motion_planning_frame]: group manipulator
[rviz2-2] [INFO] [1733216657.687625029] [moveit_ros_visualization.motion_planning_frame]: Constructing new MoveGroup connection for group 'manipulator' in namespace ''
[rviz2-2] [INFO] [1733216657.695020250] [move_group_interface]: Ready to take commands for planning group manipulator.
[rviz2-2] [INFO] [1733216657.699399642] [interactive_marker_display_106099536119232]: Sending request for interactive markers
[rviz2-2] [INFO] [1733216657.731130326] [interactive_marker_display_106099536119232]: Service response received for initialization
[move_group-1] [INFO] [1733216671.018936677] [moveit_move_group_default_capabilities.move_action_capability]: Received request
[move_group-1] [INFO] [1733216671.019133086] [moveit_move_group_default_capabilities.move_action_capability]: executing..
[move_group-1] [INFO] [1733216671.035300367] [moveit_move_group_default_capabilities.move_action_capability]: Planning request received for MoveGroup action. Forwarding to planning pipeline.
[move_group-1] [WARN] [1733216671.035413227] [moveit_move_group_capabilities_base.move_group_capability]: Couldn't find requested planning pipeline 'pilz_industrial_motion_planner'
[move_group-1] [INFO] [1733216671.035541916] [moveit_move_group_default_capabilities.move_action_capability]: Catastrophic failure




















!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!








premika robot po hello_ampak dobim error



[move_group-1] You can start planning now!
[move_group-1] 
[rviz2-2] [INFO] [1733246409.995170494] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-2] [INFO] [1733246409.995358998] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
[rviz2-2] [INFO] [1733246410.013249208] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-2] Warning: class_loader.impl: SEVERE WARNING!!! A namespace collision has occurred with plugin factory for class rviz_default_plugins::displays::InteractiveMarkerDisplay. New factory will OVERWRITE existing one. This situation occurs when libraries containing plugins are directly linked against an executable (the one running right now generating this message). Please separate plugins out into their own library or just don't link against the library and use either class_loader::ClassLoader/MultiLibraryClassLoader to open.
[rviz2-2]          at line 253 in /opt/ros/humble/include/class_loader/class_loader/class_loader_core.hpp
[rviz2-2] [ERROR] [1733246413.104530227] [moveit_ros_visualization.motion_planning_frame]: Action server: /recognize_objects not available
[rviz2-2] [INFO] [1733246413.115463077] [moveit_ros_visualization.motion_planning_frame]: MoveGroup namespace changed: / -> . Reloading params.
[rviz2-2] [WARN] [1733246413.130553075] [rcl.logging_rosout]: Publisher already registered for provided node name. If this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[rviz2-2] [WARN] [1733246413.164199250] [rcl.logging_rosout]: Publisher already registered for provided node name. If this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[rviz2-2] [INFO] [1733246413.224411200] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.0607733 seconds
[rviz2-2] [INFO] [1733246413.224469457] [moveit_robot_model.robot_model]: Loading robot model 'motoman_hc10dt'...
[rviz2-2] [INFO] [1733246413.283416112] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[rviz2-2] [INFO] [1733246413.284196276] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/monitored_planning_scene'
[rviz2-2] [INFO] [1733246413.510470507] [interactive_marker_display_99759345874160]: Connected on namespace: /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic
[rviz2-2] [INFO] [1733246413.518232159] [moveit_ros_visualization.motion_planning_frame]: group manipulator
[rviz2-2] [INFO] [1733246413.518257025] [moveit_ros_visualization.motion_planning_frame]: Constructing new MoveGroup connection for group 'manipulator' in namespace ''
[rviz2-2] [INFO] [1733246413.526453226] [move_group_interface]: Ready to take commands for planning group manipulator.
[rviz2-2] [INFO] [1733246413.527710767] [interactive_marker_display_99759345874160]: Sending request for interactive markers
[rviz2-2] [INFO] [1733246413.559853289] [interactive_marker_display_99759345874160]: Service response received for initialization
[move_group-1] [INFO] [1733246462.130826270] [moveit_move_group_default_capabilities.cartersian_path_service_capability]: Received request to compute Cartesian path
[move_group-1] [INFO] [1733246462.130957177] [moveit_move_group_default_capabilities.cartersian_path_service_capability]: Attempting to follow 8 waypoints for link 'link_6' using a step of 0.010000 m and jump threshold 0.000000 (in global reference frame)
[move_group-1] [INFO] [1733246462.137279826] [moveit_move_group_default_capabilities.cartersian_path_service_capability]: Computed Cartesian path with 85 points (followed 100.000000% of requested trajectory)
[move_group-1] [INFO] [1733246466.444358118] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Received goal request
[move_group-1] [INFO] [1733246466.444523013] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Execution request received
[move_group-1] [INFO] [1733246466.444550840] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-1] [INFO] [1733246466.444566742] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-1] [INFO] [1733246466.444673159] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 1.5
[move_group-1] [INFO] [1733246466.467562886] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-1] [INFO] [1733246466.467790058] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-1] [INFO] [1733246466.467840351] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-1] [INFO] [1733246466.468006900] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to follow_joint_trajectory
[move_group-1] [INFO] [1733246466.489022591] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: follow_joint_trajectory started execution
[move_group-1] [INFO] [1733246466.489067242] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[move_group-1] [WARN] [1733246484.504954290] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'follow_joint_trajectory' failed with error unknown error: Final position was outside tolerance. Check robot safety-limits that could be inhibiting motion. [joint_2: 0.0000 deviation] [joint_3: 0.0000 deviation]
[move_group-1] [WARN] [1733246484.505035143] [moveit_ros.trajectory_execution_manager]: Controller handle follow_joint_trajectory reports status ABORTED
[move_group-1] [INFO] [1733246484.505070925] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status ABORTED ...
[move_group-1] [INFO] [1733246484.505240907] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Execution completed: ABORTED





janperenic@janperenic-OMEN:~/moveit2$ ros2 run hello_moveit hello_moveit_node 
[INFO] [1733246462.092073067] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 1.02555 seconds
[INFO] [1733246462.092153481] [moveit_robot_model.robot_model]: Loading robot model 'motoman_hc10dt'...
[WARN] [1733246462.099708356] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1733246462.112538802] [move_group_interface]: Ready to take commands for planning group manipulator.
[INFO] [1733246462.113140240] [hello_moveit.remote_control]: RemoteControl Ready.
[INFO] [1733246462.113553874] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[INFO] [1733246462.137826933] [hello_moveit]: Visualizing plan (Cartesian path) (100.00% achieved)
[INFO] [1733246462.140507737] [hello_moveit.remote_control]: Waiting to continue: Press 'next' in the RvizVisualToolsGui window to execute Cartesian Path
[INFO] [1733246466.443845514] [hello_moveit.remote_control]: ... continuing
[INFO] [1733246466.444533344] [move_group_interface]: Execute request accepted
[INFO] [1733246484.505712577] [move_group_interface]: Execute request aborted
[ERROR] [1733246484.506435382] [move_group_interface]: MoveGroupInterface::execute() failed or timeout reached
[ERROR] [1733246484.506460963] [hello_moveit]: Failed to execute planned trajectory




















!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



ptp mors uprasat



[move_group-1] You can start planning now!
[move_group-1] 
[rviz2-2] [INFO] [1733247362.034250711] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-2] [INFO] [1733247362.034370445] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
[rviz2-2] [INFO] [1733247362.048568012] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-2] Warning: class_loader.impl: SEVERE WARNING!!! A namespace collision has occurred with plugin factory for class rviz_default_plugins::displays::InteractiveMarkerDisplay. New factory will OVERWRITE existing one. This situation occurs when libraries containing plugins are directly linked against an executable (the one running right now generating this message). Please separate plugins out into their own library or just don't link against the library and use either class_loader::ClassLoader/MultiLibraryClassLoader to open.
[rviz2-2]          at line 253 in /opt/ros/humble/include/class_loader/class_loader/class_loader_core.hpp
[rviz2-2] [ERROR] [1733247365.132164011] [moveit_ros_visualization.motion_planning_frame]: Action server: /recognize_objects not available
[rviz2-2] [INFO] [1733247365.141380736] [moveit_ros_visualization.motion_planning_frame]: MoveGroup namespace changed: / -> . Reloading params.
[rviz2-2] [WARN] [1733247365.154810664] [rcl.logging_rosout]: Publisher already registered for provided node name. If this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[rviz2-2] [WARN] [1733247365.182511592] [rcl.logging_rosout]: Publisher already registered for provided node name. If this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[rviz2-2] [INFO] [1733247365.240015448] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.0577942 seconds
[rviz2-2] [INFO] [1733247365.240048750] [moveit_robot_model.robot_model]: Loading robot model 'motoman_hc10dt'...
[rviz2-2] [INFO] [1733247365.311298604] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[rviz2-2] [INFO] [1733247365.311944251] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/monitored_planning_scene'
[rviz2-2] [INFO] [1733247365.457057193] [interactive_marker_display_104003734838784]: Connected on namespace: /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic
[rviz2-2] [INFO] [1733247365.460876891] [moveit_ros_visualization.motion_planning_frame]: group manipulator
[rviz2-2] [INFO] [1733247365.460893442] [moveit_ros_visualization.motion_planning_frame]: Constructing new MoveGroup connection for group 'manipulator' in namespace ''
[rviz2-2] [INFO] [1733247365.467371138] [interactive_marker_display_104003734838784]: Sending request for interactive markers
[rviz2-2] [INFO] [1733247365.467463280] [move_group_interface]: Ready to take commands for planning group manipulator.
[rviz2-2] [INFO] [1733247365.499012160] [interactive_marker_display_104003734838784]: Service response received for initialization
[move_group-1] [INFO] [1733247398.739391915] [moveit_move_group_default_capabilities.move_action_capability]: Received request
[move_group-1] [INFO] [1733247398.739539521] [moveit_move_group_default_capabilities.move_action_capability]: executing..
[move_group-1] [INFO] [1733247398.759254760] [moveit_move_group_default_capabilities.move_action_capability]: Planning request received for MoveGroup action. Forwarding to planning pipeline.
[move_group-1] [WARN] [1733247398.766683493] [moveit_move_group_capabilities_base.move_group_capability]: Couldn't find requested planning pipeline 'pilz_industrial_motion_planner'
[move_group-1] [INFO] [1733247398.766750949] [moveit_move_group_default_capabilities.move_action_capability]: Catastrophic failure






janperenic@janperenic-OMEN:~/moveit2$ ros2 run hello_moveit ptp_move_node
[INFO] [1733247393.719470677] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 1.0241 seconds
[INFO] [1733247393.719525710] [moveit_robot_model.robot_model]: Loading robot model 'motoman_hc10dt'...
[WARN] [1733247393.726144974] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1733247393.737209826] [move_group_interface]: Ready to take commands for planning group manipulator.
[INFO] [1733247393.737898952] [hello_moveit.remote_control]: RemoteControl Ready.
[INFO] [1733247393.737919560] [hello_moveit]: Planning pipeline: pilz_industrial_motion_planner
[INFO] [1733247393.738035767] [hello_moveit]: Planner ID: PTP
[INFO] [1733247393.738045445] [hello_moveit]: Planning time: 20.00
[INFO] [1733247398.738380698] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1733247398.739604001] [move_group_interface]: Planning request accepted
[INFO] [1733247398.767225043] [move_group_interface]: Planning request aborted
[ERROR] [1733247398.767429254] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[ERROR] [1733247398.767460603] [hello_moveit]: Planning failed for waypoint 1!
janperenic@janperenic-OMEN:~/moveit2$ 

