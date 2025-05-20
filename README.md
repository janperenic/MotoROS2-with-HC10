# Yaskawa HC10 & HC20DTP Control via ROS2 — Master's Degree Project

This project is part of my master's degree work and focuses on controlling Yaskawa HC10 and HC20 DTP robots using ROS2, and comparing the motion execution with Yaskawa's default MotoPlus programming.

The main goal is to compare PTP (MOVJ) and Linear (LIN/MOVL) motions. The testing includes:

* Moving the robot through points along X, Y, Z axes for linear moves along specific axes.
* Moving the robot in square trajectories on each of the three planes (XY, YZ, XZ).

**Note**: Every time you make changes to the project, you must rebuild and source the workspace!



## Run MoveIt2 for the HC 20 DTP robot:

### Real robot
```bash
cd moveit2
colcon build
source install/local_setup.bash
ros2 launch hc20_moveit_config demo.launch.py
```
### Simulation
```bash
cd moveit2
colcon build
source install/local_setup.bash
ros2 launch hc20_sim_moveit_config demo.launch.py
```

## Run MoveIt2 for the HC 10 robot:

### Real robot
```bash
cd moveit2
colcon build
source install/local_setup.bash
ros2 launch hc10dt_moveit_config demo.launch.py 
```

### Simulation
```bash
cd moveit2
colcon build
source install/local_setup.bash
ros2 launch hc10_movit2v3 demo.launch.py
```

## Connecting to Yaskawa Robot Controller
When connected to the Yaskawa robot controller, you must install:
* MotoROS2 (follow instruction [here](https://github.com/Yaskawa-Global/motoros2))
* micro ROS agent(use the ROS 2 package, Docker didnt work for me!)
* [motoros2_interfaces](https://github.com/yaskawa-global/motoros2_client_interface_dependencies)

Once installed, launch the micro-ROS agent to connect via Ethernet to the Yaskawa controller:

### MicroROS Agent run comand
```bash
cd micro_ros_agent_ws/
source $HOME/micro_ros_agent_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### Activate remote mode
Switch the teach pendant key to REMOTE mode, and in a new terminal activate StartTrajMode to turn on the servos and enable robot motion.

```bash
cd micro_ros_agent_ws
source install/local_setup.bash
ros2 service call /start_traj_mode motoros2_interfaces/srv/StartTrajMode 
```


## Moving the Robot with ROS2 Scripts
Build the project and run movement scripts.
The main script for me was hello_moveit_node!

```bash
cd moveit2
colcon build
source install/local_setup.bash
ros2 run hello_moveit hello_moveit_node

#script 1.
ros2 run hello_moveit hello_moveit_node

#script 2.
ros2 run hello_moveit ptp_move_node

#script 3.
ros2 run hello_moveit linear_move_node
```

## TODO:
### Launch MoveIt2 Task Contructer
Run it in a seperate terminal
The path planer should show TPC folows a 200x200mm square
u can execute the movemnt witch gear button in upper right corner
```bash
cd moveit2
colcon build
source install/local_setup.bash
ros2 launch mtc_tutorial pick_place_demo.launch.py
```

used with panda for testing
```bash
ros2 launch moveit_task_constructor_demo demo.launch.py

cd ws_moveit2
colcon build
source install/local_setup.bash
ros2 launch moveit2_tutorials mtc_demo.launch.py
```




## Debugging Tips

* If the planing with hello_moveit dost work delite all _planning files from config folder
* if you have problems with srdf in hello_moveit delite in launch files:
```bash           
        # moveit_config.robot_description,
        # moveit_config.robot_description_semantic, in rviz-node
```
* If you have problem with tf use this to see what is going on: 
```bash
ros2 run tf2_tools view_frames
```
* To get TF frames live, open a new terminal and run:
```bash
ros2 run tf2_tools view_frames
```

```bash
ros2 run rqt_gui rqt_gui
```



