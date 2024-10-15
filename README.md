# Run MoveIt2:
```bash
cd moveit2
colcon build
source install/local_setup.bash
ros2 launch hc10dt_moveit_config demo.launch.py 
```
# MicroROS Agent run comand
```bash
cd micro_ros_agent_ws/
source $HOME/micro_ros_agent_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

# To activate remote mode
Ključ nujno v PLAY mode !!!
```bash
cd micro_ros_agent_ws/
ros2 service call /start_traj_mode motoros2_interfaces/srv/StartTrajMode 
```

# Launch script for moving the robot
```bash
cd moveit2
colcon build
source install/local_setup.bash
ros2 run hello_moveit hello_moveit_node
```