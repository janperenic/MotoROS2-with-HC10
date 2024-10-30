# Run MoveIt2:
```bash
cd moveit2
colcon build
source install/local_setup.bash
ros2 launch hc10dt_moveit_config demo.launch.py 
# V demo.launch.py mors spremenit package name ko gres iz simulacije na orbota in obratno

# za robota
ros2 launch hc10dt_moveit_config demo.launch.py 

# simulacija
ros2 launch hc10_movit2v3 demo.launch.py

```
# MicroROS Agent run comand
```bash
cd micro_ros_agent_ws/
source $HOME/micro_ros_agent_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

# To activate remote mode
Ključ nujno v PLAY mode !!!
Nujno zazeni prek tega ker drugace ne dela. Ne mors nastavt tega prek robota
```bash
cd micro_ros_agent_ws
source install/local_setup.bash
ros2 service call /start_traj_mode motoros2_interfaces/srv/StartTrajMode 
```

# Launch script for moving the robot
```bash
cd moveit2
colcon build
source install/local_setup.bash
ros2 run hello_moveit ptp_move_node

#linear move on path
ros2 run hello_moveit hello_moveit_node

#want to be linear moves but are not ???
ros2 run hello_moveit linear_move_node
```

# Launch MoveIt2 Task Contructer
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



ros2 run moveit_task_constructor_demo cartesian
