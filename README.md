# Run MoveIt2:
```bash
cd moveit2
colcon build
source install/local_setup.bash
ros2 launch hc10dt_moveit_config demo.launch.py 


ros2 launch hc20_moveit_config demo.launch.py


ros2 launch hc20_sim_moveit_config demo.launch.py
# V demo.launch.py mors spremenit package name ko gres iz simulacije na robota in obratno

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


## TODO:
-naredi neko spremenljivko v ptp_move k ti doloc velikost kvadrata. mogoce lah dam vec razlicnih velikosti 20x20/40x40/60x60
-naredi spremeljviko za start kr direkt v moveit lah nastavs start1 start2 start3 in pol lah startas od lin premike od tm kjer hocs ce so ksne razlike mogoce ?
-v ptp mors nastavt start direkt v kodi tko da lah jih spremenis v ene 3 razlicne tocke

-poglej tf transorme v urdf filu zato da mas ista imena kot robot!!!


To get tf frame type this comand in another terminal
```bash
ros2 run tf2_tools view_frames
```

```bash
ros2 run rqt_gui rqt_gui
```

ros2 run moveit_task_constructor_demo cartesian












- If the planing with hello_moveit dost work delite all _planning files from config folder
- if you have problems with srdf in hello_moveit delite                
            # moveit_config.robot_description,
            # moveit_config.robot_description_semantic, in rviz-node

- If you have problem with tf use this to see what is going on: 
```bash
ros2 run tf2_tools view_frames
```
