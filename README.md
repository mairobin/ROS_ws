## Project Structure

```
catkin_ws
├── src/
│ ├── CMakeLists.txt
│ │
│ ├── serve_humanity/
│ │ ├── src/
│ │ │ ├── ur5_kinematics_node.cpp
│ │ ├── launch/
│ │ │ ├── sine_wave.launch
│ │ │ ├── ur5_kinematics_node.launch
│ │ ├── scripts/
│ │ │ ├── my_api.py
│ │ │ ├── sine_wave_joint_publisher.py
│ │ ├── use_api.py
│ │ ├── CMakeLists.txt
│ │ └── package.xml
│ │
│ ├── universal_robot/
│ ├── ros_control/
```

##  Dependencies
I installed universal_robot and ros_control from source
	
- https://github.com/ros-industrial/universal_robot
- https://github.com/ros-controls/ros_control

Installation of universal_robot with method below was missing the `ur_kinematics` folder I used for the Inverse Kinematics Solver task.

`sudo apt-get install ros-$ROS_DISTRO-universal-robots` 

## My Solution

- Task 1:
  Python script: `sine_wave_joint_publisher.py`
  To bring the UR5 robot to live:  `roslaunch serve_humanity sine_wave.launch`
  
- Task 2:
  C++ code: `ur5_kinematics_node.cpp`
  Launch: `roslaunch serve_humanity ur5_kinematics_node.launch`
  Unfortunately the node is not functional yet

- Task 3:
  User API to get Robot State from `/joint_states` topic: `python3 use_api.py` 
  No API for task 2 because the task was not completed successfully.

- Task 4:
  Unable to finish the task in the time frame. 

