# Frontier Exploration.py
## Description
The frontier_exploration package is designed for robotic systems using ROS2. It provides functionality for analyzing map data, detecting frontiers (unexplored areas), and setting goals towards these frontiers. This is particularly useful in autonomous navigation and exploration tasks in unknown environments.

## Prerequisites
* ROS2 Humble
* Python 3
* ROS2 packages: nav_msgs, geometry_msgs, action_msgs
## Installation and Build
Assuming ROS2 Humble and the required dependencies are already installed:

## Clone the package into your ROS2 workspace:
```
cd ~/ros2_ws
git clone https://github.com/Nyanziba/AutoFrontierSearch_ros2-humble.git
```
## Build the workspace:
```
cd ~/ros2_ws
colcon build --packages-select frontier_exploration
```
## Source the environment and run the frontier_exploration node:

```
cd ~/ros2_ws/install/setup.bash
ros2 run frontier_exploration exploration_node
```
## Sample
Terminal1
```
ros2 launch nav2_bringup navigation_launch.py
```
Terminal2
```
ros2 launch slam_toolbox online_async_launch.py
```
Terminal3
```
ros2 launch nav2_bringup rviz_launch.py
```
Terminal4
```
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```
## Nodes
* exploration_node
  * The exploration_node analyzes occupancy grid maps and detects frontiers. It then sets goals towards these frontiers for the robot to explore.

## Subscribed Topics
* /map ([nav_msgs/msg/OccupancyGrid])
   * The occupancy grid map from which the node detects frontiers.

* /odom ([nav_msgs/msg/Odometry]
   * The odometry data of the robot.

## Published Topics
* /goal_pose ([geometry_msgs/msg/PoseStamped](// File: ros2_humble.hpp))
   * The goal pose set towards a detected frontier.


License
Add your license information here.

