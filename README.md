Lab 1: ROS 2, Turtlesim, Actions & Services

Course: Design of Mechatronics Systems
Institution: Alexandria University, Faculty of Engineering

Team Members

Mahmoud saleh agamia-8955
Mohamed Nagy Abdullah- 9044
Mahmoud essam- 8699

Packages Description

This repository contains two ROS 2 packages built for the turtlesim assignment:

turtlesim_interfaces: A C++ (ament_cmake) package containing custom message (TargetPose.msg), service (SetSpeeds.srv), and action definitions (GoToPose.action, DrawSquare.action).

turtlesim_servers: A Python (ament_python) package containing the system's execution nodes:

go_to_pose_server: Action server to drive the turtle to a specific (x, y, yaw). Also hosts the parameter server for speed limits.

reset_server: Service to clear the screen and teleport the turtle to the center without drawing lines.

draw_square_server: High-level action server that commands the turtle to draw a square using the reset service and goToPose action.

Build Instructions

Clone this repository into the src folder of your ROS 2 workspace.

Open a terminal and navigate to the root of your workspace:

cd ~/ros2_ws


Build the packages:

colcon build


Source the setup file:

source install/setup.bash


How to Run the System

The entire system (Turtlesim + all 3 servers) is orchestrated via a single launch file.

ros2 launch turtlesim_servers turtlesim_system.launch.py


Commands to Test Requirements

Open a new terminal and source the workspace (source ~/ros2_ws/install/setup.bash) before running these commands.

1. Change Speed Parameters (Live)

ros2 service call /set_speeds turtlesim_interfaces/srv/SetSpeeds "{max_linear: 3.0, max_angular: 2.0}"


2. Test GoToPose Action

ros2 action send_goal --feedback /go_to_pose turtlesim_interfaces/action/GoToPose "{target_pose: {x: 8.0, y: 8.0, yaw: 1.57}}"


3. Test Reset Service

ros2 service call /reset_turtle std_srvs/srv/Empty


4. Test Square Action

ros2 action send_goal --feedback /draw_square turtlesim_interfaces/action/DrawSquare "{side_length: 4.0}"


Bonus Implementations

Design Patterns (State Pattern)

We refactored the GoToPose action server using the State Design Pattern. The turtle's movement logic is divided into distinct state classes: TranslatingState (handling XY movement), RotatingState (handling final yaw rotation), and FinishedState. This eliminates blocking while loops, makes the code highly modular, and allows the context (the node) to seamlessly transition behaviors as distance and angle errors converge to zero.

Bag File Recording

A rosbag file (square_bag) was recorded to capture the velocity commands (/turtle1/cmd_vel) during a square-drawing execution.
To play it back and watch the turtle repeat the motion:

Reset the turtle: ros2 service call /reset_turtle std_srvs/srv/Empty

Play the bag: ros2 bag play square_bag