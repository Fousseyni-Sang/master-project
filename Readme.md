README TO LAUNCH THE SIMULATION ON GAZEBO

Authors:
Yasmine LOUKILI
Fousseyni SANGARE
M2 IPS-SMR Sorbonne Université 2023-2024

Our project consists of designing a virtual environment for testing visual assistance devices with sensory feedback.

This Readme file describes the steps to launch it correctly on a computer with installed Ubuntu. This code has been tested only on Ubuntu 22.04 LTS with ROS2 Humble.

We hope it works on other versions of Ubuntu and even other Linux distributions with different versions of ROS 2 installed, such as ROS2 Foxy.

FOLLOW ALL THE FOLLOWING STEPS CORRECTLY WITHOUT MISSING ANY !!!!

Prerequisites:

1. Install Ubuntu 22.04 on your computer
(refer to this tutorial https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)

2. Install ROS2 (humble ideally, but it should work for other distros as well)
refer to this tutorial https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

3. Install gazebo 11 (classic)
refer to this tutorial https://classic.gazebosim.org/tutorials?tut=install_ubuntu

4. Install the joy package to use the Xbox one controller with ROS2:
open a terminal and type: sudo apt install ros-<ros2-distro>-joy
example: sudo apt install ros-humble-joy
git link of the joy node for more details: https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md

5. Install the sfml library to use sound in case of obstacle
open a terminal and type: sudo apt-get install libsfml-dev
sfml site for more details: https://www.sfml-dev.org/tutorials/2.6/start-linux.php

5. bis. Make sure pynput library is installed in order to be able to use keyboard
open a terminal and type $ pip install pynput

Preparing the workspace to launch the simulation with ROS2 and gazebo:

6. "source" the ROS2 environment: open a terminal and type
$ source /opt/ros/humble/setup.bash
To avoid doing this in every terminal, put this code at the end of the file ~/.bashrc. Thus, no need to source each time, this is done automatically.
Example: open a terminal and type:
$ echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

Refer to this tutorial for more details:
https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html

7. To enable the plugin that controls the camera to work properly, specify it in the GAZEBO_PLUGIN_PATH:
open the terminal where you want to launch the simulation, and type the following code:
$ export GAZEBO_PLUGIN_PATH=path_to_project_directory/SMR_Project/ros2_ws/src/camera_control_pkg/joy_plugin/build:$GAZEBO_PLUGIN_PATH
To avoid doing this every time you open a new terminal to launch the simulation, put the code in the /.bashrc file.
Example: Open a terminal and type:
$ echo “GAZEBO_PLUGIN_PATH=/SMR_Project/ros2_ws/src/camera_control_pkg/joy_plugin/build:$GAZEBO_PLUGIN_PATH” >> ~/.bashrc
Refer to this tutorial for more details: https://classic.gazebosim.org/tutorials?tut=plugins_model&cat=write_plugin

8. An almost final step is to do $ colcon build in the workspace so that the code can work: go to the ros_ws workspace and type $ colcon build
Example: for our case
$ cd ~/SMR_project/ros_ws
$ colcon build

9. A last step to set up the environment:
open a terminal, move to the workspace: $ cd path_to_project_directory /SMR_Project/ros2_ws and type $ source install/setup.bash so that the package can be recognized.
If this does not work, maybe redo the build in the workspace always with $ colcon build and retype the command $ source install/setup.bash.
Refer to this tutorial for more details: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html

Launch the simulation with the launch file:

10. Launch all nodes in a single terminal:
Open a terminal (where step 9 of configuration has been done. If the "source" has been added in the ~/.bashrc file, open any terminal. However, repeat step 8 each time in a new terminal, especially source install/setup.bash). Move to the launch directory to launch the launch file with the command ros2 launch virtual_environment.launch.py.
Example: for our case
$ cd ~/SMR_project/ros2_ws/
$ source install/setup.bash
$ cd src/camera_control_pkg/launch
$ ros2 launch virtual_environment.launch.py

11. Launching all nodes in different terminals:
Same as the previous step, except this time, you need to launch another launch file: 
virtual_environment_gnome_terminal.launch.py

Example: for our case
$ cd ~/SMR_project/ros2_ws/
$ source install/setup.bash
$ cd src/camera_control_pkg/launch
$ ros2 launch virtual_environment_gnome_terminal.launch.py

Rviz Configuration:


12. This step assumes that the simulation has been launched and both a gazebo window and an Rviz2 window have opened. But it is possible that the camera image is not visible in Rviz2. If that's not the case, great. If it is, then refer to this link to configure Rviz2 properly so that the images seen by the camera can be observed.
link to tutorial to configure Rviz2.
https://classic.gazebosim.org/tutorials?tut=ros_depth_camera&cat=connect_ros

Controller with Xbox One gamepad:

First, connect the Xbox One controller before proceeding to use it.
For camera translation movements, use the left joystick of the Xbox One controller (left-right for the x-axis and up-down for the y-axis), and the right joystick for rotation movements around the x and y axes of the camera.

The two LT and RT buttons on the gamepad (bottom left and right buttons) allow rotating the camera around its vertical z-axis.

Control with the computer keyboard:

To perform translation or rotation, use the right, left, up, down arrow keys, and page-up and page-down keys on the keyboard, page-up and page-down keys only for rotations around the y-axis.
By default, the selected mode is translation. To switch to rotation mode, press the "R" key, and to switch back to translation mode, press the "T" key.

A recurring issue with Gazebo:
If Gazebo refuses to start because there is another server running in the background, use this command to stop this process.
$ killall -9 gzserver




























