# Aerial-Manipulator-Gazebo
Aerial Manipulator for Gazebo Simulation

This package depends on the [RotorS Simulator](https://github.com/ethz-asl/rotors_simulator) for Gazebo-ROS simulation of an Aerial Manipulation System.
The aerial manipulator consists of the HarrierD7 Vulcan UAV and the [Kinova](https://github.com/Kinovarobotics/kinova-ros) 7DoF spherical wrist robot manipulator.

### Instructions

1. First of all, download and install the [RotorS Simulator](https://github.com/ethz-asl/rotors_simulator) in your workspace, preferably using `catkin_make`
2. Clone this repository to your workspace and run `catkin_make` once again
3. Run `roslaunch Aerial-Manipulator-Gazebo aerial_manipulator.launch` to test the default system

### Further usage
* In a new terminal, run
```
roslaunch Aerial-Manipulator-Gazebo uav_teleop.launch
```
to send discreet position and yaw orientation commands to the UAV

* In another terminal, run
```
roslaunch Aerial-Manipulator-Gazebo robot_teleop.launch
```
to send discreet joint angle commands to the robot

* For expanded usage, we suggest downloading and installing the [Kinova](https://github.com/Kinovarobotics/kinova-ros) packages, for more robot descriptions and functionalities 
