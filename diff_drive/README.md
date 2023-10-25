# ME495 Embedded Systems Homework 3
Author: Marthinus (Marno) Nel

This project includes the package `diff_drive`. A world is created in Gazebo and a robot/car is imported from a urdf.xacro file.
A node was created called flip that will publish a linear velocity that will occilate in direction and cause the robot/car to flip.

## Quickstart
1. To launch gazebo and rviz use `ros2 launch diff_drive ddrive.launch.py`
2. To launch rviz only use `ros2 launch diff_drive ddrive_rviz.launch.py view_only:=true`

3. Here is a video of the car flipping

  [Flipper.mp4](https://user-images.githubusercontent.com/60977336/201008422-60a2b429-58e1-496b-9c67-0b6132a3cd67.mp4)
