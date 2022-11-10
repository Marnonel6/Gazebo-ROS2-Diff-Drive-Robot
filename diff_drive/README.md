# ME495 Embedded Systems Homework 3
Author: Marthinus (Marno) Nel

This project includes the package `diff_drive`. A world is created in Gazebo and a robot/car is imported from a urdf.xacro file.
A node was created called flip that will publish a linear velocity that will occilate in direction and cause the robot/car to flip.

## Quickstart
1. To launch gazebo and rviz use `ros2 launch diff_drive ddrive.launch.py`
2. To launch rviz only use `ros2 launch diff_drive ddrive_rviz.launch.py view_only:=true`

3. Here is a video of the car flipping

  [Robot_Catches_Brick.webm](https://user-images.githubusercontent.com/60977336/196852556-7bbca85f-46d4-4842-99ee-a81dd363eede.webm)

Worked With: `James, Alan, Ritika, Shantao`