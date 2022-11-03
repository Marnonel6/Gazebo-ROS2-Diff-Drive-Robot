"""
Simulates the manipulation of a robot catching a brick in an arena that corresponds to turtlesim.
Broadcasts the static world and odom frame. Broadcasts the base link frame for manipulating the
robot in rviz to catch the brick and tilt the brick off.

PUBLISHERS:
    + /turtle1/cmd_vel (Twist) - Linear velocity to move the turtle forward.
    + /odom (Odometry) -
    + /joint_states (JointState) - Angles of all joints

SUBSCRIBERS:
    + /turtle1/pose (Pose) - The position of the current turtle. This is used to move
                             the robot in rviz.
    + /goal_pose (Point) - The coordinates where the robot should go to next.
    + /tilt (Tilt) - Message that contains the tilt angle for the platform.

PARAMETERS:
    + max_velocity (double) - Maximum translational velocity of the turtle/robot.
    + platform_height (double) - Height of the platform to ground in meters.
    + wheel_radius (double) - Wheel radius of the robot in meters.
"""

import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Point
from math import atan2, cos, sin
from geometry_msgs.msg import Twist, Vector3
from turtlesim.msg import Pose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from enum import Enum, auto
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import ParameterDescriptor
from nav_msgs.msg import Odometry



class Flip_node(Node):
    """
    PUBLISHERS:
    + /turtle1/cmd_vel (Twist) - Linear velocity to move the turtle forward.
    + /odom (Odometry) -
    + /joint_states (JointState) - Angles of all joints

    SUBSCRIBERS:
    + /turtle1/pose (Pose) - The position of the current turtle. This is used to move the robot
                             in rviz.
    + /goal_pose (Point) - The coordinates where the robot should go to next.
    + /tilt (Tilt) - Message that contains the tilt angle for the platform.

    PARAMETERS:
    + max_velocity (double) - Maximum translational velocity of the turtle/robot.
    + platform_height (double) - Height of the platform to ground in meters.
    + wheel_radius (double) - Wheel radius of the robot in meters.

    Static Broadcasts: world -> odom
    Dynamic Broadcasts: odom -> base_link
    """

    def __init__(self):
        super().__init__('flip')


        self.pub_Flip = self.create_publisher(Twist, "/cmd_vel", 10)
        self.Flip_Vel = Twist()

        self.maxVelocity = 5.0
        self.time_between_flips = 1.0 # Amount of seconds



        # Create a timer to do the rest of the transforms .
        self.tmr = self.create_timer(0.01, self.timer_callback)

        self.total_time = 0.0


    def flip_robot(self,direction):
        """
        Calculates the x and y velocity of the turtle as a function of the maximum velocity
        and the angle between the turtle and the goal pose to move the turtle from its
        current position to the goal position.
        """

        self.Flip_Vel = Twist(linear=Vector3(x=direction*self.maxVelocity, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))

            
        # Get the vector in x and y of the max velocity
        return self.Flip_Vel




    def timer_callback(self):
        """
        State Machine
        """

        self.total_time += 0.01

        # Move in one direction for self.time_between_flips
        if self.total_time < self.time_between_flips: 
            direction = 1
            self.pub_Flip.publish(self.flip_robot(direction))
            # Move in other direction (Thus flip) for self.time_between_flips
        elif self.total_time >= self.time_between_flips and \
                                                self.total_time < self.time_between_flips*2:
            direction = -1
            self.pub_Flip.publish(self.flip_robot(direction))
        else:
            self.total_time = 0







def flip_node_entry(args=None):
    rclpy.init(args=args)
    node = Flip_node()
    rclpy.spin(node)
    rclpy.shutdown()