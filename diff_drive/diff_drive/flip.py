"""
This node publishes a velocity for a specified time interval in one direction and then changes
the velocity direction to the opposite direction which causes the car to flip.

PUBLISHERS:
    + cmd_vel (Twist) - Linear velocity to move the car forward.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3


class Flip_node(Node):
    """
    PUBLISHERS:
    + cmd_vel (Twist) - Linear velocity to move the car forward.
    """

    def __init__(self):
        super().__init__('flip')


        self.pub_Flip = self.create_publisher(Twist, "cmd_vel", 10)
        self.Flip_Vel = Twist()

        self.maxVelocity = 2.0
        self.time_between_flips = 2.0 # Amount of seconds



        # Create a timer to do the rest of the transforms.
        self.tmr = self.create_timer(0.01, self.timer_callback)

        self.total_time = 0.0


    def flip_robot(self,direction):
        """
        Changes the x velocity direction of the car according to the direction variable.
        """
        self.Flip_Vel = Twist(linear=Vector3(x=direction*self.maxVelocity, y=0.0, z=0.0),\
             angular=Vector3(x=0.0, y=0.0, z=0.0))

        return self.Flip_Vel


    def timer_callback(self):
        """
        Timer Callback
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