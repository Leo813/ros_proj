#!/usr/bin/env python

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import motor_backend



robot = motor_backend.Robot()
class JetbotControl(Node):
    def __init__(self):
        super().__init__('jetbot_control')
        self.create_subscription(Twist, '/cmd_vel', self.cmd_twist_callback, 10)



    # listen to twist commands
    def cmd_twist_callback(self, msg):

        axis1 = msg.linear.x
        axis2 = msg.angular.z / 2
        lmot = 0.0
        rmot = 0.0

        if axis1 > 0 and axis2 == 0:  # Forward
            rmot = axis1
            lmot = axis1
        elif axis1 > 0 and axis2 > 0:  # Forward Left Turn
            rmot = axis1
            lmot = axis1 - axis2
        elif axis1 > 0 and axis2 < 0:  # Forward Right Turn
            rmot = axis1 - abs(axis2)
            lmot = axis1
        elif axis1 < 0 and axis2 == 0:  # Reverse
            rmot = axis1
            lmot = axis1
        elif axis1 < 0 and axis2 > 0:  # Reverse Left Turn
            rmot = axis1 - axis2
            lmot = axis1
        elif axis1 < 0 and axis2 < 0:  # Reverse Right Turn
            rmot = axis1
            lmot = axis1 - abs(axis2)
        elif axis1 == 0 and axis2 < 0:  # Rotate Left
            rmot = 0.0
            lmot = abs(axis2)
        elif axis1 == 0 and axis2 > 0:  # Rotate Right
            rmot = axis2
            lmot = 0.0
        elif axis1 == 0 and axis2 == 0:  # Stop
            rmot = 0.0
            lmot = 0.0

        print(lmot,rmot)
        robot.set_motors(lmot,rmot)

def main():

    #setup ros node
    rclpy.init()
    jetbot_control = JetbotControl()
    rclpy.spin(jetbot_control)

    # Stop
    jetbot_control.destroy_node()
    rclpy.shutdown()


# initialization
if __name__ == '__main__':
        main()
