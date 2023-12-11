#! /usr/bin/python
#from _typeshed import Self
import base64
from operator import pos
from os import posix_fadvise
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Empty
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
import sys

cmdlinear = []
cmdangular = []
posx = []
posy = []
lvx = []
lvy = []

class Subscribers(Node):
    def __init__(self):
        super().__init__('subscribe')
        self.cnt = 0
        self.subscription2 = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.positionX = 0.0
        self.positionY = 0.0
        self.lx = 0.0
        self.ly = 0.0
        self.lz = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0

    def odom_callback(self, msg):
        self.positionX = msg.pose.pose.position.x
        self.positionY = msg.pose.pose.position.y

        plt.figure(figsize=(16,16))
        plt.plot(self.positionX, self.positionY)
        plt.title("position")
        plt.xlabel("X",fontsize=16)
        plt.ylabel("Y",fontsize=16)
        plt.show()

def main():
    rclpy.init()
    subscribers = Subscribers()

    # Spin until ctrl + c
    rclpy.spin(subscribers)

    subscribers.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()