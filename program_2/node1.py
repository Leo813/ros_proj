#! /usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan

from matplotlib import pyplot as plt

class FeatureExtracter(Node):
    def __init__(self):
        super().__init__('feature_extracter')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.publisher_ = self.create_publisher(LaserScan, '/front_scan', 10)
  
    def polar_to_cartesian_coordinate(self, ranges, angle_min, angle_max):
        angle_step = (angle_max - angle_min) / len(ranges)
        angle = 0
        points = []
        for range in ranges:
            x = range * np.cos(angle)
            y = range * np.sin(angle)
            angle += angle_step
            points.append([x,y])

        points_np = np.array(points)
        print(points_np)
        plt.figure()
        plt.scatter(points_np[:,0], points_np[:,1])
        plt.show()
        
        return points
    
    def polar_plot(self, ranges):
        theta = np.arange(0, 360)
 
        fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
        ax.plot(theta, ranges)
        ax.set_rmax(8)
        ax.set_rticks([2, 4, 6, 8])  # Less radial ticks
        ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
        ax.grid(True)

        ax.set_title("A line plot on a polar axis", va='bottom')
        plt.show()

    def scan_callback(self,msg):

        #self.polar_to_cartesian_coordinate(msg.ranges, msg.angle_min, msg.angle_max)

        #################################
        ranges = []
        i = 0
        for r in msg.ranges:
            if i <= 60 or i >= 300:
                ranges.append(r)
            else:
                ranges.append(0.0)
            i += 1
        ################################## 

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = msg.header.frame_id
        scan.angle_min = msg.angle_min
        scan.angle_max = msg.angle_max
        scan.angle_increment = msg.angle_increment
        scan.time_increment = msg.time_increment
        scan.range_min = msg.range_min
        scan.range_max = msg.range_max 
        scan.ranges = ranges

        #print(len(ranges))
        self.polar_to_cartesian_coordinate(scan.ranges, scan.angle_min, scan.angle_max)
        self.polar_plot(scan.ranges)        
        self.publisher_.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    feature_extracter = FeatureExtracter()
    rclpy.spin(feature_extracter) 
    feature_extracter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()