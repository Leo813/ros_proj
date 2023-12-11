#! /usr/bin/env python
import math

import rclpy
from rclpy.node import Node
import numpy as np
import warnings

from sensor_msgs.msg import LaserScan

from matplotlib import pyplot as plt

xp = []
yp = []

class FeatureExtracter(Node):
    def __init__(self):
        super().__init__('feature_extracter')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.publisher_ = self.create_publisher(LaserScan, '/corner_scan', 10)
 
    def getAngle(self , a, b, c):
        ang = math.degrees(math.atan2(c[1]-b[1], c[0]-b[0]) - math.atan2(a[1]-b[1], a[0]-b[0]))
        return ang + 180 if ang < 0 else ang
  
    def polar_to_cartesian_coordinate(self, ranges, angle_min, angle_max):
        angle_step = (angle_max - angle_min) / len(ranges)
        angle = 0
        points = []

        for range in ranges:
            x = range * np.cos(angle)
            y = range * np.sin(angle)
            angle += angle_step
            points.append([x,y])
            xp.append(x)
            yp.append(y)
        '''
        points_np = np.array(points)
        print(points_np)
        plt.figure()
        plt.scatter(points_np[:,0], points_np[:,1])
        plt.show()
        '''

        return points


    def scan_callback(self,msg):

        points = self.polar_to_cartesian_coordinate(msg.ranges, msg.angle_min, msg.angle_max)
        #######################################################################################################
        '''
        points2 = []

        i = 4
        while i < 356:
            #print(self.getAngle(points[i-4], points[i], points[i+4]))
            if self.getAngle(points[i-4], points[i], points[i+4]) <= 120 and self.getAngle(points[i-4], points[i], points[i+4]) >= 70:
                points2.append(points[i])
                
            i += 1
        points_np = np.array(points2)
        print(points_np)
        plt.figure()
        plt.scatter(points_np[:,0], points_np[:,1])
        plt.show()
       
        '''
        #####################################################################################
        '''
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = msg.header.frame_id
        scan.angle_min = msg.angle_min
        scan.angle_max = msg.angle_max
        scan.angle_increment = msg.angle_increment
        scan.time_increment = msg.time_increment
        scan.range_min = msg.range_min
        scan.range_max = msg.range_max 
        scan.ranges = msg.ranges

        #print(len(ranges))
        
        self.publisher_.publish(scan)
        '''
        i = 1
        while i < 359:
            z = np.polyfit([xp[i-1], xp[i+1]], [yp[i-1], yp[i+1]], 1)
            p = np.poly1d(z)
            print(p)
            if p(xp[i]) - yp[i] >= -0.001 and p(xp[i]) - yp[i] <= 0.001:
                print('line')
                plt.plot([xp[i-1], xp[i], xp[i+1]], [yp[i-1], yp[i], yp[i+1]])
                plt.show()
            i += 1

def main(args=None):
    rclpy.init(args=args)
    feature_extracter = FeatureExtracter()
    rclpy.spin(feature_extracter) 
    feature_extracter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()