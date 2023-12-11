#! /usr/bin/python

import base64
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class CamPublisher(Node):

    def __init__(self):
        super().__init__('image_pub')
        self.image_pub = self.create_publisher(Image, 'Image_PubSubTopic', 10)

        self.cap = cv2.VideoCapture(
        'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, format=(string)NV12,framerate=(fraction)20/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !video/x-raw, format=(string)BGR ! appsink',
        cv2.CAP_GSTREAMER)

        self.cnt = 0
        self.read_image()

    def read_image(self):

        while (True):
            _, frame = self.cap.read()
            try:
                self.cnt += 1


                ## This limits the publishing rate to approx 1/second so you can receive them on your laptops
                if (self.cnt % 30 is 0):

                    #resize to save bandwidth
                    frame = cv2.resize(frame, (640, 360))
                    imgMsg = self.cv2_to_imgmsg(frame, "bgr8")

                    self.image_pub.publish(imgMsg)
            except():
                self.cap.release()
                print("Wrong with camera intial!")
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.cap.release()
                break

    def cv2_to_imgmsg(self, cv2_img, encoding = "passthrough"):
        img_msg = Image()
        img_msg.height = cv2_img.shape[0]
        img_msg.width = cv2_img.shape[1]
        img_msg.encoding = encoding

        if cv2_img.dtype.byteorder == '>':
            img_msg.is_bigendian = True
        img_msg.data = cv2_img.tostring()
        img_msg.step = len(img_msg.data) // img_msg.height
        return img_msg

def main(args=None):
    rclpy.init()
    cam_publisher = CamPublisher()
    rclpy.spin(cam_publisher)

    cam_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()