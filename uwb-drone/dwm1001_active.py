#!/usr/bin/env python3
""" 
    For more info on the documentation go to https://www.decawave.com/sites/default/files/dwm1001-api-guide.pdf
"""

import time, serial, os, sys, random

import rclpy
import rclpy.node
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.exceptions import ROSInterruptException
from rcl_interfaces.msg import ParameterType


from geometry_msgs.msg  import Pose
from geometry_msgs.msg  import PoseStamped
from sensor_msgs.msg    import Range

from dwm1001_apiCommands import DWM1001_API_COMMANDS

class dwm1001_localizer:

    def __init__(self) :
        """
        Initialize the node, open serial port
        """

        # Init node
        # rospy.init_node('DWM1001_Active_{}'.format(random.randint(0,100000)), anonymous=False)
        rclpy.init()
        self.node = rclpy.create_node('DWM1001_Active_{}'.format(random.randint(0,100000)))#, anonymous=False)

        # Get port and tag name
        try:
            self.dwm_port = self.node.get_parameter('port').value        
        except rclpy.exceptions.ParameterNotDeclaredException :
            self.dwm_port = "/dev/ttyACM0"
        try:
            self.tag_name = self.node.get_parameter('tag_name').value
        except rclpy.exceptions.ParameterNotDeclaredException :
            self.tag_name = "Tag"
        try:
            self.use_network = self.node.get_parameter('use_network').value        
        except rclpy.exceptions.ParameterNotDeclaredException :
            self.use_network = False
        try:
            self.network = self.node.get_parameter('network').value
        except rclpy.exceptions.ParameterNotDeclaredException :
            self.network = "default"
        try:
            self.verbose = self.node.get_parameter('verbose').value
        except rclpy.exceptions.ParameterNotDeclaredException :
            self.verbose = False

        
        # Set a ROS rate
        # self.rate = rospy.Rate(1)
        self.rate = self.node.create_rate(1)
        
        # Empty dictionary to store topics being published
        self.topics = {}
        
        # Serial port settings
        self.serialPortDWM1001 = serial.Serial(
            port = self.dwm_port,
            baudrate = 115200,
            parity = serial.PARITY_ODD,
            stopbits = serial.STOPBITS_TWO,
            bytesize = serial.SEVENBITS
        )
    

    def main(self) :
        """
        Initialize port and dwm1001 api
        :param:
        :returns: none
        """

        # close the serial port in case the previous run didn't closed it properly
        self.serialPortDWM1001.close()
        # sleep for one sec
        time.sleep(1)
        # open serial port
        self.serialPortDWM1001.open()


        # check if the serial port is opened
        if(self.serialPortDWM1001.isOpen()):
            self.node.get_logger().info("Port opened: "+ str(self.serialPortDWM1001.name) )
            # start sending commands to the board so we can initialize the board
            self.initializeDWM1001API()
            # give some time to DWM1001 to wake up
            time.sleep(2)
            # send command lec, so we can get positions is CSV format
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.LEC)
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            self.node.get_logger().info("Reading DWM1001 coordinates")
        else:
            self.node.get_logger().info("Can't open port: "+ str(self.serialPortDWM1001.name))

        try:
        # if(True):
            while rclpy.ok():
                # print("while")
                # just read everything from serial port
                serialReadLine = self.serialPortDWM1001.read_until().decode("utf-8") 

                try:
                    self.publishTagPositions(serialReadLine)

                except IndexError:
                    self.node.get_logger().info("Found index error in the network array! DO SOMETHING!")



        except KeyboardInterrupt:
            self.node.get_logger().info("Quitting DWM1001 Shell Mode and closing port, allow 1 second for UWB recovery")
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

        finally:
            self.node.get_logger().info("Quitting, and sending reset command to dev board")
            # self.serialPortDWM1001.reset_input_buffer()
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            self.rate.sleep()
            serialReadLine = self.serialPortDWM1001.read_until()
            if "reset" in serialReadLine:
                self.node.get_logger().info("succesfully closed ")
                self.serialPortDWM1001.close()


    def publishTagPositions(self, serialData):
        """
        Publish anchors and tag in topics using Tag and Anchor Object
        :param networkDataArray:  Array from serial port containing all informations, tag xyz and anchor xyz
        :returns: none
        """

        # print(type(serialData))
        # print(serialData)

        arrayData = [x.strip() for x in serialData.strip().split(',')]
        # print(arrayData)

        # If getting a tag position
        if "DIST" in arrayData[0] :

            # The number of elements should be 2 + 6*NUMBER_OF_ANCHORS + 5 (TAG POS)
            # number_of_anchors = int((len(arrayData) - 7)/6)

            number_of_anchors = int(arrayData[1].replace("'",""))

            # print(type(number_of_anchors))
            # print(number_of_anchors)

            for i in range(number_of_anchors) :

                node_id = arrayData[2+6*i]
                first_time = False
                if node_id not in self.topics :
                    first_time = True
                    self.topics[node_id] = self.node.create_publisher( 
                        PoseStamped,
                        '/dwm1001' + 
                        "{}".format("/"+self.network if self.use_network else "") + 
                        '/anchor/' + node_id + 
                        "/position",
                        100
                    )
                    self.topics[node_id+"_dist"] = self.node.create_publisher(
                        Range,
                        '/dwm1001' + 
                        "{}".format("/"+self.network if self.use_network else "") + 
                        '/tag/' + self.tag_name +
                        '/to/anchor/' + node_id +
                        "/distance",
                        100
                    )
                try :
                    p = PoseStamped()
                    p.header.stamp = self.node.get_clock().now().to_msg() 
                    p.pose.position.x = float(arrayData[4+6*i])
                    p.pose.position.y = float(arrayData[5+6*i])
                    p.pose.position.z = float(arrayData[6+6*i])
                    p.pose.orientation.x = 0.0
                    p.pose.orientation.y = 0.0
                    p.pose.orientation.z = 0.0
                    p.pose.orientation.w = 1.0
                    self.topics[node_id].publish(p)
                except :
                    pass
                try :
                    dist = float(arrayData[7+6*i])

                    r = Range()
                    r.header.stamp = self.node.get_clock().now().to_msg()
                    r.range =dist


                    self.topics[node_id+"_dist"].publish(range)
                except :
                    pass

                if self.verbose or first_time :
                    self.node.get_logger().info("Anchor " + node_id + ": "
                                  + " x: "
                                  + str(p.pose.position.x)
                                  + " y: "
                                  + str(p.pose.position.y)
                                  + " z: "
                                  + str(p.pose.position.z))

            # Now publish the position of the tag itself
            if "POS" in arrayData[-5] :

                # Topic is now a tag with same name as node_id
                first_time = False
                if self.tag_name not in self.topics :
                    first_time = True
                    self.topics[self.tag_name] = self.node.create_publisher(PoseStamped,'/dwm1001/tag/'+self.tag_name+"/position",100)
                p = PoseStamped()
                p.header.stamp = self.node.get_clock().now().to_msg() 
                p.pose.position.x = float(arrayData[-4])
                p.pose.position.y = float(arrayData[-3])
                p.pose.position.z = float(arrayData[-2])
                p.pose.orientation.x = 0.0
                p.pose.orientation.y = 0.0
                p.pose.orientation.z = 0.0
                p.pose.orientation.w = 1.0
                self.topics[self.tag_name].publish(p)

                if self.verbose or first_time :
                    self.node.get_logger().info("Tag " + self.tag_name + ": "
                                  + " x: "
                                  + str(p.pose.position.x)
                                  + " y: "
                                  + str(p.pose.position.y)
                                  + " z: "
                                  + str(p.pose.position.z))

    def initializeDWM1001API(self):
        """
        Initialize dwm1001 api, by sending sending bytes
        :param:
        :returns: none
        """
        # reset incase previuos run didn't close properly
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
        # send ENTER two times in order to access api
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half a second
        time.sleep(0.5)
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half second
        time.sleep(0.5)
        # send a third one - just in case
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)




if __name__ == '__main__':
    try:
        dwm1001 = dwm1001_localizer()
        dwm1001.main()
        # rclpy.spin()
    except ROSInterruptException:
        pass
