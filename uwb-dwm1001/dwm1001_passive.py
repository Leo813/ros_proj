#!/usr/bin/env python3
""" 
    For more info on the documentation go to https://www.decawave.com/sites/default/files/dwm1001-api-guide.pdf
"""

import time, serial, os
import rclpy
import rclpy.node
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.exceptions import ROSInterruptException
from rcl_interfaces.msg import ParameterType
from dwm1001_apiCommands            import DWM1001_API_COMMANDS

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg    import Range


class dwm1001_localizer:

    def __init__(self) :
        """
        Initialize the node, open serial port
        """
        
        # Init node
        rclpy.init()
        self.node = rclpy.create_node('DWM1001_Passive')#, anonymous=False)
        
        # Set a ROS rate
        # self.rate = rospy.Rate(1)
        self.rate = self.node.create_rate(1)
        
        # Empty dictionary to store topics being published
        self.topics = {}
        
        # Serial port settings
        # self.dwm_port = rclpy.declare_parameter('~port').value

        try:
            self.dwm_port = self.node.get_parameter('port').value        
        except rclpy.exceptions.ParameterNotDeclaredException :
            self.dwm_port = "/dev/ttyACM0"
        try :
            self.verbose = self.node.get_parameter('verbose').value
        except rclpy.exceptions.ParameterNotDeclaredException :
            self.verbose = False

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
                # just read everything from serial port
                serialReadLine = self.serialPortDWM1001.read_until().decode("utf-8")

                try:
                    self.publishTagPositions(serialReadLine)

                except IndexError:
                    self.node.get_logger().info("Found index error in the network array!DO SOMETHING!")



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
        arrayData = [x.strip() for x in serialData.strip().split(',')]
        # print(arrayData)

        # If getting a tag position
        if "POS" in arrayData[0] :

            tag_id = arrayData[2]

            p = PoseStamped()
            p.pose.position.x = float(arrayData[3].replace("'",""))
            p.pose.position.y = float(arrayData[4].replace("'",""))
            p.pose.position.z = float(arrayData[5].replace("'",""))
            p.pose.orientation.x = 0.0
            p.pose.orientation.y = 0.0
            p.pose.orientation.z = 0.0
            p.pose.orientation.w = 1.0
            p.header.stamp = self.node.get_clock().now().to_msg()   

            if tag_id not in self.topics :
                # self.topics[tag_id] = self.node.create_publisher(PoseStamped,'/dwm1001/tag/'+tag_id+"/position", 100)
                
                self.node.get_logger().info("New tag {}. x: {}m, y: {}m, z: {}m".format(
                    tag_id,
                    p.pose.position.x,
                    p.pose.position.y,
                    p.pose.position.z
                ))
                self.topics[tag_id] = self.node.create_publisher(PoseStamped, "test", 100)
            
            self.topics[tag_id].publish(p)

            if self.verbose :
                self.node.get_logger().info("Tag " + tag_id + ": "
                    + " x: "
                    + str(p.pose.position.x)
                    + " y: "
                    + str(p.pose.position.y)
                    + " z: "
                    + str(p.pose.position.z)
                )



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
        # rospy.spin()
    except ROSInterruptException:
        pass