
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.get_logger().info("Starting minimal_subscriber node...")

        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  				# Prevent unused variable warning

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.odom_subscription  			# Prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(msg.data)		# Simply print the data 

    def odom_callback(self, msg):
        self.get_logger().info("Odom received")	# This is going to add a lot of "noise" to your terminal !
        print(msg)


def main(args=None):
    
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
