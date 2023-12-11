
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.odom_counter = 0

        self.odom_publisher_ = self.create_publisher(Odometry, '/odom_downsample', 10)

        self.subscription = self.create_subscription(
            String,
            '/test',
            self.listener_callback,
            10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def odom_callback(self, msg):   
        self.odom_counter += 1
        if self.odom_counter % 20 == 0 :
            print("Got odom {}".format(self.odom_counter))
            print(msg.pose.pose.position.x)
            self.odom_publisher_.publish(msg)


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
