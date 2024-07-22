import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class SubscribeAndPublish(Node):

    def __init__(self):
        super().__init__('subscribe_and_publish')
        self.subscriber = self.create_subscription(
            String,
            '/line_instructions',
            self.command_callback,
            10
        )
        self.publisher = self.create_publisher(
            Twist,
            '/diffbot_base_controller/cmd_vel_unstamped',
            10
        )
        self.get_logger().info("Subscriber and Publisher created")

        self.command_mapping = {
            '1': (0.5, 0.0, 0.0, 0.0, 0.0, 0.0),  # Go Straight
            '2': (0.3, 0.0, 0.0, 0.0, 0.0, -10.0),  # Turn Right
            '3': (0.3, 0.0, 0.0, 0.0, 0.0, 10.0),   # Turn Left
            '4': (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)    # STOP
        }

    def command_callback(self, msg):
        if msg.data in self.command_mapping:
            cmd_values = self.command_mapping[msg.data]
            twist_msg = Twist()
            twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z, \
            twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = cmd_values
            self.publisher.publish(twist_msg)
            self.get_logger().info(f"Received command: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = SubscribeAndPublish()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
