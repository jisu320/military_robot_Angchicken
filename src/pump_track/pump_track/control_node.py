import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int32, Float32MultiArray  # Import Float32MultiArray for publishing floats
import time

class ODriveNode(Node):
    def __init__(self):
        super().__init__('ODriveNode')

        # Create ROS 2 subscribers
        self.real_x_subscription = self.create_subscription(
            Float32,
            '/airdrop/real_x',
            self.real_x_callback,
            10
        )
        self.yawing_angle_subscription = self.create_subscription(
            Float32,
            '/airdrop/yawing_angle',
            self.yawing_angle_callback,
            10
        )
        self.airdrop_detected_subscription = self.create_subscription(
            Bool,
            '/airdrop/airdrop_detected',
            self.airdrop_detected_callback,
            10
        )
        self.real_x_yawing_angle_publisher = self.create_publisher(
            Float32MultiArray,  # Use Float32MultiArray for publishing floats
            '/airdrop/real_x_yawing_angle',
            10
        )

        self.airdrop_detected = False
        self.sonar_detected = False
        self.order_msg = False
        self.real_x = None
        self.yawing_angle = None
        self.received_arduino_number = None  # Initialize to None
        self.number_msg = None

        # Create a subscriber to receive sonar airdrop detection status
        self.sonar_airdrop_detected_subscription = self.create_subscription(
            Bool,
            '/sonar/airdrop_detected',  # Replace with the Arduino's topic
            self.sonar_airdrop_detected_callback,
            10
        )

        # Create a publisher to send a number to the Arduino
        self.arduino_number_publisher = self.create_publisher(
            Int32,  # Use Int32 for integers
            '/arduino/number',  # Replace with your desired topic
            10
        )

        # Create a subscriber to receive a number from the Arduino
        self.arduino_number_subscription = self.create_subscription(
            Int32,  # Use Int32 for integers
            '/arduino/number',  # Replace with the Arduino's topic
            self.arduino_number_callback,
            10
        )


    def airdrop_detected_callback(self, msg):
        self.airdrop_detected = msg.data
        if self.airdrop_detected:
            self.get_logger().info('Depth Airdrop detected!')

    def real_x_callback(self, msg):
        self.real_x = msg.data
        self.get_logger().info(f'Received real_x: {msg.data} cm')

    def yawing_angle_callback(self, msg):
        self.yawing_angle = msg.data
        self.get_logger().info(f'Received yawing_angle: {msg.data} degrees')

    def sonar_airdrop_detected_callback(self, msg):
        self.sonar_detected = msg.data
        self.get_logger().info(f'Sonar detected airdrop: {msg.data}')

    def publish_number_to_arduino(self, number):
        self.number_msg = Int32()  # Create an Int32 message
        self.number_msg.data = number  # Set the data field of the message with the number
        self.get_logger().info(f'Sending number to Arduino: {number}')
        self.arduino_number_publisher.publish(self.number_msg)

    def arduino_number_callback(self, msg):
        self.received_number = msg.data
        self.get_logger().info(f'Received number from Arduino: {msg.data}')
        self.received_arduino_number = msg.data

    def control_motors_based_on_airdrop(self):
        if not self.airdrop_detected:
            self.get_logger().info('Waiting for Depth airdrop detection...')
            return
        self.get_logger().info(f'Yawing angle within range: {self.yawing_angle}')
        if self.real_x is None or self.yawing_angle is None:
            self.get_logger().info('Waiting for valid data...')
        else:
            self.get_logger().info(f'Real_x: {self.real_x}')
            data = [self.real_x, self.yawing_angle, 0]  # Add "pos" as the mode
            msg = Float32MultiArray(data=data)
            self.real_x_yawing_angle_publisher.publish(msg)
            rclpy.shutdown()
        # Rest of your code...
        self.get_logger().info('Wait for 1 second, then publish to order')
        time.sleep(1)

    def main_loop(self):
        while rclpy.ok():
            self.control_motors_based_on_airdrop()
            rclpy.spin_once(self, timeout_sec=1.0)

def main(args=None):
    print('Hi from odrive_ros2.')
    rclpy.init()
    node = ODriveNode()
    node.main_loop()
    rclpy.spin_once(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
