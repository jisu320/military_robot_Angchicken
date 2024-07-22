#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32  # Import the String message type for your custom message

class JoySubscriberNode(Node):
    def __init__(self):
        super().__init__('joy_publisher')
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

    def joy_callback(self, msg):

      # Create a publisher for your custom message
      custom_pub = self.create_publisher(Int32, '/arduino/number_pub', 10)

      # Iterate through the buttons array and publish different messages based on button presses
      for i, button_state in enumerate(msg.buttons):
          if button_state == 1:
              if i == 1:  # Check if Button 1 is pressed (Button indices start from 0)
                  custom_msg = Int32()
                  custom_msg.data = 2
                  custom_pub.publish(custom_msg)
                  self.get_logger().info("Button 2 is pressed, publishing '2'")
              elif i == 2:  # Check if Button 32 is pressed (Button indices start from 0)
                  custom_msg = Int32()
                  custom_msg.data = 3
                  custom_pub.publish(custom_msg)
                  self.get_logger().info("Button 3 is pressed, publishing '3'")
              elif i == 3:  # Check if Button 3 is pressed (Button indices start from 0)
                  custom_msg = Int32()
                  custom_msg.data = 4
                  custom_pub.publish(custom_msg)
                  self.get_logger().info("Button 4 is pressed, publishing '4'")
              elif i == 0:  # Check if Button 3 is pressed (Button indices start from 0)
                  custom_msg = Int32()
                  custom_msg.data = 1
                  custom_pub.publish(custom_msg)
                  self.get_logger().info("Button 1 is pressed, publishing '1'")




    def publish_custom_message(self):
        # Create a publisher for your custom message
        custom_pub = self.create_publisher(Int32, '/arduino/number_pub', 10)

        # Create a custom message and publish it
        custom_msg = Int32()
        custom_msg.data = self.msg.data
        custom_pub.publish(custom_msg)


def main():
    rclpy.init()
    node = JoySubscriberNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
