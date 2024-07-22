#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import odrive
from odrive.enums import *
import time
import math

class ODriveControlNode(Node):
    def __init__(self):
        super().__init__('odrive_control_node')

        # Initialize ODrive motor and calibrate it
        self.my_drive = odrive.find_any()
        self.calibrate_drive()
        # Create ROS subscriber
        self.subscriber = self.create_subscription(
            Float32MultiArray,
            '/airdrop/real_x_yawing_angle',  # Replace with your input topic
            self.callback,
            10
        )

        # Initialize control mode (0 for position, 1 for velocity)
        self.control_mode = None

    def calibrate_drive(self):
        self.get_logger().info('Starting calibration...')
        self.my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self.my_drive.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while self.my_drive.axis0.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)

        self.my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.my_drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        self.get_logger().info('Calibration completed.')

    def callback(self, msg):
        if len(msg.data) != 3:
            self.get_logger().error('Invalid message format. Please provide [real_x, yawing_angle, mode].')
            return

        real_x, yawing_angle, mode = msg.data

        if mode == 0:
            # Position control logic
            self.control_mode = 0
            # Calculate position setpoint based on real_x and yawing_angle
            position_setpoint_l = calculate_position_setpoint_l(real_x, yawing_angle)
            position_setpoint_r = calculate_position_setpoint_r(real_x, yawing_angle)
            self.position_control_drive(position_setpoint_l, position_setpoint_r)
        elif mode == 1:
            # Velocity control logic
            self.control_mode = 1
            # Implement velocity control based on your criteria
            velocity_setpoint_l = calculate_velocity_setpoint_l(yawing_angle)
            velocity_setpoint_r = calculate_velocity_setpoint_r(yawing_angle)
            self.velocity_control_drive(velocity_setpoint_l, velocity_setpoint_r)  # For example, control based on yawing_angle
        else:
            self.get_logger().error('Invalid mode. Please use 0 for position or 1 for velocity.')

    def position_control_drive(self, position_setpoint_l, position_setpoint_r):
        # Position control logic here
        self.my_drive.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        self.my_drive.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        self.my_drive.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
        self.my_drive.axis1.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
        print("Controller is " + str(self.my_drive.axis0.controller.config.control_mode))
        print("Controller is " + str(self.my_drive.axis1.controller.config.control_mode))

        # Set position setpoint for both axes
        self.my_drive.axis0.controller.input_pos = position_setpoint_l
        self.my_drive.axis1.controller.input_pos = -position_setpoint_r

        # Print the position setpoint once
        self.get_logger().info('Position setpoint is axis0: %.2f, axis1: %.2f' %
                               (self.my_drive.axis0.controller.input_pos,
                                self.my_drive.axis1.controller.input_pos))

    def velocity_control_drive(self, velocity_setpoint_l, velocity_setpoint_r):
        # Velocity control logic here
        self.my_drive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.my_drive.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.my_drive.axis0.controller.config.vel_ramp_rate = 10
        self.my_drive.axis1.controller.config.vel_ramp_rate = 10
        self.my_drive.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
        self.my_drive.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP
        print("Controller is " + str(self.my_drive.axis0.controller.config.control_mode))
        print("Controller is " + str(self.my_drive.axis1.controller.config.control_mode))

        # Calculate velocity setpoint based on yawing_angle

        # Set velocity setpoint for both axes
        self.my_drive.axis0.controller.input_vel = velocity_setpoint_l
        self.my_drive.axis1.controller.input_vel = velocity_setpoint_r

        # Print the velocity setpoint once
        self.get_logger().info('Velocity setpoint is axis0: %.2f, axis1: %.2f' %
                               (self.my_drive.axis0.controller.input_vel,
                                self.my_drive.axis1.controller.input_vel))

def calculate_position_setpoint_l(real_x, yawing_angle):
    # Calculate position setpoint based on real_x and yawing_angle
    # Modify this function according to your specific requirements

    # Initialize position setpoint to zero
    position_setpoint_l = 0.0

    # Determine the direction based on yawing_angle (positive for right, negative for left)
    if yawing_angle > 0:
        # Turn right by the yawing_angle value
        position_setpoint_l += (17/90) * yawing_angle
    elif yawing_angle < 0:
        # Turn left by the absolute value of yawing_angle
        position_setpoint_l += -(17/90) * abs(yawing_angle)

    # Move forward by the specified real_x value
    position_setpoint_l += 0.385 * (real_x) + 2 #here!!!!!!!!!!!!!!!!!!!

    return position_setpoint_l

def calculate_position_setpoint_r(real_x, yawing_angle):
    # Calculate position setpoint based on real_x and yawing_angle
    # Modify this function according to your specific requirements

    # Initialize position setpoint to zero
    position_setpoint_r = 0.0

    # Determine the direction based on yawing_angle (positive for right, negative for left)
    if yawing_angle > 0:
        # Turn right by the yawing_angle value
        position_setpoint_r += -(17/90) * yawing_angle
    elif yawing_angle < 0:
        # Turn left by the absolute value of yawing_angle
        position_setpoint_r += (17/90) * abs(yawing_angle)

    # Move forward by the specified real_x value
    position_setpoint_r += 0.385 * (real_x) + 2 #here!!!!!!!!!!!!!

    return position_setpoint_r

def calculate_velocity_setpoint_l(yawing_angle):
    # Calculate velocity setpoint based on yawing_angle or other criteria
    # Modify this function according to your specific requirements
    velocity_setpoint_l = 0
    # Determine the direction based on yawing_angle (positive for right, negative for left)
    if yawing_angle > 0:
        # Turn right by the yawing_angle value (velocity control)
        velocity_setpoint_l = 3
    elif yawing_angle < 0:
        # Turn left by the absolute value of yawing_angle (velocity control)
        velocity_setpoint_l = 1.5
    else:
        # No turning required, maintain zero velocity
        velocity_setpoint_l = 3

    return velocity_setpoint_l

def calculate_velocity_setpoint_r(yawing_angle):
    # Calculate velocity setpoint based on yawing_angle or other criteria
    # Modify this function according to your specific requirements
    velocity_setpoint_r = 0
    # Determine the direction based on yawing_angle (positive for right, negative for left)
    if yawing_angle > 0:
        # Turn right by the yawing_angle value (velocity control)
        velocity_setpoint_r = -1.5
    elif yawing_angle < 0:
        # Turn left by the absolute value of yawing_angle (velocity control)
        velocity_setpoint_r = -3
    else:
        # No turning required, maintain zero velocity
        velocity_setpoint_r = -3

    return velocity_setpoint_r




def main(args=None):
    rclpy.init(args=args)
    node = ODriveControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
