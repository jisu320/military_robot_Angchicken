import rclpy
from rclpy.node import Node
import numpy as np
import pyrealsense2 as rs
from std_msgs.msg import Int32, Float32

class CameraStatePublisher(Node):

    def __init__(self):
        super().__init__('camera_state_publisher')
        self.camera_state_publisher_ = self.create_publisher(Int32, 'camera_state', 10)
        self.accel_publisher_ = self.create_publisher(Float32, 'accel_values', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_camera_state)

        self.p = rs.pipeline()
        conf = rs.config()
        conf.enable_stream(rs.stream.accel)
        conf.enable_stream(rs.stream.gyro)
        self.prof = self.p.start(conf)

        self.alpha = 0.3 # imu 값 필터링 . 낮을수록 필터 강해짐.
        self.beta = 1 - self.alpha
        self.prev_filtered_accel = np.array([0.0, 0.0, 0.0])

        self.state_window = []
        self.state_window_size = 5 # 상태 필터 값. 높으면 필터 강해짐.

    def publish_camera_state(self):
        f = self.p.wait_for_frames()
        accel = np.array([f[0].as_motion_frame().get_motion_data().x])
        filtered_accel = self.apply_complementary_filter(accel)
        state = self.calculate_camera_state(filtered_accel)

        self.state_window.append(state)

        if len(self.state_window) > self.state_window_size:
            self.state_window.pop(0)

        avg_state = int(np.mean(self.state_window))

        msg_camera_state = Int32()
        msg_camera_state.data = avg_state

        msg_accel = Float32()
        msg_accel.data = filtered_accel[0]
        self.camera_state_publisher_.publish(msg_camera_state)
        self.accel_publisher_.publish(msg_accel)

        self.get_logger().info(f'Filtered Accel: {filtered_accel}')
        self.get_logger().info(f'Camera State: {avg_state}')

    def apply_complementary_filter(self, accel):
        filtered_accel = self.alpha * accel + self.beta * self.prev_filtered_accel
        self.prev_filtered_accel = filtered_accel
        return filtered_accel

    def calculate_camera_state(self, accel):
        threshold_accel = 0.02# 각 임계값. imu 값 튀면 증가 / 1,-1로 변화가 잘 안 되면 감소 1.5, spring, fall 1.2, winter 0.02
        if accel[0] < -threshold_accel:
            state = 1  # Right
        elif accel[0] > threshold_accel:
            state = -1  # Left
        else:
            state = 0

        return state

def main(args=None):
    rclpy.init(args=args)

    camera_state_publisher = CameraStatePublisher()

    rclpy.spin(camera_state_publisher)

    camera_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
