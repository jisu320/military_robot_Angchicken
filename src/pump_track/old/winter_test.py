import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs
from std_msgs.msg import Int32, String

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = CvBridge()
        self.state = 'slip_corse'  # Start with slip_corse
        self.publisher = self.create_publisher(Image, '/line_instructions', 10)
        self.camera_state_publisher_ = self.create_publisher(
            Int32,
            'camera_state_output',
            10
        )
        self.subscription = self.create_subscription(
            Int32,
            'camera_state',
            self.camera_state_callback,
            10
        )
        self.timer = self.create_timer(0.1, self.process_image)
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.cen_x = 320
        self.roi_start_row = int(480 * 0.35)
        self.roi_end_row = int(480 * 0.5)
        self.roi_start_col = int(640 * 0)
        self.roi_end_col = int(640 * 1)
        self.yellow_color = [0, 255, 255]
        self.bridge = CvBridge()
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        profile = self.pipeline.start(self.config)
        self.depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        self.get_logger().info(f"Depth Scale is: {self.depth_scale}")
        self.clip_dis_maxm = 1.3  # 최대 길이 범위 조정 (m)
        self.clip_dis_minm = 0.8  # 최소 길이 범위 조정 (m)
        self.clip_dis_max = self.clip_dis_maxm / self.depth_scale
        self.clip_dis_min = self.clip_dis_minm / self.depth_scale
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        self.imu_state = 1

    def camera_state_callback(self, msg):
        self.imu_state = msg.data
        self.get_logger().info(f'IMU_State: {msg.data}')

    def process_image(self):
        fs = self.pipeline.wait_for_frames()
        color_frame = fs.get_color_frame()
        align_fs = self.align.process(fs)
        depth_frame = align_fs.get_depth_frame()


        if not color_frame or not depth_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())


        if self.state == 'slip_corse':
            # Perform color-based line tracking
            fs = self.pipeline.wait_for_frames()
            color_f = fs.get_color_frame()
            color_a = np.asanyarray(color_f.get_data())
            color_i = cv2.GaussianBlur(color_a, (7, 7), 0)
            hsv_i = cv2.cvtColor(color_i, cv2.COLOR_BGR2HSV)
            roi = color_i[self.roi_start_row:self.roi_end_row, self.roi_start_col:self.roi_end_col]

            dominant_color = np.mean(roi, axis=(0, 1))
            lower_bound = dominant_color - np.array([10, 70, 70])
            upper_bound = dominant_color + np.array([130, 255, 255])

            mask = np.zeros_like(hsv_i)
            mask[self.roi_start_row:self.roi_end_row, self.roi_start_col:self.roi_end_col, :] = 255
            hsv_i = cv2.bitwise_and(color_i, mask)

            dominant_color = np.mean(roi, axis=(0, 1))
            lower_bound = dominant_color - np.array([30, 70, 70])
            upper_bound = dominant_color + np.array([130, 255, 255])

            mask = cv2.inRange(roi, lower_bound, upper_bound)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            masked_pixel_count = 0

            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 100:
                    component_mask = np.zeros_like(mask)
                    cv2.drawContours(component_mask, [contour], -1, (255, 255, 0), thickness=cv2.FILLED)
                    roi[component_mask > 0] = self.yellow_color
                    masked_pixel_count += area

            hsv_i[self.roi_start_row:self.roi_end_row, self.roi_start_col:self.roi_end_col] = roi
            cv2.rectangle(hsv_i, (self.roi_start_col, self.roi_start_row), (self.roi_end_col, self.roi_end_row), (0, 255, 0), 2)
            cv2.line(hsv_i, (self.cen_x, self.roi_start_row), (self.cen_x, self.roi_end_row), (0, 0, 255), 2)

            color_i[self.roi_start_row:self.roi_end_row, self.roi_start_col:self.roi_end_col] = roi
            cv2.rectangle(color_i, (self.roi_start_col, self.roi_start_row), (self.roi_end_col, self.roi_end_row), (0, 255, 0), 2)
            cv2.line(color_i, (self.cen_x, self.roi_start_row), (self.cen_x, self.roi_end_row), (0, 0, 255), 2)

            left_mask = mask[:, :self.cen_x - self.roi_start_col].copy()
            right_mask = mask[:, self.cen_x - self.roi_start_col:].copy()

            l_area = np.sum(left_mask == 255)
            r_area = np.sum(right_mask == 255)

            cv2.putText(hsv_i, f"L Area: {l_area}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(hsv_i, f"R Area: {r_area}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            state_msg = 0

            L_AREA = 23040
            R_AREA = 23040

            if r_area < R_AREA * 0.97:
                state_msg = -1  # Left
            elif R_AREA * 0.97 <= r_area < R_AREA * 0.98:
                state_msg = 0  # Straight
            else:
                state_msg = 1  # Right

            if self.imu_state == 0: # # 평지(평화지키미)일 때
                self.state = 'snow_corse'
            elif self.imu_state == 1:
                self.state = 'slip_corse'
            # Publish the Image message
            self.camera_state_publisher_.publish(Int32(data=state_msg))
            cv2.imshow('Track', hsv_i)

        if self.imu_state == 0: # 평지(평화지키미)일 때
            # Perform obstacle detection using depth data
            align_fs = self.align.process(fs)
            align_depth_f = align_fs.get_depth_frame()
            color_f = align_fs.get_color_frame()

            if not align_depth_f or not color_f:
                return

            depth_i = np.asanyarray(align_depth_f.get_data())
            color_i = np.asanyarray(color_f.get_data())

            mask = np.zeros_like(color_i)
            mask[self.roi_start_row:self.roi_end_row, self.roi_start_col:self.roi_end_col, :] = 255

            masked_color_i = cv2.bitwise_and(color_i, mask)

            grey_color = 0
            depth_i_3d = np.dstack((depth_i, depth_i, depth_i))
            binary_mask = np.where((depth_i_3d > self.clip_dis_max) | (depth_i_3d < self.clip_dis_min) | (depth_i_3d <= 0), 0, 1)

            l_mask = binary_mask[self.roi_start_row:self.roi_end_row, self.roi_start_col:self.cen_x]
            r_mask = binary_mask[self.roi_start_row:self.roi_end_row, self.cen_x:self.roi_end_col]

            l_area = np.sum(l_mask)
            r_area = np.sum(r_mask)

            bg_removed = np.where((depth_i_3d > self.clip_dis_max) | (depth_i_3d < self.clip_dis_min) | (depth_i_3d <= 0), grey_color, masked_color_i)
            cv2.rectangle(bg_removed, (self.roi_start_col, self.roi_start_row), (self.roi_end_col, self.roi_end_row), (0, 255, 0), 2)
            cv2.line(bg_removed, (self.cen_x, self.roi_start_row), (self.cen_x, self.roi_end_row), (0, 0, 255), 2)

            cv2.putText(bg_removed, f"L Pix: {l_area}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(bg_removed, f"R Pix: {r_area}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            if l_area > r_area * 1.1:
                state_msg = -4  # Left
            elif r_area > l_area * 1.1:
                state_msg = 4  # Right
            else:
                state_msg = 0  # Straight

            self.camera_state_publisher_.publish(Int32(data=state_msg))
            cv2.imshow('Depth Track', bg_removed)

        else:
            print('Invalid state, cannot process.')

        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
