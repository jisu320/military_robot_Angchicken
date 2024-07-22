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
        self.clip_dis_maxm = 1.35  # 최대 길이 범위 조정 (m)
        self.clip_dis_minm = 0.7  # 최소 길이 범위 조정 (m)
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

        if self.state == 'slip_corse': # 오른쪽으로 기울어있을 때
            # Perform color-based line tracking
            fs = self.pipeline.wait_for_frames()
            color_f = fs.get_color_frame()
            color_a = np.asanyarray(color_f.get_data())
            color_i = cv2.GaussianBlur(color_a, (7, 7), 0) # 블러 처리 홀수만 가능
            mask = np.zeros_like(color_i)
            roi = color_a[self.roi_start_row:self.roi_end_row, self.roi_start_col:self.roi_end_col]

            hsv_i = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            lower_bound = np.array([40, 10, 10]) #Blue 86, 40, 160, black 40,0,0, white 30, 5, 160
            upper_bound = np.array([130, 255, 255]) #Blue 100, 255, 255 black 90,40,70, white 70, 15, 190


            img_mask = cv2.inRange(hsv_i, lower_bound, upper_bound) # 범위내의 픽셀들은 흰색, 나머지 검은색

            img_result = cv2.bitwise_and(roi, roi, mask = img_mask)

            mask[self.roi_start_row:self.roi_end_row, self.roi_start_col:self.roi_end_col] = img_result

            cv2.rectangle(mask, (self.roi_start_col, self.roi_start_row), (self.roi_end_col, self.roi_end_row), (0, 255, 0), 2)

            cv2.line(mask, (self.cen_x, self.roi_start_row), (self.cen_x, self.roi_end_row), (0, 0, 255), 2)

            left_blue_area = np.sum(img_mask[:, :self.cen_x] == 255)
            right_blue_area = np.sum(img_mask[:, self.cen_x:] == 255)

            cv2.putText(mask, f"L Area: {left_blue_area}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(mask, f"R Area: {right_blue_area}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            cv2.imshow('HVS_TRACK', mask)
            #cv2.imshow('blue mask', img_result)

            state_msg = 0

            L_AREA = 23040
            R_AREA = 23040
            if left_blue_area < L_AREA * 0.4 and right_blue_area < R_AREA * 0.4: # or -> and
                if self.imu_state == 0: # 평지(평화지키미)일 때
                    self.state = 'snow_corse'

                else: #robot tilt right side
                    self.state = 'slip_corse'
                    if right_blue_area < R_AREA * 0.95:
                        state_msg = -4  # Left
                    elif R_AREA * 0.95 <= right_blue_area < R_AREA * 0.96: # 왼쪽으로 더 가게 하고 싶으면 증가 0.96
                        state_msg = 6  # Straight
                    else:
                        state_msg = 4  # Right
            elif left_blue_area < L_AREA * 0.3:
                if self.imu_state == 0: # # 평지(평화지키미)일 때
                    self.state = 'slip_corse'
                    state_msg = 4  # Rightcxxxxxxxxxxxxxxx
            else:
                # Create an Image message to publish
                if self.imu_state == 0: # # 평지(평화지키미)일 때
                    self.state = 'slip_corse'
                    if right_blue_area < R_AREA * 0.93:
                        state_msg = -4  # Left
                    elif R_AREA * 0.93 <= right_blue_area < R_AREA * 0.95: # 왼쪽으로 더 가게 하고 싶으면 증가 0.96
                        state_msg = 6  # Straight
                    else:
                        state_msg = 4  # Right
                else:
                    self.state = 'slip_corse'
                    if right_blue_area < R_AREA * 0.93:
                        state_msg = -4  # Left
                    elif R_AREA * 0.93 <= right_blue_area < R_AREA * 0.95: # 왼쪽으로 더 가게 하고 싶으면 증가 0.96
                        state_msg = 6  # Straight
                    else:
                        state_msg = 4  # Right
                # Publish the Image message

            self.camera_state_publisher_.publish(Int32(data=state_msg))



        if self.state == 'snow_corse': # 평지(평화지키미)일 때
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

            state_msg = 0

            if l_area > r_area * 1.4:
                state_msg = -3  # Left
            elif r_area > l_area * 1.4:
                state_msg = 3  # Right
            else:
                state_msg = 0  # Straight

            self.camera_state_publisher_.publish(Int32(data=state_msg))
            cv2.imshow('Depth Track', bg_removed)


        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
