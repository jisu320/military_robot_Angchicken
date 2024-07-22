import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs

class RealsenseNode(Node):
    def __init__(self):
        super().__init__('realsense_node')

        self.publisher = self.create_publisher(Int32, 'camera_state_output', 10)
        self.timer = self.create_timer(0.1, self.process_image)
        self.bridge = CvBridge()

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))
        # 해상도
        self.cen_x = 212 # 424/2
        self.roi_start_row = int(240 * 0.35)
        self.roi_end_row = int(240 * 0.5)
        self.roi_start_col = int(424 * 0.05)
        self.roi_end_col = int(424 * 0.95)

        self.found_rgb = False

        for s in self.device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                self.found_rgb = True
                break
        if not self.found_rgb:
            self.get_logger().error("require Depth / Color sensor")
            return

        self.config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 15) # 해상도 / 프레임

        if self.device_product_line == 'L500':
            self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 15)
        else:
            self.config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 15)

        profile = self.pipeline.start(self.config)

        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        self.get_logger().info(f"Depth Scale : {depth_scale}")

        self.clip_dis_maxm = 1.3  # 최대 길이 범위 조정 (m)
        self.clip_dis_minm = 0.8  # 최소 ..
        self.clip_dis_max = self.clip_dis_maxm / depth_scale
        self.clip_dis_min = self.clip_dis_minm / depth_scale

        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        self.cv_window = cv2.namedWindow('Realsense Image', cv2.WINDOW_NORMAL)

    ############################추가 필요
    ############################추가 필요

    def process_image(self):
        try:
            fs = self.pipeline.wait_for_frames()
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

            # dir 결정할 비율
            if l_area > r_area * 1.1: # 왼쪽으로 더 가게 하고 싶으면 감소
                direction = -4  # Left
            elif r_area > l_area * 1.1: # 오른쪽으로 더 가게 하고 싶으면 감소
                direction = 4  # Right
            else:
                direction = 0  # Straight
            self.get_logger().info(str(direction))

            msg = Int32()
            msg.data = direction
            self.publisher.publish(msg)
            cv2.imshow('Depth Track', bg_removed)
            cv2.waitKey(1)

        except KeyboardInterrupt:
            pass

def main(args=None):
    rclpy.init(args=args)
    realsense_node = RealsenseNode()
    rclpy.spin(realsense_node)
    realsense_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
