import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import pyrealsense2 as rs
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraStateSubscriber(Node):
    def __init__(self):
        super().__init__('track_imu_node')
        self.subscription = self.create_subscription(
            Int32,
            'camera_state',
            self.camera_state_callback,
            10
        )

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 15) # 해상도 / 프레임
        self.pipeline.start(config)

        # 해상도 변경시 cen_x=변경된 해상도 1/2 값/ 나머지=변경된 값 그대로
        self.cen_x = 212
        self.roi_start_row = int(240 * 0.35)
        self.roi_end_row = int(240 * 0.5)
        self.roi_start_col = int(424 * 0.05)
        self.roi_end_col = int(424 * 0.95)
        self.yellow_color = [0, 255, 255]

        self.camera_state_publisher_ = self.create_publisher(
            Int32,
            'camera_state_output',
            10
        )

        self.image_publisher = self.create_publisher(
            Image,
            'camera_image',
            10
        )

        self.bridge = CvBridge()

    def camera_state_callback(self, msg):
        self.state = msg.data

        fs = self.pipeline.wait_for_frames()
        color_f = fs.get_color_frame()
        color_a = np.asanyarray(color_f.get_data())
        color_i = cv2.GaussianBlur(color_a, (9, 9), 0) # 블러 처리, 홀수만.
        hsv_i = cv2.cvtColor(color_i, cv2.COLOR_BGR2HSV)
        roi = hsv_i[self.roi_start_row:self.roi_end_row, self.roi_start_col:self.roi_end_col]

        mask = np.zeros_like(hsv_i)
        mask[self.roi_start_row:self.roi_end_row, self.roi_start_col:self.roi_end_col, :] = 255
        hsv_i = cv2.bitwise_and(color_i, mask)

        dominant_color = np.mean(roi, axis=(0, 1)) # ROI 내 평균 색상
        lower_bound = dominant_color - np.array([20, 70, 70]) # 하한/ 증가=초록색 더 인식/ 감소=초록색 덜 인식
        upper_bound = dominant_color + np.array([130, 255, 255]) # 상한/ 증가=진청색 더 인식/ 감소=진청색 덜 인식

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

        left_mask = mask[:, :self.cen_x - self.roi_start_col].copy()
        right_mask = mask[:, self.cen_x - self.roi_start_col:].copy()

        l_area = np.sum(left_mask == 255)
        r_area = np.sum(right_mask == 255)

        cv2.putText(hsv_i, f"L Area: {l_area}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(hsv_i, f"R Area: {r_area}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        state_msg = 0
        
        if self.state == -1: # 왼쪽으로 기울어있을때
            roi[:, self.cen_x - self.roi_start_col:] = [0, 0, 0]
            if l_area < 4700: # 오른쪽으로 더 가게 하고 싶으면 증가 (아래도 있음)
                state_msg = 1  # Right
            elif 4700 <= l_area < 6100: # 왼쪽으로 더 가게 하고 싶으면 증가
                state_msg = 0  # Straight
            else:
                state_msg = -1  # Left

        elif self.state == 1: # 오른쪽으로 기울어있을때
            roi[:, :self.cen_x - self.roi_start_col] = [0, 0, 0]
            if r_area < 6500: # 왼쪽으로 더 가게 하고 싶으면 증가 (아래도 있음)
                state_msg = -1  # Left
            elif 6500 <= r_area < 6650: # 오른쪽으로 더 가게 하고 싶으면 증가
                state_msg = 0  # Straight
            else:
                state_msg = 1  # Right

        elif self.state == 0: # 평평할 때
            if l_area > r_area * 1.2: # 왼쪽으로 더 가게하고 싶으면 감소. 1로 하면 안 됨
                state_msg = -1  # Left
            elif r_area > l_area * 1.2: # 오른쪽으로 더 가게하고 싶으면 감소. 1로 하면 안 됨
                state_msg = 1  # Right
            else:
                state_msg = 0  # Straight

        self.camera_state_publisher_.publish(Int32(data=state_msg))
        ros_image = self.bridge.cv2_to_imgmsg(hsv_i, encoding="bgr8")
        self.image_publisher.publish(ros_image)

        cv2.imshow('Track', hsv_i)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            pass

def main(args=None):
    rclpy.init(args=args)
    camera_state_subscriber = CameraStateSubscriber()
    rclpy.spin(camera_state_subscriber)
    camera_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()