import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs
from std_msgs.msg import Int32, String

class CameraStateSubscriber(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = CvBridge()
        #self.state = 'fog_corse'  # Start with fog_corse

        self.subscription = self.create_subscription(
            Int32,
            'camera_state',
            self.camera_state_callback,
            10
        )


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

        #self.timer = self.create_timer(0.1, self.camera_state_callback)
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 15) # 해상도 / 프레임
        self.pipeline.start(config)
        self.cen_x = 212
        self.roi_start_row = int(240 * 0.35)
        self.roi_end_row = int(240 * 0.5)
        self.roi_start_col = int(424 * 0.05)
        self.roi_end_col = int(424 * 0.95)
        self.yellow_color = [0, 255, 255]


        self.bridge = CvBridge()


    def camera_state_callback(self, msg):
        self.state = msg.data


        # Perform color-based line tracking
        fs = self.pipeline.wait_for_frames()
        color_f = fs.get_color_frame()
        color_a = np.asanyarray(color_f.get_data())
        color_i = cv2.GaussianBlur(color_a, (5, 5), 0) # 블러 처리 홀수만 가능
        mask = np.zeros_like(color_i)
        roi = color_a[self.roi_start_row:self.roi_end_row, self.roi_start_col:self.roi_end_col]
        hsv_i = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)


        lower_bound = np.array([86, 40, 120])
        upper_bound = np.array([110, 255, 255])

        
        img_mask = cv2.inRange(hsv_i, lower_bound, upper_bound) # 범위내의 픽셀들은 흰색, 나머지 검은색
        img_result = cv2.bitwise_and(roi, roi, mask = img_mask)
        mask[self.roi_start_row:self.roi_end_row, self.roi_start_col:self.roi_end_col] = img_result
        cv2.rectangle(mask, (self.roi_start_col, self.roi_start_row), (self.roi_end_col, self.roi_end_row), (0, 255, 0), 2)
        cv2.line(mask, (self.cen_x, self.roi_start_row), (self.cen_x, self.roi_end_row), (0, 0, 255), 2)
        l_area = np.sum(img_mask[:, :self.cen_x] == 255)
        r_area = np.sum(img_mask[:, self.cen_x:] == 255)
        cv2.putText(mask, f"L Area: {l_area}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(mask, f"R Area: {r_area}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow('Track', mask)
        #cv2.imshow('blue mask', img_result)
        state_msg = 0
        L_AREA = 6876
        R_AREA = 6840
        if self.state == -1: # 왼쪽으로 기울어있을때
            roi[:, self.cen_x - self.roi_start_col:] = [0, 0, 0]
            if l_area < L_AREA * 0.683: # 오른쪽으로 더 가게 하고 싶으면 증가 (아래에도 있음)
                state_msg = 1  # Right
            elif L_AREA * 0.683 <= l_area < L_AREA * 0.887: # 왼쪽으로 더 가게 하고 싶으면 감소
                state_msg = 0  # Straight
            else:
                state_msg = -1  # Left

        elif self.state == 1: # 오른쪽으로 기울었을때
            roi[:, :self.cen_x - self.roi_start_col] = [0, 0, 0]
            if r_area < R_AREA * 0.97: # 왼쪽으로 더 가게 하고 싶으면 증가 (아래에도 있음) 0.96
                state_msg = -1  # Left
            elif R_AREA * 0.97 <= r_area < R_AREA * 0.98: # 오른쪽으로 더 가게 하고 싶으면 감소 0.96, 0.972
                state_msg = 0  # Straight
            else:
                state_msg = 1  # Right

        elif self.state == 0: # 평평할 때
            if l_area > 3000 and r_area > 3000:
                if l_area > r_area * 1.6: # 왼쪽으로 더 가게하고 싶으면 감소. 1로 하면 안 됨
                    state_msg = -3  # Left.
                elif r_area > l_area * 1.6: # 오른쪽으로 더 가게하고 싶으면 감소. 1로 하면 안 됨
                    state_msg = 3  # Right
                else:
                    state_msg = 0  # Straight
            else:
                state_msg = 2  # Stop

        self.camera_state_publisher_.publish(Int32(data=state_msg))
        ros_image = self.bridge.cv2_to_imgmsg(hsv_i, encoding="bgr8")
        self.image_publisher.publish(ros_image)

        #cv2.imshow('Track', hsv_i)
        #cv2.imshow('test', color_i)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            pass


        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    camera_state_subscriber = CameraStateSubscriber()
    rclpy.spin(camera_state_subscriber)
    camera_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
