import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class UsbCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node_2')
        self.publisher_combined = self.create_publisher(Image, 'usb_camera/combined_image', 10)
        self.bridge = CvBridge()
        # 인덱스 맞춰주기
        self.cap1 = cv2.VideoCapture("/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_E8FC4D2F-video-index0")
        self.cap2 = cv2.VideoCapture("/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_B0125D2F-video-index0")

        desired_fps = 15  # 프레임 수
        self.cap1.set(cv2.CAP_PROP_FPS, desired_fps)
        self.cap2.set(cv2.CAP_PROP_FPS, desired_fps)

        self.timer = self.create_timer(1.0 / desired_fps, self.publish_combined_frame)

    def publish_combined_frame(self):
        ret1, frame1 = self.cap1.read()
        ret2, frame2 = self.cap2.read()

        frame1 = cv2.resize(frame1, (424, 240))
        frame2 = cv2.resize(frame2, (424, 240))

        if ret1 and ret2: # 해상도
            # Combine
            combined_frame = np.hstack((frame1, frame2))

            img_msg_combined = self.bridge.cv2_to_imgmsg(combined_frame, encoding="bgr8")
            self.publisher_combined.publish(img_msg_combined)

            # cv2.imshow('Combined USB Camera Feed', combined_frame)
            # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = UsbCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()