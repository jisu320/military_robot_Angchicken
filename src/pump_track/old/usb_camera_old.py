import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class UsbCameraNode(Node):

    def __init__(self):
        super().__init__('usb_camera_node')
        self.publisher1 = self.create_publisher(Image, 'usb_camera/image1', 10)
        self.bridge = CvBridge()

        self.cap1 = cv2.VideoCapture(10)  # Change camera index to 0 for default camera

        self.timer = self.create_timer(1.0 / 15, self.publish_frame)
        
    def publish_frame(self):
        if self.cap1.isOpened():
            ret1, frame1 = self.cap1.read()

            if ret1:
                frame1 = cv2.resize(frame1, (1280, 720))

                img_msg1 = self.bridge.cv2_to_imgmsg(frame1, encoding="bgr8")
                self.publisher1.publish(img_msg1)

            cv2.imshow('USB Camera Feed', frame1)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = UsbCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()