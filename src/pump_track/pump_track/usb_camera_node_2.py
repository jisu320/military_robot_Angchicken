import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import atexit

class UsbCameraNode(Node):

    def __init__(self):
        super().__init__('usb_camera_node_2')
        self.publisher2 = self.create_publisher(Image, 'usb_camera/image2', 10)
        self.bridge = CvBridge()

        gst_pipeline = "v4l2src device=/dev/video6 ! videoconvert ! appsink" # 포트 번호
        self.cap2 = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        atexit.register(self.cleanup)

        self.timer = self.create_timer(1.0 / 15, self.publish_frame) # 15 프레임

    def publish_frame(self):
        if self.cap2.isOpened():
            ret2, frame2 = self.cap2.read()

            if ret2:
                frame2 = cv2.resize(frame2, (1280, 720)) # 해상도

                img_msg2 = self.bridge.cv2_to_imgmsg(frame2, encoding="bgr8")
                self.publisher2.publish(img_msg2)

            # cv2.imshow('USB Camera', frame1)
            # cv2.waitKey(1)

    def cleanup(self):
        if self.cap2.isOpened():
            self.cap2.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = UsbCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
