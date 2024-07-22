import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.bridge = CvBridge()
        self.model = YOLO('/home/choi/my_bot_ws/src/pump_track/pump_track/army/army_b_8.pt')  # 모델 파일 위치 .pt
        self.class_list = open('/home/choi/my_bot_ws/src/pump_track/pump_track/army/army.txt', 'r').read().split('\n')  # 모델 파일 텍스트 위치
        self.publisher = self.create_publisher(Image,'/army_detection',10)
        self.subscription = self.create_subscription(
            Image,
            # 'usb_camera/image1', # 카메라 1개
            'usb_camera/combined_image', # 카메라 2개
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        detection = self.model(frame)[0]

        for data in detection.boxes.data.tolist():
            xmin, ymin, xmax, ymax = map(int, data[:4])
            label = int(data[5])
            confidence = float(data[4])

            if confidence > 0.3: # and self.class_list[label] in ["friendly", "enemy"]: # 신뢰도 조절 / 군복 잘 인식 안 되면 감소시킬 것
                if self.class_list[label] == "friendly": # 아군 인식
                    cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                else: # 적군 인식
                    cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 0, 255), 2)
                text_y = ymin + 30
                cv2.putText(frame, self.class_list[label]+' '+str(round(confidence, 2)) + '%', (xmin, text_y), cv2.FONT_ITALIC, 1, (255, 255, 255), 2)

        detected_image_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.publisher.publish(detected_image_msg)

        # cv2.imshow("Object Detection1", frame)
        # cv2.waitKey(1)  # Adjust the waitKey value for the desired frame display time

def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetectionNode()
    rclpy.spin(object_detection_node)
    object_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()