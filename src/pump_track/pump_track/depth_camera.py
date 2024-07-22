import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as ROSImage
import cv2
import numpy as np
import pyrealsense2 as rs

#resized 480 * 320 version


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('line_follower')

    pipeline = rs.pipeline()
    config = rs.config()
    target_width = 424
    target_height = 240
    config.enable_stream(rs.stream.color, target_width, target_height, rs.format.bgr8, 30)
    pipeline.start(config)

    image_publisher = node.create_publisher(Image, '/depth_camera/image', 10)
    cv_bridge = CvBridge()


    try:
        while rclpy.ok():
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            image = np.array(color_frame.get_data())
            target_height = 240
            target_width = 424
            resized_image = cv2.resize(image, (target_width, target_height))

            # Convert the OpenCV image to a ROS Image message
            image_msg = cv_bridge.cv2_to_imgmsg(resized_image, encoding='bgr8')
            image_msg.header.stamp = node.get_clock().now().to_msg()
            image_publisher.publish(image_msg)


    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
