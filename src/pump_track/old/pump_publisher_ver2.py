import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as ROSImage
import cv2
import numpy as np
import pyrealsense2 as rs

#resized 1602 * 120 version

min_recognized_percentage = 22 # 인식율 한계 선정

def getHistogram(img, display=False, minVal=0.63, region=1):
    histValues = np.sum(img, axis=0)
    maxValue = np.max(histValues)
    minValue = minVal * maxValue
    indexArray = np.where(histValues >= minValue)
    basePoint = int(np.average(indexArray))

    imgHist = None

    if display:
        imgHist = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
        for x, intensity in enumerate(histValues):
            if intensity > minValue:
                color = (255, 0, 255)  # Pink color for columns above threshold
            else:
                color = (0, 0, 255)  # Red color for columns below threshold

            end_point_y = img.shape[0] - (int(intensity) // 255 // region)
            cv2.line(imgHist, (x, img.shape[0]), (x, end_point_y), color, 1)

        cv2.circle(imgHist, (basePoint, img.shape[0]), 20, (0, 255, 255), cv2.FILLED)
        cv2.line(imgHist, (basePoint, img.shape[0]), (basePoint, 0), (255, 0, 0), 10)

    return basePoint, imgHist


def getInstructions(line_position, width, threshold=0.03):
    center = width // 2
    left_limit = int(center - threshold * center)
    right_limit = int(center + threshold * center)

    if line_position < left_limit:
        return "3" # Turn Left
    elif line_position > right_limit:
        return "2" #Turn Right
    else:
        return "1" # Go straight

def calculateRecognizedPercentage(masked_edges):
    # Calculate the recognized percentage based on the white filter sections in the masked_edges image
    recognized_percentage = (np.count_nonzero(masked_edges > 0) / masked_edges.size) * 100
    return recognized_percentage


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('line_follower')

    pipeline = rs.pipeline()
    config = rs.config()
    target_width = 640
    target_height = 480
    config.enable_stream(rs.stream.color, target_width, target_height, rs.format.bgr8, 30)
    pipeline.start(config)

    image_publisher = node.create_publisher(Image, '/camera/image_raw', 10)
    instruction_publisher = node.create_publisher(String, '/line_instructions', 10)
    cv_bridge = CvBridge()
    hist_image_publisher = node.create_publisher(ROSImage, '/hist_image', 10)


    try:
        while rclpy.ok():
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            image = np.array(color_frame.get_data())
            new_target_width = 160
            new_target_height = 120
            resized_image = cv2.resize(image, (new_target_width, new_target_height))

            roi_top_left = (1, 10)
            roi_bottom_right = (119, 60)
            min_slope = 0.05
            max_slope = np.inf
            x_less_than_320 = -np.inf
            x_greater_than_320 = np.inf
            roi = resized_image[roi_top_left[1]:roi_bottom_right[1], roi_top_left[0]:roi_bottom_right[0]]
            blurred = cv2.GaussianBlur(roi, (5, 5), 0)

            hsv_image = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            lower_blue = np.array([90, 50, 50])
            upper_blue = np.array([130, 255, 255])
            blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

            blue_mask_resized = cv2.resize(blue_mask, (new_target_width, new_target_height))
            blue_mask_resized = blue_mask_resized.astype(np.uint8)  # Convert to 8-bit unsigned integer
            resized_image = resized_image.astype(np.uint8)  # Ensure resized_image is also 8-bit unsigned integer

            blue_extracted = cv2.bitwise_and(resized_image, resized_image, mask=blue_mask_resized)

        # Rest of your code...

            mask = np.zeros_like(blue_mask_resized)

            roi_vertices = np.array([[(roi_top_left[0], 10), (roi_top_left[0],60), roi_bottom_right, (roi_bottom_right[0], 10)]], dtype=np.int32)
            cv2.fillPoly(mask, roi_vertices, 255)

            masked_edges = cv2.bitwise_and(blue_mask_resized, mask)

            base_point, hist_image = getHistogram(masked_edges, display=True)
            line_position = base_point - roi_top_left[0]

            # Calculate the recognized percentage
            recognized_percentage = calculateRecognizedPercentage(masked_edges)
            #print(recognized_percentage)
            # Determine whether to display the hist_image
            display = True  # Set this to True or False based on your needs

            # Issue a stop command if the recognized percentage is below the threshold
            if recognized_percentage < min_recognized_percentage:
                instructions = "4"
            else:
                instructions = getInstructions(line_position, roi_bottom_right[0] - roi_top_left[0])


            # Convert the OpenCV image to a ROS Image message
            image_msg = cv_bridge.cv2_to_imgmsg(resized_image, encoding='bgr8')
            image_msg.header.stamp = node.get_clock().now().to_msg()
            image_publisher.publish(image_msg)
            # Convert the hist_image to a ROS Image message and publish it
            if display:
                hist_image_msg = cv_bridge.cv2_to_imgmsg(hist_image, encoding='rgb8')
                hist_image_msg.header.stamp = node.get_clock().now().to_msg()
                hist_image_publisher.publish(hist_image_msg)
            # Publish instructions as a String message
            instruction_msg = String()
            instruction_msg.data = instructions
            instruction_publisher.publish(instruction_msg)

    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
