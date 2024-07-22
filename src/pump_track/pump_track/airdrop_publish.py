import datetime
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
from cv_bridge import CvBridge
from cal_angle_imu import cal_angle
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
import time

CONFIDENCE_THRESHOLD = 0.6
GREEN, WHITE, GRAY = (0, 255, 0), (255, 255, 255), (102, 102, 102)

class AirdropDetectionNode(Node):
    def __init__(self):
        super().__init__('Airdrop_detection')
        self.image_publisher = self.create_publisher(
            Image,
            '/airdrop/image_with_boxes',
            10
        )
        self.real_x_publisher = self.create_publisher(
            Float32,
            '/airdrop/real_x',  # Update with your desired real_x topic
            10
        )
        self.yawing_angle_publisher = self.create_publisher(
            Float32,
            '/airdrop/yawing_angle',  # Update with your desired yawing_angle topic
            10
        )
        self.aridrop_publisher = self.create_publisher(
            Bool,
            '/airdrop/airdrop_detected',
            10
        )

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(0.1, self.process_image)

        self.bridge = CvBridge()

        self.coco128 = open('/home/choi/my_bot_ws/src/pump_track/pump_track/Air_drop_box/air-drop-box.txt', 'r')
        self.data = self.coco128.read()
        self.class_list = self.data.split('\n')
        self.model = YOLO('/home/choi/my_bot_ws/src/pump_track/pump_track/best_add_box.pt')

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.prof = self.pipeline.start(config)

        self.imu_pipeline = rs.pipeline()
        imu_config = rs.config()
        imu_config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 63)
        imu_config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
        self.imu_profile = self.imu_pipeline.start(imu_config)


        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        self.profile = self.pipeline.get_active_profile()
        self.dep_sensor = self.profile.get_device().first_depth_sensor()
        self.dep_scale = self.dep_sensor.get_depth_scale()

    def accel_data(self, accel):
        return np.asarray([accel.x, accel.y, accel.z])

    def process_image(self):
        start = datetime.datetime.now()
        fs = self.pipeline.wait_for_frames()
        color_f = fs.get_color_frame()
        depth_f = fs.get_depth_frame()
        align_f = self.align.process(fs)
        imu_frames = self.imu_pipeline.wait_for_frames()
        accel_frames = imu_frames.first_or_default(rs.stream.accel, rs.format.motion_xyz32f)

        color_i = np.asanyarray(color_f.get_data())
        detection = self.model(color_i)[0]

        airdrop_detected = False # Initialize airdrop_deteced
        real_x = 0.0  # Initialize real_x
        yawing_angle = 0.0 # Initialize yawing_angle

        for data in detection.boxes.data.tolist():
            confidence, label = float(data[4]), int(data[5])

            if confidence >= CONFIDENCE_THRESHOLD and self.class_list[label] == 'box':
                if confidence >= 0.7:
                    xmin, ymin, xmax, ymax = map(int, data[:4])
                    xcen, ycen = (xmin + xmax) // 2, (ymin + ymax) // 2
                    align_dep_f = align_f.get_depth_frame()
                    depth_val = align_dep_f.get_distance(xcen, ycen)
                    print(f'Depth value : {depth_val}')
                    dist = depth_val * self.dep_scale
                    detect_point = (xcen - 345, 240 - ycen)
                    print(f'xcen : {xcen}')
                    print(f'ycen : {ycen}')
                    cv2.rectangle(color_i, (xmin, ymin), (xmax, ymax), GREEN, 2)
                    cv2.circle(color_i, (xcen, ycen), 4, GRAY)
                    circle_dist = align_dep_f.get_distance(xcen, ycen)
                    circle_dist = circle_dist
                    print(f'circle_dist : {circle_dist}')
                    accel_array = self.accel_data(accel_frames.as_motion_frame().get_motion_data())
                    accel_array = np.round_(accel_array, 2)
                    accel_array = np.float32(accel_array)
                    print(f'accel_array 1 : {accel_array}')
                    return_value = cal_angle(accel_array[1], accel_array[2], detect_point[0], detect_point[1], circle_dist)
                    real_x, real_z, yawing_angle, imu_angle = float(return_value[0]*100), float(return_value[1]), float(return_value[2]*57.2958), float(return_value[3])

                    print("real_dist: %f [cm]" % real_x)
                    print("real_z: %f [meter]" % real_z)
                    print("yawing_angle: %f [degree]" % yawing_angle)
                    print("imu_angle: %f [degree]" % imu_angle)
                    cv2.putText(color_i, self.class_list[label]+' '+str(round(confidence, 2)) + '%', (xmin, ymin), cv2.FONT_ITALIC, 1, WHITE, 2)
                    cv2.putText(color_i, f'{real_x:.2f}cm',(xmin, ymin-20), cv2.FONT_HERSHEY_SIMPLEX,  0.5, GREEN, 2)
                    airdrop_detected = True

        if airdrop_detected:
            print("Airdrop box detected!")

        end = datetime.datetime.now()
        total = (end - start).total_seconds()
        print(f'Time to process 1 frame: {total * 1000:.0f} ms')
        fps = f'FPS: {1 / total:.2f}'
        cv2.putText(color_i, fps, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.imshow('frame', color_i)

        # Convert the OpenCV image to a ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(color_i, encoding="bgr8")

        # Publish the ROS Image message
        self.image_publisher.publish(ros_image)

        time.sleep(0.5)

        # Publish real_x and yawing_angle as Float32 messages if they were computed
        if real_x is not None:
            real_x_msg = Float32()
            real_x_msg.data = real_x
            self.real_x_publisher.publish(real_x_msg)

        if yawing_angle is not None:
            yawing_angle_msg = Float32()
            yawing_angle_msg.data = yawing_angle
            self.yawing_angle_publisher.publish(yawing_angle_msg)

        airdrop_detected_msg = Bool()
        airdrop_detected_msg.data = airdrop_detected
        self.aridrop_publisher.publish(airdrop_detected_msg)




def main(args=None):
    rclpy.init(args=args)
    node = AirdropDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

[0]
