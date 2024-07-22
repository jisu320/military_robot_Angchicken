import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Int32, Float32, Float32MultiArray
from cv_bridge import CvBridge
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import pyrealsense2 as rs
from .cal_angle_imu import cal_angle
import datetime
import time

CONFIDENCE_THRESHOLD = 0.6
GREEN, WHITE, GRAY = (0, 255, 0), (255, 255, 255), (102, 102, 102)


class RealsenseNode(Node):
    def __init__(self):
        super().__init__('realsense_node')

        ########################################
        #추가된 flag 파트
        self.state = 'airdrop_detection'  # Start with airdrop detection
        self.airdrop_detected = False
        ####################################
        ############################################
        #airdrop에서 가져옴
        self.image_publisher = self.create_publisher(
            Image,
            '/airdrop/image_with_boxes',  # Update with your desired output image topic
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
            '/airdrop/airdrop_detected',  # Update with your desired yawing_angle topic
            10
        )

        ###############################################
        #diffbot_postion_control추가
        self.real_x_yawing_angle_publisher = self.create_publisher(
            Float32MultiArray,  # Use Float32MultiArray for publishing floats
            '/airdrop/real_x_yawing_angle',
            10
        )
        ########################################################################3

        # Create a subscriber to receive sonar airdrop detection status
        self.sonar_airdrop_detected_subscription = self.create_subscription(
            Bool,
            '/sonar/airdrop_detected',  # Replace with the Arduino's topic
            self.sonar_airdrop_detected_callback,
            10
        )

        # Create a publisher to send a number to the Arduino
        self.arduino_number_publisher = self.create_publisher(
            Int32,  # Use Int32 for integers
            '/arduino/number_pub',  # Replace with your desired topic
            10
        )

        # Create a subscriber to receive a number from the Arduino
        self.arduino_number_subscription = self.create_subscription(
            Int32,  # Use Int32 for integers
            '/arduino/number_sub',  # Replace with the Arduino's topic
            self.arduino_number_callback,
            10
        )

        #######################추가 부분####################

        self.publisher = self.create_publisher(Int32, 'camera_state_output', 10)

        self.motor_position_subscriber = self.create_subscription(
            Bool,
            '/motor/posision',  # Replace with the Arduino's topic
            self.motor_position_callback,
            10
        )

        ##########################파이프라인 선언 부##################
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(0.1, self.AirdropDetect)
        self.bridge = CvBridge()

        ################YOLO#####################
        #airdrop에서 가져옴
        self.coco128 = open('/home/choi/my_bot_ws/src/pump_track/pump_track/Air_drop_box/air-drop-box.txt', 'r')
        self.data = self.coco128.read()
        self.class_list = self.data.split('\n')
        self.model = YOLO('/home/choi/my_bot_ws/src/pump_track/pump_track/best_add_box.pt')
        ################YOLO######################

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        #########################
        #aitdrop에서 가져옴
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        self.prof = self.pipeline.start(self.config)

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

        ###############################################
        self.cen_x = 320
        self.roi_start_row = int(480 * 0.35) #72 -> 168
        self.roi_end_row = int(480 * 0.5) # 120 -> 240
        self.roi_start_col = int(640 * 0.05) #21.2 -> 32
        self.roi_end_col = int(640 * 0.95) #402.8 -> 608
        self.clip_dis_maxm = 0.84  # 최대 길이 범위 조정 (m)
        self.clip_dis_minm = 0.8  # 최소 ..
        self.clip_dis_max = self.clip_dis_maxm / self.dep_scale
        self.clip_dis_min = self.clip_dis_minm / self.dep_scale
        ##########################
        ##########################
        # Timer for AirdropDetect
        self.timer_ad = self.create_timer(0.1, self.AirdropDetect)
        # Create a timer for TrackDepth, but don't start it yet
        self.timer_td = self.create_timer(0.1, self.TrackDepth)
        self.timer_td.cancel()  # Cancel the TrackDepth timer initiall
        ############################
        self.sonar_detected = False # need to fix Node
        self.received_arduino_number = None  # need to fix None
        self.number_msg = None # need to fix None
        self.motor_position = False
    #########################################
    # airdrop에서 가져옴
    def accel_data(self, accel):
        return np.asarray([accel.x, accel.y, accel.z])

    ############################################
    #arcuino
    def sonar_airdrop_detected_callback(self, msg):
        self.sonar_detected = msg.data
        self.get_logger().info(f'Sonar detected airdrop: {msg.data}')

    def publish_number_to_arduino(self, number):
        self.number_msg = Int32()  # Create an Int32 message
        self.number_msg.data = number  # Set the data field of the message with the number
        self.get_logger().info(f'Sending number to Arduino: {number}')
        self.arduino_number_publisher.publish(self.number_msg)

    def arduino_number_callback(self, msg):
        self.received_arduino_number = msg.data
        self.get_logger().info(f'Received number from Arduino: {msg.data}')

    ############################################

    def motor_position_callback(self, msg):
        self.motor_position = msg.data
        self.get_logger().info(f'motor_position : {msg.data}')

    ############################################################3

    def AirdropDetect(self):
        if self.state == 'airdrop_detection':
            start = datetime.datetime.now()
            fs = self.pipeline.wait_for_frames()
            color_f = fs.get_color_frame()
            depth_f = fs.get_depth_frame()
            align_f = self.align.process(fs)
            imu_frames = self.imu_pipeline.wait_for_frames()
            accel_frames = imu_frames.first_or_default(rs.stream.accel, rs.format.motion_xyz32f)
            color_i = np.asanyarray(color_f.get_data())
            detection = self.model(color_i)[0]
            airdrop_detected = False
            real_x = 0.0
            yawing_angle = 0.0
            for data in detection.boxes.data.tolist():
                confidence, label = float(data[4]), int(data[5])
                if confidence >= CONFIDENCE_THRESHOLD and self.class_list[label] == 'box':
                    if confidence >= 0.8:
                        xmin, ymin, xmax, ymax = map(int, data[:4])
                        xcen, ycen = (xmin + xmax) // 2, (ymin + ymax) // 2
                        align_dep_f = align_f.get_depth_frame()
                        depth_val = align_dep_f.get_distance(xcen, ycen)
                        dist = depth_val * self.dep_scale
                        detect_point = (xcen - 320, 240 - ycen)
                        cv2.rectangle(color_i, (xmin, ymin), (xmax, ymax), GREEN, 2)
                        cv2.circle(color_i, (xcen, ycen), 4, GRAY)
                        circle_dist = align_dep_f.get_distance(xcen, ycen)
                        circle_dist = circle_dist
                        accel_array = self.accel_data(accel_frames.as_motion_frame().get_motion_data())
                        accel_array = np.round_(accel_array, 2)
                        accel_array = np.float32(accel_array)
                        return_value = cal_angle(accel_array[1], accel_array[2], detect_point[0], detect_point[1],
                                                 circle_dist)
                        real_x, real_z, yawing_angle, imu_angle = float(return_value[0] * 100), float(return_value[1]), float(
                            return_value[2] * 57.2958), float(return_value[3] * 57.2958)
                        print("real_dist: %f [cm]" % real_x)
                        print("real_z: %f [meter]" % real_z)
                        print("yawing_angle: %f [degree]" % yawing_angle)
                        print("imu_angle: %f [degree]" % imu_angle)
                        cv2.putText(color_i, self.class_list[label] + ' ' + str(round(confidence, 2)) + '%',
                                    (xmin, ymin), cv2.FONT_ITALIC, 1, WHITE, 2)
                        cv2.putText(color_i, f'{real_x:.2f}cm', (xmin, ymin - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, GREEN, 2)
                        airdrop_detected = True
            if airdrop_detected:
                print("Airdrop box detected!")
            end = datetime.datetime.now()
            total = (end - start).total_seconds()
            print(f'Time to process 1 frame: {total * 1000:.0f} ms')
            fps = f'FPS: {1 / total:.2f}'
            cv2.putText(color_i, fps, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            ros_image = self.bridge.cv2_to_imgmsg(color_i, encoding="bgr8")
            self.image_publisher.publish(ros_image)
            if real_x != 0:
                real_x_msg = Float32()
                real_x_msg.data = real_x
                self.real_x_publisher.publish(real_x_msg)
            if yawing_angle != 0:
                yawing_angle_msg = Float32()
                yawing_angle_msg.data = yawing_angle
                self.yawing_angle_publisher.publish(yawing_angle_msg)
            airdrop_detected_msg = Bool()
            airdrop_detected_msg.data = airdrop_detected
            self.aridrop_publisher.publish(airdrop_detected_msg)

            if airdrop_detected and real_x and yawing_angle is not None:
                # After airdrop detection, switch to the depth tracking state
                self.state = 'depth_tracking' #'commuincation_odrive'
                self.timer_td.reset()  # Start the TrackDepth timer



    def TrackDepth(self):
        if self.state == 'commuincation_odrive':
            if self.motor_position == True:
                self.state = 'commuincation_arduino'
            else:
                self.get_logger().info('wait for motor moved end')

        if self.state == 'commuincation_arduino': #commuincation with arduino
            # Publish the number 1 to the Arduino
            self.get_logger().info('order move gripper')
            self.publish_number_to_arduino(5) # need to change number
            # Wait for sonar_airdrop_detected_subscription to return True
            if self.received_arduino_number == 5: #need to change True
                self.state = 'depth_tracking'
                self.get_logger().info('go running depth')
                # Wait for the Arduino to respond with the number 2
            else:
                self.get_logger().info(f'current subscribed number : {self.received_arduino_number}')
                self.get_logger().info('Wait for gripper moving finish')
                time.sleep(1.0)

        if self.state == 'depth_tracking':
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
            grey_color = 0  # 범위 내에 없는 영역. 구분하기 위해 100으로 줬음. 나중에 0으로 바꿀것.
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
            if l_area > r_area * 1.4:
                direction = -1  # left Turn
            elif r_area > l_area * 1.4:
                direction = 1  # right Turn
            else:
                direction = 0  # Straight
            self.get_logger().info(str(direction))
            real_x = 0.5
            yawing_angle = 2.5*direction
            data = [real_x, yawing_angle, 1]  # Add "pos" as the mode
            msg = Float32MultiArray(data=data)
            self.real_x_yawing_angle_publisher.publish(msg)
            cv2.imshow('Depth Track', bg_removed)
            cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)

    # Create a ROS 2 node
    node = rclpy.create_node('combined_node')
    node.get_logger().info('Received order. Starting RealsenseNode...')
    realsense_node = RealsenseNode()
    rclpy.spin(realsense_node)
    realsense_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
