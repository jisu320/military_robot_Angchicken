import numpy as np

def cal_angle(accel_y, accel_z, pixel_x, pixel_y, depth_point):
    angle_const = (121.0/160.0)/458.0

    theta = np.arctan(accel_z/-accel_y)

    real_x = angle_const * depth_point * pixel_x
    cam_y = angle_const * depth_point * pixel_y

    if depth_point == 0:
        print("Depth is zero, Can't calculate.")
        obj_dist = 0
        h = 0
        angle = 0
        dir = 0
        return obj_dist, h, angle, dir

    else :
        alpha = np.arctan(cam_y/depth_point)
        real_dist = depth_point * np.cos(theta + alpha)
        real_z = real_dist * np.tan(theta + alpha)

        angle = np.arctan(real_x/real_dist)

        ye = np.float32(angle)
        x = np.float32(real_dist)
        z = np.float32(real_z)
        imu_angle = np.float32(theta)

        return x, z, ye, imu_angle
