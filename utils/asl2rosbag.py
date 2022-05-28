import rosbag
import rospy
import PIL.Image
import numpy as np
import torch
import torchvision
import torchvision.transforms.functional as torchvision_transforms
import os
import pandas as pd
from std_msgs.msg import Int32, String
from sensor_msgs.msg import Imu, Image

if os.path.exists("stairs.bag"):
        os.remove("stairs.bag")

bag = rosbag.Bag('stairs.bag', 'w')
df_imu0 = pd.read_csv('stairs/imu0/data.csv')
df_cam0 = pd.read_csv('stairs/cam0/data.csv')
df_cam1 = pd.read_csv('stairs/cam1/data.csv')
df_depth0 = pd.read_csv('stairs/depth0/data.csv')

try:

    # # depth0 topic
    # counter = 0
    # for row in range(df_depth0.shape[0]):
    #     timestamp = rospy.Time.from_sec(1e-9*df_depth0['#timestamp [ns]'][row])
    #     counter = counter + 1

    #     depth0_msg = Image()
    #     depth0_msg.header.stamp = timestamp
    #     depth0_msg.header.seq = counter

    #     # find image named by filename
    #     file_name = 'stairs/depth0/data/' + df_depth0['filename'][row]
    #     # print(file_name)
    #     with PIL.Image.open(file_name) as image:
    #         #print(image.mode) #depth is mono 32 int
    #         image.convert("F")
    #         width, height = image.size
    #         depth0_msg.height = height
    #         depth0_msg.width = width
    #         depth0_msg.encoding = "32FC1"
    #         #depth0_msg.encoding = "32SC1"
    #         depth0_msg.step = width * 4
    #         depth0_msg.is_bigendian = False
    #         im_array = np.array(image)
    #         depth0_msg.data = np.divide(im_array,1000.0).tobytes()
    #         #depth0_msg.data = np.array(image).tobytes()
    #         bag.write("/depth0", depth0_msg, timestamp)

    # simon depth topic
    counter = 0
    for row in range(df_depth0.shape[0]):
        timestamp = rospy.Time.from_sec(1e-9*df_depth0['#timestamp [ns]'][row])
        counter = counter + 1

        depth0_msg = Image()
        depth0_msg.header.stamp = timestamp
        depth0_msg.header.seq = counter

        # find image named by filename
        file_name = 'stairs/depth0/data/' + df_depth0['filename'][row]
        # print(file_name)
        with PIL.Image.open(file_name) as image:

            image = torchvision_transforms.to_tensor(image) / 1000  # TUM-format with 1000, not 5000 / m
            im = image.cpu().detach().numpy().astype(np.float32)
            depth0_msg.step = 4 * im.shape[-1]
            depth0_msg.height = im.shape[1]
            depth0_msg.width = im.shape[2]
            depth0_msg.encoding = "32FC1"
            depth0_msg.is_bigendian = False
            depth0_msg.data = np.transpose(im, (1, 2, 0)).tobytes()  # ROS has (H, W, C) format
            bag.write("/depth0", depth0_msg, timestamp)

    # imu topic
    counter = 0
    for row in range(df_imu0.shape[0]):
        timestamp = rospy.Time.from_sec(1e-9*df_imu0['#timestamp [ns]'][row])
        counter = counter + 1

        imu_msg = Imu()
        imu_msg.header.stamp = timestamp
        imu_msg.header.seq = counter
        imu_msg.angular_velocity.x = df_imu0['w_RS_S_x [rad s^-1]'][row]
        imu_msg.angular_velocity.y = df_imu0['w_RS_S_y [rad s^-1]'][row]
        imu_msg.angular_velocity.z = df_imu0['w_RS_S_z [rad s^-1]'][row]
        imu_msg.linear_acceleration.x = df_imu0['a_RS_S_x [m s^-2]'][row]
        imu_msg.linear_acceleration.y = df_imu0['a_RS_S_y [m s^-2]'][row]
        imu_msg.linear_acceleration.z = df_imu0['a_RS_S_z [m s^-2]'][row]

        bag.write("/imu0", imu_msg, timestamp)

    # cam0 topic
    counter = 0
    for row in range(df_cam0.shape[0]):
        timestamp = rospy.Time.from_sec(1e-9*df_cam0['#timestamp [ns]'][row])
        counter = counter + 1

        cam0_msg = Image()
        cam0_msg.header.stamp = timestamp
        cam0_msg.header.seq = counter

        # find image named by filename
        file_name = 'stairs/cam0/data/' + df_cam0['filename'][row]
        # print(file_name)
        with PIL.Image.open(file_name) as image:
            # print(image.mode) mode is L = mono 8
            width, height = image.size
            cam0_msg.height = height
            cam0_msg.width = width
            cam0_msg.encoding = "mono8"
            cam0_msg.step = width
            cam0_msg.is_bigendian = False
            cam0_msg.data = np.array(image).tobytes()

        bag.write("/cam0", cam0_msg, timestamp)

    # cam1 topic
    counter = 0
    for row in range(df_cam1.shape[0]):
        timestamp = rospy.Time.from_sec(1e-9*df_cam1['#timestamp [ns]'][row])
        counter = counter + 1

        cam1_msg = Image()
        cam1_msg.header.stamp = timestamp
        cam1_msg.header.seq = counter

        # find image named by filename
        file_name = 'stairs/cam1/data/' + df_cam1['filename'][row]
        # print(file_name)
        with PIL.Image.open(file_name) as image:
            # print(image.mode) mode is L = mono 8
            width, height = image.size
            cam1_msg.height = height
            cam1_msg.width = width
            cam1_msg.encoding = "mono8"
            cam1_msg.step = width
            cam1_msg.is_bigendian = False
            cam1_msg.data = np.array(image).tobytes()

        bag.write("/cam1", cam1_msg, timestamp)


finally:
    print("saving bag...")
    bag.close()
