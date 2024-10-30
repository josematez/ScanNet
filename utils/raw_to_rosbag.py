import os
import argparse
import numpy as np
import cv2
import matplotlib.pyplot as plt
from copy import deepcopy
import rosbag
import tf
import csv
from tqdm import tqdm
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped

from generator import GenerateRosbag
from transformations import *

class ROSbag_generator():

    def __init__(self, data_path):
        
        self.dataset_path = os.path.abspath(data_path)
    
    def converter(self):

        n_data = len(os.listdir(self.dataset_path + "/color/"))

        # Depth, RGB, segmentation_img, colors.txt (wall class code), extrinsic.txt (poses), intrinsic.txt (poses), camera_number
        with open(self.dataset_path + "/intrinsic/intrinsic_depth.txt", 'r') as file:
            data = file.read()
        matrix_list = [list(map(float, row.split())) for row in data.strip().split('\n')]
        matrix_array = np.array(matrix_list)
        intrinsics = [matrix_array[0, 0], matrix_array[1, 1], matrix_array[0, 2], matrix_array[1, 2]]

        rb = GenerateRosbag()
        rosbag_path = os.path.dirname(os.path.abspath(__file__)) + "/../to_ros/ROS1_bags/"
        bag = rosbag.Bag(rosbag_path + self.dataset_path.split("/")[-1] + ".bag", "w")

        pose_1 = []
        pose_2 = []
        pose_3 = []

        origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
        Rx = rotation_matrix(np.radians(-90), xaxis)
        Ry = rotation_matrix(np.radians(90), yaxis)
        Rz = rotation_matrix(np.radians(-90), zaxis)

        for i in tqdm(range(n_data)):
            
            header = Header()
            header.seq = i
            header.stamp.secs = i
            header.stamp.nsecs = 0
            header.frame_id = "camera"

            img_rgb = cv2.imread(self.dataset_path + "/color/" + str(i) + ".jpg", -1)
            #img_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2RGB)
            img_depth = cv2.imread(self.dataset_path + "/depth/" + str(i) + ".png", cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH) / 1000.
            img_rgb = cv2.resize(img_rgb, (img_depth.shape[1], img_depth.shape[0]), interpolation=cv2.INTER_AREA)
            
            with open(self.dataset_path + "/pose/" + str(i) + ".txt", 'r') as file:
                data = file.read()
                matrix_list = [list(map(float, row.split())) for row in data.strip().split('\n')]
            #pose = convert_4x4_matrix_robotics_to_cv(np.array(matrix_list))
            pose = np.array(matrix_list) @ np.linalg.inv(Ry @ Rz)

            pose_quat = quaternion_from_matrix(pose)
            pose_t = pose[:3,3].reshape(-1)
            pose = [*pose_quat] + [*pose_t]

            pose_1.append(pose_t[0])
            pose_2.append(pose_t[1])
            pose_3.append(pose_t[2])

            rgb_msg = rb.create_image_msg(img_rgb, deepcopy(header))
            depth_msg = rb.create_image_msg(img_depth, deepcopy(header))
            cam_info_msg = rb.create_camera_info_msg(img_rgb, intrinsics, deepcopy(header))
            pose_msg = rb.create_pose_msg(pose, deepcopy(header))
            #tf_msg = rb.create_tf_msg(pose, deepcopy(header))


            bag.write("camera/rgb", rgb_msg)
            bag.write("camera/depth", depth_msg)
            bag.write("camera/camera_info", cam_info_msg)
            #bag.write("tf", tf_msg)
            bag.write("amcl_pose", pose_msg)

        bag.close()

        print("ROS1 bag saved in: {}".format(rosbag_path + self.dataset_path.split("/")[-1] + ".bag"))

    
    @staticmethod
    def convert_4x4_matrix_robotics_to_cv(matrix):
        """
        Converts a 4x4 transformation matrix from robotics convention
        (X forward, Y left, Z up) to computer vision convention
        (X right, Y down, Z forward).
        
        Args:
        matrix (np.ndarray): A 4x4 transformation matrix.
        
        Returns:
        np.ndarray: The converted 4x4 transformation matrix.
        """
        # Rotation matrix to convert from robotics to computer vision convention
        R_conversion = np.array([
            [0,  1,  0],
            [1,  0,  0],
            [0,  0, -1]
        ])

        # Extract the 3x3 rotation part and the 3x1 translation part
        R_robotics = matrix[:3, :3]
        t_robotics = matrix[:3, 3]

        # Convert the rotation matrix
        R_cv = R_conversion @ R_robotics

        # Convert the translation vector
        t_cv = R_conversion @ t_robotics

        # Construct the new 4x4 transformation matrix
        matrix_cv = np.eye(4)
        matrix_cv[:3, :3] = R_cv
        matrix_cv[:3, 3] = t_cv

        return matrix_cv

    @staticmethod
    def convert_4x4_matrix_cv_to_robotics(matrix):
        """
        Converts a 4x4 transformation matrix from computer vision convention
        (X right, Y down, Z forward) to robotics convention
        (X forward, Y left, Z up).
        
        Args:
        matrix (np.ndarray): A 4x4 transformation matrix.
        
        Returns:
        np.ndarray: The converted 4x4 transformation matrix.
        """
        # Inverse rotation matrix to convert from computer vision to robotics convention
        R_conversion = np.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0, -1]
        ])

        # Extract the 3x3 rotation part and the 3x1 translation part
        R_cv = matrix[:3, :3]
        t_cv = matrix[:3, 3]

        # Convert the rotation matrix
        R_robotics = R_conversion @ R_cv @ R_conversion.T

        # Convert the translation vector
        t_robotics = R_conversion @ t_cv

        # Construct the new 4x4 transformation matrix
        matrix_robotics = np.eye(4)
        matrix_robotics[:3, :3] = R_robotics
        matrix_robotics[:3, 3] = t_robotics

        return matrix_robotics

    @staticmethod
    def rotateZ(matrix, degrees):
        theta = np.radians(degrees)
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        rotation_matrix_z = np.array([
            [cos_theta, -sin_theta, 0, 0],
            [sin_theta, cos_theta, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        return np.dot(rotation_matrix_z, matrix)
    

# params
parser = argparse.ArgumentParser()
parser.add_argument('--datapath', required=True, help='path to the raw data of the scene')

opt = parser.parse_args()

if __name__ == '__main__':

    rb_generator = ROSbag_generator(opt.datapath)
    rb_generator.converter()