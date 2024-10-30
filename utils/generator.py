import numpy as np
from copy import deepcopy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion, TransformStamped, Vector3
from tf2_msgs.msg import TFMessage

class GenerateRosbag(object):

    def __init__(self):
        self._bridge = CvBridge()

    def create_image_msg(self, img, header):

        img_msg = self._bridge.cv2_to_imgmsg(img, "passthrough")
        if img_msg.encoding == "8UC3":
            img_msg.encoding = "bgr8"
        img_msg.header = header

        return img_msg
    
    def create_camera_info_msg(self, img, intrinsics, header):

        cam_info_msg = CameraInfo()
        cam_info_msg.header = header
        cam_info_msg.height = img.shape[0]
        cam_info_msg.width = img.shape[1]
        cam_info_msg.distortion_model = "plumb_bob"
        cam_info_msg.D = [0., 0., 0., 0., 0.]
        cam_info_msg.K = [intrinsics[0], 0., intrinsics[2], 0., intrinsics[1], intrinsics[3], 0., 0., 1.]
        cam_info_msg.R = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        cam_info_msg.P = [intrinsics[0], 0., intrinsics[2], 0., 0., intrinsics[1], intrinsics[3], 0., 0., 0., 1., 0.]

        return cam_info_msg

    def create_pose_msg(self, pose, header):

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = header
        pose_msg.header.frame_id = "map"

        pose_msg.pose.covariance = 36 * [0.]
        pose_msg.pose.pose.position = Point(pose[4], pose[5], pose[6])
        pose_msg.pose.pose.orientation = Quaternion(pose[0], pose[1], pose[2], pose[3])


        return pose_msg

    def create_tf_msg(self, pose, header):

        tf_msg = TFMessage()
        tf_msg.transforms = []

        transform = TransformStamped()
        transform.header = deepcopy(header)
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "camera"
        
        transform.transform.translation = Vector3(0., 0., 0.)
        transform.transform.rotation = Quaternion(0., 0., 0., 1.)

        tf_msg.transforms.append(transform)

        transform = TransformStamped()
        transform.header = deepcopy(header)
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"

        transform.transform.translation = Vector3(pose[4], pose[5], pose[6])
        transform.transform.rotation = Quaternion(pose[0], pose[1], pose[2], pose[3])

        tf_msg.transforms.append(transform)

        return tf_msg
