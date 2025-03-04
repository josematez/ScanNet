import numpy as np
from cv_bridge import CvBridge
from rosbags.typesys.stores.ros1_noetic import (
    sensor_msgs__msg__Image as Image,
    sensor_msgs__msg__CameraInfo as CameraInfo,
    sensor_msgs__msg__RegionOfInterest as RegionOfInterest,
    geometry_msgs__msg__PoseWithCovarianceStamped as PoseWithCovarianceStamped,
    geometry_msgs__msg__PoseWithCovariance as PoseWithCovariance,
    geometry_msgs__msg__Pose as Pose,
    geometry_msgs__msg__Quaternion as QuaternionROS1,
    geometry_msgs__msg__Point as Point
)

class GenerateRosbag(object):

    def __init__(self):
        self._bridge = CvBridge()

    def create_image_msg(self, img, header):

        img_msg = self._bridge.cv2_to_imgmsg(img, "passthrough")
        if img_msg.encoding == "8UC3":
            img_msg.encoding = "bgr8"
        
        ros1_image = Image(
            header = header,
            height = img_msg.height,
            width = img_msg.width,
            encoding = img_msg.encoding,
            is_bigendian = img_msg.is_bigendian,
            step = img_msg.step,
            data = np.array(img_msg.data, dtype=np.uint8)
        )

        return ros1_image
    
    def create_camera_info_msg(self, img, intrinsics, header):

        cam_info_msg = CameraInfo(
            header=header,
            height=img.shape[0],
            width=img.shape[1],
            distortion_model="plumb_bob",
            D=np.array([0., 0., 0., 0., 0.], dtype=np.float64),
            K=np.array([intrinsics[0], 0., intrinsics[2], 0., intrinsics[1], intrinsics[3], 0., 0., 1.], dtype=np.float64),
            R=np.array([1., 0., 0., 0., 1., 0., 0., 0., 1.], dtype=np.float64),
            P=np.array([intrinsics[0], 0., intrinsics[2], 0., 0., intrinsics[1], intrinsics[3], 0., 0., 0., 1., 0.], dtype=np.float64),
            binning_x=0, # Forced to add these fields
            binning_y=0,
            roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False)
        )

        return cam_info_msg

    def create_pose_msg(self, pose, header):

        header.frame_id = "map"

        pose_msg = PoseWithCovarianceStamped(
            header=header,
            pose=PoseWithCovariance(
                pose = Pose(
                    position = Point(x = pose[4], y = pose[5], z = pose[6]),
                    orientation = QuaternionROS1(x = pose[0], y = pose[1], z = pose[2], w = pose[3])
                ),
                covariance = np.array(36 * [0.], dtype=np.float64)
            )
        )


        return pose_msg
