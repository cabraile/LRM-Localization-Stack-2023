#!/usr/bin/python3
import numpy as np
from scipy.spatial.transform import Rotation

import rospy
import tf2_ros

from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose

def ros_pose_to_transformation_matrix(pose : Pose) -> np.ndarray:
    translation = np.array([pose.position.x, pose.position.y, pose.position.z])
    rotation = Rotation.from_quat( [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w] ).as_matrix()
    T = np.eye(4)
    T[:3,:3] = rotation
    T[:3,3] = translation
    return T

def invert_transformation_matrix(transform : np.ndarray) -> np.ndarray:
    t = transform[:3,3]
    R = transform[:3,:3]
    transform_inverse = np.eye(4)
    transform_inverse[:3,:3] = R.T
    transform_inverse[:3,3] = (-R.T @ t.reshape(3,1)).flatten()
    return transform_inverse

class GroundtruthComparisonNode:

    def __init__(self) -> None:
        self.base_link_T_gps = None
        self.gps_T_base_link = None
        self.subscriber_groundtruth = Subscriber("/carina/sensor/ins_utm_pose_gt", PoseWithCovarianceStamped)
        self.subscriber_estimations = Subscriber("/localization/ekf/global", PoseWithCovarianceStamped)
        self.publisher_absolute_translational_error = rospy.Publisher("/localization/metrics/absolute_translational_error_meters", Float64, queue_size=1000)
        self.publisher_absolute_rotational_error = rospy.Publisher("/localization/metrics/absolute_rotational_error_degrees", Float64, queue_size=1000)
        self.time_synchronizer = ApproximateTimeSynchronizer([self.subscriber_groundtruth, self.subscriber_estimations], queue_size=100, slop=0.01)
        self.time_synchronizer.registerCallback(self.callback)

    def wait_for_calibration_tf(self) -> None:
        # Load the latest TF available
        tf_Buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_Buffer)
        flag_received_calibration_tf = False
        transform = None
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not flag_received_calibration_tf:
            try:
                transform = tf_Buffer.lookup_transform("gps", "base_link", rospy.Time(0))
                flag_received_calibration_tf = True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue    

        # Split components
        translation = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]
        rotation = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]

        # Convert to numpy array
        self.base_link_T_gps = np.eye(4)
        self.base_link_T_gps[:3,:3] = Rotation.from_quat(rotation).as_matrix()
        self.base_link_T_gps[:3,3] = translation
        self.gps_T_base_link = invert_transformation_matrix(self.base_link_T_gps)

    def callback(self,groundtruth_msg : PoseWithCovarianceStamped, estimation_msg : PoseWithCovarianceStamped) -> None:
        # Convert groundtruth pose to array
        map_T_gps_groundtruth = ros_pose_to_transformation_matrix(groundtruth_msg.pose.pose)

        # Transform groundtruth pose to the base_link frame
        map_T_base_link_groundtruth = map_T_gps_groundtruth @ self.gps_T_base_link

        # Convert estimation pose to array
        map_T_base_link_estimated = ros_pose_to_transformation_matrix(estimation_msg.pose.pose)

        # Compute the transformation matrix of the pose between the estimated and the groundtruth
        groundtruth_T_estimated = invert_transformation_matrix(map_T_base_link_groundtruth) @ map_T_base_link_estimated

        # Compare their divergence in translation
        divergence_translation = groundtruth_T_estimated[:3,3]
        absolute_translation_error = np.linalg.norm(divergence_translation[:2])

        # Compare their divergence in rotation
        divergence_rotation_matrix = groundtruth_T_estimated[:3,:3]
        divergence_roll, divergence_pitch, divergence_yaw = Rotation.from_matrix(divergence_rotation_matrix).as_euler("xyz", degrees=True)
        absolute_rotational_error = np.abs(divergence_yaw) # TODO: consider the remaining angles

        # Publish
        absolute_translational_error_msg = Float64()
        absolute_translational_error_msg.data = absolute_translation_error
        self.publisher_absolute_translational_error.publish(absolute_translational_error_msg)

        absolute_rotational_error_msg = Float64()
        absolute_rotational_error_msg.data = absolute_rotational_error
        self.publisher_absolute_rotational_error.publish(absolute_rotational_error_msg)

    def spin(self) -> None:
        self.wait_for_calibration_tf()
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node("groundtruth_comparison_node")
    node = GroundtruthComparisonNode()
    node.spin()