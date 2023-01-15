#!/usr/bin/python3
import numpy as np

import utm
import rospy
from message_filters import ApproximateTimeSynchronizer, Subscriber

from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped

class FixToUTMPoseNode:

  def __init__(self) -> None:
    rospy.init_node("gps_to_odometry_node")
    self.subscriber_gps = Subscriber("/carina/localization/nav_sat_fix", NavSatFix)
    self.subscriber_imu = Subscriber("/carina/sensor/imu_corrected", Imu)
    self.publisher = rospy.Publisher("/carina/sensor/ins_utm_pose", PoseWithCovarianceStamped, queue_size=1000)
    self.time_synchronizer = ApproximateTimeSynchronizer([self.subscriber_gps, self.subscriber_imu], queue_size=100, slop=0.01)
    self.time_synchronizer.registerCallback(self.callback)

  def callback(self, gps_msg : NavSatFix, imu_msg : Imu ) -> None:
    # Header
    out_msg = PoseWithCovarianceStamped()
    out_msg.header = gps_msg.header
    out_msg.header.frame_id = "gps"

    # Position and orientation values
    x,y, _,_ = utm.from_latlon(gps_msg.latitude, gps_msg.longitude)
    
    out_msg.pose.pose.position.x = x
    out_msg.pose.pose.position.y = y
    out_msg.pose.pose.position.z = gps_msg.altitude
    out_msg.pose.pose.orientation.x = imu_msg.orientation.x
    out_msg.pose.pose.orientation.y = imu_msg.orientation.y
    out_msg.pose.pose.orientation.z = imu_msg.orientation.z
    out_msg.pose.pose.orientation.w = imu_msg.orientation.w

    # Covariance
    out_covariance = np.eye(6)
    out_covariance[:3,:3] *= 10.0
    out_covariance[3:,3:] *= 1.0
    out_msg.pose.covariance = tuple(out_covariance.flatten())
    self.publisher.publish(out_msg)

if __name__ == "__main__":
  node = FixToUTMPoseNode()
  rospy.spin()