#!/usr/bin/python3
from typing import Tuple
import numpy as np

import rospy
from message_filters import ApproximateTimeSynchronizer, Subscriber

from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped

import math
from std_msgs.msg import Bool

def geodesic_to_mercator(lat_ref : float, lon_ref : float, lat : float, lon : float) -> Tuple[float, float, float]:
  EARTH_RADIUS_EQUA = 6378137.0
  scale = math.cos(lat_ref * math.pi / 180.0)

  mx_ref = scale * lon_ref * math.pi * EARTH_RADIUS_EQUA / 180
  my_ref = scale * EARTH_RADIUS_EQUA * math.log( math.tan( (90.0 + lat_ref) * math.pi / 360 ) )
  
  mx = scale * lon * math.pi * EARTH_RADIUS_EQUA / 180.0
  my = scale * EARTH_RADIUS_EQUA * math.log( math.tan( (90.0 + lat) * math.pi / 360 ) )
  x = mx - mx_ref
  y = my - my_ref

  return [0, y, -x]

class FixToUTMPoseNode:

  def __init__(self) -> None:
    rospy.init_node("gps_to_odometry_node", anonymous=True)
    self.subscriber_gps = Subscriber("/carina/localization/nav_sat_fix", NavSatFix)
    self.subscriber_imu = Subscriber("/carina/sensor/imu", Imu)
    self.publisher = rospy.Publisher("/carina/sensor/ins_utm_pose", PoseWithCovarianceStamped, queue_size=1000)
    self.time_synchronizer = ApproximateTimeSynchronizer([self.subscriber_gps, self.subscriber_imu], queue_size=100, slop=0.01)
    self.time_synchronizer.registerCallback(self.callback)
    self.shutdown_sub = rospy.Subscriber('/carina/vehicle/shutdown', Bool, self.shutdown_cb, queue_size=1)

  def callback(self, gps_msg : NavSatFix, imu_msg : Imu ) -> None:
    # Header
    out_msg = PoseWithCovarianceStamped()
    out_msg.header = gps_msg.header
    out_msg.header.frame_id = "map"

    # Position and orientation values
    #x,y, _,_ = utm.from_latlon(gps_msg.latitude, gps_msg.longitude)
    _, x, y = geodesic_to_mercator(0.0, 0.0, gps_msg.latitude, gps_msg.longitude)
    
    out_msg.pose.pose.position.x = x
    out_msg.pose.pose.position.y = y
    out_msg.pose.pose.position.z = gps_msg.altitude
    out_msg.pose.pose.orientation.x = imu_msg.orientation.x
    out_msg.pose.pose.orientation.y = imu_msg.orientation.y
    out_msg.pose.pose.orientation.z = imu_msg.orientation.z
    out_msg.pose.pose.orientation.w = imu_msg.orientation.w

    # Covariance
    out_covariance = np.eye(6)
    out_covariance[:3,:3] *= 5.0
    out_covariance[3:,3:] *= 0.1
    out_msg.pose.covariance = tuple(out_covariance.flatten())
    self.publisher.publish(out_msg)

  def shutdown_cb(self, msg):
    if msg.data:
      print ("Bye!")
      del self.subscriber_gps 
      del self.subscriber_imu 
      del self.publisher 
      del self.time_synchronizer 

      del self.shutdown_sub
      rospy.signal_shutdown("finished route") 
      
if __name__ == "__main__":
  node = FixToUTMPoseNode()
  rospy.spin()