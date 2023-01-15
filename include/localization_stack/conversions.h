#pragma once

#include <gtsam/geometry/Pose3.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>

namespace lrm
{

gtsam::Pose3 fromROSPoseToGTSAMPose( const geometry_msgs::Pose & ros_pose );

geometry_msgs::Pose fromGTSAMPoseToROSPose( const gtsam::Pose3 & gtsam_pose );

tf::Transform fromROSPoseToTFTransform( const geometry_msgs::Pose & ros_pose );

gtsam::Pose3 fromTFTransformToGTSAMPose( const tf::Transform & tf_transform );

tf::Transform fromGTSAMPoseToTFTransform( const gtsam::Pose3 & gtsam_pose );

}