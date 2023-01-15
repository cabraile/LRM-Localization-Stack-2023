#include "localization_stack/conversions.h"

namespace lrm
{

gtsam::Pose3 fromROSPoseToGTSAMPose( const geometry_msgs::Pose & ros_pose )
{
    gtsam::Vector3 translation;
    translation << ros_pose.position.x, ros_pose.position.y, ros_pose.position.z;
    gtsam::Quaternion rotation( ros_pose.orientation.w, ros_pose.orientation.x, ros_pose.orientation.y, ros_pose.orientation.z );
    gtsam::Pose3 gtsam_pose(gtsam::Rot3(rotation.normalized()), translation);
    return gtsam_pose;
}

geometry_msgs::Pose fromGTSAMPoseToROSPose( const gtsam::Pose3 & gtsam_pose )
{
    geometry_msgs::Pose ros_pose;
    ros_pose.position.x = gtsam_pose.x();
    ros_pose.position.y = gtsam_pose.y();
    ros_pose.position.z = gtsam_pose.z();
    gtsam::Quaternion gtsam_quaternion = gtsam_pose.rotation().toQuaternion().normalized();
    ros_pose.orientation.w = gtsam_quaternion.w();
    ros_pose.orientation.x = gtsam_quaternion.x();
    ros_pose.orientation.y = gtsam_quaternion.y();
    ros_pose.orientation.z = gtsam_quaternion.z();
    return ros_pose;
}

gtsam::Pose3 fromTFTransformToGTSAMPose( const tf::Transform & tf_transform )
{
    tf::Vector3 tf_translation = tf_transform.getOrigin();
    tf::Quaternion tf_rotation = tf_transform.getRotation();

    gtsam::Vector3 gtsam_translation;
    gtsam_translation << tf_translation.x(), tf_translation.y(), tf_translation.z();

    gtsam::Quaternion gtsam_rotation( tf_rotation.w(), tf_rotation.x(), tf_rotation.y(), tf_rotation.z() );
    
    gtsam::Pose3 gtsam_pose(gtsam::Rot3(gtsam_rotation.normalized()), gtsam_translation);

    return gtsam_pose;
}

tf::Transform fromGTSAMPoseToTFTransform( const gtsam::Pose3 & gtsam_pose )
{
    tf::Vector3 tf_translation(gtsam_pose.x(),gtsam_pose.y(),gtsam_pose.z());
    gtsam::Quaternion gtsam_quaternion = gtsam_pose.rotation().toQuaternion().normalized();
    tf::Quaternion tf_rotation(gtsam_quaternion.x(),gtsam_quaternion.y(),gtsam_quaternion.z(),gtsam_quaternion.w());
    tf::Transform tf_transform(tf_rotation, tf_translation);
    return tf_transform;
}

}