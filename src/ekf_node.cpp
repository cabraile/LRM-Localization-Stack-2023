#include<memory>
#include<vector>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "localization_stack/ekf.h"
#include "localization_stack/conversions.h"

class EKFNode
{

public:
    EKFNode()
    {
        _node_handler = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("~"));
        _count_global_pose_msgs_received = 0;
        _count_visual_odom_msgs_received = 0;
    }

    void callbackVisualOdometryMessage(const nav_msgs::Odometry::ConstPtr & odom_ptr )
    {
        // Convert from ROS pose to gtsam pose (already in the base_link frame)
        gtsam::Pose3 visual_odom_T_base_link = lrm::fromROSPoseToGTSAMPose(odom_ptr->pose.pose);

        // Get covariance diagonals from the message
        gtsam::Vector diagonal_components_vector = gtsam::Vector::Zero(6);
        for(size_t i = 0; i < 6; i++)
        {
            diagonal_components_vector(i) = odom_ptr->pose.covariance[7 * i];
        }

        // Update latest timestamp
        _timestamp_last_callback = odom_ptr->header.stamp;

        // EKF prediction
        _ekf.predict(visual_odom_T_base_link, diagonal_components_vector);

        // Update control attributes
        _count_visual_odom_msgs_received++;

        // Update the distance from the last update
        gtsam::Pose3 between_pose = _visual_odom_T_base_link_prev.inverse() * visual_odom_T_base_link;
        _distance_since_last_update += between_pose.translation().norm();
        _visual_odom_T_base_link_prev = visual_odom_T_base_link;

        ROS_DEBUG("Received %ld visual odometry pose(s)", _count_global_pose_msgs_received);

        this->publish();
    }

    void callbackGlobalPoseMessage(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & pose_ptr )
    {
        // Convert from ROS pose to gtsam pose
        gtsam::Pose3 utm_T_gps = lrm::fromROSPoseToGTSAMPose(pose_ptr->pose.pose);
        // Transform to the base_link frame
        gtsam::Pose3 utm_T_base_link = utm_T_gps * _gps_T_base_link;

        // Get covariance diagonals from the message
        gtsam::Vector diagonal_components_vector = gtsam::Vector::Zero(6);
        for(size_t i = 0; i < 6; i++)
        {
            diagonal_components_vector(i) = pose_ptr->pose.covariance[7 * i];
        }

        // Handle first time GPS is received
        if (_count_global_pose_msgs_received == 0)
        {
            // Initialize EKF
            _ekf.initialize( utm_T_base_link, diagonal_components_vector );

            // Update control attributes
            _distance_since_last_update = 0.0;
            _count_global_pose_msgs_received++;

            ROS_DEBUG("Received %ld global pose(s)", _count_global_pose_msgs_received);
            return ;
        }

        // Disregard messages when the vehicle didn't move enough
        if (_distance_since_last_update < _min_accepted_distance_between_corrections)
        {
            ROS_DEBUG("Disregarding global pose: moved %.2fm from %.2fm before correction", _distance_since_last_update, _min_accepted_distance_between_corrections);
            return ;
        }

        _ekf.update(utm_T_base_link, diagonal_components_vector);
        _count_global_pose_msgs_received++;
        ROS_DEBUG("Received %ld global pose(s)", _count_global_pose_msgs_received);
        
        // Update control variables
        _timestamp_last_callback = pose_ptr->header.stamp;
        _distance_since_last_update = 0.0;

        this->publish();
    }

    void spin()
    {
        tf::TransformListener tf_listener;

        // Wait for GPS to base_link transform and camera to base_link
        tf::StampedTransform tf_base_link_T_gps;
        tf::StampedTransform tf_base_link_T_cam;
        bool flag_received_gps_transform = false;
        bool flag_received_transforms = false;
        while(!flag_received_transforms)
        {
            try
            {
                tf_listener.lookupTransform("/base_link", "/gps", ros::Time(0), tf_base_link_T_gps);
                tf_listener.lookupTransform("/base_link", "/stereo", ros::Time(0), tf_base_link_T_cam);
                flag_received_transforms = true;
            }
            catch (tf::TransformException exception)
            {
                ROS_ERROR("%s",exception.what());
                ros::Duration(1.0).sleep();
            }
        }

        // Convert them to GTSAM poses
        _base_link_T_gps = lrm::fromTFTransformToGTSAMPose(tf_base_link_T_gps);
        _gps_T_base_link = _base_link_T_gps.inverse();
        _base_link_T_cam = lrm::fromTFTransformToGTSAMPose(tf_base_link_T_cam);
        _cam_T_base_link = _base_link_T_cam.inverse();

        // Wait first global pose message
        ros::Subscriber global_pose_sub = _node_handler->subscribe("/carina/sensor/ins_utm_pose", 10, &EKFNode::callbackGlobalPoseMessage, this);
        ROS_INFO("Waiting for global pose");
        while(!_ekf.isInitialized())
        {
            ros::spinOnce();
        }

        // Subscribe to the odometry topic
        ros::Subscriber visual_odometry_pose_sub = _node_handler->subscribe("/localization/odometry/visual", 10, &EKFNode::callbackVisualOdometryMessage, this);

        // Initialize publishers
        _estimated_local_pose_pub = _node_handler->advertise<geometry_msgs::PoseWithCovarianceStamped>("/localization/ekf/local", 1000);
        _estimated_global_pose_pub = _node_handler->advertise<geometry_msgs::PoseWithCovarianceStamped>("/localization/ekf/global", 1000);

        // Loop
        ROS_WARN("Started loop");
        ros::spin();
    }

    void publish()
    {
        // Publish local frame pose
        gtsam::Pose3 base_link_origin_T_base_link = _ekf.getCurrentLocalState();
        geometry_msgs::PoseWithCovarianceStamped pose_local_msg;
        pose_local_msg.pose.pose = lrm::fromGTSAMPoseToROSPose(base_link_origin_T_base_link);
        _estimated_local_pose_pub.publish(pose_local_msg);

        // Publish UTM frame pose
        gtsam::Pose3 utm_T_base_link_origin = _ekf.getOriginToUTMTransform();
        gtsam::Pose3 utm_T_base_link = utm_T_base_link_origin * base_link_origin_T_base_link;
        geometry_msgs::PoseWithCovarianceStamped pose_global_msg;
        pose_global_msg.pose.pose = lrm::fromGTSAMPoseToROSPose(utm_T_base_link);
        _estimated_global_pose_pub.publish(pose_global_msg);

        // Publish global to local frame TF
        tf::Transform transform_utm_T_odom = lrm::fromGTSAMPoseToTFTransform(utm_T_base_link_origin);
        _tf_broadcaster.sendTransform(
            tf::StampedTransform(
                transform_utm_T_odom,
                _timestamp_last_callback,
                "utm",
                "odom"
            )
        );

        // Publish local to base_link TF
        tf::Transform transform_odom_T_base_link = lrm::fromGTSAMPoseToTFTransform(base_link_origin_T_base_link);
        _tf_broadcaster.sendTransform(
            tf::StampedTransform(
                transform_odom_T_base_link,
                _timestamp_last_callback,
                "odom",
                "base_link"
            )
        );
    }

private:
    // --------------
    // ROS
    // --------------

    std::shared_ptr<ros::NodeHandle> _node_handler;
    
    ros::Publisher _estimated_local_pose_pub;
    ros::Publisher _estimated_global_pose_pub;

    tf::TransformBroadcaster _tf_broadcaster;

    // --------------
    // CONTROL ATTRIBUTES
    // --------------

    ros::Time _timestamp_last_callback;

    lrm::EKF _ekf;

    gtsam::Pose3 _base_link_T_gps;
    gtsam::Pose3 _gps_T_base_link;
    gtsam::Pose3 _base_link_T_cam;
    gtsam::Pose3 _cam_T_base_link;

    gtsam::Pose3 _visual_odom_T_base_link_prev;

    // Distance the vehicle moved (estimated from the odometry) between global poses
    double _distance_since_last_update;

    // The least distance a global pose must be from the pose in the last correction in order to accept.
    // Prevents pose jumps from noisy GPS.
    double _min_accepted_distance_between_corrections = 25.0;

    // --------------
    // DIAGNOSTICS TOPICS
    // --------------

    size_t _count_global_pose_msgs_received;
    size_t _count_visual_odom_msgs_received;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "lrm_ekf_node");
    EKFNode node;
    node.spin();
    return 0;
}