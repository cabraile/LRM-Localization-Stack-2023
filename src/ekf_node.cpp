#include<memory>
#include<vector>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "localization_stack/ekf.h"
#include "localization_stack/conversions.h"
#include <std_msgs/Bool.h>

// @brief Check if the diagonals used for a covariance matrix are valid (positive, not zero and not too big).
bool areCovarianceDiagonalValuesValid(const gtsam::Vector & diagonal_values, double max_diagonal_value = 1e2)
{
    // Check if all values are positive and not too close from zero
    bool all_positive = (diagonal_values.array() > 1e-8).all();

    // Check if all of them are lower than the maximum diagonal value
    bool all_bounded = (diagonal_values.array() <= max_diagonal_value).all();

    return (all_positive && all_bounded);
}

class EKFNode
{

public:

    EKFNode()
    {
        _node_handler = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("~"));
        
        this->loadROSParams();

        _count_global_pose_msgs_received = 0;
        _count_visual_odom_msgs_received = 0;
    }

    void loadROSParams()
    {
        // Retrieve the frames used.
        _node_handler->getParam("frames/local_frame_id",_local_frame_id);
        _node_handler->getParam("frames/global_frame_id",_global_frame_id);

        // Retrieve the input filter params.
        _node_handler->getParam("input_filter/min_accepted_distance_between_corrections",_min_accepted_distance_between_corrections);
        _node_handler->getParam("input_filter/max_acceptable_covariance_diagonal_value_visual_odom_pose",_max_acceptable_covariance_diagonal_value_visual_odom_pose);
        _node_handler->getParam("input_filter/max_acceptable_covariance_diagonal_value_global_pose",_max_acceptable_covariance_diagonal_value_global_pose);
        _node_handler->getParam("input_filter/max_input_poses_velocity_kmph",_max_input_poses_velocity_kmph);
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

        // Outlier rejection by covariance
        bool accept_by_covariance = areCovarianceDiagonalValuesValid(diagonal_components_vector, _max_acceptable_covariance_diagonal_value_visual_odom_pose);
        if(!accept_by_covariance)
        {
            ROS_ERROR("[ekf_node.cpp] Dropped odometry pose due to invalid covariance.");
            for(size_t i = 0; i < 6; i++)
            {
                ROS_WARN_STREAM( "diagonal_components_vector["<< i <<"]"<<diagonal_components_vector(i));
            }
            return ;
        }

        // Outlier rejection by position jumps
        double translated_distance = 0.0;
        if (_count_visual_odom_msgs_received > 0)
        {
            gtsam::Pose3 between_pose = _visual_odom_T_base_link_prev.inverse() * visual_odom_T_base_link;
            translated_distance = between_pose.translation().norm();
            double time_delta = (odom_ptr->header.stamp - _timestamp_last_accepted_odom).toSec();
            double velocity_m_s = translated_distance / time_delta;
            double velocity_kmph = velocity_m_s * 3.6;
            if (velocity_kmph > _max_input_poses_velocity_kmph)
            {
                ROS_ERROR("[ekf_node.cpp] Dropped odometry message due to position jump.");
                ROS_ERROR_STREAM("[ekf_node.cpp] Odometry jump speed: " << velocity_m_s << " meters/second");
            }
        }

        // EKF prediction
        try 
        {
            _ekf.predict(visual_odom_T_base_link, diagonal_components_vector);
        }
        catch (const gtsam::IndeterminantLinearSystemException& ex) 
        {
            ROS_ERROR_STREAM("[ekf_node.cpp] IndeterminantLinearSystemException caught during EKF prediction: " << ex.what());
        }

        // Update latest timestamp
        _timestamp_last_callback = odom_ptr->header.stamp;
        _timestamp_last_accepted_odom = odom_ptr->header.stamp;

        // Update control attributes
        _count_visual_odom_msgs_received++;

        // Update the distance from the last update
        _distance_since_last_update += translated_distance;
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

        // Outlier rejection by covariance
        bool accept_by_covariance = areCovarianceDiagonalValuesValid(diagonal_components_vector, _max_acceptable_covariance_diagonal_value_global_pose);
        if(!accept_by_covariance)
        {
            ROS_ERROR("[ekf_node.cpp] Dropped global pose due to invalid covariance.");
            return ;
        }

        // Outlier rejection by position jumps
        double translated_distance = 0.0;
        if (_count_visual_odom_msgs_received > 0)
        {
            gtsam::Pose3 between_pose = _utm_T_base_link_prev.inverse() * utm_T_base_link;
            translated_distance = between_pose.translation().norm();
            double time_delta = (pose_ptr->header.stamp - _timestamp_last_accepted_global_pose).toSec();
            double velocity_m_s = translated_distance / time_delta;
            double velocity_kmph = velocity_m_s * 3.6;
            if (velocity_kmph > _max_input_poses_velocity_kmph)
            {
                ROS_ERROR("[ekf_node.cpp] Dropped global pose message due to position jump.");
                ROS_ERROR_STREAM("[ekf_node.cpp] Global pose jump speed: " << velocity_m_s << " meters/second");
            }
        }

        try
        {
            _ekf.update(utm_T_base_link, diagonal_components_vector);
        } 
        catch (const gtsam::IndeterminantLinearSystemException& ex) 
        {
            ROS_ERROR_STREAM("[ekf_node.cpp] IndeterminantLinearSystemException caught during EKF update: " << ex.what());
        }

        _count_global_pose_msgs_received++;
        ROS_DEBUG("Received %ld global pose(s)", _count_global_pose_msgs_received);
        
        // Update control variables
        _utm_T_base_link_prev = utm_T_base_link;
        _timestamp_last_callback = pose_ptr->header.stamp;
        _timestamp_last_accepted_global_pose = pose_ptr->header.stamp;
        _distance_since_last_update = 0.0;

        this->publish();
    }

    void shutdown_cb(const std_msgs::Bool::ConstPtr &msg){
        if (msg->data){
          
          this->global_pose_sub.shutdown();
          this->shutdown_sub.shutdown();
          this->visual_odometry_pose_sub.shutdown();
          this->_estimated_local_pose_pub.shutdown();
          this->_estimated_global_pose_pub.shutdown();
          this->_estimated_local_trajectory_pub.shutdown();

          std::cout<<"Bye!"<<std::endl;
          ros::shutdown();
        }
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
        global_pose_sub = _node_handler->subscribe("/carina/sensor/ins_utm_pose", 10, &EKFNode::callbackGlobalPoseMessage, this);
        ROS_INFO("Waiting for global pose");
        while(!_ekf.isInitialized())
        {
            ros::spinOnce();
        }

        // Subscribe to the odometry topic
        visual_odometry_pose_sub = _node_handler->subscribe("/localization/odometry/visual", 10, &EKFNode::callbackVisualOdometryMessage, this);

        // Initialize publishers
        _estimated_local_pose_pub = _node_handler->advertise<geometry_msgs::PoseWithCovarianceStamped>("/localization/ekf/local", 1000);
        _estimated_global_pose_pub = _node_handler->advertise<geometry_msgs::PoseWithCovarianceStamped>("/localization/ekf/global", 1000);
        _estimated_local_trajectory_pub = _node_handler->advertise<nav_msgs::Path>("/localization/ekf/local_path", 1000);

        // Subscribe to the shotdown topic
        shutdown_sub = _node_handler->subscribe("/carina/vehicle/shutdown", 1, &EKFNode::shutdown_cb, this);
        // Loop
        ROS_WARN("[ekf_node.cpp] Started loop");
        ros::spin();
    }

    void publish()
    {
        // Avoids annoying "REPEATED_TF" messages.
        if ( (_timestamp_last_callback - _timestamp_last_published).toSec() < 0.001 )
        {
            return ;
        }
        // Publish local frame pose
        gtsam::Pose3 base_link_origin_T_base_link = _ekf.getCurrentLocalState();
        geometry_msgs::PoseWithCovarianceStamped pose_local_msg;
        pose_local_msg.header.stamp = _timestamp_last_callback;
        pose_local_msg.header.frame_id = "odom";
        pose_local_msg.pose.pose = lrm::fromGTSAMPoseToROSPose(base_link_origin_T_base_link);
        _estimated_local_pose_pub.publish(pose_local_msg);

        // Publish UTM frame pose
        gtsam::Pose3 utm_T_base_link_origin = _ekf.getOriginToUTMTransform();
        gtsam::Pose3 utm_T_base_link = utm_T_base_link_origin * base_link_origin_T_base_link;
        geometry_msgs::PoseWithCovarianceStamped pose_global_msg;
        pose_global_msg.header.stamp = _timestamp_last_callback;
        pose_local_msg.header.frame_id = "map";
        pose_global_msg.pose.pose = lrm::fromGTSAMPoseToROSPose(utm_T_base_link);
        _estimated_global_pose_pub.publish(pose_global_msg);

        // Publish global to local frame TF
        tf::Transform transform_utm_T_odom = lrm::fromGTSAMPoseToTFTransform(utm_T_base_link_origin);
        _tf_broadcaster.sendTransform(
            tf::StampedTransform(
                transform_utm_T_odom,
                _timestamp_last_callback,
                "map",
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

        // Update trajectory
        geometry_msgs::PoseStamped pose_local_current;
        pose_local_current.header.frame_id = "odom";
        pose_local_current.header.stamp = _timestamp_last_callback;
        pose_local_current.pose = pose_local_msg.pose.pose;
        _trajectory_local.push_back(pose_local_current);

        // Publish trajectory
        nav_msgs::Path trajectory_local;
        trajectory_local.header.frame_id = "odom";
        trajectory_local.header.stamp = _timestamp_last_callback;
        trajectory_local.poses = _trajectory_local;
        _estimated_local_trajectory_pub.publish(trajectory_local);

        _timestamp_last_published = _timestamp_last_callback;
    }

private:
    // --------------
    // ROS
    // --------------

    std::shared_ptr<ros::NodeHandle> _node_handler;
    
    ros::Publisher _estimated_local_pose_pub;
    ros::Publisher _estimated_global_pose_pub;
    ros::Publisher _estimated_local_trajectory_pub;


    ros::Subscriber global_pose_sub;
    ros::Subscriber visual_odometry_pose_sub;
    ros::Subscriber shutdown_sub;

    tf::TransformBroadcaster _tf_broadcaster;

    // Frame ID centered at the base_link's origin.
    std::string _local_frame_id;
    // Frame ID centered at the world's origin.
    std::string _global_frame_id;

    // --------------
    // CONTROL ATTRIBUTES
    // --------------

    ros::Time _timestamp_last_callback;
    ros::Time _timestamp_last_accepted_odom;
    ros::Time _timestamp_last_accepted_global_pose;
    ros::Time _timestamp_last_published;

    lrm::EKF _ekf;

    gtsam::Pose3 _base_link_T_gps;
    gtsam::Pose3 _gps_T_base_link;
    gtsam::Pose3 _base_link_T_cam;
    gtsam::Pose3 _cam_T_base_link;

    // Last visual odometry pose (transform of the base_link w.r.t. the odometry origin)
    gtsam::Pose3 _visual_odom_T_base_link_prev;
    // Last received GPS pose w.r.t. the world
    gtsam::Pose3 _utm_T_base_link_prev;

    // Distance the vehicle moved (estimated from the odometry) between global poses
    double _distance_since_last_update;

    // --------------
    // INPUT FILTERING
    // --------------

    // The least distance a global pose must be from the pose in the last correction in order to accept.
    // Prevents pose jumps from noisy GPS.
    double _min_accepted_distance_between_corrections;

    // Refuses odometry poses which have covariance values larger than this.
    double _max_acceptable_covariance_diagonal_value_visual_odom_pose;

    // Refuses global poses which have covariance values larger than this.
    double _max_acceptable_covariance_diagonal_value_global_pose;

    // Max vehicle velocity in km/h. Used for rejecting outlier poses.
    double _max_input_poses_velocity_kmph;

    // --------------
    // DIAGNOSTICS TOPICS
    // --------------

    size_t _count_global_pose_msgs_received;
    size_t _count_visual_odom_msgs_received;
    std::vector<geometry_msgs::PoseStamped> _trajectory_local;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "lrm_ekf_node");
    EKFNode node;
    node.spin();
    return 0;
}