#include <deque>
#include <memory>
#include <cmath>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <gtsam/geometry/Pose3.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include "localization_stack/conversions.h"

// Type Definitions and namespaces
using pcl::PointCloud;
using gtsam::Pose3;
typedef pcl::PointXYZ PointT;

class MappingNode
{

public:
    MappingNode()
    {
        _node_handle_ptr = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("~"));
        this->loadROSParams();

        _total_scans_received = 0;
    }

    void loadROSParams()
    {
        _node_handle_ptr->getParam("input/topic_scan",_scans_topic);
        _node_handle_ptr->getParam("input/topic_odometry",_odometry_topic);
        _node_handle_ptr->getParam("output/map_local",_out_map_local_topic);
        _node_handle_ptr->getParam("mapping/max_clouds_queue",_max_clouds_queue);
        _node_handle_ptr->getParam("mapping/downsample_resolution",_downsample_resolution);
        _node_handle_ptr->getParam("communication/min_synchronization_time_diff_sec",_min_synchronization_time_diff_sec);
    }

    void spin()
    {

        ROS_DEBUG("[mapping_node.cpp] Waiting for TF");
        // Wait for static transform from the base_link frame to LiDAR
        tf::TransformListener tf_listener;
        tf::StampedTransform tf_base_link_T_lidar;
        bool flag_received_transforms = false;
        while(!flag_received_transforms)
        {
            try
            {
                tf_listener.lookupTransform("/base_link", "/velodyne", ros::Time(0), tf_base_link_T_lidar);
                flag_received_transforms = true;
            }
            catch (tf::TransformException exception)
            {
                ROS_ERROR("%s",exception.what());
                ros::Duration(0.1).sleep();
            }
        }
        _base_link_T_lidar = lrm::fromTFTransformToGTSAMPose(tf_base_link_T_lidar);

        // Instantiate synchronized subscribers
        ROS_DEBUG("[mapping_node.cpp] Starting synchronized subscriber");
        // Start ROS communication
        _publisher_local_window = _node_handle_ptr->advertise<sensor_msgs::PointCloud2>(_out_map_local_topic, 10);
        ros::Subscriber subscriber_scan = _node_handle_ptr->subscribe(_scans_topic, 100, &MappingNode::callbackScan, this);
        ros::Subscriber subscriber_odometry = _node_handle_ptr->subscribe(_odometry_topic, 100, &MappingNode::callbackOdometry, this);

        // Spin
        ROS_DEBUG("[mapping_node.cpp] Started loop");
        ros::spin();
    }

    // @brief Registers the point clouds w.r.t. the last point cloud.
    PointCloud<PointT>::Ptr buildLocalMap()
    {
        // Declare a point cloud PointCloud<PointT>::Ptr local_window for storing the registered points
        PointCloud<PointT>::Ptr local_window(new PointCloud<PointT>);

        // For each cloud in the std::deque<std::pair<Pose3, PointCloud<PointT>::Ptr>> _cloud_pair_queue structure do:
        Pose3 origin_T_lidar_last = _cloud_pair_queue.back().first;
        Pose3 lidar_last_T_origin = origin_T_lidar_last.inverse();
        for (const std::pair<Pose3, PointCloud<PointT>::Ptr> & pair : _cloud_pair_queue)
        {
            // 1. Split the pose and the cloud
            Pose3 origin_T_lidar_i = pair.first;
            PointCloud<PointT>::Ptr cloud = pair.second;

            // 2. Compute the pose of the current cloud with respect to the last pose, last_T_curr
            Pose3 lidar_last_T_lidar_i = lidar_last_T_origin * origin_T_lidar_i;

            // 3. Transform the point cloud points with the last_T_curr pose
            pcl::transformPointCloud(*cloud, *cloud, lidar_last_T_lidar_i.matrix());

            // 4. Merge the registered points in the local window
            *local_window += *cloud;

            // 5. Apply a downsample considering the _downsample_resolution value
            pcl::VoxelGrid<PointT> vg;
            vg.setInputCloud(local_window);
            vg.setLeafSize(_downsample_resolution, _downsample_resolution, _downsample_resolution);
            vg.filter(*local_window);
        }

        return local_window;
    }

private:

    ros::Publisher _publisher_local_window;

    // Queue of point clouds and their origin_T_lidar transforms' pairs
    std::deque<std::pair<Pose3, PointCloud<PointT>::Ptr>> _cloud_pair_queue;

    // Maximum number of clouds for storing in the queue
    int _max_clouds_queue;

    // Resolution in meters
    float _downsample_resolution;

    // Transform from the LiDAR to the base_link frame
    Pose3 _base_link_T_lidar;

    //
    size_t _total_scans_received;
    

    float _min_synchronization_time_diff_sec;


    // ROS node handler
    std::shared_ptr< ros::NodeHandle > _node_handle_ptr;

    // Timestamp of the last synchronized pair's point cloud.
    ros::Time _timestamp_curr;

    // 
    std::string _scans_topic;

    //
    std::string _out_map_local_topic;

    //
    std::string _odometry_topic;

    // Last scan message for synchronization
    sensor_msgs::PointCloud2 _last_scan_msg;

    // Last odometry message for synchronization
    nav_msgs::Odometry _last_odom_msg;

    ////////////////////////
    // Callbacks
    ////////////////////////

    void callbackScan(const sensor_msgs::PointCloud2 & scan_msg)
    {
        float time_diff = abs( (scan_msg.header.stamp - _last_odom_msg.header.stamp).toSec() );
        // Run the synchronized callback
        if (time_diff < _min_synchronization_time_diff_sec)
        {
            this->callbackSynchronizedScan(scan_msg, _last_odom_msg);
        }
        // Store for later usage
        else
        {
            _last_scan_msg = scan_msg;
        }
    }

    void callbackOdometry(const nav_msgs::Odometry & odom_msg)
    {
        float time_diff = abs( (_last_scan_msg.header.stamp - odom_msg.header.stamp).toSec() );
        // Run the synchronized callback
        if (time_diff < _min_synchronization_time_diff_sec)
        {
            this->callbackSynchronizedScan(_last_scan_msg, odom_msg);
        }
        // Store for later usage
        else
        {   
            _last_odom_msg = odom_msg;
        }
    }

    void callbackSynchronizedScan(const sensor_msgs::PointCloud2 & scan_msg, const nav_msgs::Odometry & odom_msg )
    {
        // Convert the scan msg to a pcl::PointCloud<pcl::PointXYZI>
        PointCloud<PointT>::Ptr cloud(new PointCloud<PointT>);
        pcl::fromROSMsg(scan_msg, *cloud);

        // Downsample in a given resolution
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud (cloud);
        vg.setLeafSize (_downsample_resolution, _downsample_resolution, _downsample_resolution);
        vg.filter (*cloud);

        // Convert the odometry pose to a gtsam::Pose3
        Pose3 origin_T_base_link;
        origin_T_base_link = Pose3(
            gtsam::Rot3::Quaternion(odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z),
            gtsam::Point3(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z)
        );

        // Transform the pose to the LiDAR frame
        Pose3 origin_T_lidar = origin_T_base_link * _base_link_T_lidar;

        // Store pair in the deque maintaining the maximum size
        if (_cloud_pair_queue.size() >= _max_clouds_queue)
        {
            _cloud_pair_queue.pop_front();
        }
        _cloud_pair_queue.push_back(std::make_pair(origin_T_lidar, cloud));

        // Updates the timestamp
        _timestamp_curr = scan_msg.header.stamp;

        // Increments the number of scans received
        _total_scans_received++;

        // Publishes the local window
        this->publishLocalMap();
    }

    ////////////////////////
    // Publishers
    ////////////////////////

    void publishLocalMap()
    {
        // Builds the local window
        PointCloud<PointT>::Ptr local_window = this->buildLocalMap();
        ROS_WARN("[mapping_node.cpp] Publishing the local window");

        // Publish local window
        sensor_msgs::PointCloud2 local_window_msg;
        pcl::toROSMsg(*local_window, local_window_msg);
        local_window_msg.header.frame_id = "velodyne";
        local_window_msg.header.stamp = _timestamp_curr;
        local_window_msg.header.seq = _total_scans_received;
        _publisher_local_window.publish(local_window_msg);
    }

};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "lrm_mapping_node");
    MappingNode node;
    node.spin();
    return 0;
}