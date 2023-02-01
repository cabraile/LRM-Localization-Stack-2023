#include <deque>
#include <memory>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <gtsam/geometry/Pose3.h>

#include <ros/ros.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include "localization_stack/conversions.h"

// Type Definitions and namespaces
using pcl::PointCloud;
using gtsam::Pose3;
using namespace message_filters;
typedef pcl::PointXYZ PointT;
typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyT;
typedef Synchronizer<SyncPolicyT> SyncT;

class MappingNode
{

public:
    MappingNode()
    {

        // Load parameters
        string _scans_topic = "";
        string _odometry_topic = "/localization/odometry/visual";

        // Load node handler
        _node_handle_ptr = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("~"));

        // Instantiate publishers
        _publisher_local_window = _node_handle_ptr->advertise<sensor_msgs::PointCloud2>("/mapping/maps/local", 1000);
    }

    void spin()
    {
        // Wait for static transform from the base_link frame to LiDAR
        tf::TransformListener tf_listener;
        // Wait for LiDAR to base_link transform 
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
                ros::Duration(1.0).sleep();
            }
        }
        _base_link_T_lidar = lrm::fromTFTransformToGTSAMPose(tf_base_link_T_lidar);

        // Instantiate synchronized subscribers
        _subscriber_scan_ptr = std::shared_ptr<Subscriber<sensor_msgs::PointCloud2>>( new Subscriber<sensor_msgs::PointCloud2>( *_node_handle_ptr, _scans_topic, 10 ) );
        _subscriber_odometry_ptr = std::shared_ptr<Subscriber<nav_msgs::Odometry>>( new Subscriber<nav_msgs::Odometry>( *_node_handle_ptr, _odometry_topic, 10 ) );
        _message_synchronizer_ptr = std::shared_ptr<SyncT>( new SyncT(SyncPolicyT(10),*_subscriber_scan_ptr, *_subscriber_odometry_ptr) );

        // Spin
        ros::Rate rate(1);
        while(ros::ok())
        {
            // Checks if the cloud can be published
            if(_count_callbacks_after_published > _publish_every_N_callbacks)
            {
                // Builds the local window
                PointCloud<PointT>::Ptr local_window = this->buildLocalMap();

                // Publish
                sensor_msgs::PointCloud2 local_window_msg;
                pcl::toROSMsg(local_window, local_window_msg);
                _publisher_local_window.publish(local_window_msg);

                // Resets the condition counter
                _count_callbacks_after_published = 0;
            }

            ros::spinOnce();
            rate.sleep();
        }
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
    // Queue of point clouds and their origin_T_lidar transforms' pairs
    std::deque<std::pair<Pose3, PointCloud<PointT>::Ptr>> _cloud_pair_queue;

    // Maximum number of clouds for storing in the queue
    int _max_clouds_queue = 10;

    // Resolution in meters
    float _downsample_resolution = 0.1f;

    // Transform from the LiDAR to the base_link frame
    Pose3 _base_link_T_lidar;

    // 
    size_t _count_callbacks_after_published = 0;

    //
    size_t _publish_every_N_callbacks = 3;

    // 
    std::shared_ptr< ros::NodeHandle > _node_handle_ptr;


    string _scans_topic;
    string _odometry_topic;

    // Declare: local point cloud publisher
    ros::Publisher _publisher_local_window;

    // Scan subscriber
    std::shared_ptr< Subscriber<sensor_msgs::PointCloud2> > _subscriber_scan_ptr;

    // Odometry subscriber
    std::shared_ptr< Subscriber<nav_msgs::Odometry> > _subscriber_odometry_ptr;
    
    std::shared_ptr< SyncT > _message_synchronizer_ptr;

    ////////////////////////
    // Callbacks
    ////////////////////////

    void callbackSynchronizedScan(const sensor_msgs::PointCloud2::Ptr & scan_msg_ptr, const nav_msgs::Odometry::Ptr & odom_msg_ptr )
    {
        // Convert the scan msg to a pcl::PointCloud<pcl::PointXYZI>
        PointCloud<PointT>::Ptr cloud(new PointCloud<PointT>);
        pcl::fromROSMsg(*scan_msg_ptr, *cloud);

        // Downsample in a given resolution
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud (cloud);
        vg.setLeafSize (_downsample_resolution, _downsample_resolution, _downsample_resolution);
        vg.filter (*cloud);

        // Convert the odometry pose to a gtsam::Pose3
        Pose3 origin_T_base_link;
        origin_T_base_link = Pose3(
            gtsam::Rot3::Quaternion(odom_msg_ptr->pose.pose.orientation.w, odom_msg_ptr->pose.pose.orientation.x, odom_msg_ptr->pose.pose.orientation.y, odom_msg_ptr->pose.pose.orientation.z),
            gtsam::Point3(odom_msg_ptr->pose.pose.position.x, odom_msg_ptr->pose.pose.position.y, odom_msg_ptr->pose.pose.position.z)
        );

        // Transform the pose to the LiDAR frame
        Pose3 origin_T_lidar = origin_T_base_link * _base_link_T_lidar;

        // Store pair in the deque maintaining the maximum size
        if (_cloud_pair_queue.size() >= _max_clouds_queue)
        {
            _cloud_pair_queue.pop_front();
        }
        _cloud_pair_queue.push_back(std::make_pair(origin_T_lidar, cloud));

        // Increments the counter for publishing
        _count_callbacks_after_published++;
    }
    

};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "lrm_mapping_node");
    MappingNode node;
    node.spin();
    return 0;
}