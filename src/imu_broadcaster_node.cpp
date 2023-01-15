#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>

class IMUBroadcasterNode {

private:
    ros::Subscriber sub_imu;
    ros::Publisher pub_imu;
    ros::Publisher pub_diagnostics_euler_orientation_msg;
    std::string frame_id;

    void imuCallback(const sensor_msgs::Imu::ConstPtr & msg) {
        // Get RPY from message
        geometry_msgs::Quaternion q_in_msg = msg->orientation;
        Eigen::Quaterniond q_in(
            q_in_msg.w,
            q_in_msg.x, 
            q_in_msg.y, 
            q_in_msg.z
        );
        
        Eigen::Matrix3d R(q_in);
        Eigen::Vector3d r_vec_in = R.eulerAngles(0,1,2);
        double roll = r_vec_in(0);
        double pitch = r_vec_in(1);
        double heading = -r_vec_in(2); 

        // Process yaw: heading convetion
        double yaw = M_PI_2 - heading;
        
        // Convert back to quaternion - z-last and x-first convention
        Eigen::AngleAxisd roll_angle( roll, Eigen::Vector3d::UnitX() );
        Eigen::AngleAxisd pitch_angle( pitch, Eigen::Vector3d::UnitY() );
        Eigen::AngleAxisd yaw_angle( yaw, Eigen::Vector3d::UnitZ() );
        Eigen::Quaterniond q_out = roll_angle * pitch_angle * yaw_angle;

        // Display
        double roll_comp = q_out.toRotationMatrix().eulerAngles(0,1,2)(0);
        double pitch_comp = q_out.toRotationMatrix().eulerAngles(0,1,2)(1);
        double yaw_comp = q_out.toRotationMatrix().eulerAngles(0,1,2)(2);
        geometry_msgs::Point diagnostics_euler_orientation_msg;
        diagnostics_euler_orientation_msg.x = roll_comp * (180 / M_PI);
        diagnostics_euler_orientation_msg.y = pitch_comp * (180 / M_PI);
        diagnostics_euler_orientation_msg.z = yaw_comp * (180 / M_PI);
        this->pub_diagnostics_euler_orientation_msg.publish(diagnostics_euler_orientation_msg);
       
        // Prepare output message
        geometry_msgs::Quaternion q_out_msg;
        q_out_msg.x = q_out.x();
        q_out_msg.y = q_out.y();
        q_out_msg.z = q_out.z();
        q_out_msg.w = q_out.w();

        sensor_msgs::Imu imu_out;
        imu_out.header = msg->header;
        imu_out.header.frame_id = this->frame_id;
        imu_out.orientation = q_out_msg;
        imu_out.orientation_covariance = msg->orientation_covariance;
        imu_out.angular_velocity = msg->angular_velocity;
        imu_out.angular_velocity_covariance = msg->angular_velocity_covariance;
        imu_out.linear_acceleration = msg->linear_acceleration;
        imu_out.linear_acceleration_covariance = msg->linear_acceleration_covariance;

        this->pub_imu.publish(imu_out);
        return ;
    }

public:

    IMUBroadcasterNode(ros::NodeHandle &nh) {
        std::string rcv_imu_topic = "/carina/sensor/imu";
        this->frame_id = "/imu";
        this->sub_imu = nh.subscribe(rcv_imu_topic, 100, &IMUBroadcasterNode::imuCallback, this );
        this->pub_imu = nh.advertise<sensor_msgs::Imu>("/carina/sensor/imu_corrected", 100, false);
        this->pub_diagnostics_euler_orientation_msg = nh.advertise<geometry_msgs::Point>("/diagnostics/carina/sensors/imu/orientation_euler_xyz_deg", 100, false);
        return ;
    }

};

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "imu_broadcaster_node");
    ros::NodeHandle n;
    IMUBroadcasterNode node(n);
    ros::spin();
    return 0;
}