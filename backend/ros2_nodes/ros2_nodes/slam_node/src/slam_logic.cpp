#include <Eigen/Dense>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

class SlamNode : public rclcpp::Node {
public:
    SlamNode() : Node("slam_node") {
        // --- ROS2 Publishers and Subscribers ---
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("imu/data", 10, std::bind(&SlamNode::imu_callback, this, std::placeholders::_1));
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&SlamNode::scan_callback, this, std::placeholders::_1));
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // --- EKF State Initialization ---
        state_ = Eigen::Vector3d::Zero();
        covariance_ = Eigen::Matrix3d::Identity() * 0.1;
        last_time_ = this->get_clock()->now();

        // --- Map Initialization ---
        map_resolution_ = 0.1; // 10cm per cell
        map_width_ = 200;      // 20 meters
        map_height_ = 200;     // 20 meters
        map_.info.resolution = map_resolution_;
        map_.info.width = map_width_;
        map_.info.height = map_height_;
        map_.info.origin.position.x = - (map_width_ * map_resolution_) / 2.0;
        map_.info.origin.position.y = - (map_height_ * map_resolution_) / 2.0;
        map_.data.assign(map_width_ * map_height_, -1); // -1 for unknown

        RCLCPP_INFO(this->get_logger(), "Advanced SLAM Node has started.");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // --- EKF PREDICTION STEP ---
        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0) return;
        last_time_ = current_time;

        double angular_velocity_z = msg->angular_velocity.z;
        double velocity = 1.0; 

        double x = state_(0);
        double y = state_(1);
        double theta = state_(2);

        state_(0) = x + velocity * cos(theta) * dt;
        state_(1) = y + velocity * sin(theta) * dt;
        state_(2) = theta + angular_velocity_z * dt;

        // Publish odometry and transform
        publish_odometry();
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // --- MAPPING STEP ---
        double vehicle_x = state_(0);
        double vehicle_y = state_(1);
        double vehicle_theta = state_(2);

        // Iterate through each laser beam in the scan
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float range = msg->ranges[i];
            if (range < msg->range_min || range > msg->range_max) continue;

            double angle = msg->angle_min + i * msg->angle_increment;
            // Calculate the world coordinates of the obstacle hit by the laser
            double obstacle_x = vehicle_x + range * cos(vehicle_theta + angle);
            double obstacle_y = vehicle_y + range * sin(vehicle_theta + angle);
            
            // Convert world coordinates to map coordinates
            int map_x = (obstacle_x - map_.info.origin.position.x) / map_resolution_;
            int map_y = (obstacle_y - map_.info.origin.position.y) / map_resolution_;

            if (map_x >= 0 && map_x < map_width_ && map_y >= 0 && map_y < map_height_) {
                int index = map_y * map_width_ + map_x;
                map_.data[index] = 100; // 100 for occupied
            }
        }
        
        // Publish the updated map
        map_.header.stamp = this->get_clock()->now();
        map_.header.frame_id = "map";
        map_publisher_->publish(map_);
    }

    void publish_odometry() {
        // Create Odometry message
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = state_(0);
        odom_msg.pose.pose.position.y = state_(1);
        
        tf2::Quaternion q;
        q.setRPY(0, 0, state_(2));
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        
        odom_publisher_->publish(odom_msg);

        // Create and broadcast the transform
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        t.transform.translation.x = state_(0);
        t.transform.translation.y = state_(1);
        t.transform.rotation = odom_msg.pose.pose.orientation;
        tf_broadcaster_->sendTransform(t);
    }

    // --- Member Variables ---
    rclcpp::Time last_time_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    Eigen::Vector3d state_;
    Eigen::Matrix3d covariance_;
    
    nav_msgs::msg::OccupancyGrid map_;
    double map_resolution_;
    int map_width_, map_height_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamNode>());
    rclcpp::shutdown();
    return 0;
}
