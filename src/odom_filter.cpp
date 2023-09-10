#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

// Global variables for storing the previous odometry message
std::shared_ptr<nav_msgs::Odometry> prev_odom(new nav_msgs::Odometry);
std::shared_ptr<nav_msgs::Odometry> filtered_odom(new nav_msgs::Odometry);

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Apply low-pass filter
    double alpha = 0.9;  // Filter coefficient (between 0 and 1)

    filtered_odom->pose.pose.position.x = alpha * prev_odom->pose.pose.position.x + (1 - alpha) * msg->pose.pose.position.x;
    filtered_odom->pose.pose.position.y = alpha * prev_odom->pose.pose.position.y + (1 - alpha) * msg->pose.pose.position.y;
    filtered_odom->pose.pose.position.z = alpha * prev_odom->pose.pose.position.z + (1 - alpha) * msg->pose.pose.position.z;

    // Store current odometry for next iteration
    *prev_odom = *filtered_odom;

    // Publish tf
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(filtered_odom->pose.pose.position.x, filtered_odom->pose.pose.position.y, filtered_odom->pose.pose.position.z));
    tf::Quaternion q(
        filtered_odom->pose.pose.orientation.x,
        filtered_odom->pose.pose.orientation.y,
        filtered_odom->pose.pose.orientation.z,
        filtered_odom->pose.pose.orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry_filter_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("odom", 10, odometryCallback); 
    ros::spin();
    return 0;
}