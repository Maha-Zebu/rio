#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class SyncNode {
public:
  SyncNode() {
    // Initialize ROS node handle
    nh_ = ros::NodeHandle("~");

    // Subscribe to IMU and PointCloud topics
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_(nh_, "/imu", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_(nh_, "/ti_mmwave/radar_scan_pcl_0", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imu_sub_, pc_sub_);
    // Synchronize IMU and PointCloud messages using ApproxTimeSync
    // sync_ = std::make_shared<Sync>(SyncPolicy(10), imu_sub_, pc_sub_);

    sync.registerCallback(boost::bind(&SyncNode::callback, this, _1, _2));

    // Advertise the synchronized topic
    synced_radar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ti_mmwave/radar_scan_pcl_synced", 1);
    synced_imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/synced", 1);
    synced_trigger_pub_ = nh_.advertise<std_msgs::Header>("/ti_mmwave/radar_center/trigger", 1);
  }

  // Callback function for synchronized messages
  void callback(const sensor_msgs::ImuConstPtr& imu_msg, const sensor_msgs::PointCloud2ConstPtr& pc_msg) {
    // Process synchronized messages here
    // ...
    sensor_msgs::Imu imu_msg_copy = *imu_msg;
    // Republish the synchronized messages
    imu_msg_copy.header.stamp = pc_msg->header.stamp;
    // Set synced_msg fields based on imu_msg and pc_msg
    // ...
    std_msgs::Header header = imu_msg_copy.header;

    header.frame_id = "PB3";

    synced_radar_pub_.publish(pc_msg);
    synced_imu_pub_.publish(imu_msg);
    synced_trigger_pub_.publish(header);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher synced_radar_pub_, synced_imu_pub_, synced_trigger_pub_;
};

int main(int argc, char** argv) {
  // Initialize ROS node
  ros::init(argc, argv, "sync_node");

  // Create SyncNode object
  SyncNode sync_node;

  // Spin the node
  ros::spin();

  return 0;
}
