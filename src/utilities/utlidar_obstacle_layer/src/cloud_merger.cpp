#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <mutex>

// Merges MID360 registered scan (map frame) with UTLidar body-frame cloud
// to produce a denser terrain map input.
//
// Subscriptions:
//   /registered_scan            - MID360 registered cloud in map frame (from Point-LIO)
//   /utlidar/transformed_cloud  - UTLidar cloud in robot body frame (from transform_sensors)
//   /state_estimation           - Point-LIO odometry (MID360/sensor pose in map frame)
//
// Publications:
//   /merged_scan                - combined cloud in map frame

static std::mutex odom_mutex;
static Eigen::Vector3d sensor_pos(0, 0, 0);
static Eigen::Quaterniond sensor_rot(1, 0, 0, 0);
static bool odom_received = false;

// Stores the latest UTLidar body-frame cloud
static std::mutex utlidar_mutex;
static sensor_msgs::msg::PointCloud2::SharedPtr latest_utlidar_cloud;

static double mid360_to_body_tx;
static double mid360_to_body_ty;
static double mid360_to_body_tz;

void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex);
  sensor_pos.x() = msg->pose.pose.position.x;
  sensor_pos.y() = msg->pose.pose.position.y;
  sensor_pos.z() = msg->pose.pose.position.z;
  sensor_rot.x() = msg->pose.pose.orientation.x;
  sensor_rot.y() = msg->pose.pose.orientation.y;
  sensor_rot.z() = msg->pose.pose.orientation.z;
  sensor_rot.w() = msg->pose.pose.orientation.w;
  odom_received = true;
}

void utlidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(utlidar_mutex);
  latest_utlidar_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
}

void registeredScanCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan_msg,
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub)
{
  if (!odom_received) {
    pub->publish(*scan_msg);
    return;
  }

  // Get latest UTLidar cloud
  sensor_msgs::msg::PointCloud2::SharedPtr utlidar_msg;
  {
    std::lock_guard<std::mutex> lock(utlidar_mutex);
    utlidar_msg = latest_utlidar_cloud;
  }

  if (!utlidar_msg) {
    pub->publish(*scan_msg);
    return;
  }

  // Build body→map transform (same pattern as utlidar_obstacle_publisher)
  Eigen::Vector3d pos;
  Eigen::Quaterniond rot;
  {
    std::lock_guard<std::mutex> lock(odom_mutex);
    pos = sensor_pos;
    rot = sensor_rot;
  }

  Eigen::Isometry3d T_sensor_map = Eigen::Isometry3d::Identity();
  T_sensor_map.translate(pos);
  T_sensor_map.rotate(rot);

  Eigen::Isometry3d T_body_sensor = Eigen::Isometry3d::Identity();
  T_body_sensor.translate(Eigen::Vector3d(mid360_to_body_tx, mid360_to_body_ty, mid360_to_body_tz));

  Eigen::Isometry3d T_body_map = T_sensor_map * T_body_sensor;

  // Transform UTLidar body-frame cloud to map frame
  pcl::PointCloud<pcl::PointXYZI> utlidar_body;
  pcl::fromROSMsg(*utlidar_msg, utlidar_body);

  pcl::PointCloud<pcl::PointXYZI> utlidar_map;
  utlidar_map.reserve(utlidar_body.points.size());

  for (const auto& pt : utlidar_body.points) {
    Eigen::Vector3d p_map = T_body_map * Eigen::Vector3d(pt.x, pt.y, pt.z);
    pcl::PointXYZI out;
    out.x = static_cast<float>(p_map.x());
    out.y = static_cast<float>(p_map.y());
    out.z = static_cast<float>(p_map.z());
    out.intensity = pt.intensity;
    utlidar_map.points.push_back(out);
  }

  // Concatenate with MID360 registered scan
  pcl::PointCloud<pcl::PointXYZI> mid360_cloud;
  pcl::fromROSMsg(*scan_msg, mid360_cloud);

  mid360_cloud += utlidar_map;

  sensor_msgs::msg::PointCloud2 out_msg;
  pcl::toROSMsg(mid360_cloud, out_msg);
  out_msg.header.stamp = scan_msg->header.stamp;
  out_msg.header.frame_id = "map";
  pub->publish(out_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cloud_merger");

  node->declare_parameter<double>("mid360_to_body_tx", 0.0);
  node->declare_parameter<double>("mid360_to_body_ty", 0.0);
  node->declare_parameter<double>("mid360_to_body_tz", 0.0);

  node->get_parameter("mid360_to_body_tx", mid360_to_body_tx);
  node->get_parameter("mid360_to_body_ty", mid360_to_body_ty);
  node->get_parameter("mid360_to_body_tz", mid360_to_body_tz);

  auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_scan", 5);

  auto sub_odom = node->create_subscription<nav_msgs::msg::Odometry>(
    "/state_estimation", 5, odometryCallback);

  auto sub_utlidar = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/utlidar/transformed_cloud", 5, utlidarCallback);

  auto sub_scan = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/registered_scan", 5,
    [pub](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      registeredScanCallback(msg, pub);
    });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
