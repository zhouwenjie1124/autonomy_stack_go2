#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <mutex>

// Merges MID360 registered scan (map frame) with raw UTLidar cloud
// using URDF-derived transforms directly — no transform_sensors node needed.
//
// Transform chain:
//   utlidar/cloud (radar frame)
//     → base_link  via Go2W L2 extrinsics:   xyz=[0.28216, 0, 0]  rpy=[-2.92072, -0.141324, -1.01053]
//     → mid360     via mid360_to_base_link:  xyz=[-0.12971, 0, -0.15579]  rpy=[0, -0.226893, 0]
//     → map        via SLAM odometry (/state_estimation)
//
// Subscriptions:  /registered_scan  /utlidar/cloud  /state_estimation
// Publication:    /merged_scan

// base_link → radar (UTLidar) — Go2W L2 extrinsics
static constexpr double kRadarTx    =  0.28216;
static constexpr double kRadarTy    =  0.0;
static constexpr double kRadarTz    =  0.0;
static constexpr double kRadarRoll  = -2.92072;   // Rx (rad)
static constexpr double kRadarPitch = -0.141324;  // Ry (rad)
static constexpr double kRadarYaw   = -1.01053;   // Rz (rad)

// URDF mid360_to_base_link joint: mid360 → base_link
static constexpr double kMid360Tx    = -0.12971;
static constexpr double kMid360Ty    =  0.0;
static constexpr double kMid360Tz    = -0.15579;
static constexpr double kMid360Pitch = -0.226893; // Ry(rad), from rpy="0 -0.226893 0"

static std::mutex odom_mutex;
static Eigen::Vector3d sensor_pos(0, 0, 0);
static Eigen::Quaterniond sensor_rot(1, 0, 0, 0);
static bool odom_received = false;

static std::mutex utlidar_mutex;
static sensor_msgs::msg::PointCloud2::SharedPtr latest_utlidar_cloud;

// Precomputed constant: T(mid360 ← radar) = T_mid360_base × T_base_radar
static Eigen::Isometry3d T_mid360_radar = Eigen::Isometry3d::Identity();

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

  sensor_msgs::msg::PointCloud2::SharedPtr utlidar_msg;
  {
    std::lock_guard<std::mutex> lock(utlidar_mutex);
    utlidar_msg = latest_utlidar_cloud;
  }
  if (!utlidar_msg) {
    pub->publish(*scan_msg);
    return;
  }

  // T_map_mid360 from SLAM odometry
  Eigen::Vector3d pos;
  Eigen::Quaterniond rot;
  {
    std::lock_guard<std::mutex> lock(odom_mutex);
    pos = sensor_pos;
    rot = sensor_rot;
  }
  Eigen::Isometry3d T_map_mid360 = Eigen::Isometry3d::Identity();
  T_map_mid360.translate(pos);
  T_map_mid360.rotate(rot);

  // Full chain: radar → mid360 → map
  Eigen::Isometry3d T_map_radar = T_map_mid360 * T_mid360_radar;

  // Transform raw UTLidar cloud to map frame
  pcl::PointCloud<pcl::PointXYZI> utlidar_raw;
  pcl::fromROSMsg(*utlidar_msg, utlidar_raw);

  pcl::PointCloud<pcl::PointXYZI> utlidar_map;
  utlidar_map.reserve(utlidar_raw.points.size());
  for (const auto & pt : utlidar_raw.points) {
    Eigen::Vector3d p = T_map_radar * Eigen::Vector3d(pt.x, pt.y, pt.z);
    pcl::PointXYZI out;
    out.x = static_cast<float>(p.x());
    out.y = static_cast<float>(p.y());
    out.z = static_cast<float>(p.z());
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cloud_merger");

  // Build constant transform: T(mid360 ← radar) = T_mid360_base × T_base_radar
  // T_base_radar: base_link → radar  (Go2W L2 extrinsics, full RPY)
  Eigen::Quaterniond q_radar =
    Eigen::AngleAxisd(kRadarYaw,   Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(kRadarPitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(kRadarRoll,  Eigen::Vector3d::UnitX());
  Eigen::Isometry3d T_base_radar = Eigen::Isometry3d::Identity();
  T_base_radar.translate(Eigen::Vector3d(kRadarTx, kRadarTy, kRadarTz));
  T_base_radar.rotate(q_radar);

  // T_mid360_base: mid360 → base_link  (from URDF mid360_to_base_link joint)
  Eigen::Isometry3d T_mid360_base = Eigen::Isometry3d::Identity();
  T_mid360_base.translate(Eigen::Vector3d(kMid360Tx, kMid360Ty, kMid360Tz));
  T_mid360_base.rotate(Eigen::AngleAxisd(kMid360Pitch, Eigen::Vector3d::UnitY()));

  T_mid360_radar = T_mid360_base * T_base_radar;

  auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_scan", 5);

  auto sub_odom = node->create_subscription<nav_msgs::msg::Odometry>(
    "/state_estimation", 5, odometryCallback);

  auto sub_utlidar = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/utlidar/cloud", 5, utlidarCallback);

  auto sub_scan = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/registered_scan", 5,
    [pub](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      registeredScanCallback(msg, pub);
    });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
