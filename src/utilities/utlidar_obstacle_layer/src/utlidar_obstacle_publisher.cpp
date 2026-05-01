#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <mutex>

// Transforms Unitree L1 UTLidar point cloud (body frame, from transform_sensors)
// into map frame using Point-LIO odometry, then filters by range and height
// before publishing to /added_obstacles for local_planner.
//
// Subscriptions:
//   /utlidar/transformed_cloud  - L1 cloud in robot body frame
//   /state_estimation           - Point-LIO odometry (MID360/sensor pose in map)
//
// Publications:
//   /added_obstacles            - filtered obstacle cloud in map frame

static std::mutex odom_mutex;
static Eigen::Vector3d sensor_pos(0, 0, 0);
static Eigen::Quaterniond sensor_rot(1, 0, 0, 0);
static bool odom_received = false;

static double obstacle_range_max;
static double obstacle_range_min;
static double obstacle_height_min;
static double obstacle_height_max;

// MID360→body translation (measured extrinsic, default identity)
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

void cloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub)
{
  if (!odom_received) return;

  Eigen::Vector3d pos;
  Eigen::Quaterniond rot;
  {
    std::lock_guard<std::mutex> lock(odom_mutex);
    pos = sensor_pos;
    rot = sensor_rot;
  }

  // T_sensor_map: MID360 sensor pose in map frame (from Point-LIO odometry)
  Eigen::Isometry3d T_sensor_map = Eigen::Isometry3d::Identity();
  T_sensor_map.translate(pos);
  T_sensor_map.rotate(rot);

  // T_body_sensor: body frame relative to MID360 sensor frame (extrinsic offset)
  Eigen::Isometry3d T_body_sensor = Eigen::Isometry3d::Identity();
  T_body_sensor.translate(Eigen::Vector3d(mid360_to_body_tx, mid360_to_body_ty, mid360_to_body_tz));

  // T_body_map = T_sensor_map * T_body_sensor
  Eigen::Isometry3d T_body_map = T_sensor_map * T_body_sensor;

  pcl::PointCloud<pcl::PointXYZI> cloud_body;
  pcl::fromROSMsg(*cloud_msg, cloud_body);

  pcl::PointCloud<pcl::PointXYZI> cloud_out;
  cloud_out.reserve(cloud_body.points.size());

  double robot_z = T_body_map.translation().z();

  for (const auto& pt : cloud_body.points)
  {
    Eigen::Vector3d p_body(pt.x, pt.y, pt.z);
    Eigen::Vector3d p_map = T_body_map * p_body;

    // Range filter (distance from robot in map frame)
    double dx = p_map.x() - T_body_map.translation().x();
    double dy = p_map.y() - T_body_map.translation().y();
    double dz = p_map.z() - T_body_map.translation().z();
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist < obstacle_range_min || dist > obstacle_range_max) continue;

    // Height filter (relative to robot z)
    double rel_z = p_map.z() - robot_z;
    if (rel_z < obstacle_height_min || rel_z > obstacle_height_max) continue;

    pcl::PointXYZI out_pt;
    out_pt.x = static_cast<float>(p_map.x());
    out_pt.y = static_cast<float>(p_map.y());
    out_pt.z = static_cast<float>(p_map.z());
    out_pt.intensity = pt.intensity;
    cloud_out.points.push_back(out_pt);
  }

  sensor_msgs::msg::PointCloud2 out_msg;
  pcl::toROSMsg(cloud_out, out_msg);
  out_msg.header.stamp = cloud_msg->header.stamp;
  out_msg.header.frame_id = "map";
  pub->publish(out_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("utlidar_obstacle_publisher");

  node->declare_parameter<double>("obstacle_range_max", 3.0);
  node->declare_parameter<double>("obstacle_range_min", 0.3);
  node->declare_parameter<double>("obstacle_height_min", 0.1);
  node->declare_parameter<double>("obstacle_height_max", 1.5);
  node->declare_parameter<double>("mid360_to_body_tx", 0.0);
  node->declare_parameter<double>("mid360_to_body_ty", 0.0);
  node->declare_parameter<double>("mid360_to_body_tz", 0.0);

  node->get_parameter("obstacle_range_max", obstacle_range_max);
  node->get_parameter("obstacle_range_min", obstacle_range_min);
  node->get_parameter("obstacle_height_min", obstacle_height_min);
  node->get_parameter("obstacle_height_max", obstacle_height_max);
  node->get_parameter("mid360_to_body_tx", mid360_to_body_tx);
  node->get_parameter("mid360_to_body_ty", mid360_to_body_ty);
  node->get_parameter("mid360_to_body_tz", mid360_to_body_tz);

  auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/added_obstacles", 5);

  auto sub_odom = node->create_subscription<nav_msgs::msg::Odometry>(
    "/state_estimation", 5, odometryCallback);

  auto sub_cloud = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/utlidar/transformed_cloud", 5,
    [pub](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      cloudCallback(msg, pub);
    });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
