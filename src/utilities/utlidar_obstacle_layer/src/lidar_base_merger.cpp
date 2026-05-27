#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <mutex>

// Merges MID360 (/livox/lidar) and UTLidar (/utlidar/cloud) into a single
// cloud expressed in base_link frame. Publishing in base_link means the
// onboard_detector can subscribe directly with body_to_lidar = identity.
//
// Transform chain:
//   /livox/lidar  (mid360 frame)
//     → base_link  via inv(T_mid360←base):  pitch=+0.226893, t=[0.16143, 0, 0.12263]
//   /utlidar/cloud (radar frame)
//     → base_link  via Go2W L2 extrinsics:   xyz=[0.28216, 0, 0]  rpy=[-2.92072, -0.141324, -1.01053]
//
// Publication: /merged_lidar_base (frame_id = base_link)

// base_link → radar (UTLidar) — Go2W L2 extrinsics
static constexpr double kRadarTx    =  0.28216;
static constexpr double kRadarTy    =  0.0;
static constexpr double kRadarTz    =  0.0;
static constexpr double kRadarRoll  = -2.92072;
static constexpr double kRadarPitch = -0.141324;
static constexpr double kRadarYaw   = -1.01053;

// mid360 → base_link — inverse of URDF mid360_to_base_link joint
// (T_base←mid360 = inv(T_mid360←base), pitch=+0.226893, t=[0.16143, 0, 0.12263])
static constexpr double kMid360Tx    =  0.16143;
static constexpr double kMid360Ty    =  0.0;
static constexpr double kMid360Tz    =  0.12263;
static constexpr double kMid360Pitch =  0.226893;  // +pitch = inverse of -0.226893

// Body self-filter box in base_link frame (tune to match robot body geometry)
static constexpr double kBodyXMin = -0.7;
static constexpr double kBodyXMax =  0.4;
static constexpr double kBodyYMin = -0.4;
static constexpr double kBodyYMax =  0.4;
static constexpr double kBodyZMin = -0.6;
static constexpr double kBodyZMax =  0.1;

static std::mutex utlidar_mutex;
static sensor_msgs::msg::PointCloud2::SharedPtr latest_utlidar;

// Precomputed constant transforms (set in main)
static Eigen::Isometry3d T_base_radar  = Eigen::Isometry3d::Identity(); // radar → base_link
static Eigen::Isometry3d T_base_mid360 = Eigen::Isometry3d::Identity(); // mid360 → base_link

static void transformAndFilter(
    const pcl::PointCloud<pcl::PointXYZI> & in,
    const Eigen::Isometry3d & T,
    pcl::PointCloud<pcl::PointXYZI> & out)
{
    for (const auto & pt : in.points) {
        Eigen::Vector3d p = T * Eigen::Vector3d(pt.x, pt.y, pt.z);
        if (p.x() > kBodyXMin && p.x() < kBodyXMax &&
            p.y() > kBodyYMin && p.y() < kBodyYMax &&
            p.z() > kBodyZMin && p.z() < kBodyZMax) {
            continue;
        }
        pcl::PointXYZI o;
        o.x = static_cast<float>(p.x());
        o.y = static_cast<float>(p.y());
        o.z = static_cast<float>(p.z());
        o.intensity = pt.intensity;
        out.points.push_back(o);
    }
}

void utlidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(utlidar_mutex);
    latest_utlidar = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
}

void mid360Callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub)
{
    // MID360 cloud → base_link
    pcl::PointCloud<pcl::PointXYZI> mid360_raw;
    pcl::fromROSMsg(*msg, mid360_raw);

    pcl::PointCloud<pcl::PointXYZI> merged;
    merged.reserve(mid360_raw.points.size());
    transformAndFilter(mid360_raw, T_base_mid360, merged);

    // UTLidar cloud → base_link (use latest cached scan)
    sensor_msgs::msg::PointCloud2::SharedPtr utlidar_msg;
    {
        std::lock_guard<std::mutex> lock(utlidar_mutex);
        utlidar_msg = latest_utlidar;
    }
    if (utlidar_msg) {
        pcl::PointCloud<pcl::PointXYZI> utlidar_raw;
        pcl::fromROSMsg(*utlidar_msg, utlidar_raw);
        transformAndFilter(utlidar_raw, T_base_radar, merged);
    }

    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(merged, out_msg);
    out_msg.header.stamp    = msg->header.stamp;
    out_msg.header.frame_id = "base_link";
    pub->publish(out_msg);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("lidar_base_merger");

    // T_base_radar: radar → base_link (Go2W L2 extrinsics, full RPY)
    Eigen::Quaterniond q_radar =
        Eigen::AngleAxisd(kRadarYaw,   Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(kRadarPitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(kRadarRoll,  Eigen::Vector3d::UnitX());
    T_base_radar.translate(Eigen::Vector3d(kRadarTx, kRadarTy, kRadarTz));
    T_base_radar.rotate(q_radar);

    // T_base_mid360: mid360 → base_link (inverse of URDF mid360_to_base_link)
    T_base_mid360.translate(Eigen::Vector3d(kMid360Tx, kMid360Ty, kMid360Tz));
    T_base_mid360.rotate(Eigen::AngleAxisd(kMid360Pitch, Eigen::Vector3d::UnitY()));

    auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_lidar_base", 5);

    auto sub_utlidar = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/utlidar/cloud", 5, utlidarCallback);

    auto sub_mid360 = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", 5,
        [pub](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            mid360Callback(msg, pub);
        });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
