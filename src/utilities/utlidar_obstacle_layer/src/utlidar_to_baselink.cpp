#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>


//   xyz="0.28945 0 -0.046825"  rpy="0 2.8782 0"
static constexpr double kTx    =  0.28216;
static constexpr double kTy    =  0.0;
static constexpr double kTz    =  0.0;
static constexpr double kRoll  = -2.92072;   // Rx (rad)
static constexpr double kPitch = -0.141324;  // Ry (rad)
static constexpr double kYaw   = -1.01053;   // Rz (rad)

class UtlidarToBaselink : public rclcpp::Node
{
public:
  UtlidarToBaselink()
  : Node("utlidar_to_baselink"),
    tf_broadcaster_(this)
  {
    publish_static_tf();

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/utlidar/cloud_laser", 5);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/utlidar/cloud", 5,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { callback(msg); });

    RCLCPP_INFO(get_logger(),
      "Publishing static TF: base_link → laser_link");
    RCLCPP_INFO(get_logger(),
      "Republishing /utlidar/cloud as /utlidar/cloud_laser (frame_id=laser_link)");
  }

private:
  void publish_static_tf()
  {
    // RPY → quaternion (intrinsic ZYX / extrinsic XYZ convention used by ROS)
    double cr = std::cos(kRoll  / 2.0), sr = std::sin(kRoll  / 2.0);
    double cp = std::cos(kPitch / 2.0), sp = std::sin(kPitch / 2.0);
    double cy = std::cos(kYaw   / 2.0), sy = std::sin(kYaw   / 2.0);
    double qw = cr*cp*cy + sr*sp*sy;
    double qx = sr*cp*cy - cr*sp*sy;
    double qy = cr*sp*cy + sr*cp*sy;
    double qz = cr*cp*sy - sr*sp*cy;

    // base_link → laser_link
    geometry_msgs::msg::TransformStamped tf_laser;
    tf_laser.header.stamp    = now();
    tf_laser.header.frame_id = "base_link";
    tf_laser.child_frame_id  = "laser_link";
    tf_laser.transform.translation.x = kTx;
    tf_laser.transform.translation.y = kTy;
    tf_laser.transform.translation.z = kTz;
    tf_laser.transform.rotation.x = qx;
    tf_laser.transform.rotation.y = qy;
    tf_laser.transform.rotation.z = qz;
    tf_laser.transform.rotation.w = qw;

    // laser_link → utlidar_lidar  (identity: same physical frame, different SDK name)
    geometry_msgs::msg::TransformStamped tf_alias;
    tf_alias.header.stamp    = now();
    tf_alias.header.frame_id = "laser_link";
    tf_alias.child_frame_id  = "utlidar_lidar";
    tf_alias.transform.rotation.w = 1.0;   // identity quaternion

    tf_broadcaster_.sendTransform({tf_laser, tf_alias});
  }

  void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 out = *msg;
    out.header.frame_id = "laser_link";
    pub_->publish(out);
  }

  tf2_ros::StaticTransformBroadcaster tf_broadcaster_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UtlidarToBaselink>());
  rclcpp::shutdown();
  return 0;
}
