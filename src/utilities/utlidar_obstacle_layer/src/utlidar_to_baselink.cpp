#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

// URDF radar_joint: parent=base_link, child=radar
//   xyz="0.28945 0 -0.046825"  rpy="0 2.8782 0"
// We publish this same transform as base_link → laser_link so that RViz
// (and any TF consumer) can resolve laser_link automatically.
static constexpr double kTx    =  0.28945;
static constexpr double kTy    =  0.0;
static constexpr double kTz    = -0.046825;
static constexpr double kPitch =  2.8782;   // Ry (rad)

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
    // Ry(pitch) quaternion: q = (0, sin(p/2), 0, cos(p/2))
    double half = kPitch / 2.0;
    double qy = std::sin(half);
    double qw = std::cos(half);

    // base_link → laser_link  (radar joint from URDF)
    geometry_msgs::msg::TransformStamped tf_laser;
    tf_laser.header.stamp    = now();
    tf_laser.header.frame_id = "base_link";
    tf_laser.child_frame_id  = "laser_link";
    tf_laser.transform.translation.x = kTx;
    tf_laser.transform.translation.y = kTy;
    tf_laser.transform.translation.z = kTz;
    tf_laser.transform.rotation.x = 0.0;
    tf_laser.transform.rotation.y = qy;
    tf_laser.transform.rotation.z = 0.0;
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
