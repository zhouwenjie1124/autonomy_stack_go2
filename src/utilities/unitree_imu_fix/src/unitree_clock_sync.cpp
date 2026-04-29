#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <mutex>

// Re-stamps both the LowState IMU and the UTLidar point cloud with the local
// ROS system clock so that downstream nodes (e.g. LI-Init) see consistent
// timestamps across the two sensors.
//
// Subscriptions  → Publications
//   /lowstate          → /lowstate/imu        (system clock stamp, strictly monotonic)
//   /utlidar/cloud     → /utlidar/cloud_synced (system clock stamp)
//
// Uses a MultiThreadedExecutor so that the heavy PointCloud2 copy does not
// block the IMU callback and cause timestamp bursts.

class UnitreeClockSync : public rclcpp::Node
{
public:
  UnitreeClockSync() : Node("unitree_clock_sync"), last_imu_stamp_(0, 0, RCL_ROS_TIME)
  {
    this->declare_parameter<std::string>("lowstate_topic",  "/lowstate");
    this->declare_parameter<std::string>("imu_out_topic",   "/lowstate/imu");
    this->declare_parameter<std::string>("cloud_in_topic",  "/utlidar/cloud");
    this->declare_parameter<std::string>("cloud_out_topic", "/utlidar/cloud_synced");
    this->declare_parameter<std::string>("imu_frame_id",    "body");

    std::string ls_topic  = this->get_parameter("lowstate_topic").as_string();
    std::string imu_out   = this->get_parameter("imu_out_topic").as_string();
    std::string cloud_in  = this->get_parameter("cloud_in_topic").as_string();
    std::string cloud_out = this->get_parameter("cloud_out_topic").as_string();
    imu_frame_id_         = this->get_parameter("imu_frame_id").as_string();

    // Separate callback groups so cloud processing never delays IMU
    imu_cb_group_   = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cloud_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions imu_opts;
    imu_opts.callback_group = imu_cb_group_;

    rclcpp::SubscriptionOptions cloud_opts;
    cloud_opts.callback_group = cloud_cb_group_;

    // LowState → /lowstate/imu
    lowstate_sub_ = this->create_subscription<unitree_go::msg::LowState>(
      ls_topic, rclcpp::QoS(50).reliable(),
      [this](const unitree_go::msg::LowState::SharedPtr msg) {
        sensor_msgs::msg::Imu imu;

        // Monotonic guard: ensure stamp never goes backwards
        {
          std::lock_guard<std::mutex> lock(stamp_mutex_);
          rclcpp::Time now = this->get_clock()->now();
          if (now <= last_imu_stamp_) {
            now = last_imu_stamp_ + rclcpp::Duration(0, 1000);  // +1 µs
          }
          last_imu_stamp_   = now;
          imu.header.stamp  = now;
        }
        imu.header.frame_id = imu_frame_id_;

        // quaternion[0]=w, [1]=x, [2]=y, [3]=z
        imu.orientation.w = msg->imu_state.quaternion[0];
        imu.orientation.x = msg->imu_state.quaternion[1];
        imu.orientation.y = msg->imu_state.quaternion[2];
        imu.orientation.z = msg->imu_state.quaternion[3];

        imu.angular_velocity.x = msg->imu_state.gyroscope[0];
        imu.angular_velocity.y = msg->imu_state.gyroscope[1];
        imu.angular_velocity.z = msg->imu_state.gyroscope[2];

        imu.linear_acceleration.x = msg->imu_state.accelerometer[0];
        imu.linear_acceleration.y = msg->imu_state.accelerometer[1];
        imu.linear_acceleration.z = msg->imu_state.accelerometer[2];

        imu.orientation_covariance[0]         = -1.0;
        imu.angular_velocity_covariance[0]    = -1.0;
        imu.linear_acceleration_covariance[0] = -1.0;

        imu_pub_->publish(imu);
      },
      imu_opts);

    // /utlidar/cloud → /utlidar/cloud_synced
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_in, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        sensor_msgs::msg::PointCloud2 out = *msg;
        out.header.stamp = this->get_clock()->now();
        cloud_pub_->publish(out);
      },
      cloud_opts);

    imu_pub_   = this->create_publisher<sensor_msgs::msg::Imu>(imu_out, rclcpp::QoS(50));
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_out, rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(),
      "unitree_clock_sync: %s -> %s | %s -> %s",
      ls_topic.c_str(), imu_out.c_str(), cloud_in.c_str(), cloud_out.c_str());
  }

private:
  std::string imu_frame_id_;
  rclcpp::Time last_imu_stamp_;
  std::mutex   stamp_mutex_;
  rclcpp::CallbackGroup::SharedPtr imu_cb_group_;
  rclcpp::CallbackGroup::SharedPtr cloud_cb_group_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr     lowstate_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr            imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    cloud_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UnitreeClockSync>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
