#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <unitree_go/msg/low_state.hpp>

// Converts unitree_go/LowState imu_state to standard sensor_msgs/Imu.
// Source: /lowstate (500 Hz high-frequency channel).
// Unitree quaternion order: [w, x, y, z] → mapped to ROS orientation fields.

class LowstateImuFix : public rclcpp::Node
{
public:
  LowstateImuFix() : Node("lowstate_imu_fix")
  {
    this->declare_parameter<std::string>("input_topic",  "/lowstate");
    this->declare_parameter<std::string>("output_topic", "/lowstate/imu");
    this->declare_parameter<std::string>("frame_id",     "body");

    std::string in       = this->get_parameter("input_topic").as_string();
    std::string out      = this->get_parameter("output_topic").as_string();
    frame_id_            = this->get_parameter("frame_id").as_string();

    sub_ = this->create_subscription<unitree_go::msg::LowState>(
      in, rclcpp::QoS(50).reliable(),
      [this](const unitree_go::msg::LowState::SharedPtr msg) {
        sensor_msgs::msg::Imu imu;
        imu.header.stamp    = this->get_clock()->now();
        imu.header.frame_id = frame_id_;

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

        // -1 indicates covariance unknown
        imu.orientation_covariance[0]         = -1.0;
        imu.angular_velocity_covariance[0]    = -1.0;
        imu.linear_acceleration_covariance[0] = -1.0;

        pub_->publish(imu);
      });

    pub_ = this->create_publisher<sensor_msgs::msg::Imu>(out, rclcpp::QoS(50));

    RCLCPP_INFO(this->get_logger(), "lowstate_imu_fix: %s -> %s", in.c_str(), out.c_str());
  }

private:
  std::string frame_id_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr        pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LowstateImuFix>());
  rclcpp::shutdown();
  return 0;
}
