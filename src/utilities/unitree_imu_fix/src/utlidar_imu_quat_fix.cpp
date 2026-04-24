#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

// The Unitree lidar driver incorrectly fills wxyz into ROS's xyzw quaternion fields.
// Driver layout:  orientation.x = w,  .y = x,  .z = y,  .w = z
// Correct layout: orientation.x = x,  .y = y,  .z = z,  .w = w
// Fix: correct.x = raw.y, correct.y = raw.z, correct.z = raw.w, correct.w = raw.x

class ImuQuatFix : public rclcpp::Node
{
public:
  ImuQuatFix() : Node("imu_quat_fix")
  {
    this->declare_parameter<std::string>("input_topic",  "/utlidar/imu");
    this->declare_parameter<std::string>("output_topic", "/utlidar/imu_corrected");

    std::string in  = this->get_parameter("input_topic").as_string();
    std::string out = this->get_parameter("output_topic").as_string();

    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      in, rclcpp::QoS(300).best_effort(),
      [this](sensor_msgs::msg::Imu::UniquePtr msg) {
        const double w = msg->orientation.x;  // driver put w here
        const double x = msg->orientation.y;  // driver put x here
        const double y = msg->orientation.z;  // driver put y here
        const double z = msg->orientation.w;  // driver put z here

        msg->orientation.x = x;
        msg->orientation.y = y;
        msg->orientation.z = z;
        msg->orientation.w = w;

        pub_->publish(std::move(msg));
      });

    pub_ = this->create_publisher<sensor_msgs::msg::Imu>(out, rclcpp::QoS(300));

    RCLCPP_INFO(this->get_logger(), "imu_quat_fix: %s -> %s", in.c_str(), out.c_str());
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr    pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuQuatFix>());
  rclcpp::shutdown();
  return 0;
}
