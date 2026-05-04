#ifndef MAP_TOOLS_PANEL_H
#define MAP_TOOLS_PANEL_H

#ifndef Q_MOC_RUN
# include <rclcpp/rclcpp.hpp>
# include <rviz_common/panel.hpp>
#endif

#include <QPushButton>
#include <QVBoxLayout>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float32.hpp>

namespace map_tools_rviz_plugin
{

class MapToolsPanel : public rviz_common::Panel
{
Q_OBJECT
public:
  MapToolsPanel(QWidget* parent = 0);

protected Q_SLOTS:
  void clearGoalPoint();
  void clearTerrainMap();

private:
  void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom);

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr way_point_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr map_clearing_pub_;

  QPushButton* clear_goal_button_;
  QPushButton* clear_terrain_button_;

  float vehicle_x_;
  float vehicle_y_;
  float vehicle_z_;
};

}  // namespace map_tools_rviz_plugin

#endif  // MAP_TOOLS_PANEL_H
