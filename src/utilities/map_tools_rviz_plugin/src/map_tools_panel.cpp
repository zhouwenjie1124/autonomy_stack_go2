#include "map_tools_panel.h"
#include <rclcpp/time.hpp>

namespace map_tools_rviz_plugin
{

MapToolsPanel::MapToolsPanel(QWidget* parent)
  : rviz_common::Panel(parent)
  , vehicle_x_(0)
  , vehicle_y_(0)
  , vehicle_z_(0)
{
  node_ = rclcpp::Node::make_shared("map_tools_panel_node");

  QVBoxLayout* layout = new QVBoxLayout;
  clear_goal_button_ = new QPushButton("Clear Goal Point", this);
  layout->addWidget(clear_goal_button_);
  clear_terrain_button_ = new QPushButton("Clear Terrain Map", this);
  layout->addWidget(clear_terrain_button_);
  setLayout(layout);

  connect(clear_goal_button_,    SIGNAL(pressed()), this, SLOT(clearGoalPoint()));
  connect(clear_terrain_button_, SIGNAL(pressed()), this, SLOT(clearTerrainMap()));

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/state_estimation", 5,
    std::bind(&MapToolsPanel::odometryHandler, this, std::placeholders::_1));

  goal_point_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>("/goal_point", 5);
  way_point_pub_  = node_->create_publisher<geometry_msgs::msg::PointStamped>("/way_point",  5);
  map_clearing_pub_ = node_->create_publisher<std_msgs::msg::Float32>("/map_clearing", 5);
}

void MapToolsPanel::odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  vehicle_x_ = odom->pose.pose.position.x;
  vehicle_y_ = odom->pose.pose.position.y;
  vehicle_z_ = odom->pose.pose.position.z;
}

void MapToolsPanel::clearGoalPoint()
{
  rclcpp::spin_some(node_);

  geometry_msgs::msg::PointStamped pt;
  pt.header.stamp    = node_->now();
  pt.header.frame_id = "map";
  pt.point.x = vehicle_x_;
  pt.point.y = vehicle_y_;
  pt.point.z = vehicle_z_;

  // publish to both topics to cover with-FAR-Planner and direct-waypoint configurations
  goal_point_pub_->publish(pt);
  way_point_pub_->publish(pt);
}

void MapToolsPanel::clearTerrainMap()
{
  std_msgs::msg::Float32 msg;
  msg.data = 8.0f;  // matches default clearingDis in terrain_analysis
  map_clearing_pub_->publish(msg);
}

}  // namespace map_tools_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(map_tools_rviz_plugin::MapToolsPanel, rviz_common::Panel)
