#include "rm_behavior_tree/plugins/action/get_location.hpp"
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

namespace rm_behavior_tree {
GetLocationAction::GetLocationAction(
    const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : BT::RosTopicSubNode<nav_msgs::msg::Odometry>(name, conf, params)
{
}

BT::NodeStatus GetLocationAction::onTick(
    const std::shared_ptr<nav_msgs::msg::Odometry>& last_msg)
{
    geometry_msgs::msg::PoseStamped robot_location;
    robot_location.header.stamp = node_->now();
    robot_location.header.frame_id = "map";
    robot_location.pose.position.x = 0.0;
    robot_location.pose.position.y = 0.0;
    robot_location.pose.position.z = 0.0;
    robot_location.pose.orientation.x = 0.0;
    robot_location.pose.orientation.y = 0.0;
    robot_location.pose.orientation.z = 0.0;
    robot_location.pose.orientation.w = 1.0;

    if (last_msg) {
        robot_location.pose.position.x = last_msg->pose.pose.position.x;
        robot_location.pose.position.y = last_msg->pose.pose.position.y;
        robot_location.pose.position.z = last_msg->pose.pose.position.z;
        robot_location.pose.orientation = last_msg->pose.pose.orientation;

        setOutput("robot_location", robot_location);

        RCLCPP_DEBUG(logger(), "[GetLocation] pose.position.x: %f", robot_location.pose.position.x);
        RCLCPP_DEBUG(logger(), "[GetLocation] pose.position.y: %f", robot_location.pose.position.y);

        return BT::NodeStatus::SUCCESS;
    } else {
        RCLCPP_WARN(logger(), "[GetLocation] Waiting for /odometry message...");
        return BT::NodeStatus::FAILURE;
    }
}
} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"

CreateRosNodePlugin(rm_behavior_tree::GetLocationAction, "GetLocation");