#include "rm_behavior_tree/plugins/action/get_location.hpp"
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>

#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <memory>

#include <nav_msgs/msg/detail/odometry__struct.hpp>
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
    geometry_msgs::msg::TransformStamped robot_location;
    robot_location.header.stamp = node_->now();
    robot_location.header.frame_id = "map";
    robot_location.transform.translation.x = 0.0;
    robot_location.transform.translation.y = 0.0;
    robot_location.transform.translation.z = 0.0;
    robot_location.transform.rotation.x = 0.0;
    robot_location.transform.rotation.y = 0.0;
    robot_location.transform.rotation.z = 0.0;
    robot_location.transform.rotation.w = 1.0;

    if (last_msg) {
        robot_location.transform.translation.x = last_msg->pose.pose.position.x;
        robot_location.transform.translation.y = last_msg->pose.pose.position.y;
        // 只使用平面位置信息，z轴和旋转保持默认值

        setOutput("robot_location", robot_location);


        RCLCPP_INFO(logger(), "[GetLocation] pose.position.x: %f", robot_location.transform.translation.x);
        RCLCPP_INFO(logger(), "[GetLocation] pose.position.y: %f", robot_location.transform.translation.y);

        return BT::NodeStatus::SUCCESS;
    } else {
        RCLCPP_WARN(logger(), "[GetLocation] Waiting for /odometry message...");
        return BT::NodeStatus::FAILURE;
    }
}
} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"

CreateRosNodePlugin(rm_behavior_tree::GetLocationAction, "GetLocation");