#include "rm_behavior_tree/plugins/action/armor_to_goal.hpp"
#include <armor_interfaces/msg/detail/armor__struct.hpp>
#include <behaviortree_cpp/basic_types.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <cmath>

namespace rm_behavior_tree
{

ArmorToGoalAction::ArmorToGoalAction(const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config), rclcpp::Node("armor_to_goal_node")
{
    // 从 XML 读取战术参数
    getInput<double>("offset_distance", offset_distance_);
    getInput<double>("min_confidence", min_confidence_);

    // 设置默认值
    if (offset_distance_ <= 0.0) offset_distance_ = 2.0;
    if (min_confidence_ <= 0.0) min_confidence_ = 0.4;

    RCLCPP_INFO(this->get_logger(), "ArmorToGoalAction initialized:");
    RCLCPP_INFO(this->get_logger(), "  offset_distance: %.2f m", offset_distance_);
    RCLCPP_INFO(this->get_logger(), "  min_confidence: %.2f", min_confidence_);
}

BT::NodeStatus ArmorToGoalAction::tick()
{
    // 1. 获取敌人位置 (target_message 已在 map 坐标系)
    armor_interfaces::msg::Target target;
    auto target_result = getInput<armor_interfaces::msg::Target>("target_message");
    if (!target_result.has_value()) {
        RCLCPP_DEBUG(this->get_logger(), "No target_message received");
        return BT::NodeStatus::FAILURE;
    }
    target = target_result.value();

    // 2. 置信度检查
    if (target.confidence < min_confidence_) {
        RCLCPP_WARN(this->get_logger(), "Target confidence too low: %.2f < %.2f",
                    target.confidence, min_confidence_);
        return BT::NodeStatus::FAILURE;
    }

    // 3. 检查是否正在跟踪
    if (!target.tracking && target.confidence < min_confidence_) {
        RCLCPP_DEBUG(this->get_logger(), "Target not tracking and confidence too low");
        return BT::NodeStatus::FAILURE;
    }

    // 4. 获取机器人位置
    auto robot_pose_result = getInput<geometry_msgs::msg::PoseStamped>("robot_pose");
    if (!robot_pose_result.has_value()) {
        RCLCPP_WARN(this->get_logger(), "No robot_pose available for tactical offset calculation");
        return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::msg::PoseStamped robot_pose = robot_pose_result.value();

    // 5. 坐标系一致性检查
    if (robot_pose.header.frame_id != target.header.frame_id) {
        RCLCPP_ERROR(this->get_logger(),
            "Coordinate frame mismatch! robot: %s, target: %s",
            robot_pose.header.frame_id.c_str(), target.header.frame_id.c_str());
        return BT::NodeStatus::FAILURE;
    }

    // 6. 计算战术偏移目标
    geometry_msgs::msg::PoseStamped offset_goal = calculateOffsetGoal(target, robot_pose);

    // 7. 设置输出端口
    setOutput("goal_pose", offset_goal);

    RCLCPP_INFO(this->get_logger(), "Tactical goal calculated:");
    RCLCPP_INFO(this->get_logger(), "  Enemy: (%.2f, %.2f)", target.position.x, target.position.y);
    RCLCPP_INFO(this->get_logger(), "  Robot: (%.2f, %.2f)", robot_pose.pose.position.x, robot_pose.pose.position.y);
    RCLCPP_INFO(this->get_logger(), "  Goal: (%.2f, %.2f)", offset_goal.pose.position.x, offset_goal.pose.position.y);
    RCLCPP_INFO(this->get_logger(), "  Confidence: %.2f", target.confidence);

    return BT::NodeStatus::SUCCESS;
}

geometry_msgs::msg::PoseStamped ArmorToGoalAction::calculateOffsetGoal(
    const armor_interfaces::msg::Target &target,
    const geometry_msgs::msg::PoseStamped &robot_pose)
{
    geometry_msgs::msg::PoseStamped goal;

    goal.header.stamp = this->now();
    goal.header.frame_id = target.header.frame_id;

    // 敌人位置
    double target_x = target.position.x;
    double target_y = target.position.y;

    // 机器人位置
    double robot_x = robot_pose.pose.position.x;
    double robot_y = robot_pose.pose.position.y;

    // 计算方向向量：从敌人指向机器人
    double dx = robot_x - target_x;
    double dy = robot_y - target_y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // 计算战术偏移位置
    double offset_x, offset_y;

    if (distance <= offset_distance_) {
        // 敌人很近，保持在机器人当前位置（不要后退）
        offset_x = robot_x;
        offset_y = robot_y;

        RCLCPP_DEBUG(this->get_logger(), "Close range: %.2f m <= %.2f m, holding position",
                    distance, offset_distance_);
    } else {
        // 正常情况：在敌人朝向机器人的方向上，放置偏移目标
        // 这样目标点位于机器人和敌人之间
        double scale = offset_distance_ / distance;
        offset_x = target_x + dx * scale;
        offset_y = target_y + dy * scale;
    }

    // 设置计算出的位置
    goal.pose.position.x = offset_x;
    goal.pose.position.y = offset_y;
    goal.pose.position.z = 0.0;

    // 计算朝向（始终面向敌人）
    double yaw = std::atan2(target_y - offset_y, target_x - offset_x);

    // 转换为四元数
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = std::sin(yaw / 2.0);
    goal.pose.orientation.w = std::cos(yaw / 2.0);

    RCLCPP_DEBUG(this->get_logger(), "Offset goal calculated:");
    RCLCPP_DEBUG(this->get_logger(), "  Target: (%.2f, %.2f)", target_x, target_y);
    RCLCPP_DEBUG(this->get_logger(), "  Robot: (%.2f, %.2f)", robot_x, robot_y);
    RCLCPP_DEBUG(this->get_logger(), "  Distance: %.2f m, Offset: %.2f m", distance, offset_distance_);
    RCLCPP_DEBUG(this->get_logger(), "  Goal: (%.2f, %.2f)", offset_x, offset_y);

    return goal;
}

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory){
    factory.registerNodeType<rm_behavior_tree::ArmorToGoalAction>("ArmorToGoal");
}
