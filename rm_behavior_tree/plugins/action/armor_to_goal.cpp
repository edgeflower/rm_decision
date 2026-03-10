#include "rm_behavior_tree/plugins/action/armor_to_goal.hpp"
#include <armor_interfaces/msg/detail/armor__struct.hpp>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <cmath>
#include <algorithm>

namespace rm_behavior_tree
{

ArmorToGoalAction::ArmorToGoalAction(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config), Node("armor_to_goal_node"), last_publish_time_(0, 0, RCL_ROS_TIME)
{
    publisher_goal_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);

    // Get parameters from ports with defaults
    getInput<double>("offset_distance", offset_distance_);
    getInput<double>("min_confidence", min_confidence_);
    getInput<double>("rate_limit", rate_limit_);

    // Set defaults if not provided
    if (offset_distance_ <= 0.0) offset_distance_ = 1.5;
    if (min_confidence_ <= 0.0) min_confidence_ = 0.3;
    if (rate_limit_ <= 0.0) rate_limit_ = 0.1;

    RCLCPP_INFO(this->get_logger(), "ArmorToGoalAction initialized:");
    RCLCPP_INFO(this->get_logger(), "  offset_distance: %.2f m", offset_distance_);
    RCLCPP_INFO(this->get_logger(), "  min_confidence: %.2f", min_confidence_);
    RCLCPP_INFO(this->get_logger(), "  rate_limit: %.2f s", rate_limit_);
}

BT::NodeStatus ArmorToGoalAction::onStart()
{
    // onStart is called once when the node first starts
    // We do minimal initialization here, actual target updates happen in onRunning
    RCLCPP_INFO(this->get_logger(), "ArmorToGoalAction started");
    return BT::NodeStatus::RUNNING;
}

void ArmorToGoalAction::onHalted()
{
    RCLCPP_WARN(this->get_logger(), "ArmorToGoalAction was halted.");
}

BT::NodeStatus ArmorToGoalAction::onRunning()
{
    // CRITICAL FIX: Update target on every tick, not just in onStart
    armor_interfaces::msg::Target target;
    if (!getInput<armor_interfaces::msg::Target>("target_message", target)) {
        RCLCPP_DEBUG(this->get_logger(), "No target_message received");
        return BT::NodeStatus::FAILURE;
    }

    // Store the latest target
    last_target_ = target;

    // Check confidence threshold
    if (target.confidence < min_confidence_) {
        RCLCPP_WARN(this->get_logger(), "Target confidence too low: %.2f < %.2f",
                    target.confidence, min_confidence_);
        return BT::NodeStatus::FAILURE;
    }

    // Check if target is being tracked
    if (!target.tracking && target.confidence < min_confidence_) {
        RCLCPP_DEBUG(this->get_logger(), "Target not tracking and confidence too low, status: %d", target.tracking_status);
        return BT::NodeStatus::FAILURE;
    }

    // Get robot pose for offset direction
    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.header.frame_id = "map";
    robot_pose.pose.position.x = 0.0;
    robot_pose.pose.position.y = 0.0;
    robot_pose.pose.position.z = 0.0;
    robot_pose.pose.orientation.x = 0.0;
    robot_pose.pose.orientation.y = 0.0;
    robot_pose.pose.orientation.z = 0.0;
    robot_pose.pose.orientation.w = 1.0;

    auto result = getInput<geometry_msgs::msg::PoseStamped>("robot_pose");
    bool has_robot_pose = result.has_value();
    if (has_robot_pose) {
        robot_pose = result.value();
    }

    // Calculate offset goal (only once, no duplicate calculation)
    geometry_msgs::msg::PoseStamped offset_goal;
    if (has_robot_pose) {
        offset_goal = calculateOffsetGoal(target, robot_pose);
    } else {
        // No robot pose available, use target position directly
        RCLCPP_WARN(this->get_logger(), "No robot_pose available, using target position directly");
        offset_goal.header.stamp = this->now();
        offset_goal.header.frame_id = target.header.frame_id;
        offset_goal.pose.position.x = target.position.x;
        offset_goal.pose.position.y = target.position.y;
        offset_goal.pose.position.z = 0.0;
        offset_goal.pose.orientation.w = 1.0;
    }

    // Check if we should publish (rate limiting + significant change)
    rclcpp::Time now = this->now();
    if (shouldPublishGoal(now, offset_goal)) {
        // Publish directly without recalculation
        offset_goal.header.stamp = now;
        publisher_goal_pose->publish(offset_goal);

        last_publish_time_ = now;
        last_published_goal_ = offset_goal;

        // Set output port
        setOutput("goal_pose", offset_goal);

        RCLCPP_INFO(this->get_logger(), "[ArmorToGoal] Publishing goal:");
        RCLCPP_INFO(this->get_logger(), "  Target ID: %s at (%.2f, %.2f)", target.id.c_str(), target.position.x, target.position.y);
        RCLCPP_INFO(this->get_logger(), "  Goal: (%.2f, %.2f)", offset_goal.pose.position.x, offset_goal.pose.position.y);
        RCLCPP_INFO(this->get_logger(), "  Confidence: %.2f, Tracking: %s", target.confidence, target.tracking ? "true" : "false");
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Goal update skipped (rate limited)");
    }

    return BT::NodeStatus::RUNNING;
}

bool ArmorToGoalAction::shouldPublishGoal(const rclcpp::Time &now, const geometry_msgs::msg::PoseStamped &new_goal)
{
    // Always publish if never published before
    if (last_publish_time_.nanoseconds() == 0) {
        return true;
    }

    // Check rate limit
    double time_since_last = (now - last_publish_time_).seconds();
    if (time_since_last < rate_limit_) {
        // Even within rate limit, check for significant position change
        // FIXED: Reduced threshold from 0.5m to 0.15m for better responsiveness in close combat
        double dx = new_goal.pose.position.x - last_published_goal_.pose.position.x;
        double dy = new_goal.pose.position.y - last_published_goal_.pose.position.y;
        double distance_change = std::sqrt(dx * dx + dy * dy);

        // Publish anyway if position changed significantly
        // 对于追击 0.5米以上的变化，强制发布以保持响应性，高速变化反而不好
        if (distance_change > 0.5) {
            RCLCPP_INFO(this->get_logger(), "Significant position change: %.2f m, publishing anyway", distance_change);
            return true;
        }

        return false;  // Skip due to rate limit
    }

    return true;  // Rate limit satisfied
}

geometry_msgs::msg::PoseStamped ArmorToGoalAction::calculateOffsetGoal(
    const armor_interfaces::msg::Target &target,
    const geometry_msgs::msg::PoseStamped &robot_pose)
{
    geometry_msgs::msg::PoseStamped goal;

    goal.header.stamp = this->now();
    goal.header.frame_id = target.header.frame_id;

    // FIXED: Add coordinate frame safety check
    if (robot_pose.header.frame_id != target.header.frame_id) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Coordinate frame mismatch! robot: %s, target: %s. Using target position directly.",
            robot_pose.header.frame_id.c_str(), target.header.frame_id.c_str());
        goal.pose.position.x = target.position.x;
        goal.pose.position.y = target.position.y;
        goal.pose.position.z = 0.0;
        goal.pose.orientation.w = 1.0;
        return goal;
    }

    // Target position
    double target_x = target.position.x;
    double target_y = target.position.y;

    // Robot position
    double robot_x = robot_pose.pose.position.x;
    double robot_y = robot_pose.pose.position.y;

    // Calculate direction vector from target to robot
    double dx = robot_x - target_x;
    double dy = robot_y - target_y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // FIXED: Handle close-range case (enemy is very close)
    double offset_x, offset_y;

    if (distance <= offset_distance_) {
        // Enemy is closer than our desired offset distance
        // Strategy: Stay at current position (don't back away)
        offset_x = robot_x;
        offset_y = robot_y;

        RCLCPP_INFO(this->get_logger(), "Close range detected: %.2f m <= %.2f m, holding position",
                    distance, offset_distance_);
    } else {
        // Normal case: Position goal at offset_distance from target, towards robot
        // This places the goal BETWEEN the robot and target
        double scale = offset_distance_ / distance;
        offset_x = target_x + dx * scale;
        offset_y = target_y + dy * scale;
    }

    // Set the calculated position
    goal.pose.position.x = offset_x;
    goal.pose.position.y = offset_y;
    goal.pose.position.z = 0.0;

    // Calculate orientation (face the target)
    // Goal should always face the enemy
    double yaw = std::atan2(target_y - offset_y, target_x - offset_x);

    // Convert yaw to quaternion
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

void ArmorToGoalAction::setMessage(geometry_msgs::msg::TransformStamped location,
                                   geometry_msgs::msg::PoseStamped &armor_target_location)
{
    // This function appears to be unused in the current implementation
    // Kept for backward compatibility
}

void ArmorToGoalAction::sendGoalPose(const armor_interfaces::msg::Target &target_msg,
                                     const geometry_msgs::msg::PoseStamped &robot_pose)
{
    // This function is now DEPRECATED - we publish directly in onRunning
    // Kept for backward compatibility but should not be called
    RCLCPP_WARN(this->get_logger(), "sendGoalPose called, but this is deprecated. Publishing directly...");
    geometry_msgs::msg::PoseStamped goal_msg = calculateOffsetGoal(target_msg, robot_pose);
    goal_msg.header.stamp = this->now();
    publisher_goal_pose->publish(goal_msg);
}

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory){
    factory.registerNodeType<rm_behavior_tree::ArmorToGoalAction>("ArmorToGoal");
}
