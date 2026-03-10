// IsGoalReached Condition Node - Check if a goal point has been reached
// Refactored to use RosTopicSubNode
// File: is_goal_reached.cpp

#include "rm_behavior_tree/plugins/condition/is_goal_reached.hpp"

namespace rm_behavior_tree
{

IsGoalReachedCondition::IsGoalReachedCondition(
    const std::string & name,
    const BT::NodeConfiguration & config,
    const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_decision_interfaces::msg::ObservationPoints>(name, config, params)
{
    RCLCPP_INFO(node_->get_logger(), "IsGoalReachedCondition initialized");
}

BT::NodeStatus IsGoalReachedCondition::onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::ObservationPoints>& last_msg)
{
    // Note: This is a simplified implementation that tracks point availability
    // In a full implementation, you would subscribe to a goal_status topic
    if (last_msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        RCLCPP_DEBUG(node_->get_logger(), "Received %zu observation points", last_msg->points.size());
    }

    // Get goal_id from input port
    uint32_t goal_id = 0;
    if (!getInput<uint32_t>("goal_id", goal_id)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get goal_id from input port");
        return BT::NodeStatus::FAILURE;
    }

    // Check if we should verify the current goal
    bool check_current = false;
    getInput<bool>("check_current", check_current);

    if (check_current) {
        uint32_t current_goal_id = 0;
        if (getInput<uint32_t>("current_goal_id", current_goal_id)) {
            if (current_goal_id != goal_id) {
                // Not the current goal, return FAILURE
                RCLCPP_DEBUG(node_->get_logger(), "Goal ID %u is not the current goal (%u)",
                             goal_id, current_goal_id);
                return BT::NodeStatus::FAILURE;
            }
        }
    }

    std::lock_guard<std::mutex> lock(data_mutex_);

    if (isGoalReached(goal_id)) {
        RCLCPP_DEBUG(node_->get_logger(), "Goal ID %u has been reached", goal_id);
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_DEBUG(node_->get_logger(), "Goal ID %u not yet reached", goal_id);
    return BT::NodeStatus::FAILURE;
}

bool IsGoalReachedCondition::isGoalReached(uint32_t goal_id) const
{
    // In the full implementation, this would check against a goal_status topic
    // For now, we return false (not reached) as the default
    // The actual status tracking is done by the GoalManager node

    auto it = goal_statuses_.find(goal_id);
    if (it != goal_statuses_.end()) {
        // Status: 0=IDLE, 1=VISITING, 2=DONE, 3=BLOCKED, 4=RETRYING
        return it->second == 2;  // DONE
    }

    return false;
}

} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::IsGoalReachedCondition, "IsGoalReached")
