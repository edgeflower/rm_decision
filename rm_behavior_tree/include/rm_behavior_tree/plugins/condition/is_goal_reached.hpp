#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GOAL_REACHED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GOAL_REACHED_HPP_

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include <rm_decision_interfaces/msg/observation_points.hpp>
#include <map>
#include <mutex>

namespace rm_behavior_tree
{

class IsGoalReachedCondition : public BT::RosTopicSubNode<rm_decision_interfaces::msg::ObservationPoints>
{
public:
    IsGoalReachedCondition(const std::string & name,
                          const BT::NodeConfiguration & config,
                          const BT::RosNodeParams & params);

    static BT::PortsList providedPorts()
    {
        BT::PortsList custom_ports = {
            BT::InputPort<uint32_t>("goal_id", "ID of the goal to check"),
            BT::InputPort<bool>("check_current", false, "Check if currently selected goal is reached"),
            BT::InputPort<uint32_t>("current_goal_id", 0, "Current goal ID from GoalManager")
        };
        return BT::RosTopicSubNode<rm_decision_interfaces::msg::ObservationPoints>::providedBasicPorts(custom_ports);
    }

    BT::NodeStatus onTick(const std::shared_ptr<rm_decision_interfaces::msg::ObservationPoints>& last_msg) override;

private:
    // Track goal statuses locally (simplified version)
    std::map<uint32_t, uint8_t> goal_statuses_;
    mutable std::mutex data_mutex_;

    // Helper to check if goal is reached (DONE status)
    bool isGoalReached(uint32_t goal_id) const;
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GOAL_REACHED_HPP_
