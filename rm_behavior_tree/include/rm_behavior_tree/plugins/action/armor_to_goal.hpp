#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__ARMOR_TO_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__ARMOR_TO_GOAL_HPP_

#include "armor_interfaces/msg/armor.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <armor_interfaces/msg/detail/target__struct.hpp>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <rclcpp/publisher.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <rclcpp/node.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <armor_interfaces/msg/target.hpp>
#include <rm_decision_interfaces/msg/target.hpp>
#include <chrono>
#include <cmath>

namespace rm_behavior_tree {
class ArmorToGoalAction : public BT::StatefulActionNode, rclcpp::Node
{
public:
    ArmorToGoalAction(const std::string &name, const BT::NodeConfig & config);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<armor_interfaces::msg::Target>("target_message", "Target message with position and confidence"),
            BT::InputPort<double>("offset_distance", 1.5, "Distance to offset from target (meters)"),
            BT::InputPort<double>("min_confidence", 0.3, "Minimum confidence threshold"),
            BT::InputPort<double>("rate_limit", 0.1, "Minimum time between goal updates (seconds)"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose", "Current robot pose for offset direction"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "Output goal pose")
        };
    }

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

    // DEPRECATED: Kept for backward compatibility
    void setMessage(geometry_msgs::msg::TransformStamped location,
                   geometry_msgs::msg::PoseStamped &armor_target_location);

    // DEPRECATED: Now publishing directly in onRunning to avoid duplicate calculation
    void sendGoalPose(const armor_interfaces::msg::Target &target_msg,
                      const geometry_msgs::msg::PoseStamped &robot_pose);

private:
    int goal_count;
    geometry_msgs::msg::PoseStamped armor_target_location;
    armor_interfaces::msg::Target last_target_;
    nav_msgs::msg::Odometry sentry_current_location;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_goal_pose;
    geometry_msgs::msg::PoseStamped robot_location;

    // Parameters
    double offset_distance_;
    double min_confidence_;
    double rate_limit_;

    // Rate limiting
    rclcpp::Time last_publish_time_;
    geometry_msgs::msg::PoseStamped last_published_goal_;

    // Helper functions
    /**
     * @brief Check if goal should be published based on rate limit and position change
     * @param now Current time
     * @param new_goal Newly calculated goal
     * @return true if goal should be published
     *
     * FIXED: Reduced position change threshold from 0.5m to 0.15m for better
     * responsiveness in close-range combat scenarios.
     */
    bool shouldPublishGoal(const rclcpp::Time &now, const geometry_msgs::msg::PoseStamped &new_goal);

    /**
     * @brief Calculate offset goal position between target and robot
     * @param target Target position and info
     * @param robot_pose Robot pose (must be in same frame as target)
     * @return Offset goal pose
     *
     * FIXED: Handles close-range case (distance < offset_distance) by holding
     * position instead of backing away. Also adds coordinate frame safety check.
     */
    geometry_msgs::msg::PoseStamped calculateOffsetGoal(
        const armor_interfaces::msg::Target &target,
        const geometry_msgs::msg::PoseStamped &robot_pose);
};

} // rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__ARMOR_TO_GOAL_HPP_
