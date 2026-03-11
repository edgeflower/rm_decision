#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__ARMOR_TO_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__ARMOR_TO_GOAL_HPP_

#include "armor_interfaces/msg/armor.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <armor_interfaces/msg/detail/target__struct.hpp>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <rclcpp/node.hpp>
#include <armor_interfaces/msg/target.hpp>
#include <cmath>

namespace rm_behavior_tree {

/**
 * @brief ArmorToGoalAction - 战术偏移计算节点
 *
 * 输入: target_message (已在 map 坐标系的敌人位置)
 *       robot_pose (机器人位置)
 * 输出: goal_pose (战术偏移后的目标点)
 *
 * 功能: 在敌人朝向机器人的方向上，放置一个偏移距离的目标点
 *       让机器人保持在有利位置进行攻击
 */
class ArmorToGoalAction : public BT::SyncActionNode, public rclcpp::Node
{
public:
    ArmorToGoalAction(const std::string &name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<armor_interfaces::msg::Target>("target_message", "Target message with position and confidence"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose", "Current robot pose for offset direction"),
            BT::InputPort<double>("offset_distance", 2.0, "Distance to offset from target (meters)"),
            BT::InputPort<double>("min_confidence", 0.4, "Minimum confidence threshold"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "Output goal pose with tactical offset")
        };
    }

    BT::NodeStatus tick() override;

private:
    // 战术偏移参数
    double offset_distance_ = 2.0;
    double min_confidence_ = 0.4;

    /**
     * @brief Calculate offset goal position between target and robot
     *
     * 战术原理: 在敌人朝向机器人的方向上，放置一个偏移目标
     *           这样机器人就能保持在敌人前方，保持射击优势
     *
     * @param target Target position (must be in map frame)
     * @param robot_pose Robot pose (must be in map frame)
     * @return Offset goal pose
     */
    geometry_msgs::msg::PoseStamped calculateOffsetGoal(
        const armor_interfaces::msg::Target &target,
        const geometry_msgs::msg::PoseStamped &robot_pose);
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__ARMOR_TO_GOAL_HPP_
