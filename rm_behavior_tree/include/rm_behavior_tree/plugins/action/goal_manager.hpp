#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__GOAL_MANAGER_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__GOAL_MANAGER_HPP_

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rm_decision_interfaces/msg/observation_points.hpp>
#include <rm_decision_interfaces/msg/observation_point.hpp>
#include <rm_decision_interfaces/msg/goal_status.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/client.hpp>
#include <nav2_msgs/srv/get_costmap.hpp>
#include <vector>
#include <map>
#include <mutex>
#include <chrono>

namespace rm_behavior_tree
{

enum GoalStatusEnum : uint8_t {
    IDLE = 0,
    VISITING = 1,
    DONE = 2,
    BLOCKED = 3,
    RETRYING = 4
};

// Point visit tracking
struct PointVisitInfo {
    uint32_t point_id;
    rclcpp::Time visit_start_time;
    double max_duration_seconds;
    int retry_count;
    int max_retries;

    PointVisitInfo() : point_id(0), max_duration_seconds(30.0), retry_count(0), max_retries(2) {}
};

// GoalManagerAction now inherits from RosTopicSubNode
// It will receive observation_points messages automatically
class GoalManagerAction : public BT::RosTopicSubNode<rm_decision_interfaces::msg::ObservationPoints>
{
public:
    GoalManagerAction(const std::string & name,
                     const BT::NodeConfiguration & config,
                     const BT::RosNodeParams & params);

    // Must provide ports including topic_name port from base class
    static BT::PortsList providedPorts()
    {
        // Combine custom ports with base class ports
        BT::PortsList custom_ports = {
            BT::InputPort<geometry_msgs::msg::TransformStamped>("robot_pose"),
            BT::InputPort<double>("lethal_threshold", 252, "Lethal cost threshold"),
            BT::InputPort<double>("high_cost_threshold", 200, "High cost threshold for delay"),
            BT::InputPort<double>("visit_timeout", 30.0, "Max time to mark point as DONE (seconds)"),
            BT::InputPort<int>("max_retries", 2, "Maximum retry attempts for blocked points"),
            BT::InputPort<bool>("reset_requested", false, "Request to reset all points to IDLE"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("best_goal"),
            BT::OutputPort<uint32_t>("selected_id"),
            BT::OutputPort<bool>("should_reset", "True if all points completed"),
            BT::OutputPort<uint32_t>("idle_count", "Number of IDLE points remaining"),
            BT::OutputPort<uint32_t>("done_count", "Number of DONE points")
        };
        return BT::RosTopicSubNode<rm_decision_interfaces::msg::ObservationPoints>::providedBasicPorts(custom_ports);
    }

    // onTick is called automatically when the behavior tree ticks this node
    // last_msg contains the latest ObservationPoints message (or nullptr if no message yet)
    BT::NodeStatus onTick(const std::shared_ptr<rm_decision_interfaces::msg::ObservationPoints>& last_msg) override;

    // Public methods for BT condition nodes
    bool isGoalReached(uint32_t point_id) const;
    bool shouldResetAll() const;
    void resetAllPoints();

private:
    // Costmap service client (created lazily)
    rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;

    // Data storage
    std::map<uint32_t, rm_decision_interfaces::msg::ObservationPoint> observation_points_;
    std::map<uint32_t, rm_decision_interfaces::msg::GoalStatus> goal_status_list_;
    std::map<uint32_t, PointVisitInfo> visit_info_map_;
    mutable std::mutex data_mutex_;

    // Parameters
    double lethal_threshold_;
    double high_cost_threshold_;
    double visit_timeout_;
    int max_retries_;
    bool all_points_completed_;

    // Helper functions
    uint32_t findNearestIdlePoint(const geometry_msgs::msg::TransformStamped & robot_pose);
    uint32_t findNearestPointConsideringRetry(const geometry_msgs::msg::TransformStamped & robot_pose);
    double euclideanDistance(const geometry_msgs::msg::TransformStamped & pose1,
                              const rm_decision_interfaces::msg::ObservationPoint & point2);

    // Enhanced costmap checking with delay logic
    enum CostmapStatus { REACHABLE, HIGH_COST, LETHAL, OUT_OF_BOUNDS };
    CostmapStatus checkCostmapStatus(const geometry_msgs::msg::PoseStamped & goal, double & cost_value);

    // Status management
    void updateGoalStatus(uint32_t point_id, GoalStatusEnum status);
    void checkVisitingTimeouts();
    void checkAllPointsCompleted();
    bool retryBlockedPoint(uint32_t point_id);

    // TSP nearest neighbor algorithm
    std::vector<uint32_t> sortPointsByNearestNeighbor(const geometry_msgs::msg::TransformStamped & robot_pose);
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__GOAL_MANAGER_HPP_
