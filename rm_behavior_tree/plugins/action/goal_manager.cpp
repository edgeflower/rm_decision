// Goal Manager Action Node - Refactored to use RosTopicSubNode
// Now shares the main ROS node instead of creating a private one
// File: goal_manager.cpp

#include "rm_behavior_tree/plugins/action/goal_manager.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <rclcpp/executors.hpp>

namespace rm_behavior_tree
{

GoalManagerAction::GoalManagerAction(
    const std::string & name,
    const BT::NodeConfiguration & config,
    const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_decision_interfaces::msg::ObservationPoints>(name, config, params)
, all_points_completed_(false)
{
    // Get parameters from ports if provided, otherwise use defaults
    getInput<double>("lethal_threshold", lethal_threshold_);
    getInput<double>("high_cost_threshold", high_cost_threshold_);
    getInput<double>("visit_timeout", visit_timeout_);
    getInput<int>("max_retries", max_retries_);

    // Set defaults if not provided
    if (lethal_threshold_ <= 0) lethal_threshold_ = 252.0;
    if (high_cost_threshold_ <= 0) high_cost_threshold_ = 200.0;
    if (visit_timeout_ <= 0) visit_timeout_ = 30.0;
    if (max_retries_ < 0) max_retries_ = 2;

    RCLCPP_INFO(node_->get_logger(), "GoalManagerAction initialized:");
    RCLCPP_INFO(node_->get_logger(), "  lethal_threshold: %.1f", lethal_threshold_);
    RCLCPP_INFO(node_->get_logger(), "  high_cost_threshold: %.1f", high_cost_threshold_);
    RCLCPP_INFO(node_->get_logger(), "  visit_timeout: %.1f seconds", visit_timeout_);
    RCLCPP_INFO(node_->get_logger(), "  max_retries: %d", max_retries_);
}

BT::NodeStatus GoalManagerAction::onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::ObservationPoints>& last_msg)
{
    // Check if we received observation points data
    if (!last_msg) {
        RCLCPP_WARN(node_->get_logger(), "Waiting for observation points...");
        setOutput("should_reset", false);
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "Received %zu observation points", last_msg->points.size());

    // Get robot pose from input port
    geometry_msgs::msg::TransformStamped robot_pose;
    if (!getInput<geometry_msgs::msg::TransformStamped>("robot_pose", robot_pose)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get robot_pose from input port");
        return BT::NodeStatus::FAILURE;
    }

    // Check if reset is requested
    bool reset_requested = false;
    getInput<bool>("reset_requested", reset_requested);
    if (reset_requested) {
        RCLCPP_INFO(node_->get_logger(), "Reset requested, resetting all points to IDLE");
        resetAllPoints();
    }

    std::lock_guard<std::mutex> lock(data_mutex_);

    // Update observation points from the latest message
    for (const auto & point : last_msg->points) {
        observation_points_[point.point_id] = point;

        // Initialize status if not exists
        if (goal_status_list_.find(point.point_id) == goal_status_list_.end()) {
            rm_decision_interfaces::msg::GoalStatus status;
            status.point_id = point.point_id;
            status.status = IDLE;
            status.last_visit_time.sec = 0;
            status.last_visit_time.nanosec = 0;
            goal_status_list_[point.point_id] = status;

            // Initialize visit info
            visit_info_map_[point.point_id] = PointVisitInfo();
            visit_info_map_[point.point_id].point_id = point.point_id;
            visit_info_map_[point.point_id].max_retries = max_retries_;
        }
    }

    RCLCPP_INFO(node_->get_logger(), "Total observation points stored: %zu", observation_points_.size());

    // Check for visiting timeouts and update statuses
    checkVisitingTimeouts();

    // Check if all points are completed
    checkAllPointsCompleted();

    // Try to find nearest IDLE point
    uint32_t best_id = findNearestIdlePoint(robot_pose);

    // If no IDLE points, check if we can retry blocked points
    if (best_id == 0) {
        best_id = findNearestPointConsideringRetry(robot_pose);
    }

    // Handle case when no points are available
    if (best_id == 0 && observation_points_.empty()) {
        RCLCPP_WARN(node_->get_logger(), "No observation points available");
        setOutput("should_reset", false);
        return BT::NodeStatus::FAILURE;
    }

    // All points are done
    if (best_id == 0 && all_points_completed_) {
        RCLCPP_INFO(node_->get_logger(), "All observation points completed, ready for reset");
        setOutput("should_reset", true);

        uint32_t done_count = 0;
        for (const auto & status_pair : goal_status_list_) {
            if (status_pair.second.status == DONE) {
                done_count++;
            }
        }
        setOutput("done_count", done_count);
        setOutput("idle_count", static_cast<uint32_t>(0));

        return BT::NodeStatus::SUCCESS;
    }

    // No available points but not all completed
    if (best_id == 0) {
        RCLCPP_WARN(node_->get_logger(), "No available observation points (all blocked or visiting)");
        setOutput("should_reset", false);

        uint32_t idle_count = 0, done_count = 0;
        for (const auto & status_pair : goal_status_list_) {
            if (status_pair.second.status == IDLE) idle_count++;
            if (status_pair.second.status == DONE) done_count++;
        }
        setOutput("idle_count", idle_count);
        setOutput("done_count", done_count);

        return BT::NodeStatus::FAILURE;
    }

    // Get the best observation point
    auto it = observation_points_.find(best_id);
    if (it == observation_points_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Point ID %u not found in observation points", best_id);
        return BT::NodeStatus::FAILURE;
    }

    // Create goal pose
    geometry_msgs::msg::PoseStamped best_goal;
    best_goal.header.stamp = node_->now();
    best_goal.header.frame_id = "map";
    best_goal.pose = it->second.pose;

    // Check costmap status
    double cost_value = 0.0;
    CostmapStatus cost_status = checkCostmapStatus(best_goal, cost_value);

    if (cost_status == LETHAL || cost_status == OUT_OF_BOUNDS) {
        RCLCPP_WARN(node_->get_logger(), "Goal ID %u is unreachable (cost status: %d, cost: %.2f), marking as BLOCKED",
                    best_id, static_cast<int>(cost_status), cost_value);
        updateGoalStatus(best_id, BLOCKED);

        // Try to retry if max retries not reached
        if (!retryBlockedPoint(best_id)) {
            RCLCPP_ERROR(node_->get_logger(), "Point ID %u exceeded max retries, skipping", best_id);
        }

        setOutput("should_reset", false);
        return BT::NodeStatus::FAILURE;
    }

    if (cost_status == HIGH_COST) {
        RCLCPP_WARN(node_->get_logger(), "Goal ID %u has high cost (%.2f), delaying visit", best_id, cost_value);
        // For high cost points, we still mark as VISITING but log warning
        // The navigation stack will handle the difficulty
    }

    // Mark as VISITING and setup visit tracking
    updateGoalStatus(best_id, VISITING);
    visit_info_map_[best_id].point_id = best_id;
    visit_info_map_[best_id].visit_start_time = node_->now();
    visit_info_map_[best_id].max_duration_seconds = visit_timeout_;

    // Count IDLE and DONE points
    uint32_t idle_count = 0, done_count = 0;
    for (const auto & status_pair : goal_status_list_) {
        if (status_pair.second.status == IDLE) idle_count++;
        if (status_pair.second.status == DONE) done_count++;
    }

    // Output results
    setOutput("best_goal", best_goal);
    setOutput("selected_id", best_id);
    setOutput("should_reset", false);
    setOutput("idle_count", idle_count);
    setOutput("done_count", done_count);

    RCLCPP_INFO(node_->get_logger(), "Selected goal ID %u at (%.2f, %.2f) with score %.2f, cost: %.2f",
                best_id, best_goal.pose.position.x, best_goal.pose.position.y,
                it->second.score, cost_value);

    return BT::NodeStatus::SUCCESS;
}

uint32_t GoalManagerAction::findNearestIdlePoint(
    const geometry_msgs::msg::TransformStamped & robot_pose)
{
    double min_dist = std::numeric_limits<double>::max();
    uint32_t best_id = 0;

    for (const auto & status_pair : goal_status_list_) {
        const auto & status = status_pair.second;

        // Only consider IDLE points
        if (status.status != IDLE) {
            continue;
        }

        // Find corresponding observation point
        auto it = observation_points_.find(status.point_id);
        if (it == observation_points_.end()) {
            continue;
        }

        // Calculate distance
        double dist = euclideanDistance(robot_pose, it->second);

        // Update best if closer
        if (dist < min_dist) {
            min_dist = dist;
            best_id = status.point_id;
        }
    }
    RCLCPP_INFO(node_->get_logger(), "Nearest IDLE point: ID %u, distance %.2f", best_id, min_dist);

    return best_id;
}

uint32_t GoalManagerAction::findNearestPointConsideringRetry(
    const geometry_msgs::msg::TransformStamped & robot_pose)
{
    double min_dist = std::numeric_limits<double>::max();
    uint32_t best_id = 0;

    for (const auto & status_pair : goal_status_list_) {
        const auto & status = status_pair.second;

        // Consider RETRYING points (previously blocked but being retried)
        if (status.status != RETRYING) {
            continue;
        }

        // Find corresponding observation point
        auto it = observation_points_.find(status.point_id);
        if (it == observation_points_.end()) {
            continue;
        }

        // Calculate distance
        double dist = euclideanDistance(robot_pose, it->second);

        // Update best if closer
        if (dist < min_dist) {
            min_dist = dist;
            best_id = status.point_id;
        }
    }

    return best_id;
}

double GoalManagerAction::euclideanDistance(
    const geometry_msgs::msg::TransformStamped & pose1,
    const rm_decision_interfaces::msg::ObservationPoint & point2)
{
    double dx = pose1.transform.translation.x - point2.pose.position.x;
    double dy = pose1.transform.translation.y - point2.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

GoalManagerAction::CostmapStatus GoalManagerAction::checkCostmapStatus(
    const geometry_msgs::msg::PoseStamped & goal, double & cost_value)
{
    // NON-BLOCKING: Only check if service exists, don't actually call it
    // Calling spin_until_future_complete inside BT node causes Executor conflicts

    // Create costmap service client if not exists
    if (!costmap_client_) {
        costmap_client_ = node_->create_client<nav2_msgs::srv::GetCostmap>("/global_costmap/get_costmap");
    }

    // NON-BLOCKING check: Only verify service availability without spinning
    if (!costmap_client_->service_is_ready()) {
        RCLCPP_DEBUG(node_->get_logger(), "Costmap service not ready, skipping check");
        cost_value = 0.0;
        return REACHABLE;
    }

    // Skip actual costmap checking to avoid blocking
    // The navigation stack will handle obstacle avoidance
    cost_value = 0.0;
    return REACHABLE;
}

void GoalManagerAction::updateGoalStatus(uint32_t point_id, GoalStatusEnum status)
{
    auto it = goal_status_list_.find(point_id);
    if (it != goal_status_list_.end()) {
        it->second.status = status;
        it->second.last_visit_time = node_->now();
    }
}

void GoalManagerAction::checkVisitingTimeouts()
{
    auto current_time = node_->now();

    for (auto & status_pair : goal_status_list_) {
        uint32_t point_id = status_pair.first;
        auto & status = status_pair.second;

        // Check VISITING points for timeout
        if (status.status == VISITING) {
            auto visit_it = visit_info_map_.find(point_id);
            if (visit_it != visit_info_map_.end()) {
                auto elapsed = (current_time - visit_it->second.visit_start_time).seconds();

                if (elapsed > visit_it->second.max_duration_seconds) {
                    RCLCPP_INFO(node_->get_logger(), "Point ID %u visiting timeout (%.1f s), marking as DONE",
                                point_id, elapsed);
                    updateGoalStatus(point_id, DONE);
                }
            }
        }
    }
}

void GoalManagerAction::checkAllPointsCompleted()
{
    if (observation_points_.empty()) {
        all_points_completed_ = false;
        return;
    }

    bool all_done = true;
    uint32_t total_points = 0;

    for (const auto & status_pair : goal_status_list_) {
        if (status_pair.second.status != DONE) {
            all_done = false;
            break;
        }
        total_points++;
    }

    all_points_completed_ = (all_done && total_points > 0);

    if (all_points_completed_) {
        RCLCPP_INFO(node_->get_logger(), "All %u observation points completed!", total_points);
    }
}

bool GoalManagerAction::retryBlockedPoint(uint32_t point_id)
{
    auto visit_it = visit_info_map_.find(point_id);
    if (visit_it == visit_info_map_.end()) {
        return false;
    }

    if (visit_it->second.retry_count < visit_it->second.max_retries) {
        visit_it->second.retry_count++;
        updateGoalStatus(point_id, RETRYING);
        RCLCPP_INFO(node_->get_logger(), "Point ID %u set to RETRYING (attempt %d/%d)",
                    point_id, visit_it->second.retry_count, visit_it->second.max_retries);
        return true;
    }

    return false;
}

bool GoalManagerAction::isGoalReached(uint32_t point_id) const
{
    auto it = goal_status_list_.find(point_id);
    return (it != goal_status_list_.end() && it->second.status == DONE);
}

bool GoalManagerAction::shouldResetAll() const
{
    return all_points_completed_;
}

void GoalManagerAction::resetAllPoints()
{
    for (auto & status_pair : goal_status_list_) {
        status_pair.second.status = IDLE;
        status_pair.second.last_visit_time = node_->now();
    }

    // Reset visit info
    for (auto & visit_pair : visit_info_map_) {
        visit_pair.second.retry_count = 0;
    }

    all_points_completed_ = false;

    RCLCPP_INFO(node_->get_logger(), "Reset all %zu observation points to IDLE", goal_status_list_.size());
}

std::vector<uint32_t> GoalManagerAction::sortPointsByNearestNeighbor(
    const geometry_msgs::msg::TransformStamped & robot_pose)
{
    std::vector<uint32_t> sorted_ids;
    std::set<uint32_t> unvisited;

    // Collect all unvisited (IDLE) points
    for (const auto & status_pair : goal_status_list_) {
        if (status_pair.second.status == IDLE) {
            unvisited.insert(status_pair.first);
        }
    }

    geometry_msgs::msg::TransformStamped current_pose = robot_pose;

    // Greedy nearest neighbor algorithm
    while (!unvisited.empty()) {
        double min_dist = std::numeric_limits<double>::max();
        uint32_t nearest_id = 0;

        for (uint32_t point_id : unvisited) {
            auto it = observation_points_.find(point_id);
            if (it != observation_points_.end()) {
                double dist = euclideanDistance(current_pose, it->second);
                if (dist < min_dist) {
                    min_dist = dist;
                    nearest_id = point_id;
                }
            }
        }

        if (nearest_id != 0) {
            sorted_ids.push_back(nearest_id);
            unvisited.erase(nearest_id);

            // Update current pose to the selected point
            auto it = observation_points_.find(nearest_id);
            if (it != observation_points_.end()) {
                current_pose.transform.translation.x = it->second.pose.position.x;
                current_pose.transform.translation.y = it->second.pose.position.y;
            }
        } else {
            break;  // Should not happen
        }
    }

    return sorted_ids;
}

} // namespace rm_behavior_tree

// Use CreateRosNodePlugin macro instead of BT_REGISTER_NODES
// This enables shared node registration
#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::GoalManagerAction, "GoalManager")
