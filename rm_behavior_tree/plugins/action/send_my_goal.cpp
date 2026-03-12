#include "rm_behavior_tree/plugins/action/send_my_goal.hpp"
#include <sstream>
#include <vector>

namespace rm_behavior_tree
{

SendMyGoalAction::SendMyGoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
{
}

bool SendMyGoalAction::setGoal(nav2_msgs::action::NavigateToPose::Goal & goal)
{
  std::string goal_str;
  if (!getInput<std::string>("my_goal_pose", goal_str)) {
    throw BT::RuntimeError("error reading port [my_goal_pose]");
  }

  // 手动解析字符串: "x;y;z;qx;qy;qz;qw"
  std::vector<std::string> parts;
  std::stringstream ss(goal_str);
  std::string token;
  while (std::getline(ss, token, ';')) {
    parts.push_back(token);
  }

  if (parts.size() != 7) {
    throw BT::RuntimeError("Invalid my_goal_pose format. Expected: x;y;z;qx;qy;qz;qw");
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = std::stod(parts[0]);
  pose.pose.position.y = std::stod(parts[1]);
  pose.pose.position.z = std::stod(parts[2]);
  pose.pose.orientation.x = std::stod(parts[3]);
  pose.pose.orientation.y = std::stod(parts[4]);
  pose.pose.orientation.z = std::stod(parts[5]);
  pose.pose.orientation.w = std::stod(parts[6]);

  goal.pose = pose;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = rclcpp::Clock().now();

  /*
  // clang-format off
  std::cout << "My_goal_pose: [ "
    << std::fixed << std::setprecision(1)
    << goal.pose.pose.position.x << ", "
    << goal.pose.pose.position.y << ", "
    << goal.pose.pose.position.z << ", "
    << goal.pose.pose.orientation.x << ", "
    << goal.pose.pose.orientation.y << ", "
    << goal.pose.pose.orientation.z << ", "
    << goal.pose.pose.orientation.w << " ]\n";
  // clang-format on
  */

  return true;
}

void SendMyGoalAction::onHalt()
{
  RCLCPP_INFO(node_->get_logger(), "SendMyGoalAction has been halted.");
}

BT::NodeStatus SendMyGoalAction::onResultReceived(const WrappedResult & wr)
{
  switch (wr.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "Success!!!");
      return BT::NodeStatus::SUCCESS;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(node_->get_logger(), "Goal was aborted");
      return BT::NodeStatus::FAILURE;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(node_->get_logger(), "Goal was canceled");
      std::cout << "Goal was canceled" << '\n';
      return BT::NodeStatus::FAILURE;
      break;
    default:
      RCLCPP_INFO(node_->get_logger(), "Unknown result code");
      return BT::NodeStatus::FAILURE;
      break;
  }
}

BT::NodeStatus SendMyGoalAction::onFeedback(
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> /*feedback*/)
{
  // std::cout << "Distance remaining: " << feedback->distance_remaining << '\n';
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SendMyGoalAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(node_->get_logger(), "SendGoalAction failed with error code: %d", error);
  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SendMyGoalAction, "SendMyGoal");