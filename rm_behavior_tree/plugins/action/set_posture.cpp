#include "rm_behavior_tree/plugins/action/set_posture.hpp"

namespace rm_behavior_tree
{

SetPosture::SetPosture(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosServiceNode<rm_decision_interfaces::srv::SetSentryPosture>(name, conf, params)
{
    // 修复：使用正确的服务名，与串口节点一致
    std::string service_name = "set_sentry_posture";
    RCLCPP_INFO(node_->get_logger(), "[%s] 正在连接服务: %s", name.c_str(), service_name.c_str());

    // 创建服务客户端
    auto client = node_->create_client<rm_decision_interfaces::srv::SetSentryPosture>(service_name);
    if (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(node_->get_logger(), "[%s] 服务 [%s] 暂时不可用，将在运行时重试。",
                    name.c_str(), service_name.c_str());
    } else {
        RCLCPP_INFO(node_->get_logger(), "[%s] 服务已连接", name.c_str());
    }
}

bool SetPosture::setRequest(Request::SharedPtr & request)
{
    // 获取目标姿态参数
    if (!getInput("posture", target_posture_)) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] 缺少姿态参数!", name().c_str());
        return false;
    }

    bool override_mode = false;
    getInput("override", override_mode);

    // 首次发送请求时记录开始时间
    if (!is_executing_timer_) {
        start_time_ = node_->now();
        is_executing_timer_ = true;
        RCLCPP_INFO(node_->get_logger(), "[%s] 开始姿态切换计时", name().c_str());
    }

    // 填充请求
    request->posture = static_cast<uint8_t>(target_posture_);
    request->override_mode = override_mode;

    RCLCPP_INFO(node_->get_logger(), "[%s] 发送姿态请求: posture = %d, override = %s",
                name().c_str(), target_posture_, override_mode ? "true" : "false");
    return true;
}

BT::NodeStatus SetPosture::onResponseReceived(const Response::SharedPtr & response)
{
    if (!response->accepted) {
        RCLCPP_WARN(node_->get_logger(), "[%s] 姿态切换请求被拒绝!", name().c_str());
        // 重置状态以便重试
        is_executing_timer_ = false;
        return BT::NodeStatus::FAILURE;
    }

    // 请求已被接受，现在检查实际姿态是否已切换成功
    int current_posture = 0;
    auto result = getInput("current_posture", current_posture);
    bool has_current_posture = static_cast<bool>(result);

    if (has_current_posture && current_posture == target_posture_) {
        // 姿态已确认切换成功
        RCLCPP_INFO(node_->get_logger(), "[%s] 姿态切换成功: %d, 耗时: %.2f秒",
                    name().c_str(), target_posture_,
                    (node_->now() - start_time_).seconds());

        // 重置状态以便下次使用
        is_executing_timer_ = false;
        return BT::NodeStatus::SUCCESS;
    }

    // 检查是否超时
    double elapsed = (node_->now() - start_time_).seconds();
    if (elapsed >= POSTURE_TIMEOUT_SECONDS_) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[%s] 姿态切换超时: 目标=%d, 当前=%d, 耗时: %.2f秒",
                     name().c_str(), target_posture_,
                     has_current_posture ? current_posture : -1, elapsed);

        // 超时失败，重置状态
        is_executing_timer_ = false;
        return BT::NodeStatus::FAILURE;
    }

    // 继续等待姿态切换完成
    if (has_current_posture) {
        RCLCPP_DEBUG(node_->get_logger(), "[%s] 等待姿态切换: 目标=%d, 当前=%d, 已等待: %.2f秒",
                     name().c_str(), target_posture_, current_posture, elapsed);
    } else {
        RCLCPP_DEBUG(node_->get_logger(), "[%s] 等待姿态反馈, 已等待: %.2f秒",
                     name().c_str(), elapsed);
    }

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetPosture::onFailure(BT::ServiceNodeErrorCode error)
{
    RCLCPP_ERROR(node_->get_logger(), "[%s] 服务调用失败，错误码: %d", name().c_str(), static_cast<int>(error));

    // 重置状态以便重试
    is_executing_timer_ = false;

    return BT::NodeStatus::FAILURE;
}

} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SetPosture, "SetPosture");
