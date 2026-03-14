#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SET_POSTURE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SET_POSTURE_HPP_

#include "behaviortree_ros2/bt_service_node.hpp"
#include "rm_decision_interfaces/srv/set_sentry_posture.hpp"

namespace rm_behavior_tree
{

class SetPosture : public BT::RosServiceNode<rm_decision_interfaces::srv::SetSentryPosture>
{
public:
    SetPosture(const std::string & name,
               const BT::NodeConfig & conf,
               const BT::RosNodeParams & params);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("posture", "要切换的机器人姿态"),
            BT::InputPort<bool>("override", false, "是否强制覆盖当前状态"),
            BT::InputPort<int>("current_posture", 0, "当前实际姿态（从SubRobotPosture获取）")
        };
    }

    // 发送请求前调用，用于填充请求数据
    bool setRequest(Request::SharedPtr & request) override;

    // 收到响应后调用
    BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override;

    // 发生错误（如服务不可用）时调用
    BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

private:
    enum Posture
    {
        POSTURE_ATTACK = 1,   // 攻击姿态
        POSTURE_DEFENSE = 2,  // 防御姿态
        POSTURE_MOVE = 3,     // 移动姿态
    };

    // 目标姿态
    int target_posture_ {0};
    // 请求发送时间
    rclcpp::Time start_time_;
    // 是否已记录开始时间
    bool is_executing_timer_ {false};
    // 超时时间（秒）
    const double POSTURE_TIMEOUT_SECONDS_ = 15.0;

};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SET_POSTURE_HPP_
