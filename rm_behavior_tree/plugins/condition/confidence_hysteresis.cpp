#include "rm_behavior_tree/plugins/condition/confidence_hysteresis.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>

namespace rm_behavior_tree
{

ConfidenceHysteresis::ConfidenceHysteresis(const std::string &name, const BT::NodeConfiguration &config)
    : BT::SimpleConditionNode(name, std::bind(&ConfidenceHysteresis::checkConfidenceHysteresis, this), config)
{
    // Get parameters with defaults
    getInput<double>("upper_threshold", upper_threshold_);
    getInput<double>("lower_threshold", lower_threshold_);
    getInput<double>("grace_period", grace_period_);
    getInput<bool>("enable_hold_position", enable_hold_position_);

    // Validate thresholds
    if (upper_threshold_ <= lower_threshold_) {
        upper_threshold_ = lower_threshold_ + 0.1;
        RCLCPP_WARN(rclcpp::get_logger("ConfidenceHysteresis"),
                    "Upper threshold <= lower threshold, adjusted to %.2f", upper_threshold_);
    }

    RCLCPP_INFO(rclcpp::get_logger("ConfidenceHysteresis"),
                "Initialized with thresholds: [%.2f, %.2f], grace_period: %.1fs",
                lower_threshold_, upper_threshold_, grace_period_);
}

BT::NodeStatus ConfidenceHysteresis::checkConfidenceHysteresis()
{
    rclcpp::Time now = rclcpp::Clock().now();

    // Get current target
    armor_interfaces::msg::Target target;
    if (!getInput<armor_interfaces::msg::Target>("target", target)) {
        // No target data, reset to low confidence but continue execution
        // 返回 SUCCESS 让行为树继续，下游节点应该检查 hysteresis_state 来决定是否巡逻
        current_state_ = State::LOW_CONFIDENCE;
        setOutput("hysteresis_state", std::string("LOW")); // 使用LOW就行，下游节点根据这个状态决定是否进入巡逻模式
        setOutput("current_confidence", 0.0);
        return BT::NodeStatus::SUCCESS;
    }

    double confidence = target.confidence;
    setOutput("current_confidence", confidence);

    // 状态机逻辑
    switch (current_state_) {
        case State::HIGH_CONFIDENCE:
            // 高置信度状态：检查是否应该进入观察期
            if (shouldEnterGracePeriod(confidence)) {
                current_state_ = State::GRACE_PERIOD;
                grace_period_start_ = now;

                RCLCPP_INFO(rclcpp::get_logger("ConfidenceHysteresis"),
                            "State: HIGH -> GRACE_PERIOD (confidence: %.2f <= %.2f)",
                            confidence, upper_threshold_);

                // 保存最后有效目标用于保持位置
                if (enable_hold_position_) {
                    last_valid_target_ = target;
                }
            } else {
                RCLCPP_DEBUG(rclcpp::get_logger("ConfidenceHysteresis"),
                             "State: HIGH (confidence: %.2f)", confidence);
            }
            setOutput("hysteresis_state", stateToString(current_state_));
            return BT::NodeStatus::SUCCESS;

        case State::GRACE_PERIOD:
            // 观察期：检查是否应该回到高置信度或进入低置信度
            if (shouldReturnToHighConfidence(confidence)) {
                // 置信度回升，立即回到高置信度状态
                current_state_ = State::HIGH_CONFIDENCE;

                RCLCPP_INFO(rclcpp::get_logger("ConfidenceHysteresis"),
                            "State: GRACE_PERIOD -> HIGH (confidence recovered: %.2f)",
                            confidence);

                setOutput("hysteresis_state", stateToString(current_state_));
                return BT::NodeStatus::SUCCESS;
            }

            if (shouldExitGracePeriod(now, confidence)) {
                // 观察期结束，置信度仍然低，切换到巡逻模式
                current_state_ = State::LOW_CONFIDENCE;

                double elapsed = getGracePeriodElapsed(now);
                RCLCPP_INFO(rclcpp::get_logger("ConfidenceHysteresis"),
                            "State: GRACE_PERIOD -> LOW (grace period ended: %.1fs, final confidence: %.2f)",
                            elapsed, confidence);

                setOutput("hysteresis_state", stateToString(current_state_));
                return BT::NodeStatus::FAILURE;
            }

            // 仍在观察期
            RCLCPP_DEBUG(rclcpp::get_logger("ConfidenceHysteresis"),
                        "State: GRACE_PERIOD (elapsed: %.1fs/%.1fs, confidence: %.2f)",
                        getGracePeriodElapsed(now), grace_period_, confidence);

            // 观察期内继续返回 SUCCESS（保持追逐模式）
            setOutput("hysteresis_state", stateToString(current_state_));
            return BT::NodeStatus::SUCCESS;

        case State::LOW_CONFIDENCE:
            // 低置信度状态：检查是否应该回到高置信度
            if (shouldReturnToHighConfidence(confidence)) {
                // 置信度回升，立即切换回追逐模式
                current_state_ = State::HIGH_CONFIDENCE;
                last_high_confidence_time_ = now;

                RCLCPP_INFO(rclcpp::get_logger("ConfidenceHysteresis"),
                            "State: LOW -> HIGH (confidence recovered: %.2f)",
                            confidence);

                setOutput("hysteresis_state", stateToString(current_state_));
                return BT::NodeStatus::SUCCESS;
            }

            RCLCPP_DEBUG(rclcpp::get_logger("ConfidenceHysteresis"),
                        "State: LOW (confidence: %.2f)", confidence);

            setOutput("hysteresis_state", stateToString(current_state_));
            return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::FAILURE;  // Should not reach here
}

void ConfidenceHysteresis::reset()
{
    current_state_ = State::LOW_CONFIDENCE;
    grace_period_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_high_confidence_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    RCLCPP_INFO(rclcpp::get_logger("ConfidenceHysteresis"), "State reset to LOW");
}

std::string ConfidenceHysteresis::stateToString(State state) const
{
    switch (state) {
        case State::HIGH_CONFIDENCE: return "HIGH";
        case State::GRACE_PERIOD: return "GRACE_PERIOD";
        case State::LOW_CONFIDENCE: return "LOW";
        default: return "UNKNOWN";
    }
}

bool ConfidenceHysteresis::isInGracePeriod(const rclcpp::Time &now) const
{
    return current_state_ == State::GRACE_PERIOD;
}

double ConfidenceHysteresis::getGracePeriodElapsed(const rclcpp::Time &now) const
{
    if (grace_period_start_.nanoseconds() == 0) {
        return 0.0;
    }
    return (now - grace_period_start_).seconds();
}

bool ConfidenceHysteresis::shouldEnterGracePeriod(double confidence)
{
    // 从高置信度进入观察期的条件：置信度低于上阈值
    return confidence < upper_threshold_;
}

bool ConfidenceHysteresis::shouldExitGracePeriod(const rclcpp::Time &now, double confidence)
{
    // 从观察期切换到低置信度的条件：
    // 1. 观察期时间已满
    // 2. 置信度仍然低于下阈值
    double elapsed = getGracePeriodElapsed(now);
    return (elapsed >= grace_period_) && (confidence < lower_threshold_);
}

bool ConfidenceHysteresis::shouldReturnToHighConfidence(double confidence)
{
    // 从任何状态回到高置信度的条件：置信度高于上阈值
    // 这提供了即时响应（不用担心抖动，因为我们刚从高置信度掉下来）
    return confidence >= upper_threshold_;
}

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::ConfidenceHysteresis>("ConfidenceHysteresis");
}
