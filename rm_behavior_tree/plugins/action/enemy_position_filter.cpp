#include "rm_behavior_tree/plugins/action/enemy_position_filter.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <algorithm>

namespace rm_behavior_tree {

EnemyPositionFilter::EnemyPositionFilter(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
    , initialized_(false)
{
    RCLCPP_INFO(rclcpp::get_logger("EnemyPositionFilter"),
        "EnemyPositionFilter initialized (Data Cleaning Layer)");
}

BT::NodeStatus EnemyPositionFilter::tick()
{
    // 获取输入参数
    double alpha = 0.3;
    double max_distance = 1.5;  // CHANGED: Default from 10.0 to 1.5 for RoboMaster 3v3
    int history_size = 5;
    double zero_threshold = 0.1;
    double min_output_interval = 0.1;  // 默认100ms (10Hz)

    getInput("alpha", alpha);
    getInput("max_distance", max_distance);
    getInput("history_size", history_size);
    getInput("zero_threshold", zero_threshold);
    getInput("min_output_interval", min_output_interval);

    alpha_ = std::clamp(alpha, 0.01, 1.0);
    max_history_size_ = std::max(2, history_size);
    min_output_interval_ = min_output_interval;

    RCLCPP_DEBUG(rclcpp::get_logger("EnemyPositionFilter"),
        "Params: alpha=%.2f, max_distance=%.2fm, history_size=%zu, min_interval=%.2fs",
        alpha_, max_distance, max_history_size_, min_output_interval_);

    // 方案1: 检查输出时间间隔，防止频繁输出导致导航抖动
    rclcpp::Time current_time = rclcpp::Clock().now();
    if (initialized_ && last_output_time_.nanoseconds() > 0) {
        double elapsed = (current_time - last_output_time_).seconds();
        if (elapsed < min_output_interval_) {
            // 未达到最小输出间隔，直接返回上次值，不重新计算
            RCLCPP_DEBUG(rclcpp::get_logger("EnemyPositionFilter"),
                "Output throttled: %.3fs < %.3fs, using last value",
                elapsed, min_output_interval_);
            setOutput("filtered_position", filtered_target_);
            return BT::NodeStatus::SUCCESS;
        }
    }

    // 获取输入的敌人位置 (完整 Target 消息)
    armor_interfaces::msg::Target enemy_position;
    if (!getInput("enemy_position", enemy_position)) {
        // 没有输入数据，如果已有滤波值则继续使用
        if (initialized_) {
            // 输出上次的有效目标 (保持元数据)
            setOutput("filtered_position", filtered_target_);
            RCLCPP_DEBUG(rclcpp::get_logger("EnemyPositionFilter"),
                "No input, using last valid target");
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_WARN(rclcpp::get_logger("EnemyPositionFilter"),
            "No input and no history available");
        return BT::NodeStatus::FAILURE;
    }

    // 检查位置是否有效（不是NaN或无穷大）
    if (!std::isfinite(enemy_position.position.x) ||
        !std::isfinite(enemy_position.position.y) ) {
        // 无效值，但如果有历史值则继续使用
        if (initialized_) {
            setOutput("filtered_position", filtered_target_);
            RCLCPP_DEBUG(rclcpp::get_logger("EnemyPositionFilter"),
                "Invalid position (NaN/Inf), using last valid target");
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_ERROR(rclcpp::get_logger("EnemyPositionFilter"),
            "Invalid position and no history");
        return BT::NodeStatus::FAILURE;
    }

    // 检查是否为0值（视觉未检测到目标或预测丢失时）
    double abs_x = std::abs(enemy_position.position.x);
    double abs_y = std::abs(enemy_position.position.y);
    if (abs_x < zero_threshold && abs_y < zero_threshold) {
        // 0值：保持上一次的有效位置
        if (initialized_) {
            // 关键配合：当串口节点的置信度预测输出0值时，
            // 这里保持最后一次有效坐标，让ArmorToGoal继续使用
            RCLCPP_DEBUG(rclcpp::get_logger("EnemyPositionFilter"),
                "Zero value detected, keeping last valid position [%.2f, %.2f]",
                filtered_position_.x, filtered_position_.y);
            setOutput("filtered_position", filtered_target_);
            return BT::NodeStatus::SUCCESS;
        }
        /*
        RCLCPP_WARN(rclcpp::get_logger("EnemyPositionFilter"),
            "Zero value on first input, cannot initialize");
            */
        return BT::NodeStatus::FAILURE;
    }

    // 检查是否为异常值（仅在历史数据充足时启用）
    // 配合预测逻辑：即使是预测坐标也要经过异常检测
    if (initialized_ && history_.size() >= 2 && isOutlier(enemy_position)) {
        // 异常值：保持当前滤波值，但记录到历史
        RCLCPP_DEBUG(rclcpp::get_logger("EnemyPositionFilter"),
            "Outlier detected: raw[%.2f, %.2f], keeping filtered[%.2f, %.2f]",
            enemy_position.position.x, enemy_position.position.y,
            filtered_position_.x, filtered_position_.y);

        // 输出当前滤波值 (保持原始元数据)
        setOutput("filtered_position", filtered_target_);

        // 记录原始值到历史（用于后续判断）
        history_.push_back({
            enemy_position.header.stamp,
            enemy_position.position.x,
            enemy_position.position.y
        });
        while (history_.size() > max_history_size_) {
            history_.pop_front();
        }
        return BT::NodeStatus::SUCCESS;
    }

    // 正常值：更新滤波器 (只过滤 position.x 和 position.y)
    updateFilteredPosition(enemy_position);

    // 添加到历史记录
    history_.push_back({
        enemy_position.header.stamp,
        enemy_position.position.x,
        enemy_position.position.y
    });

    // 限制历史记录大小
    while (history_.size() > max_history_size_) {
        history_.pop_front();
    }

    // 创建并输出完整的 Target 消息
    // 关键：只修改坐标，透传所有元数据
    filtered_target_ = createFilteredTarget(
        filtered_position_.x,
        filtered_position_.y,
        enemy_position
    );

    setOutput("filtered_position", filtered_target_);

    // 更新最后输出时间
    last_output_time_ = rclcpp::Clock().now();

    RCLCPP_DEBUG(rclcpp::get_logger("EnemyPositionFilter"),
        "Filtered: raw[%.2f, %.2f] -> filtered[%.2f, %.2f], id=%s, conf=%.2f, tracking=%d",
        enemy_position.position.x, enemy_position.position.y,
        filtered_target_.position.x, filtered_target_.position.y,
        filtered_target_.id.c_str(), filtered_target_.confidence,
        filtered_target_.tracking);

    return BT::NodeStatus::SUCCESS;
}

void EnemyPositionFilter::reset()
{
    initialized_ = false;
    history_.clear();
    last_output_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    RCLCPP_INFO(rclcpp::get_logger("EnemyPositionFilter"),
        "Filter reset - will require fresh input");
}

double EnemyPositionFilter::calculateDistance(const PositionHistory& p1, const PositionHistory& p2) const
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

bool EnemyPositionFilter::isOutlier(const armor_interfaces::msg::Target& pos) const
{
    if (history_.size() < 2) {
        return false;
    }

    // 计算与历史数据的平均距离
    double total_distance = 0.0;
    int count = 0;

    for (const auto& hist : history_) {
        double dist = std::sqrt(
            std::pow(pos.position.x - hist.x, 2) +
            std::pow(pos.position.y - hist.y, 2)
        );
        total_distance += dist;
        count++;
    }

    double avg_distance = total_distance / count;

    // 获取阈值
    double max_distance = 1.5;  // CHANGED: Default for RoboMaster 3v3
    getInput("max_distance", max_distance);

    // 如果平均距离超过阈值，认为是异常值
    // 在RoboMaster 3v3赛场上，单帧跳变1.5米以上通常是视觉误识别
    return avg_distance > max_distance;
}

void EnemyPositionFilter::updateFilteredPosition(const armor_interfaces::msg::Target& new_pos)
{
    if (!initialized_) {
        // 第一次接收数据，直接使用
        filtered_position_.x = new_pos.position.x;
        filtered_position_.y = new_pos.position.y;
        filtered_position_.last_update = new_pos.header.stamp;
        initialized_ = true;

        RCLCPP_INFO(rclcpp::get_logger("EnemyPositionFilter"),
            "Initialized with position [%.2f, %.2f], id=%s",
            filtered_position_.x, filtered_position_.y, new_pos.id.c_str());
    } else {
        // 指数移动平均滤波 (只对 x, y 坐标)
        // filtered = alpha * new + (1 - alpha) * old
        // Alpha = 0.3 提供平滑与实时性的平衡
        filtered_position_.x = alpha_ * new_pos.position.x + (1.0 - alpha_) * filtered_position_.x;
        filtered_position_.y = alpha_ * new_pos.position.y + (1.0 - alpha_) * filtered_position_.y;
        filtered_position_.last_update = new_pos.header.stamp;

        RCLCPP_DEBUG(rclcpp::get_logger("EnemyPositionFilter"),
            "EMA: [%.2f, %.2f] -> [%.2f, %.2f] (alpha=%.2f)",
            new_pos.position.x, new_pos.position.y,
            filtered_position_.x, filtered_position_.y, alpha_);
    }
}

armor_interfaces::msg::Target EnemyPositionFilter::createFilteredTarget(
    double x, double y,
    const armor_interfaces::msg::Target& original)
{
    armor_interfaces::msg::Target filtered;

    // 拷贝完整头部信息
    filtered.header = original.header;
    filtered.header.stamp = rclcpp::Clock().now();  // 更新为当前时间

    // 设置滤波后的坐标
    filtered.position.x = x;
    filtered.position.y = y;
    filtered.position.z = 0.0;  // 2D平面

    // 透传所有元数据 (保持原始信息)
    filtered.tracking = original.tracking;
    filtered.tracking_status = original.tracking_status;
    filtered.confidence = original.confidence;
    filtered.id = original.id;
    filtered.armors_num = original.armors_num;

    // 透传目标描述信息
    filtered.yaw = original.yaw;
    filtered.v_yaw = original.v_yaw;
    filtered.radius_1 = original.radius_1;
    filtered.radius_2 = original.radius_2;
    filtered.dz = original.dz;

    // 透传速度信息 (速度也保持原始值，不做滤波)
    filtered.velocity = original.velocity;

    return filtered;
}

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::EnemyPositionFilter>("EnemyPositionFilter");
}
