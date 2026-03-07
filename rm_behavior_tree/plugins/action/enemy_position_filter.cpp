#include "rm_behavior_tree/plugins/action/enemy_position_filter.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <algorithm>

namespace rm_behavior_tree {

EnemyPositionFilter::EnemyPositionFilter(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
    , initialized_(false)
{
}

BT::NodeStatus EnemyPositionFilter::tick()
{
    // 获取输入参数
    double alpha = 0.3;
    double max_distance = 10.0;
    int history_size = 5;

    getInput("alpha", alpha);
    getInput("max_distance", max_distance);
    getInput("history_size", history_size);

    alpha_ = std::clamp(alpha, 0.01, 1.0);
    max_history_size_ = std::max(2, history_size);

    // 获取输入的敌人位置
    armor_interfaces::msg::Target enemy_position;
    if (!getInput("enemy_position", enemy_position)) {
        // 没有输入数据，重置滤波器
        reset();
        return BT::NodeStatus::FAILURE;
    }

    // 检查位置是否有效（不是NaN或无穷大）
    if (!std::isfinite(enemy_position.position.x) ||
        !std::isfinite(enemy_position.position.y) ) {
        return BT::NodeStatus::FAILURE;
    }

    // 检查是否为异常值
    if (initialized_ && isOutlier(enemy_position)) {
        // 异常值：使用预测值而不是测量值
        // 这里简单使用上一个滤波值
        RCLCPP_WARN(rclcpp::get_logger("EnemyPositionFilter"),
                    "Outlier detected at [%.2f, %.2f], using filtered value",
                    enemy_position.position.x, enemy_position.position.y);
        // 不更新，直接输出上一个滤波值
    } else {
        // 正常值：更新滤波器
        updateFilteredPosition(enemy_position);
    }

    // 添加到历史记录
    history_.push_back({
        enemy_position.header.stamp,
        enemy_position.position.x,
        enemy_position.position.y
        //enemy_position.position.z
    });

    // 限制历史记录大小
    while (history_.size() > max_history_size_) {
        history_.pop_front();
    }

    // 输出滤波后的位置
    setOutput("filtered_position", filtered_position_);

    return BT::NodeStatus::SUCCESS;
}

void EnemyPositionFilter::reset()
{
    initialized_ = false;
    history_.clear();
}

double EnemyPositionFilter::calculateDistance(const PositionHistory& p1, const PositionHistory& p2) const
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    //double dz = p1.z - p2.z;
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
            std::pow(pos.position.y - hist.y, 2) //+
            //std::pow(pos.position.z - hist.z, 2)
        );
        total_distance += dist;
        count++;
    }

    double avg_distance = total_distance / count;

    // 获取阈值
    double max_distance = 10.0;
    getInput("max_distance", max_distance);

    // 如果平均距离超过阈值，认为是异常值
    return avg_distance > max_distance;
}

void EnemyPositionFilter::updateFilteredPosition(const armor_interfaces::msg::Target& new_pos)
{
    if (!initialized_) {
        // 第一次接收数据，直接使用
        filtered_position_.point.x = new_pos.position.x;
        filtered_position_.point.y = new_pos.position.y;
        //filtered_position_.point.z = new_pos.position.z;
        initialized_ = true;
    } else {
        // 指数移动平均滤波
        // filtered = alpha * new + (1 - alpha) * old
        filtered_position_.point.x = alpha_ * new_pos.position.x + (1.0 - alpha_) * filtered_position_.point.x;
        filtered_position_.point.y = alpha_ * new_pos.position.y + (1.0 - alpha_) * filtered_position_.point.y;
        //filtered_position_.point.z = alpha_ * new_pos.position.z + (1.0 - alpha_) * filtered_position_.point.z;
        //filtered_position_.header = new_pos.header;
    }
}

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::EnemyPositionFilter>("EnemyPositionFilter");
}
