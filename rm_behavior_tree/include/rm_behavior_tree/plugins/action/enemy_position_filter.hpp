#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__ENEMY_POSITION_FILTER_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__ENEMY_POSITION_FILTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <behaviortree_cpp/action_node.h>
#include <deque>
#include <cmath>
#include <armor_interfaces/msg/target.hpp>

namespace rm_behavior_tree
{
    /**
    * @brief 对敌人位置进行滤波的节点
    *
    * 使用指数移动平均和异常值检测来平滑敌人位置数据
    *
    * @param[in] enemy_position 输入的敌人位置 (armor_interfaces::msg::Target)
    * @param[out] filtered_position 滤波后的敌人位置 (geometry_msgs::msg::PointStamped)
    * @param[in] alpha 滤波系数 (0.0-1.0)，越小越平滑，默认0.3
    * @param[in] max_distance 最大有效距离阈值，超过则视为异常值
    * @param[in] history_size 历史数据窗口大小，用于异常检测
    */

    class EnemyPositionFilter : public BT::SyncActionNode
    {
    public:
        EnemyPositionFilter(const std::string& name, const BT::NodeConfig& config);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<armor_interfaces::msg::Target>("enemy_position"),
                BT::OutputPort<geometry_msgs::msg::PointStamped>("filtered_position"),  // 输出滤波后的位置 ，使用PointStamped 而不使用Target是为了简化数据结构，专注于位置部分
                BT::InputPort<double>("alpha", 0.3, "滤波系数 (0.0-1.0)，默认0.3"),
                BT::InputPort<double>("max_distance", 10.0, "最大有效距离阈值(米)"),
                BT::InputPort<int>("history_size", 5, "历史数据窗口大小"),
                BT::InputPort<double>("zero_threshold", 0.1, "0值判断阈值(米)，小于此值视为无效检测")
            };
        }

        BT::NodeStatus tick() override;
        void reset();

    private:
        // 滤波后的位置
        geometry_msgs::msg::PointStamped filtered_position_;

        // 指数移动平均滤波器
        double alpha_ = 0.3;  // 滤波系数
        bool initialized_ = false;

        // 历史数据用于异常检测
        struct PositionHistory {
            rclcpp::Time time;
            double x, y;
        }; // 移除了z坐标，因为我们主要关注平面位置
        std::deque<PositionHistory> history_;
        size_t max_history_size_ = 5;

        // 辅助函数
        double calculateDistance(const PositionHistory& p1, const PositionHistory& p2) const;
        bool isOutlier(const armor_interfaces::msg::Target& pos) const;
        void updateFilteredPosition(const armor_interfaces::msg::Target& new_pos);
    };
}

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__ENEMY_POSITION_FILTER_HPP_
