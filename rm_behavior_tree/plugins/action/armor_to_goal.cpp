#include "rm_behavior_tree/plugins/action/armor_to_goal.hpp"
#include <armor_interfaces/msg/detail/armor__struct.hpp>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
namespace rm_behavior_tree
{
ArmorToGoalAction::ArmorToGoalAction(const std::string & name,const BT::NodeConfig & config)
: BT::StatefulActionNode(name,config) , Node("armor_to_goal_node"){
    publisher_goal_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose",10);
}

BT::NodeStatus ArmorToGoalAction::onStart(){
    // 在串口里已经进行了tf变换，所以这里直接使用sentry_message中的坐标作为当前坐标

    /*
    sentry_current_location.header.frame_id = "map";
    sentry_current_location.pose.pose.position.x = 0.0;
    sentry_current_location.pose.pose.position.y = 0.0;
    sentry_current_location.pose.pose.position.z = 0.0;

    sentry_current_location.pose.pose.orientation.x = 0.0;
    sentry_current_location.pose.pose.orientation.y = 0.0;
    sentry_current_location.pose.pose.orientation.z = 0.0;
    sentry_current_location.pose.pose.orientation.w = 1.0;
    

    if(!getInput("sentry_message",sentry_current_location)){
        RCLCPP_DEBUG(this->get_logger(), "没接收到 sentry_message ");
        return BT::NodeStatus::FAILURE;
    }

    
    if (getInput("robot_pose", robot_location)) {
        sentry_current_location.pose.pose.position.x = robot_location.first;
        sentry_current_location.pose.pose.position.y = robot_location.second;
    } else {
        RCLCPP_DEBUG(this->get_logger(),"没有得到 robot_location");
        return BT::NodeStatus::FAILURE;
    }
    */

    if(!getInput("armor_message",armor_relative_current_location)){
        RCLCPP_DEBUG(this->get_logger(), "没接收到 armor_message ");
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

void ArmorToGoalAction::onHalted(){
    RCLCPP_WARN(this->get_logger(),"ArmorToGoalAction was halted.");
}

BT::NodeStatus ArmorToGoalAction::onRunning(){
    sendGoalPose(armor_target_location);
    return BT::NodeStatus::RUNNING;
}


void ArmorToGoalAction::sendGoalPose(geometry_msgs::msg::PoseStamped & msg)
{   
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = "map";  // 暂定 map作为坐标系，后续根据实际情况调整
    msg.pose.position.x =  armor_relative_current_location.pose.position.x;
    msg.pose.position.y =  armor_relative_current_location.pose.position.y;
    msg.pose.position.z =  0.0; // 2D平面，z轴设为0

    msg.pose.orientation.x = 0.0; // 2D平面，朝向可以根据需要设定，这里暂时设为0
    msg.pose.orientation.y = 0.0; // 2D平面，朝向可以根据需要设定，这里暂时设为0
    msg.pose.orientation.z = 0.0; // 2D平面，朝向可以根据需要设定，这里暂时设为0
    msg.pose.orientation.w = 1.0; // 2D平面，朝向可以根据需要设定，这里暂时设为1

    // DEBUG
    RCLCPP_DEBUG(this->get_logger(),"[ArmorToGoal] 敌人目标点位");        
    RCLCPP_DEBUG(this->get_logger(),"[ArmorToGoal] pose.position.x %f",msg.pose.position.x);
    RCLCPP_DEBUG(this->get_logger(),"[ArmorToGoal] pose.position.y %f",msg.pose.position.y);
    RCLCPP_DEBUG(this->get_logger(),"[ArmorToGoal] pose.position.z %f",msg.pose.position.z);

    RCLCPP_DEBUG(this->get_logger(),"[ArmorToGoal] pose.orientation.x %f",msg.pose.orientation.x);        
    RCLCPP_DEBUG(this->get_logger(),"[ArmorToGoal] pose.orientation.y %f",msg.pose.orientation.y);        
    RCLCPP_DEBUG(this->get_logger(),"[ArmorToGoal] pose.orientation.z %f",msg.pose.orientation.z);        
    RCLCPP_DEBUG(this->get_logger(),"[ArmorToGoal] pose.orientation.w %f",msg.pose.orientation.w);        
     
    publisher_goal_pose->publish(msg);
}

}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory){
    factory.registerNodeType<rm_behavior_tree::ArmorToGoalAction>("ArmorToGoal");
}