// Map Processor Node - Offline/Periodic Map Analysis for Observation Point Generation
// File: map_processor_node.cpp

#include "rm_behavior_tree/map_processor_node.hpp"
#include <cmath>
#include <algorithm>

namespace rm_behavior_tree
{

MapProcessorNode::MapProcessorNode(const rclcpp::NodeOptions & options)
: Node("map_processor_node", options), first_map_received_(false)
{
    // Declare parameters
    this->declare_parameter("map_topic", "/map");
    this->declare_parameter("sample_step", 2.0);
    this->declare_parameter("robot_radius", 0.3);
    this->declare_parameter("ray_count", 36);
    this->declare_parameter("update_rate", 0.1);

    // Get parameters
    map_topic_ = this->get_parameter("map_topic").as_string();
    sample_step_ = this->get_parameter("sample_step").as_double();
    robot_radius_ = this->get_parameter("robot_radius").as_double();
    ray_count_ = this->get_parameter("ray_count").as_int();
    update_rate_ = this->get_parameter("update_rate").as_double();

    RCLCPP_INFO(this->get_logger(), "Map Processor Node started with params:");
    RCLCPP_INFO(this->get_logger(), "  map_topic: %s", map_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  sample_step: %.2f m", sample_step_);
    RCLCPP_INFO(this->get_logger(), "  robot_radius: %.2f m", robot_radius_);
    RCLCPP_INFO(this->get_logger(), "  ray_count: %d", ray_count_);
    RCLCPP_INFO(this->get_logger(), "  update_rate: %.2f Hz", update_rate_);

    // Create subscription and publisher
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, rclcpp::QoS(10).transient_local().reliable(),
        std::bind(&MapProcessorNode::mapCallback, this, std::placeholders::_1));

    observation_points_pub_ = this->create_publisher<rm_decision_interfaces::msg::ObservationPoints>(
        "/observation_points", 10);

    // Create timer for periodic updates
    auto timer_period = std::chrono::duration<double>(1.0 / update_rate_);
    timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&MapProcessorNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Waiting for map data on topic: %s", map_topic_.c_str());
}

void MapProcessorNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    current_map_ = msg;
    RCLCPP_INFO(this->get_logger(), "Received map: %d x %d, resolution: %.3f m/pixel",
                msg->info.width, msg->info.height, msg->info.resolution);

    // Process and publish observation points immediately on first map receipt
    if (!first_map_received_) {
        first_map_received_ = true;
        RCLCPP_INFO(this->get_logger(), "First map received, processing observation points immediately...");
        processAndPublishObservationPoints();
    }
}

void MapProcessorNode::timerCallback()
{
    if (!current_map_) {
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Processing map for observation points...");
    processAndPublishObservationPoints();
}

void MapProcessorNode::processAndPublishObservationPoints()
{
    if (!current_map_) {
        return;
    }

    // Step 1: Binarize map (threshold > 50 for obstacles)
    processed_map_ = binarizeMap(*current_map_);

    // Step 2: Dilate map for robot radius
    processed_map_ = dilateMap(processed_map_, robot_radius_, current_map_->info.resolution);

    // Step 3: Sample grid points
    auto grid_points = sampleGridPoints(*current_map_, sample_step_);

    RCLCPP_INFO(this->get_logger(), "Sampled %zu candidate observation points", grid_points.size());

    // Step 4: Score each point with ray-casting
    rm_decision_interfaces::msg::ObservationPoints obs_points;
    obs_points.header.stamp = this->now();
    obs_points.header.frame_id = current_map_->header.frame_id;

    uint32_t point_id = 0;
    for (const auto & pt : grid_points) {
        double world_x, world_y;
        mapToWorld(current_map_->info, static_cast<int>(pt.x), static_cast<int>(pt.y),
                   world_x, world_y);

        double score = scoreObservationPoint(processed_map_, current_map_->info,
                                              static_cast<float>(world_x), static_cast<float>(world_y));

        // Only include points with reasonable visibility
        if (score > 0.5) {
            rm_decision_interfaces::msg::ObservationPoint obs_pt;
            obs_pt.point_id = point_id++;
            obs_pt.pose.position.x = world_x;
            obs_pt.pose.position.y = world_y;
            obs_pt.pose.position.z = 0.0;
            obs_pt.pose.orientation.w = 1.0;  // Default orientation
            obs_pt.score = static_cast<float>(score);

            obs_points.points.push_back(obs_pt);
        }
    }

    observation_points_pub_->publish(obs_points);
    RCLCPP_INFO(this->get_logger(), "Published %zu observation points",
                obs_points.points.size());
}

cv::Mat MapProcessorNode::binarizeMap(const nav_msgs::msg::OccupancyGrid & map)
{
    int width = map.info.width;
    int height = map.info.height;

    cv::Mat binary_map(height, width, CV_8UC1);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = y * width + x;
            int8_t value = map.data[idx];

            // Threshold > 50 for obstacles, -1 (unknown) treated as free
            if (value > 50) {
                binary_map.at<uint8_t>(y, x) = 255;  // Obstacle
            } else {
                binary_map.at<uint8_t>(y, x) = 0;    // Free space
            }
        }
    }

    return binary_map;
}

cv::Mat MapProcessorNode::dilateMap(const cv::Mat & binary_map, double robot_radius,
                                     double resolution)
{
    int kernel_size = static_cast<int>(robot_radius / resolution) * 2 + 1;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                               cv::Size(kernel_size, kernel_size));

    cv::Mat dilated;
    cv::dilate(binary_map, dilated, kernel);

    return dilated;
}

std::vector<cv::Point2f> MapProcessorNode::sampleGridPoints(
    const nav_msgs::msg::OccupancyGrid & map, double step)
{
    std::vector<cv::Point2f> points;
    int step_pixels = static_cast<int>(step / map.info.resolution);

    for (int y = step_pixels / 2; y < static_cast<int>(map.info.height); y += step_pixels) {
        for (int x = step_pixels / 2; x < static_cast<int>(map.info.width); x += step_pixels) {
            // Only add points that are not in obstacles
            if (processed_map_.at<uint8_t>(y, x) == 0) {
                points.push_back(cv::Point2f(static_cast<float>(x), static_cast<float>(y)));
            }
        }
    }

    return points;
}

double MapProcessorNode::scoreObservationPoint(const cv::Mat & map,
                                                const nav_msgs::msg::MapMetaData & info,
                                                float x, float y)
{
    double total_distance = 0.0;
    int valid_rays = 0;
    double max_range = 5.0;  // Maximum ray casting range in meters

    for (int i = 0; i < ray_count_; i++) {
        double angle = 2.0 * M_PI * i / ray_count_;
        double dist = castRayOnGrid(map, info, x, y, angle, max_range);

        if (dist > 0) {
            total_distance += dist;
            valid_rays++;
        }
    }

    return (valid_rays > 0) ? (total_distance / valid_rays) / max_range : 0.0;
}

double MapProcessorNode::castRayOnGrid(const cv::Mat & map,
                                        const nav_msgs::msg::MapMetaData & info,
                                        float start_x, float start_y,
                                        double angle, double max_range)
{
    double step_size = info.resolution * 0.5;
    double distance = 0.0;
    double dx = std::cos(angle) * step_size;
    double dy = std::sin(angle) * step_size;

    double current_x = start_x;
    double current_y = start_y;

    while (distance < max_range) {
        int grid_x, grid_y;
        if (!worldToMap(info, current_x, current_y, grid_x, grid_y)) {
            break;  // Out of map bounds
        }

        if (isObstacle(map, grid_x, grid_y)) {
            return distance;
        }

        current_x += dx;
        current_y += dy;
        distance += step_size;
    }

    return max_range;
}

bool MapProcessorNode::isObstacle(const cv::Mat & map, int grid_x, int grid_y)
{
    if (grid_x < 0 || grid_x >= map.cols || grid_y < 0 || grid_y >= map.rows) {
        return true;  // Out of bounds treated as obstacle
    }
    return map.at<uint8_t>(grid_y, grid_x) > 0;
}

bool MapProcessorNode::worldToMap(const nav_msgs::msg::MapMetaData & info,
                                   double wx, double wy, int & mx, int & my)
{
    mx = static_cast<int>((wx - info.origin.position.x) / info.resolution);
    my = static_cast<int>((wy - info.origin.position.y) / info.resolution);

    return (mx >= 0 && mx < static_cast<int>(info.width) &&
            my >= 0 && my < static_cast<int>(info.height));
}

void MapProcessorNode::mapToWorld(const nav_msgs::msg::MapMetaData & info,
                                   int mx, int my, double & wx, double & wy)
{
    wx = info.origin.position.x + (mx + 0.5) * info.resolution;
    wy = info.origin.position.y + (my + 0.5) * info.resolution;
}

} // namespace rm_behavior_tree

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rm_behavior_tree::MapProcessorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
