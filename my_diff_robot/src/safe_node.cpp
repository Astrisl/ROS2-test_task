#include <memory>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class SafeNode : public rclcpp::Node
{
public:
    SafeNode() : Node("safe_node")
    {
        sub_raw_cmd_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel_raw",
            10,
            std::bind(&SafeNode::cmd_callback, this, std::placeholders::_1));

        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            std::bind(&SafeNode::scan_callback, this, std::placeholders::_1));

        auto qos = rclcpp::QoS(10).best_effort();
        pub_safe_cmd_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/diff_drive_controller/cmd_vel", qos);

        RCLCPP_INFO(this->get_logger(), "Safe Node with strict requirements loaded!");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (msg->ranges.empty()) return;

        float current_min = msg->range_max;
        bool valid_found = false;

        const float min_distance_from_lidar = 0.20f; 
        const float robot_width = 0.5f;        
        const float side_clearance = 0.05f;     
        const float max_y = (robot_width / 2.0f) + side_clearance;
        const float max_x = 1.0f; 

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float r = msg->ranges[i];

            if (std::isnan(r) || std::isinf(r) || r < min_distance_from_lidar || r > msg->range_max)
                continue;

            float raw_angle = msg->angle_min + i * msg->angle_increment;
            float angle = std::atan2(std::sin(raw_angle), std::cos(raw_angle));

            float x = r * std::cos(angle);
            float y = r * std::sin(angle);

            if (x > min_distance_from_lidar && x <= max_x && std::abs(y) <= max_y)
            {
                if (x < current_min)
                {
                    current_min = x;
                    valid_found = true;
                }
            }
        }

        if (valid_found)
        {
            min_dist_ = current_min;
        }
        else
        {
            min_dist_ = msg->range_max; 
        }
    }

    void cmd_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        geometry_msgs::msg::TwistStamped safe_msg = *msg;
        safe_msg.header.stamp = this->now();

        float current_speed = msg->twist.linear.x;

        if (current_speed > 0.0)
        {
            float stop_threshold = safety_distance_ + (current_speed * 0.2f);

            if (min_dist_ <= stop_threshold)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                    "STOP! Obstacle distance: %.2f m <= Threshold: %.2f m", min_dist_, stop_threshold);
                safe_msg.twist.linear.x = 0.0;
            }
        }

        pub_safe_cmd_->publish(safe_msg);
    }

    float min_dist_ = 10.0f; 
    const float safety_distance_ = 0.50f;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_raw_cmd_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_safe_cmd_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafeNode>());
    rclcpp::shutdown();
    return 0;
}