#include <memory>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class SafeNode : public rclcpp::Node
{
public:
    SafeNode() : Node("safe_node")
    {
        sub_raw_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_raw",
            10,
            std::bind(&SafeNode::cmd_callback, this, std::placeholders::_1));

        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&SafeNode::scan_callback, this, std::placeholders::_1));

        pub_safe_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Safe Node loaded!");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int total_deg = msg->ranges.size();
        if (total_deg == 0) return;

        int center = total_deg / 2;
        int window = 30;

        int start = std::max(0, center - window);
        int end = std::min(total_deg - 1, center + window);

        float current_min = msg->range_max;
        bool valid_found = false;

        for (int i = start; i <= end; ++i)
        {
            float val = msg->ranges[i];
            if (!std::isnan(val) && !std::isinf(val) && val >= msg->range_min && val <= msg->range_max)
            {
                if (val < current_min)
                {
                    current_min = val;
                    valid_found = true;
                }
            }
        }
        if (valid_found)
        {
            min_dist_ = current_min;
        }
    }

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto safe_msg = *msg; 

        if (min_dist_ <= safety_distance_ && msg->linear.x > 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "WARNING! Forward movement blocked! Distance: %.2f м", min_dist_);
            safe_msg.linear.x = 0.0; 
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Transferred speed: linear=%.2f, angular=%.2f", safe_msg.linear.x, safe_msg.angular.z);
        }
        pub_safe_cmd_->publish(safe_msg);
    }

    float min_dist_ = 10.0f; 
    const float safety_distance_ = 0.5f;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_raw_cmd_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_safe_cmd_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafeNode>());
    rclcpp::shutdown();
    return 0;
}