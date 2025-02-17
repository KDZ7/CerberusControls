#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class zmp_stabilizer : public rclcpp::Node
{
public:
    zmp_stabilizer() : Node("zmp_stabilizer")
    {
        zmp_stabilizer_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "zmp_stabilizer/leg_positions", 10,
            std::bind(&zmp_stabilizer::zmp_stabilizer_callback, this, std::placeholders::_1));
        zmp_stabilizer_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "zmp_stabilizer/validate_positions", 10);
        RCLCPP_INFO(this->get_logger(), "ZMP Stabilizer has been started.");
    }

private:
    void zmp_stabilizer_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty leg positions.");
            return;
        }
        auto validate_positions = std::make_unique<std_msgs::msg::Float64MultiArray>();
        validate_positions->data = msg->data;
        zmp_stabilizer_pub_->publish(std::move(validate_positions));
    }
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr zmp_stabilizer_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr zmp_stabilizer_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<zmp_stabilizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}