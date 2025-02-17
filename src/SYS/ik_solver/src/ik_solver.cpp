#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#define __clamp__(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

class ik_solver : public rclcpp::Node
{
public:
    ik_solver() : Node("ik_solver")
    {
        this->declare_parameter("L1", 23.55);
        this->declare_parameter("L2", 181.11);
        this->declare_parameter("L3", 179.12);

        L1_ = this->get_parameter("L1").as_double();
        L2_ = this->get_parameter("L2").as_double();
        L3_ = this->get_parameter("L3").as_double();

        gait_generator_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "gait_generator/leg_positions", 10, std::bind(&ik_solver::gait_generator_callback, this, std::placeholders::_1));
        zmp_stabilizer_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("zmp_stabilizer/leg_positions", 10);

        RCLCPP_INFO(this->get_logger(), "IK Solver has been started.");
        RCLCPP_INFO(this->get_logger(), "Leg Lengths: L1 = %f, L2 = %f, L3 = %f", L1_, L2_, L3_);
    }

private:
    void gait_generator_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (msg->poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty leg positions.");
            return;
        }
        auto angles = std::make_unique<std_msgs::msg::Float64MultiArray>();
        angles->data.resize(3 * msg->poses.size());
        for (size_t leg_idx = 0; leg_idx < msg->poses.size(); leg_idx++)
        {
            const auto &pose = msg->poses[leg_idx];
            auto q = calculate_ik(pose.position.x, pose.position.y, pose.position.z);

            angles->data[leg_idx * 3] = q[0];
            angles->data[leg_idx * 3 + 1] = q[1];
            angles->data[leg_idx * 3 + 2] = q[2];
        }
        zmp_stabilizer_pub_->publish(std::move(angles));
    }
    std::vector<double> calculate_ik(double x, double y, double z)
    {
        std::vector<double> q(3);
        double A = std::sqrt(z * z + y * y);
        double B = std::sqrt(__clamp__(A * A - L1_ * L1_, 0, 1e6));
        double C = std::sqrt(x * x + B * B);
        q[0] = std::atan2(y, z) + std::atan2(B, L1_);
        q[2] = std::acos(__clamp__((L2_ * L2_ + L3_ * L3_ - C * C) / (2 * L2_ * L3_), -1.0, 1.0));
        q[1] = std::atan2(x, B) + std::asin(__clamp__(L3_ * std::sin(q[2]) / C, -1.0, 1.0));
        return q;
    }
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gait_generator_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr zmp_stabilizer_pub_;
    double L1_, L2_, L3_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ik_solver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}