#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class zmp_stabilizer : public rclcpp::Node
{
public:
    zmp_stabilizer() : Node("zmp_stabilizer")
    {
        zmp_stabilizer_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("zmp_stabilizer/leg_positions", 10, std::bind(&zmp_stabilizer::zmp_stabilizer_callback, this, std::placeholders::_1));
        zmp_stabilizer_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("zmp_stabilizer/validate_positions", 10);
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
        std::vector<std::string> joints_ = {
            "SeFR_M", "ThFR_SeFR", "ArFR_ThFR",
            "SeFL_M", "ThFL_SeFL", "ArFL_ThFL",
            "SeBR_M", "ThBR_SeBR", "ArBR_ThBR",
            "SeBL_M", "ThBL_SeBL", "ArBL_ThBL"};
        if (msg->data.size() != joints_.size())
        {
            RCLCPP_WARN(this->get_logger(), "Received %zu joint angles, expected %zu.", msg->data.size(), joints_.size());
            return;
        }
        auto trajectory_msg = std::make_unique<trajectory_msgs::msg::JointTrajectory>();
        trajectory_msg->header.stamp = this->now();
        trajectory_msg->header.frame_id = "world";
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.resize(msg->data.size());
        std::copy(msg->data.begin(), msg->data.end(), point.positions.begin());

        point.time_from_start = rclcpp::Duration(1, 0);
        trajectory_msg->points.push_back(point);
        zmp_stabilizer_pub_->publish(std::move(trajectory_msg));
    }
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr zmp_stabilizer_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr zmp_stabilizer_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<zmp_stabilizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}