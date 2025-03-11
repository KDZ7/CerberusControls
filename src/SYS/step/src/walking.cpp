#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "yaml-cpp/yaml.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

class Walking : public rclcpp::Node
{
public:
    Walking() : Node("walking")
    {
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Walking::timer_callback, this));

        try
        {
            YAML::Node config = YAML::LoadFile(ament_index_cpp::get_package_share_directory("step") + "/config/walking.yaml");
            if (config["walking_cycle"])
            {
                for (const auto step : config["walking_cycle"])
                {
                    std::vector<double> joint_positions;
                    for (const auto &leg : {"FL", "FR", "BL", "BR"})
                    {
                        joint_positions.push_back(step[leg]["SeXX_M"].as<double>());
                        joint_positions.push_back(step[leg]["ThXX_SeXX"].as<double>());
                        joint_positions.push_back(step[leg]["ArXX_ThXX"].as<double>());
                    }
                    walking_cycle_.push_back(joint_positions);
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load walking cycle: %s", e.what());
        }
    }

private:
    void timer_callback()
    {
        static size_t step = 0;
        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = this->now();
        joint_state.name = {"SeFL_M", "ThFL_SeFL", "ArFL_ThFL",
                            "SeFR_M", "ThFR_SeFR", "ArFR_ThFR",
                            "SeBL_M", "ThBL_SeBL", "ArBL_ThBL",
                            "SeBR_M", "ThBR_SeBR", "ArBR_ThBR"};
        joint_state.position = walking_cycle_[step];
        joint_state_publisher_->publish(joint_state);
        step = (step + 1) % walking_cycle_.size();
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::vector<double>> walking_cycle_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Walking>());
    rclcpp::shutdown();
    return 0;
}