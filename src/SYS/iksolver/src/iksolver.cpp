#include "log/log.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "yaml-cpp/yaml.h"
#include "iksolver/iksolver.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace cerberus_iksolver
{
    class IKSolver : public rclcpp::Node
    {
    public:
        IKSolver() : Node("iksolver")
        {
            this->declare_parameter("config_file_path", ament_index_cpp::get_package_share_directory("iksolver") + "/config/ikconfig.yaml");
            config_file_path_ = this->get_parameter("config_file_path").as_string();
            waypoint_sub_ = this->create_subscription<geometry_msgs::msg::Point>("~/waypoint", 10, std::bind(&IKSolver::waypoint_callback, this, std::placeholders::_1));
            iksolver_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
            try
            {
                LOG_INFO("IKSolver loading configuration file ...");
                YAML::Node config = YAML::LoadFile(config_file_path_);
                if (!config["groups"])
                {
                    LOG_ERROR("groups section missing in configuration file");
                    return;
                }
                for (auto it = config["groups"].begin(); it != config["groups"].end(); ++it)
                {
                    std::string group_name = it->first.as<std::string>();
                    YAML::Node group_node = it->second;
                    if (!group_node["joints"])
                    {
                        LOG_ERROR("joints section missing for group: " << group_name);
                        return;
                    }
                    for (const auto &joint : group_node["joints"])
                        group_joints_[group_name].push_back(joint.as<std::string>());
                    if (!group_node["structs"])
                    {
                        LOG_ERROR("structs section missing for group: " << group_name);
                        return;
                    }
                    for (const auto &structure : group_node["structs"])
                        group_structs_[group_name].push_back(structure.as<double>());
                }
                if (!config["joint_limits"])
                {
                    LOG_ERROR("joint_limits section missing in configuration file");
                    return;
                }
                for (const auto &joint : config["joint_limits"])
                {
                    joint_min_positions_[joint.first.as<std::string>()] = joint.second["min"].as<double>();
                    joint_max_positions_[joint.first.as<std::string>()] = joint.second["max"].as<double>();
                }
                if (!config["joint_offsets"])
                {
                    LOG_ERROR("joint_offsets section missing in configuration file");
                    return;
                }
                for (const auto &joint : config["joint_offsets"])
                    joint_offsets_[joint.first.as<std::string>()] = joint.second.as<double>();

                LOG_INFO("IKSolver initialized successfully");
            }
            catch (const std::exception &e)
            {
                LOG_ERROR("Failed to load configuration file: " << config_file_path_ << " with exception: " << e.what());
            }
        }

    private:
        void waypoint_callback(const geometry_msgs::msg::Point::SharedPtr msg)
        {
            for (const auto &group : group_joints_)
            {
                if (group.second.size() != 3)
                {
                    LOG_ERROR("Solver only supports 3DOF groups, you gave " << group.second.size() << "DOF group");
                    return;
                }
                if (group_structs_[group.first].size() != 3)
                {
                    LOG_ERROR("Solver only supports 3DOF structures, you gave " << group_structs_[group.first].size() << "DOF structure");
                    return;
                }
                auto solution = iksolver3d(msg->x, msg->y, msg->z,
                                           group_structs_[group.first][0],
                                           group_structs_[group.first][1],
                                           group_structs_[group.first][2]);
                if (std::isnan(std::get<0>(solution)) || std::isnan(std::get<1>(solution)) || std::isnan(std::get<2>(solution)))
                {
                    LOG_ERROR("Failed to solve inverse kinematics for group: " << group.first);
                    return;
                }
                double q1 = std::get<0>(solution) + joint_offsets_[group.second[0]];
                double q2 = std::get<1>(solution) + joint_offsets_[group.second[1]];
                double q3 = std::get<2>(solution) + joint_offsets_[group.second[2]];
                sensor_msgs::msg::JointState joint_state;
                joint_state.name = group.second;
                joint_state.position = {q1, q2, q3};
                iksolver_pub_->publish(joint_state);
            }
        }
        std::string config_file_path_;
        std::map<std::string, std::vector<std::string>> group_joints_;
        std::map<std::string, std::vector<double>> group_structs_;
        std::map<std::string, double> joint_min_positions_;
        std::map<std::string, double> joint_max_positions_;
        std::map<std::string, double> joint_offsets_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr waypoint_sub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr iksolver_pub_;
    };
} // namespace cerberus_iksolver

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cerberus_iksolver::IKSolver>());
    rclcpp::shutdown();
    return 0;
}
