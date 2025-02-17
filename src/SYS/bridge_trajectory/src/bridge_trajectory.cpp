#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "cerberus_msgs/msg/joint_trajectory_array.hpp"

class Bridge_Trajectory : public rclcpp::Node
{
public:
    Bridge_Trajectory() : Node("bridge_trajectory")
    {
        this->declare_parameter("mimic_joints", std::vector<std::string>({"ShFL_ThFL", "ShFR_ThFR", "ShBL_ThBL", "ShBR_ThBR"}));
        this->declare_parameter("master_joints", std::vector<std::string>({"ArFL_ThFL", "ArFR_ThFR", "ArBL_ThBL", "ArBR_ThBR"}));
        this->declare_parameter("mimic_factors", std::vector<double>({-0.2824, -0.2824, -0.2824, -0.2824}));
        this->declare_parameter("period", 0.1);

        auto mimic_joints = this->get_parameter("mimic_joints").as_string_array();
        auto master_joints = this->get_parameter("master_joints").as_string_array();
        auto mimic_factors = this->get_parameter("mimic_factors").as_double_array();

        if (mimic_joints.size() != master_joints.size() || mimic_joints.size() != mimic_factors.size())
        {
            RCLCPP_ERROR(this->get_logger(), "Inconsistent mimic joints, master joints, and mimic factors.");
            throw std::runtime_error("Inconsistent mimic joints, master joints, and mimic factors.");
        }

        for (size_t i = 0; i < mimic_joints.size(); ++i)
        {
            mimic_factor_[mimic_joints[i]] = {master_joints[i], mimic_factors[i]};
            RCLCPP_INFO(this->get_logger(), "Mimic joint configured: %s -> %s (factor: %f)", mimic_joints[i].c_str(), master_joints[i].c_str(), mimic_factors[i]);
        }

        period_ = this->get_parameter("period").as_double();
        joint_trajectory_sub_ = this->create_subscription<cerberus_msgs::msg::JointTrajectoryArray>("/ik_solver/joint_trajectories", 10, std::bind(&Bridge_Trajectory::joint_trajectories_callback, this, std::placeholders::_1));
        position_controller_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_group_position_controller/commands", 10);
        joint_trajectory_controller_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);
        timer_ = this->create_wall_timer(std::chrono::duration<double>(period_), std::bind(&Bridge_Trajectory::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Bridge Trajectory has been started with period %f.", period_);
    }

private:
    void joint_trajectories_callback(const cerberus_msgs::msg::JointTrajectoryArray::SharedPtr msg)
    {
        if (msg->joint_trajectories.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty joint trajectories.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Received new following waypoints with %zu trajectories.", msg->joint_trajectories.size());
        joint_trajectories_ = msg->joint_trajectories;
        current_trajectory_ = joint_trajectories_.begin();
    }

    void timer_callback()
    {
        if (joint_trajectories_.empty() || current_trajectory_ == joint_trajectories_.end())
        {
            RCLCPP_INFO(this->get_logger(), "No waypoints to follow.");
            return;
        }
        if (current_trajectory_->q_values.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Current trajectory has no joint values.");
            ++current_trajectory_;
            return;
        }

        size_t num_joints = current_trajectory_->joint_names.size();

        if (current_trajectory_->q_values.size() != num_joints)
        {
            RCLCPP_ERROR(this->get_logger(), "inconsistent trajectory data.");
            ++current_trajectory_;
            return;
        }

        std::vector<double> processed_q_values = current_trajectory_->q_values;

        for (size_t i = 0; i < num_joints; ++i)
        {
            auto mimic_it = mimic_factor_.find(current_trajectory_->joint_names[i]);
            if (mimic_it != mimic_factor_.end())
            {
                auto master_joint = mimic_it->second.first;
                auto master_joint_it = std::find(current_trajectory_->joint_names.begin(), current_trajectory_->joint_names.end(), master_joint);
                if (master_joint_it != current_trajectory_->joint_names.end())
                {
                    size_t master_joint_index = std::distance(current_trajectory_->joint_names.begin(), master_joint_it);
                    processed_q_values[i] = current_trajectory_->q_values[master_joint_index] / mimic_it->second.second;
                }
            }
        }

        std_msgs::msg::Float64MultiArray joints_values;
        joints_values.data = processed_q_values;
        position_controller_pub_->publish(joints_values);

        trajectory_msgs::msg::JointTrajectory joint_trajectory;
        joint_trajectory.header = current_trajectory_->header;
        joint_trajectory.joint_names = current_trajectory_->joint_names;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = processed_q_values;
        point.velocities = current_trajectory_->q_velocities;
        point.accelerations = current_trajectory_->q_accelerations;
        point.time_from_start = current_trajectory_->time_from_start;

        joint_trajectory.points.push_back(point);
        joint_trajectory_controller_pub_->publish(joint_trajectory);

        ++current_trajectory_;

        if (current_trajectory_ == joint_trajectories_.end())
        {
            RCLCPP_INFO(this->get_logger(), "Completed following waypoints.");
            current_trajectory_ = joint_trajectories_.begin();
        }
    }

    double period_;
    std::map<std::string, std::pair<std::string, double>> mimic_factor_;
    rclcpp::Subscription<cerberus_msgs::msg::JointTrajectoryArray>::SharedPtr joint_trajectory_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_controller_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_controller_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<cerberus_msgs::msg::JointTrajectory> joint_trajectories_;
    std::vector<cerberus_msgs::msg::JointTrajectory>::iterator current_trajectory_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Bridge_Trajectory>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}