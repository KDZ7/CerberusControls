#include <memory>
#include <vector>
#include <cmath>

#include "ik_solver/ik_solver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cerberus_msgs/msg/pose_trajectory_array.hpp"
#include "cerberus_msgs/msg/joint_trajectory_array.hpp"

class IK_Solver : public rclcpp::Node
{
public:
    IK_Solver() : Node("ik_solver")
    {
        this->declare_parameter("L1", 0.02355); // 23.55 mm
        this->declare_parameter("L2", 0.18111); // 181.11 mm
        this->declare_parameter("L3", 0.17912); // 179.12 mm

        L1_ = this->get_parameter("L1").as_double();
        L2_ = this->get_parameter("L2").as_double();
        L3_ = this->get_parameter("L3").as_double();

        gait_generator_sub_ = this->create_subscription<cerberus_msgs::msg::PoseTrajectoryArray>(
            "~/waypoints", 10, std::bind(&IK_Solver::gait_generator_callback, this, std::placeholders::_1));

        zmp_stabilizer_pub_ = this->create_publisher<cerberus_msgs::msg::JointTrajectoryArray>("~/joint_trajectories", 10);

        RCLCPP_INFO(this->get_logger(), "IK Solver has been started.");
        RCLCPP_INFO(this->get_logger(), "Leg Lengths: L1 = %f, L2 = %f, L3 = %f", L1_, L2_, L3_);
    }

private:
    void gait_generator_callback(const cerberus_msgs::msg::PoseTrajectoryArray::SharedPtr msg)
    {
        if (msg->pose_trajectories.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty pose trajectories.");
            return;
        }

        auto joint_trajectory_array = std::make_unique<cerberus_msgs::msg::JointTrajectoryArray>();
        joint_trajectory_array->header = msg->header;

        for (const auto &pose_trajectory : msg->pose_trajectories)
        {
            cerberus_msgs::msg::JointTrajectory joint_trajectory;
            joint_trajectory.header = pose_trajectory.header;
            joint_trajectory.joint_names = pose_trajectory.joint_names;
            joint_trajectory.time_from_start = pose_trajectory.time_from_start;

            for (const auto &pose : pose_trajectory.poses)
            {
                std::vector<double> q = cerberus_solver::solver(pose.linear.x, pose.linear.y, pose.linear.z, L1_, L2_, L3_);
                joint_trajectory.q_values.insert(joint_trajectory.q_values.end(), q.begin(), q.end());
                joint_trajectory.q_velocities.insert(joint_trajectory.q_velocities.end(), {0.0, 0.0, 0.0});
                joint_trajectory.q_accelerations.insert(joint_trajectory.q_accelerations.end(), {0.0, 0.0, 0.0});
            }

            joint_trajectory_array->joint_trajectories.push_back(joint_trajectory);
        }

        zmp_stabilizer_pub_->publish(std::move(joint_trajectory_array));
    }
    rclcpp::Subscription<cerberus_msgs::msg::PoseTrajectoryArray>::SharedPtr gait_generator_sub_;
    rclcpp::Publisher<cerberus_msgs::msg::JointTrajectoryArray>::SharedPtr zmp_stabilizer_pub_;

    double L1_, L2_, L3_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IK_Solver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}