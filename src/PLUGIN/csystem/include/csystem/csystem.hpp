#ifndef __CSYSTEM_HPP__
#define __CSYSTEM_HPP__

#include "log/log.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "controller_interface/controller_interface.hpp"
#include "csystem/visibility_control.h"

namespace csystem
{

  // class Csystem : public controller_interface::ControllerInterface
  // {
  // public:
  //   controller_interface::CallbackReturn on_init() override;
  //   controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  //   controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  //   controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  //   controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
  //   controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
  //   controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
  //   controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  //   controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  //   controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  // private:
  //   std::unordered_map<std::string, std::vector<std::string>> joint_configs_;
  //   std::unordered_map<std::string, std::vector<double>> joint_offsets_;
  //   std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState>> joint_command_sub_;
  //   std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> rt_joint_state_pub_;
  //   realtime_tools::RealtimeBuffer<std::vector<double>> rt_command_buffer_;
  //   std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> command_interfaces_;
  //   std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> state_interfaces_;
  //   void command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

} // namespace csystem

#endif // __CSYSTEM_HPP__
