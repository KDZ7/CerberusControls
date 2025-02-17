#include "hwsystem/hwsystem.hpp"

namespace hwsystem
{

    CallbackReturn Hwsystem::on_init(const hardware_interface::HardwareInfo &info)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Hwsystem::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Hwsystem::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Hwsystem::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Hwsystem::on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Hwsystem::on_error(const rclcpp_lifecycle::State &previous_state)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Hwsystem::on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type Hwsystem::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type Hwsystem::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface::ConstSharedPtr> Hwsystem::on_export_state_interfaces()
    {
        return {};
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr> Hwsystem::on_export_command_interfaces()
    {
        return {};
    }

} // namespace hwsystem

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hwsystem::Hwsystem, hardware_interface::SystemInterface)