#include "hwsystem/hwsystem.hpp"
#include <yaml-cpp/yaml.h>
#include "pluginlib/class_loader.hpp"

#define DEFAULT_MIN_POSITION -2.0944
#define DEFAULT_MAX_POSITION 2.0944
#define DEFAULT_MIN_VELOCITY 0.0
#define DEFAULT_MAX_VELOCITY 5.8178
#define DEFAULT_COMMON_POSITION 0.0
#define DEFAULT_COMMON_TIME 100

namespace hwsystem
{
    CallbackReturn Hwsystem::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        try
        {
            if (info_.hardware_parameters.find("config_file_path") == info_.hardware_parameters.end())
            {
                LOG_ERROR("Parameter 'config_file_path' missing in configuration");
                return CallbackReturn::ERROR;
            }

            config_file_path = info_.hardware_parameters.at("config_file_path");

            YAML::Node config = YAML::LoadFile(config_file_path);
            if (!config["idmap"])
            {
                LOG_ERROR("idmap section missing in configuration file");
                return CallbackReturn::ERROR;
            }

            YAML::Node idmap = config["idmap"];
            LOG_INFO("Joint ID mapping:");
            for (const auto &joint : idmap)
            {
                joint_idx_map[joint.first.as<std::string>()] = joint.second.as<uint8_t>();
                LOG_INFO("> Joint: " << joint.first.as<std::string>() << " <=> ID: " << (int)joint.second.as<uint8_t>());
            }

            LOG_INFO("Loaded (" << joint_idx_map.size() << ") joint ID mappings from configuration file: " << config_file_path);

            LOG_INFO("Joint settings:");

            for (const auto &joint : info.joints)
            {
                if (!joint.state_interfaces.empty())
                    joint_state_positions_[joint.name] = 0.0;
                if (!joint.command_interfaces.empty())
                {
                    joint_command_positions_[joint.name] = 0.0;
                    joint_command_velocities_[joint.name] = 0.0;
                    last_joint_command_positions_[joint.name] = 0.0;

                    for (const auto &limit : joint.command_interfaces)
                    {
                        if (limit.name == hardware_interface::HW_IF_POSITION)
                        {
                            if (limit.parameters.find("min") != limit.parameters.end())
                            {
                                joint_min_positions_[joint.name] = std::stod(limit.parameters.at("min"));
                                LOG_INFO("> Joint: " << joint.name << " Min Position: " << joint_min_positions_[joint.name]);
                            }
                            else
                            {
                                joint_min_positions_[joint.name] = DEFAULT_MIN_POSITION;
                                LOG_WARN("> Joint: " << joint.name << " Min Position not found, using default: " << joint_min_positions_[joint.name]);
                            }

                            if (limit.parameters.find("max") != limit.parameters.end())
                            {
                                joint_max_positions_[joint.name] = std::stod(limit.parameters.at("max"));
                                LOG_INFO("> Joint: " << joint.name << " Max Position: " << joint_max_positions_[joint.name]);
                            }
                            else
                            {
                                joint_max_positions_[joint.name] = DEFAULT_MAX_POSITION;
                                LOG_WARN("> Joint: " << joint.name << " Max Position not found, using default: " << joint_max_positions_[joint.name]);
                            }
                        }
                        if (limit.name == hardware_interface::HW_IF_VELOCITY)
                        {
                            if (limit.parameters.find("min") != limit.parameters.end())
                            {
                                joint_min_velocities_[joint.name] = std::stod(limit.parameters.at("min"));
                                LOG_INFO("> Joint: " << joint.name << " Min Velocity: " << joint_min_velocities_[joint.name]);
                            }
                            else
                            {
                                joint_min_velocities_[joint.name] = DEFAULT_MIN_VELOCITY;
                                LOG_WARN("> Joint: " << joint.name << " Min Velocity not found, using default: " << joint_min_velocities_[joint.name]);
                            }

                            if (limit.parameters.find("max") != limit.parameters.end())
                            {
                                joint_max_velocities_[joint.name] = std::stod(limit.parameters.at("max"));
                                LOG_INFO("> Joint: " << joint.name << " Max Velocity: " << joint_max_velocities_[joint.name]);
                            }
                            else
                            {
                                joint_max_velocities_[joint.name] = DEFAULT_MAX_VELOCITY;
                                LOG_WARN("> Joint: " << joint.name << " Max Velocity not found, using default: " << joint_max_velocities_[joint.name]);
                            }
                        }
                    }
                }
            }
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("Failed to load configuration file: " << config_file_path << " with exception: " << e.what());
            return CallbackReturn::ERROR;
        }

        LOG_INFO("Hwsystem initialized successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Hwsystem::on_configure([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        try
        {
            pluginlib::ClassLoader<idevice::Servo> loader("idevice", "idevice::Servo");
            servo = loader.createSharedInstance("lsc_servo::Lsc_servo");

            LOG_INFO("Driver device loaded successfully");

            if (!servo->connect())
            {
                LOG_ERROR("Failed to connect to device");
                return CallbackReturn::ERROR;
            }
            LOG_INFO("Driver device connected successfully");
        }
        catch (const pluginlib::PluginlibException &e)
        {
            LOG_ERROR("Failed to load the driver device with exception: " << e.what());
            return CallbackReturn::ERROR;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("Failed to connect to device with exception: " << e.what());
            return CallbackReturn::ERROR;
        }

        LOG_INFO("Hwsystem configured successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Hwsystem::on_activate([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        LOG_INFO("Hwsystem activated successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Hwsystem::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        if (servo)
        {
            std::vector<std::tuple<uint8_t, double>> servos;
            for (const auto &joint : joint_idx_map)
                servos.push_back({joint.second, DEFAULT_COMMON_POSITION});
            if (!servos.empty())
            {
                servo->moveServo(servos, DEFAULT_COMMON_TIME);
                LOG_INFO("Reset all servos to common position: " << DEFAULT_COMMON_POSITION);
            }
        }

        LOG_INFO("Hwsystem deactivated successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Hwsystem::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        if (servo && servo->isConnected())
        {
            servo->disconnect();
            LOG_INFO("Driver device disconnected during cleanup");
        }
        joint_state_positions_.clear();
        joint_command_positions_.clear();
        joint_command_velocities_.clear();
        joint_max_positions_.clear();
        joint_min_positions_.clear();
        joint_min_velocities_.clear();
        joint_max_velocities_.clear();
        last_joint_command_positions_.clear();

        LOG_INFO("Hwsystem cleaned up successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Hwsystem::on_error([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        LOG_ERROR("Hwsystem encountered an error");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Hwsystem::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        LOG_INFO("Hwsystem shutdown successfully");
        return CallbackReturn::SUCCESS;
    }

    // hardware_interface::return_type Hwsystem::read([[maybe_unused]] const rclcpp::Time &time, [[maybe_unused]] const rclcpp::Duration &period)
    // {
    //     if (!servo)
    //     {
    //         LOG_ERROR("Driver device not loaded");
    //         return hardware_interface::return_type::ERROR;
    //     }

    //     try
    //     {
    //         std::vector<uint8_t> servo_ids;
    //         servo_ids.reserve(joint_idx_map.size());
    //         for (const auto &joint : joint_idx_map)
    //             servo_ids.push_back(joint.second);

    //         std::map<uint8_t, double> state_servos = servo->stateServo(servo_ids);
    //         for (const auto &joint : joint_idx_map)
    //         {
    //             auto servo_state = state_servos.find(joint.second);
    //             if (servo_state != state_servos.end())
    //                 joint_state_positions_.at(joint.first) = servo_state->second;
    //         }
    //     }
    //     catch (const std::out_of_range &e)
    //     {
    //         LOG_ERROR("Out of range error during read: " << e.what());
    //         return hardware_interface::return_type::ERROR;
    //     }
    //     catch (const std::exception &e)
    //     {
    //         LOG_ERROR("Read failed with exception: " << e.what());
    //         return hardware_interface::return_type::ERROR;
    //     }

    //     return hardware_interface::return_type::OK;
    // }

    // Juste for test one leg
    hardware_interface::return_type Hwsystem::read([[maybe_unused]] const rclcpp::Time &time, [[maybe_unused]] const rclcpp::Duration &period)
    {
        if (!servo)
        {
            LOG_ERROR("Driver device not loaded");
            return hardware_interface::return_type::ERROR;
        }

        try
        {
            std::vector<uint8_t> servo_ids = {31, 32, 33};
            servo_ids.reserve(joint_idx_map.size());
            for (const auto &joint : joint_idx_map)
                servo_ids.push_back(joint.second);

            std::map<uint8_t, double> state_servos = servo->stateServo(servo_ids);
            for (const auto &joint : joint_idx_map)
            {
                auto servo_state = state_servos.find(joint.second);
                if (servo_state != state_servos.end())
                {
                    joint_state_positions_.at(joint.first) = servo_state->second;
                    // LOG_INFO("> Joint (" << joint.first << ") ID (" << (int)joint.second << ") position: " << joint_state_positions_.at(joint.first));
                }
                else
                {
                    joint_state_positions_.at(joint.first) = joint_command_positions_.at(joint.first);
                    // LOG_WARN("> Just dummy value joint (" << joint.first << ") ID (" << (int)joint.second << ") position: " << joint_state_positions_.at(joint.first));
                }
            }
        }
        catch (const std::out_of_range &e)
        {
            LOG_ERROR("Out of range error during read: " << e.what());
            return hardware_interface::return_type::ERROR;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("Read failed with exception: " << e.what());
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type Hwsystem::write([[maybe_unused]] const rclcpp::Time &time, [[maybe_unused]] const rclcpp::Duration &period)
    {
        if (!servo)
        {
            LOG_ERROR("Driver device not loaded");
            return hardware_interface::return_type::ERROR;
        }

        try
        {
            std::vector<std::tuple<uint8_t, double>> servo_commands;
            servo_commands.reserve(joint_command_positions_.size());
            uint16_t move_time = DEFAULT_COMMON_TIME;

            for (const auto &joint : joint_command_positions_)
            {
                auto servo_id = joint_idx_map.find(joint.first);
                if (servo_id != joint_idx_map.end())
                {
                    double current_position = joint_state_positions_.at(joint.first);
                    double target_position = joint_command_positions_.at(joint.first);
                    double velocity = joint_command_velocities_.at(joint.first);

                    target_position = std::max(
                        joint_min_positions_.at(joint.first),
                        std::min(joint_max_positions_.at(joint.first), target_position));

                    velocity = std::max(
                        joint_min_velocities_.at(joint.first),
                        std::min(joint_max_velocities_.at(joint.first), velocity));

                    if (velocity > 0)
                    {
                        double delta_position = std::abs(target_position - current_position);
                        move_time = static_cast<uint16_t>(std::max(
                            (delta_position / velocity) * 1000.0,
                            static_cast<double>(DEFAULT_COMMON_TIME)));
                    }

                    LOG_INFO("> Joint (" << joint.first << ") ID (" << (int)servo_id->second << ") position: " << target_position << " velocity: " << velocity << " move time: " << move_time);
                    servo_commands.push_back({servo_id->second, target_position});
                }
            }

            if (!servo_commands.empty())
                servo->moveServo(servo_commands, move_time);
        }
        catch (const std::out_of_range &e)
        {
            LOG_ERROR("Out of range error during write: " << e.what());
            return hardware_interface::return_type::ERROR;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("Write failed with exception: " << e.what());
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> Hwsystem::on_export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
        for (const auto &joint : joint_state_positions_)
        {
            state_interfaces.emplace_back(
                std::make_shared<hardware_interface::StateInterface>(
                    joint.first,
                    hardware_interface::HW_IF_POSITION,
                    &joint_state_positions_.at(joint.first)));
        }

        LOG_INFO("Exported (" << state_interfaces.size() << ") state interfaces");
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr> Hwsystem::on_export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
        for (const auto &joint : joint_command_positions_)
        {
            command_interfaces.emplace_back(
                std::make_shared<hardware_interface::CommandInterface>(
                    joint.first,
                    hardware_interface::HW_IF_POSITION,
                    &joint_command_positions_.at(joint.first)));
            command_interfaces.emplace_back(
                std::make_shared<hardware_interface::CommandInterface>(
                    joint.first,
                    hardware_interface::HW_IF_VELOCITY,
                    &joint_command_velocities_.at(joint.first)));
        }

        LOG_INFO("Exported (" << command_interfaces.size() << ") command interfaces");
        return command_interfaces;
    }
} // namespace hwsystem

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hwsystem::Hwsystem, hardware_interface::SystemInterface)