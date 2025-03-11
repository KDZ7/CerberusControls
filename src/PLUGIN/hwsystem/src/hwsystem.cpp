#include <cmath>
#include "hwsystem/hwsystem.hpp"
#include "yaml-cpp/yaml.h"
#include "pluginlib/class_loader.hpp"

#define DEFAULT_MOVETIME 25.0
#define DEFAULT_TIMEOUT 25.0
#define DEFAULT_MIN_POSITION -2.0944
#define DEFAULT_MAX_POSITION 2.0944

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
            YAML::Node hwconfig = YAML::LoadFile(config_file_path);
            if (!hwconfig["idmap"])
            {
                LOG_ERROR("idmap section missing in configuration file");
                return CallbackReturn::ERROR;
            }

            YAML::Node idmap = hwconfig["idmap"];
            LOG_INFO("Joint ID mapping:");
            for (const auto &joint : idmap)
            {
                joint_idmap[joint.first.as<std::string>()] = joint.second.as<uint8_t>();
                LOG_INFO("> Joint: " << joint.first.as<std::string>() << " <=> ID: " << (int)joint.second.as<uint8_t>());
            }

            LOG_INFO("Loaded (" << joint_idmap.size() << ") joint ID mappings from configuration file: " << config_file_path);
            LOG_INFO("Joint settings:");

            for (const auto &joint : info.joints)
            {
                if (!joint.state_interfaces.empty())
                    joint_state_positions_[joint.name] = 0.0;
                if (!joint.command_interfaces.empty())
                {
                    joint_command_positions_[joint.name] = 0.0;
                    for (const auto &limit : joint.command_interfaces)
                    {
                        if (limit.name == hardware_interface::HW_IF_POSITION)
                        {
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
                        }
                    }
                }
            }
            if (hwconfig["home_positions"])
            {
                YAML::Node offsets = hwconfig["home_positions"];
                for (const auto &joint : offsets)
                {
                    home_positions_[joint.first.as<std::string>()] = joint.second.as<double>();
                    LOG_INFO("> Joint: " << joint.first.as<std::string>() << " home position: " << joint.second.as<double>());
                }
            }
            else
                for (const auto &map : joint_idmap)
                    home_positions_[map.first] = 0.0;
            if (hwconfig["movetime"])
            {
                state_movetime_ = hwconfig["movetime"].as<double>();
                command_movetime_ = state_movetime_;
                LOG_INFO("> State move time: " << state_movetime_);
            }
            else
            {
                state_movetime_ = DEFAULT_MOVETIME;
                command_movetime_ = state_movetime_;
                LOG_WARN("> Movetime not found, using default: " << state_movetime_);
            }
            if (hwconfig["timeout"])
            {
                timeout_ = hwconfig["timeout"].as<double>();
                LOG_INFO("> Timeout: " << timeout_);
            }
            else
            {
                timeout_ = DEFAULT_TIMEOUT;
                LOG_WARN("> Timeout not found, using default: " << timeout_);
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
            LOG_INFO("Loading driver device ...");
            pluginlib::ClassLoader<idevice::Servo> loader("idevice", "idevice::Servo");
            servo = loader.createSharedInstance("lsc_servo::Lsc_servo");

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
        for (const auto &joint : joint_idmap)
        {
            joint_state_positions_[joint.first] = home_positions_.at(joint.first);
            LOG_INFO("Home position joint: " << joint.first << " Position: " << joint_state_positions_.at(joint.first));
        }
        LOG_INFO("Hwsystem activated successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Hwsystem::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        if (servo)
        {
            std::vector<std::tuple<uint8_t, double>> servos;
            for (const auto &joint : joint_idmap)
            {
                servos.push_back({joint.second, home_positions_.at(joint.first)});
                LOG_INFO("Go to home position joint: " << joint.first << " Position: " << home_positions_.at(joint.first));
            }
            if (!servos.empty())
                servo->moveServo(servos, timeout_);
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
        joint_max_positions_.clear();
        joint_min_positions_.clear();
        home_positions_.clear();
        joint_idmap.clear();
        servo.reset();

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

    hardware_interface::return_type Hwsystem::read([[maybe_unused]] const rclcpp::Time &time, [[maybe_unused]] const rclcpp::Duration &period)
    {
        if (!servo)
        {
            LOG_ERROR("Driver device not loaded");
            return hardware_interface::return_type::ERROR;
        }

        try
        {
            std::vector<uint8_t> servo_ids;
            servo_ids.reserve(joint_idmap.size());
            for (const auto &map : joint_idmap)
                servo_ids.push_back(map.second);

            std::map<uint8_t, double> state_servos = servo->stateServo(servo_ids, (uint16_t)timeout_);
            for (const auto &map : joint_idmap)
            {
                auto servo_state = state_servos.find(map.second);
                if (servo_state != state_servos.end())
                {
                    joint_state_positions_.at(map.first) = servo_state->second;
                    state_movetime_ = command_movetime_;
                    LOG_INFO("> Read joint: " << map.first
                                              << " | Position: " << joint_state_positions_.at(map.first)
                                              << " | Timeout: " << timeout_);
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
            for (const auto &joint : joint_command_positions_)
            {
                auto servo_id = joint_idmap.find(joint.first);
                if (servo_id != joint_idmap.end())
                {
                    double target_position = joint_command_positions_.at(joint.first);
                    target_position = std::max(
                        joint_min_positions_.at(joint.first),
                        std::min(joint_max_positions_.at(joint.first), target_position));
                    servo_commands.push_back({servo_id->second, target_position});
                    LOG_INFO("> Write joint: " << joint.first
                                               << " | Position: " << target_position
                                               << " | Time: " << command_movetime_);
                }
            }
            if (!servo_commands.empty())
                servo->moveServo(servo_commands, (uint16_t)command_movetime_);
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
        state_interfaces.emplace_back(
            std::make_shared<hardware_interface::StateInterface>(
                "movetime",
                "time",
                &state_movetime_));
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
        }
        command_interfaces.emplace_back(
            std::make_shared<hardware_interface::CommandInterface>(
                "movetime",
                "time",
                &command_movetime_));
        LOG_INFO("Exported (" << command_interfaces.size() << ") command interfaces");
        return command_interfaces;
    }
} // namespace hwsystem

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hwsystem::Hwsystem, hardware_interface::SystemInterface)