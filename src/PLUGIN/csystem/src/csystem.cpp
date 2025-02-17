// #include "csystem/csystem.hpp"

// namespace csystem
// {
//     controller_interface::CallbackReturn Csystem::on_init()
//     {
//         try
//         {
//             auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
//             auto_declare<std::string>("state_topic", "joint_states");
//             auto_declare<std::string>("command_topic", "joint_commands");
//         }
//         catch (const std::exception &e)
//         {
//             LOG_ERROR("Initialization failed: " << e.what());
//             return controller_interface::CallbackReturn::ERROR;
//         }

//         LOG_INFO("Csystem initialized successfully");
//         return controller_interface::CallbackReturn::SUCCESS;
//     }

//     controller_interface::CallbackReturn Csystem::on_configure([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
//     {
//         auto node = get_node();
//         try
//         {
//             auto joints = node->get_parameter("joints").as_string_array();
//             joint_configs_.clear();
//             joint_offsets_.clear();

//             for (const auto &joint : joints)
//             {
//                 auto interfaces = node->get_parameter(joint + ".interfaces").as_string_array();
//                 joint_configs_[joint] = interfaces;
//                 auto offsets = node->get_parameter(joint + ".offsets").as_double_array();
//                 joint_offsets_[joint] = offsets;
//             }
//             std::string state_topic = node->get_parameter("state_topic").as_string();
//             std::string command_topic = node->get_parameter("command_topic").as_string();

//             auto pub = node->create_publisher<sensor_msgs::msg::JointState>(state_topic, rclcpp::QoS(10));
//             rt_joint_state_pub_ = std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(pub);

//             joint_command_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(command_topic, 10, std::bind(&Csystem::command_callback, this, std::placeholders::_1));

//             LOG_INFO("Csystem configured successfully");
//             return controller_interface::CallbackReturn::SUCCESS;
//         }
//         catch (const std::exception &e)
//         {
//             LOG_ERROR("Configuration failed: " << e.what());
//             return controller_interface::CallbackReturn::ERROR;
//         }
//     }

//     controller_interface::CallbackReturn Csystem::on_activate([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
//     {
//         if (command_interfaces_.size() != joint_configs_.size())
//         {
//             LOG_ERROR("Number of command interfaces (" << command_interfaces_.size() << ") doesn't match number of configured joints (" << joint_configs_.size() << ")");
//             return controller_interface::CallbackReturn::ERROR;
//         }

//         if (state_interfaces_.size() != joint_configs_.size())
//         {
//             LOG_ERROR("Number of state interfaces (" << state_interfaces_.size() << ") doesn't match number of configured joints (" << joint_configs_.size() << ")");
//             return controller_interface::CallbackReturn::ERROR;
//         }

//         std::vector<double> initial_positions(joint_configs_.size());
//         for (size_t i = 0; i < state_interfaces_.size(); i++)
//         {
//             initial_positions[i] = state_interfaces_[i].get().get_value();
//         }

//         rt_command_buffer_.writeFromNonRT(initial_positions);

//         LOG_INFO("Csystem activated successfully");
//         return controller_interface::CallbackReturn::SUCCESS;
//     }

//     controller_interface::CallbackReturn Csystem::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
//     {
//         joint_command_sub_.reset();
//         rt_joint_state_pub_.reset();
//         rt_command_buffer_.reset();
//         command_interfaces_.clear();
//         state_interfaces_.clear();
//         LOG_INFO("Csystem deactivated successfully");
//         return controller_interface::CallbackReturn::SUCCESS;
//     }

//     controller_interface::CallbackReturn Csystem::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
//     {
//         joint_configs_.clear();
//         joint_offsets_.clear();
//         joint_command_sub_.reset();
//         rt_joint_state_pub_.reset();
//         rt_command_buffer_.reset();
//         command_interfaces_.clear();
//         state_interfaces_.clear();
//         LOG_INFO("Csystem cleaned up successfully");
//         return controller_interface::CallbackReturn::SUCCESS;
//     }

//     controller_interface::CallbackReturn Csystem::on_error([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
//     {
//         joint_command_sub_.reset();
//         rt_joint_state_pub_.reset();
//         rt_command_buffer_.reset();
//         command_interfaces_.clear();
//         state_interfaces_.clear();
//         LOG_ERROR("Csystem encountered an error");
//         return controller_interface::CallbackReturn::SUCCESS;
//     }

//     controller_interface::CallbackReturn Csystem::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
//     {
//         joint_configs_.clear();
//         joint_offsets_.clear();
//         joint_command_sub_.reset();
//         rt_joint_state_pub_.reset();
//         rt_command_buffer_.reset();
//         command_interfaces_.clear();
//         state_interfaces_.clear();

//         LOG_INFO("Csystem shutdown successfully");
//         return controller_interface::CallbackReturn::SUCCESS;
//     }

//     controller_interface::InterfaceConfiguration Csystem::command_interface_configuration() const
//     {
//         controller_interface::InterfaceConfiguration config;
//         config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

//         for (const auto &[joint, interfaces] : joint_configs_)
//             for (const auto &interface_type : interfaces)
//                 config.names.push_back(joint + "/" + interface_type);

//         LOG_INFO("Csystem command interface configured successfully");
//         return config;
//     }

//     controller_interface::InterfaceConfiguration Csystem::state_interface_configuration() const
//     {
//         controller_interface::InterfaceConfiguration config;
//         config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

//         for (const auto &[joint, interfaces] : joint_configs_)
//             config.names.push_back(joint + "/" + "position");

//         LOG_INFO("Csystem state interface configured successfully");
//         return config;
//     }

//     controller_interface::return_type Csystem::update([[maybe_unused]] const rclcpp::Time &time, [[maybe_unused]] const rclcpp::Duration &period)
//     {
//         auto command_positions = rt_command_buffer_.readFromRT();

//         if (!command_positions || command_positions->size() != joint_configs_.size())
//         {
//             LOG_ERROR("Failed to read command positions");
//             return controller_interface::return_type::ERROR;
//         }

//         for (size_t i = 0; i < command_interfaces_.size(); i++)
//         {
//             [[maybe_unused]] bool result = command_interfaces_[i].get().set_value((*command_positions)[i]);
//         }

//         if (rt_joint_state_pub_ && rt_joint_state_pub_->trylock())
//         {
//             auto &joint_state = rt_joint_state_pub_->msg_;
//             joint_state.header.stamp = time;
//             joint_state.name.clear();
//             joint_state.position.clear();

//             for (const auto &[joint, offsets] : joint_offsets_)
//             {
//                 double offset = offsets.empty() ? 0.0 : offsets[0];
//                 joint_state.name.push_back(joint);
//                 joint_state.position.push_back((*command_positions)[0] - offset);
//             }
//             rt_joint_state_pub_->unlockAndPublish();
//         }
//         return controller_interface::return_type::OK;
//     }

//     void Csystem::command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
//     {
//         if (msg->name.empty() || msg->position.empty())
//         {
//             LOG_WARN("Received empty command message");
//             return;
//         }

//         std::vector<double> commands(joint_configs_.size(), 0.0);

//         for (size_t i = 0; i < msg->name.size(); ++i)
//         {
//             auto joint_it = joint_configs_.find(msg->name[i]);
//             if (joint_it != joint_configs_.end())
//             {
//                 size_t joint_index = std::distance(std::begin(joint_configs_), joint_it);
//                 commands[joint_index] = msg->position[i];
//             }
//         }

//         rt_command_buffer_.writeFromNonRT(commands);
//     }
// } // namespace csystem

// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(csystem::Csystem, controller_interface::ControllerInterface)
