#include "gmotion/gmotion.hpp"

namespace cerberus_gait
{
    GMotion::GMotion() : LifecycleNode("gmotion")
    {
        this->declare_parameter("config_file_path", "");
        LOG_INFO("GMotion created.");
    }

    CallbackReturn GMotion::on_configure([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        LOG_INFO("GMotion configuring ...");
        config_file_path_ = this->get_parameter("config_file_path").as_string();

        if (config_file_path_.empty())
        {
            LOG_ERROR("<config_file_path> parameter must be set.");
            return CallbackReturn::FAILURE;
        }
        if (!loadConfig())
        {
            LOG_ERROR("Failed to load configuration.");
            return CallbackReturn::FAILURE;
        }
        LOG_INFO("GMotion configured successfully.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn GMotion::on_activate([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        LOG_INFO("GMotion activating ...");

        command_sub_ = this->create_subscription<std_msgs::msg::UInt8>("~/command", rclcpp::QoS(10), std::bind(&GMotion::command_callback, this, std::placeholders::_1));
        solutions_sub_ = this->create_subscription<msgroup::msg::GroupState>("~/solutions", rclcpp::QoS(10), std::bind(&GMotion::solutions_callback, this, std::placeholders::_1));
        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("~/in", rclcpp::QoS(10), std::bind(&GMotion::joint_states_callback, this, std::placeholders::_1));
        joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("~/out", rclcpp::QoS(10));
        LOG_INFO("GMotion activated successfully.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn GMotion::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        LOG_INFO("GMotion deactivating ...");

        command_sub_.reset();
        solutions_sub_.reset();
        joint_states_sub_.reset();
        joint_states_pub_.reset();
        if (executor_)
        {
            executor_->cancel();
            executor_.reset();
        }
        LOG_INFO("GMotion deactivated successfully.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn GMotion::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        LOG_INFO("GMotion cleaning up ...");

        gaits_.clear();

        LOG_INFO("GMotion cleaned up successfully.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn GMotion::on_error([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        LOG_INFO("GMotion encountered an error.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn GMotion::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        LOG_INFO("GMotion shutdown successfully.");
        return CallbackReturn::SUCCESS;
    }

    bool GMotion::loadConfig()
    {
        LOG_INFO("GMotion loading configuration from: " << config_file_path_);
        YAML::Node config = YAML::LoadFile(config_file_path_);
        try
        {
            if (!config["gaits"])
            {
                LOG_ERROR("No gaits found in configuration.");
                return false;
            }
            YAML::Node gaits = config["gaits"];
            for (const auto &gait : gaits)
            {
                std::string gait_name = gait.first.as<std::string>();
                YAML::Node content = gait.second;
                gait_t new_gait;
                new_gait.name = gait_name;
                for (const auto &sequence : content)
                {
                    if (sequence["goto"])
                    {
                        new_gait.goto_gait = sequence["goto"].as<std::string>();
                        continue;
                    }
                    sequence_t new_sequence;
                    if (sequence["sync"])
                        for (const auto &leg : sequence["sync"])
                            new_sequence.sync_legs.push_back(leg.as<std::string>());
                    if (sequence["step"])
                        for (const auto &step : sequence["step"])
                        {
                            step_t new_step;
                            new_step.leg = step["leg"].as<std::string>();
                            new_step.x = step["x"].as<double>();
                            new_step.y = step["y"].as<double>();
                            new_step.z = step["z"].as<double>();
                            new_step.wait = step["wait"].as<double>();
                            auto it = std::find(new_sequence.sync_legs.begin(), new_sequence.sync_legs.end(), new_step.leg);
                            if (it != new_sequence.sync_legs.end())
                                new_sequence.steps.push_back(new_step);
                            else
                                LOG_WARN("Leg " << new_step.leg << " not in sync list for this sequence. Ignoring step.");
                        }
                    new_gait.sequences.push_back(new_sequence);
                }
                gaits_[new_gait.name] = new_gait;
                LOG_INFO("> Gait loaded [ " << new_gait.name << " ]");
            }
            if (config["solver"])
            {
                solver_name_ = config["solver"].as<std::string>();
                LOG_INFO("> Solver loaded [ " << solver_name_ << " ]");
            }
            else
            {
                LOG_WARN("No solver block found in configuration. Default solver will be used.");
                solver_name_ = "GRID";
                LOG_INFO("> Solver loaded [ " << solver_name_ << " ]");
            }
            LOG_INFO("GMotion configuration loaded successfully.");
            return true;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("Failed to load configuration: " << e.what());
            return false;
        }
    }

    void GMotion::command_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        LOG_INFO("GMotion command received: " << int(msg->data));
        switch (msg->data)
        {
        case GAIT_STOP:
            LOG_INFO("GMotion: action [stop] is requesting ...");
            action(gait_t{"stop", {}, ""});
            break;
        case GAIT_STAND:
            LOG_INFO("GMotion: action [stand] is requesting ...");
            action(gaits_["stand"]);
            break;
        case GAIT_SIT:
            LOG_INFO("GMotion: action [sit] is requesting ...");
            action(gaits_["sit"]);
            break;
        case GAIT_CRAWL:
            LOG_INFO("GMotion: action [crawl] is requesting ...");
            action(gaits_["crawl"]);
            break;
        case GAIT_WALK:
            LOG_INFO("GMotion: action [walk] is requesting ...");
            action(gaits_["walk"]);
            break;
        case GAIT_TROT:
            LOG_INFO("GMotion: action [trot] is requesting ...");
            action(gaits_["trot"]);
            break;
        case GAIT_BOUND:
            LOG_INFO("GMotion: action [bound] is requesting ...");
            action(gaits_["bound"]);
            break;
        case GAIT_GALLOP:
            LOG_INFO("GMotion: action [gallop] is requesting ...");
            action(gaits_["gallop"]);
            break;
        case GAIT_RAMP:
            LOG_INFO("GMotion: action [ramp] is requesting ...");
            action(gaits_["ramp"]);
            break;
        case GAIT_TURN_YAW:
            LOG_INFO("GMotion: action [turn_yaw] is requesting ...");
            action(gaits_["turn_yaw"]);
            break;
        case GAIT_SIDE_RIGHT:
            LOG_INFO("GMotion: action [side_right] is requesting ...");
            action(gaits_["side_right"]);
            break;
        case GAIT_SIDE_LEFT:
            LOG_INFO("GMotion: action [side_left] is requesting ...");
            action(gaits_["side_left"]);
            break;
        case GAIT_BACKWARD:
            LOG_INFO("GMotion: action [backward] is requesting ...");
            action(gaits_["backward"]);
            break;
        case GAIT_SNIFF:
            LOG_INFO("GMotion: action [sniff] is requesting ...");
            action(gaits_["sniff"]);
            break;
        default:
            LOG_ERROR("GMotion: unknown action requested.");
            break;
        }
    }
    void GMotion::solutions_callback(const msgroup::msg::GroupState::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(solutions_mutex_);
            solutions_cache_[msg->group] = *msg;
        }
        LOG_INFO("GMotion: solutions updated cache [ " << msg->group << " ]");
    }
    void GMotion::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(solutions_mutex_);
        if (solutions_cache_.empty())
            return;
        sensor_msgs::msg::JointState joint_state = *msg;
        for (auto &solution : solutions_cache_)
        {
            const auto &group_state = solution.second;
            for (size_t i = 0; i < joint_state.name.size(); i++)
            {
                auto it = std::find(group_state.name.begin(), group_state.name.end(), joint_state.name[i]);
                if (it != group_state.name.end())
                {
                    joint_state.position[i] = group_state.position[std::distance(group_state.name.begin(), it)];
                    joint_state.velocity[i] = group_state.velocity[std::distance(group_state.name.begin(), it)];
                    joint_state.effort[i] = group_state.effort[std::distance(group_state.name.begin(), it)];
                }
            }
        }

        joint_states_pub_->publish(joint_state);
    }
    void GMotion::action(const gait_t &gait)
    {
        if (gait.name == "stop")
        {
            if (executor_)
            {
                executor_->cancel();
                executor_.reset();
            }
            LOG_INFO("GMotion: action [stop] completed.");
            return;
        }

        LOG_INFO("GMotion: action [ " << gait.name << " ] preparing ...");

        rclcpp::NodeOptions options;
        options.automatically_declare_parameters_from_overrides(true);
        auto backend = std::make_shared<rclcpp::Node>("gmotion", "backend", options);
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        auto waypoint_pub = backend->create_publisher<mswaypoint::msg::Waypoint>("~/waypoint", rclcpp::QoS(10));
        executor_->add_node(backend);

        std::thread([this, gait, waypoint_pub, backend]() -> void
                    { 
        std::string name = gait.name;
        while(rclcpp::ok() && executor_) {
            auto it = gaits_.find(name);
            if (it == gaits_.end())
            {
                LOG_ERROR("Gait [ " << name << " ] not found. Aborting sequence.");
                break;
            }
            const gait_t &current_gait = it->second;
            for (const auto &sequence : current_gait.sequences)
            {
                if(!rclcpp::ok() || !executor_) break;
                std::map<std::string, std::vector<step_t>> leg_steps;
                for (const auto &step : sequence.steps)
                    leg_steps[step.leg].push_back(step);

                size_t max_steps = 0;
                for (const auto &pair : leg_steps) 
                    if (pair.second.size() > max_steps) 
                        max_steps = pair.second.size();
                
                for (size_t step_idx = 0; step_idx < max_steps; step_idx++)
                {
                    if(!rclcpp::ok() || !executor_) break;
                    std::vector<mswaypoint::msg::Waypoint> waypoints;
                    double max_wait = 0.0;
                    for (const auto &leg_name : sequence.sync_legs)
                    {
                        auto it = leg_steps.find(leg_name);
                        if (it != leg_steps.end() && step_idx < it->second.size()) {
                            const step_t &step = it->second[step_idx];
                            mswaypoint::msg::Waypoint waypoint;
                            waypoint.group = leg_name;
                            waypoint.solver = solver_name_;
                            waypoint.x = step.x;
                            waypoint.y = step.y;
                            waypoint.z = step.z;
                            waypoints.push_back(waypoint);
                            max_wait = std::max(max_wait, step.wait);
                            LOG_INFO("Prepare step: [ " << name << " ] --> [ " << leg_name << " ] : ( " << step.x << ", " << step.y << ", " << step.z << " )");
                        }
                    }
                    if (!waypoints.empty() && executor_)
                    {
                        if(!rclcpp::ok() || !executor_) break;

                        for (const auto &waypoint : waypoints)
                            waypoint_pub->publish(waypoint);

                        if(max_wait > 0 && executor_)
                            rclcpp::sleep_for(std::chrono::milliseconds(int(max_wait * 1000)));
                    }
                }
            }

            if(!current_gait.goto_gait.empty())
                name = current_gait.goto_gait;
            else 
                break;
        }
        LOG_INFO("GMotion: execution completed."); })
            .detach();
    }
} // namespace cerberus_gait