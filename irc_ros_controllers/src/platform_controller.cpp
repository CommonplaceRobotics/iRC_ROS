// Inspired by DIO Controller
// 
// Not a controller, only forwards the twist command published on /cmd_vel to the system_interface
#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "irc_ros_controllers/platform_controller.hpp"

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
// #include "tf2_msgs/msg/tf_message.hpp"

namespace irc_ros_controllers
{
    controller_interface::InterfaceConfiguration PlatformController::command_interface_configuration() const
    {
        std::vector<std::string> conf_names;

        for (std::string name : digital_outputs)
        {
            conf_names.emplace_back(name);
        }
        
        for (std::string name : commandedTwist_pltf)
        {
            conf_names.emplace_back(name);
        }
        
        for (std::string name : commandedTwist_arm)
        {
            conf_names.emplace_back(name);
        }

        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::InterfaceConfiguration PlatformController::state_interface_configuration() const
    {
        std::vector<std::string> conf_names;

        for (std::string name : digital_inputs)
        {
            conf_names.emplace_back(name);
        }

        for (std::string name : currentTwist)
        {
            conf_names.emplace_back(name);
        }
        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }
    
    void PlatformController::receive_velocity_callback_sub_(const geometry_msgs::msg::Twist & cmd)
    {
        int counter = 0;
        for (auto && ci : command_interfaces_)
        {   
            if(ci.get_name() == "platform/forward_cmd")
            {   
                ci.set_value(cmd.linear.x);
                continue;
            }
            if(ci.get_name() == "platform/lateral_cmd")
            {
                ci.set_value(cmd.linear.y);
                continue;
            }
            if(ci.get_name() == "platform/angular_cmd")
            {
                ci.set_value(cmd.angular.z);
                continue;
            }
            if(ci.get_name() == "rebel/lin_x")
            {   
                ci.set_value(cmd.linear.x);
                continue;
            }
            if(ci.get_name() == "rebel/lin_y")
            {
                ci.set_value(cmd.linear.y);
                continue;
            }
            if(ci.get_name() == "rebel/lin_z")
            {
                ci.set_value(cmd.linear.z);
                continue;
            }
            if(ci.get_name() == "rebel/rot_x")
            {
                ci.set_value(cmd.angular.x);
                continue;
            }
            if(ci.get_name() == "rebel/rot_y")
            {
                ci.set_value(cmd.angular.y);
                continue;
            }
            if(ci.get_name() == "rebel/rot_z")
            {
                ci.set_value(cmd.angular.z);
            }
        }
    }
    
    
    void PlatformController::set_outputs_sub_callback(const irc_ros_msgs::msg::DioCommand & cmd)
    {
        if (cmd.names.size() == cmd.outputs.size()) {
            // TODO: Find an elegant solution to iterate over cmd instead of command_interfaces_
            for (auto && ci : command_interfaces_) {
                
                if (cmd.names[0] == ci.get_name())
                {   
                    double value = cmd.outputs[0];
                    ci.set_value(value);
                }
            }
        } else {
            RCLCPP_ERROR(
            rclcpp::get_logger("iRC_ROS::PlatformController"), "Number of outputs and names are not equal");
        }
    }

    void PlatformController::set_motiontype_sub_callback(const irc_ros_msgs::msg::DioIntCommand & cmd)
    {
        if (cmd.names.size() == cmd.outputs.size()) {
            // TODO: Find an elegant solution to iterate over cmd instead of command_interfaces_
            for (auto && ci : command_interfaces_) {
                if (cmd.names[0] == ci.get_name())
                {   
                    int value = cmd.outputs[0];
                    ci.set_value(value);
                }
            }
        } else {
            RCLCPP_ERROR(
            rclcpp::get_logger("iRC_ROS::PlatformController"), "Number of outputs and names are not equal");
        }
    }


    controller_interface::CallbackReturn PlatformController::on_configure(const rclcpp_lifecycle::State & previous_state)
    {
        std::string name = get_node()->get_name();
        get_node()->get_parameter("curr_vels", currentTwist);
        get_node()->get_parameter("cmd_vels", commandedTwist_pltf);

        get_node()->get_parameter("cartesian_velocity_cmd", commandedTwist_arm);

        get_node()->get_parameter("digital_inputs", digital_inputs);
        get_node()->get_parameter("digital_outputs", digital_outputs);

        velocity_command_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
            ("~/cmd_vel"), 10,
        std::bind(
            &PlatformController::receive_velocity_callback_sub_, this, std::placeholders::_1));
        
        reset_enable_subscriber = get_node()->create_subscription<irc_ros_msgs::msg::DioCommand>(
            ("~/set_outputs"), 10,
        std::bind(
            &PlatformController::set_outputs_sub_callback, this, std::placeholders::_1));
        
        change_motiontype_subscriber = get_node()->create_subscription<irc_ros_msgs::msg::DioIntCommand>(
            ("~/set_motionType"), 10, 
        std::bind(
            &PlatformController::set_motiontype_sub_callback, this, std::placeholders::_1));

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PlatformController::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        is_halted = false;
        subscriber_is_active_ = true;

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PlatformController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {   
        subscriber_is_active_ = false;
        if (!is_halted)
        {
            halt();
            is_halted = true;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type PlatformController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
    {

        return controller_interface::return_type::OK;
    }
    
    
    void PlatformController::halt()
    {
        for (auto && ci : command_interfaces_)
        {   
            ci.set_value(0.0);
        }
    }

    controller_interface::CallbackReturn PlatformController::on_init()
    {
        return controller_interface::CallbackReturn::SUCCESS;
    } 

// bool subscriber_is_active_ = false;
// rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_command_subscriber_ = nullptr;

// realtime_tools::RealtimeBox<std::shared_ptr<geometry_msgs::msg::Twist>> received_velocity_msg_ptr_{nullptr};

// std::queue<geometry_msgs::msg::Twist> previous_commands_;  // last two commands

}  // namespace irc_ros_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  irc_ros_controllers::PlatformController, controller_interface::ControllerInterface)