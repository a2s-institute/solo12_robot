#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "include/solo12_h_interface.hpp"


namespace gazebo_ros2_control
{
    hardware_interface::CallbackReturn Solo12HardwareInterface::on_init(
        const hardware_interface::HardwareInfo & info)
    {
        if(hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS){
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    /*Here we can pass the parameters required for the hardware
    This will convert the string obtained from yml to double and 
    pass the value.*/

    // ??? At this sentence double should not be used,clarify,compare with example2 ???
    // Is this because we used it in the private section in the class 
    
    // std::vector<double> solo12_h_position_(info_.joints.size(),std::numeric_limits<double>::quiet_NaN());
    // std::vector<double> solo12_h_velocity_(info_.joints.size(),std::numeric_limits<double>::quiet_Nan());
    for(cosnt hardware_interface::ComponentInfo & joint : info_.joints){
        if(joint.command_interfaces.size()!=1)
        { 
            RCLCPP_FATAL(rclcpp::get_logger("Solo12HardwareInterface"),"Joint '%s' has %zu command interfaces found. 1 expected.",joint.name.c_str(),joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if(joint.command_interface[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(rclcpp::get_logger("Solo12HardwareInterface"),"Joint '%s' has command interface '%s'. '%s' expected.",joint.name.c_str(),joint.command_interfaces[0].name.c_str(),hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if(joint.state_interfaces.size()!=2)
        {
            RCLCPP_FATAL(rclcpp::get_logger("Solo12HardwareInterface"),"Joint '%s' has %zu state interfaces found. 2 expected.",joint.name.c_str(),joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if(joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(rclcpp::get_logger("Solo12HardwareInterface"),"Joint '%s' has state interface '%s'. '%s' expected.",joint.name.c_str(),joint.state_interfaces[0].name.c_str(),hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if(joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(rclcpp::get_logger("Solo12HardwareInterface"),"Joint '%s' has state interface '%s'. '%s' expected.",joint.name.c_str(),joint.state_interfaces[1].name.c_str(),hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    return hardware_interface::CallbackReturn::SUCCESS;


}