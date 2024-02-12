#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "solo12_hardware_interface/solo12_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
    const rclcpp::Logger LOGGER = rclcpp::get_logger("Solo12HardwareInterface");
}

namespace solo12_hardware_interface
{
    hardware_interface::CallbackReturn Solo12HardwareInterface::on_init(
            const hardware_interface::HardwareInfo & info)
    {
        if(hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS){
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(LOGGER, "Running on_init function");


        /* Here we can pass the parameters required for the hardware
           This will convert the string obtained from yml to double and 
           pass the value.*/

        // ??? At this sentence double should not be used,clarify,compare with example2 ???
        // Is this because we used it in the private section in the class 

        /*
           std::vector<double> solo12_h_position_(info_.joints.size(),std::numeric_limits<double>::quiet_NaN());
           std::vector<double> solo12_h_velocity_(info.joints.size(),std::numeric_limits<double>::quiet_NaN());
        // std::vector<double> solo12_h_velocity_command_(info.joints.size(),std::numeric_limits<double>::quiet_NaN());
        */

        /*
           for(hardware_interface::ComponentInfo & joint : info_.joints){
           if(joint.command_interfaces.size()!=1)
           { 
           RCLCPP_FATAL(rclcpp::get_logger("Solo12HardwareInterface"),"Joint '%s' has %zu command interfaces found. 1 expected.",joint.name.c_str(),joint.command_interfaces.size());
           return hardware_interface::CallbackReturn::ERROR;
           }

           if(joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
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
           */

        // can assign some particular values to the hardware, to get the exact values from the hardware
        // Example: solo12_h_position_[i] = 0.0;
        //
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> Solo12HardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        RCLCPP_INFO(LOGGER, "Running export_state_interfaces function");

        /*
        for(size_t i=0; i<info_.joints.size();i++)
        {
            //emplcae back is used to construct the object in place at the end of the vector
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,hardware_interface::HW_IF_POSITION,&solo12_h_position_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,hardware_interface::HW_IF_VELOCITY,&solo12_h_velocity_[i]));
            // can add more state interfaces if required wrt to sensors or other state variables
        }
        */
        return state_interfaces;
    }

    // the velocity command parameter is passed to the hardware
    //std::vector<double> solo12_h_velocity_command_(info.joints.size(),std::numeric_limits<double>::quiet_NaN());

    std::vector<hardware_interface::CommandInterface> Solo12HardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        RCLCPP_INFO(LOGGER, "Running export_command_interfaces function");

        /*
        for(size_t i=0; i<info_.joints.size();i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,hardware_interface::HW_IF_VELOCITY, &solo12_h_velocity_command_[i]));
            // Similarly can add more velocity command interfaces if required
        }
        */

        return command_interfaces;
    }

    hardware_interface::return_type Solo12HardwareInterface::read(
        const rclcpp::Time & time, const rclcpp::Duration & period)
        {

            RCLCPP_INFO(LOGGER, "Running read function");

            /*
            for(size_t i=0; i<info_.joints.size();i++)
            {
                solo12_h_position_[i] = solo12_h_velocity_[i] + period.seconds()*solo12_h_velocity_command_[i];

                RCLCPP_INFO(rclcpp::get_logger("Solo12HardwareInterface"),"Joint '%s' position: %.3f, velocity: %.3f, velocity command: %.3f",info_.joints[i].name.c_str(),solo12_h_position_[i],solo12_h_velocity_[i],solo12_h_velocity_command_[i]);
            }
            */
            return hardware_interface::return_type::OK;
        }

    hardware_interface::return_type Solo12HardwareInterface::write(
        const rclcpp::Time & time, const rclcpp::Duration & period)
        {

            RCLCPP_INFO(LOGGER, "Running write function");

            /*
            RCLCPP_INFO(rclcpp::get_logger("Solo12HardwareInterface"),"Writing to hardware");

            for(size_t i=0; i<info_.joints.size();i++)
            {
                RCLCPP_INFO(rclcpp::get_logger("Solo12HardwareInterface"),"Joint '%s' velocity command: %.3f",info_.joints[i].name.c_str(),solo12_h_velocity_command_[i]);
                solo12_h_velocity_[i] = solo12_h_velocity_command_[i];
            }
            RCLCPP_INFO(rclcpp::get_logger("Solo12HardwareInterface"),"Velocity commads are passed to the hardware!");
            */

            return hardware_interface::return_type::OK;
        }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  solo12_hardware_interface::Solo12HardwareInterface, hardware_interface::SystemInterface)
