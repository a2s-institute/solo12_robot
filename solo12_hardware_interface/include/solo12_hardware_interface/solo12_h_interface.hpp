#ifndef HARDWARE_INTERFACE__SOLO12_INTERFACE_HPP
#define HARDWARE_INTERFACE__SOLO12_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>

// These are the libraries which are existing and we dont need to find in other folders

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Any additonal libraries can be added below

/*Master_board_interfaces can be inlcuded to this and we can call
them to access the interface parameters*/

using hardware_interface::return_type;

namespace solo12_hardware_interface
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class Solo12HardwareInterface : public hardware_interface::SystemInterface
    {
        public:
        RCLCPP_SHARED_PTR_DEFINITIONS(Solo12HardwareInterface);

        /*use the exmaple 7 to write the hpp*/
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo & info) override;
        
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type read(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

        hardware_interface::return_type write(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
        // Parameters for the SOLO12 Hardware
        
        /*It is mandatory to select between the current or
        other positional or its derivative commands*/

        double solo12_h_curr;       // Current Reference
        double solo12_h_pos_ref;    // Position Reference
        double solo12_h_vel_ref;    // Velocity Reference

        // Parameters stored from the SOLO12 Hardware
        
        /*In this I guess we have to add all the imu struct which
        is available in the "define.h" to store the data, for now a 
        dummy variables are provided here.
        
        ??? I also dont know how to use it because it is a 1X3 data
        which is available for all the joints, so should I need to
        pass it as normal parameter or list of it???*/
        std::vector<double> solo12_imu_accelerometer_;
        // std::vector<double> solo12_imu_gyroscope_;
        // std::vector<double> solo12_imu_altitude_;
        std::vector<double> solo12_h_position_;
        std::vector<double> solo12_h_velocity_;
        
        // It is just an idea!!!

        /*std::vector<double> imu_accelerometer_;
        std::vector<double> imu_gyroscope_;
        std::vector<double> imu_attitude_;
        std::vector<double> imu_linear_acceleration*/

        // There are also various other things available
        
    };
}

#endif