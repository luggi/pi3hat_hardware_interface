/**
 * @file pi3hat_hardware_interface.hpp
 * @author John Bush (johncbus@usc.edu)
 * @brief This file defines the Pi3HatHardwareInterface class, which is a 
 *          ROS2 Control Hardware System Type Interface.
 * @details implementation follows the ROS2 documentation found at: 
 *              <https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html>
 * @version 0.1
 * @date 2024-03-21
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef PI3HAT_HARDWARE_INTERFACE__PI3HAT_HARDWARE_INTERFACE_HPP_
#define PI3HAT_HARDWARE_INTERFACE__PI3HAT_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <functional>

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

#include "pi3hat/pi3hat.h"
#include "actuators/actuator_base.h"
#include "actuators/odrive_actuator.h"
#include "actuators/moteus_actuator.h"

#define MAX_NUM_CAN_FRAMES 12*8

namespace pi3hat_hardware_interface
{
    class Pi3HatHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(Pi3HatHardwareInterface)

        /**
         * @details on_init is expected to vary depending on the URDF file. 
         *          The SystemInterface::on_init(info) call fills out the info 
         *          object with specifics from the URDF. For example, 
         *          the info object has fields for joints, sensors, gpios, and more. 
         *          Suppose the sensor field has a name value of tcp_force_torque_sensor. 
         *          Then the on_init must try to establish communication with that sensor. 
         *          If it fails, then an error value is returned.
         * 
         * @param info 
         * @return hardware_interface::CallbackReturn 
         */
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        /**
         * @brief Called after the on_init function. Setup the communication to the hardware and set everything up so that the hardware can be activated.
         * 
         * @param previous_state 
         * @return hardware_interface::CallbackReturn 
         */
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        /**
         * Does opposite of on_configure()
        */
        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;


        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        /**
         * @brief Called after on_configure. This is where the hardware is activated -- "powered up"
         * 
         * @param previous_state 
         * @return hardware_interface::CallbackReturn 
         */
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Opposite of on_activate. This is where the hardware is deactivated -- "powered down"
         * 
         * @param previous_state 
         * @return hardware_interface::CallbackReturn 
         */
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State &previous_state) override;


        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;


        std::shared_ptr<mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>> allocateTxSpan(size_t size);
        
    private:
        // Utility functions for converting between double and uint
        int double_to_uint(double x, double x_min, double x_max,
                          int bits);
        double uint_to_double(int x_int, double x_min, double x_max,
                            int bits);

        // Utility functions for dealing with angle wrapping
        double wrap_angle(double angle, double angle_min, double angle_max);
        double unwrap_angle(double angle, double prev_angle, double angle_min,
                           double angle_max);

        // Enum for CAN protocol types: cheetah, myactuator, or moteus
        enum class CanProtocol
        {
            CHEETAH,    // MIT Mini Cheetah actuators, SteadyWin, CubeMars
            MYACTUATOR, // MyActuator, LKMTech
            MOTEUS,      // Moteus (non-FD CAN)
            ODRIVE,      // ODrive
        };

        enum TxAllocation
        {
            CHEETAH     = 1,
            MYACTUATOR  = 1,
            MOTEUS      = 1,
            ODRIVE      = 6,
        };

        const unsigned char cheetahEnableMsg[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                                                   0xFF, 0xFF, 0xFF, 0xFC};

        const unsigned char cheetahDisableMsg[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                                                    0xFF, 0xFF, 0xFF, 0xFD};

        const unsigned char cheetahSetZeroPositionMsg[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                                                            0xFF, 0xFF, 0xFF, 0xFE};

        const unsigned char cheetahSetIdleCmdMsg[8] = {0x7F, 0xFF, 0x7F, 0xF0,
                                                       0x00, 0x00, 0x07, 0xFF};

        mjbots::pi3hat::Pi3Hat *pi3hat_;
        mjbots::pi3hat::Pi3Hat::Input pi3hat_input_;
        mjbots::pi3hat::Attitude attitude_;
        // mjbots::pi3hat::CanFrame tx_can_frames_[MAX_NUM_CAN_FRAMES];
        // mjbots::pi3hat::CanFrame rx_can_frames_[MAX_NUM_CAN_FRAMES];

        // Pi3Hat input handling
        std::vector<mjbots::pi3hat::CanFrame> tx_can_frames_;
        std::vector<mjbots::pi3hat::CanFrame> rx_can_frames_;
        size_t nextTxStart = 0; // Tracks the next start position for Tx allocations
        size_t nextRxStart = 0; // Tracks the next start position for Rx allocations

        // ! DO THESE NEED TO BE STATIC??
        size_t tx_capacity_ = 36; // Default initial capacity
        size_t rx_capacity_ = 36; // Default initial capacity

        // IMU state
        std::array<double, 4> hw_state_imu_orientation_;         // x, y, z, w
        std::array<double, 3> hw_state_imu_angular_velocity_;    // x, y, z
        std::array<double, 3> hw_state_imu_linear_acceleration_; // x, y, z

        // Actuator CAN config
        std::vector<int> hw_actuator_can_channels_;
        std::vector<int> hw_actuator_can_ids_;
        std::vector<CanProtocol> hw_actuator_can_protocols_;
        std::vector<std::shared_ptr<ActuatorBase>> hw_actuators_;

        // Actuator parameters
        std::vector<int> hw_actuator_axis_directions_;
        std::vector<double> hw_actuator_position_offsets_;
        std::vector<double> hw_actuator_gear_ratios_;
        std::vector<double> hw_actuator_torque_constants_;
        std::vector<int> hw_actuator_soft_start_durations_ms_;

        // Actuator limits
        std::vector<double> hw_actuator_position_mins_; 
        std::vector<double> hw_actuator_position_maxs_;
        std::vector<double> hw_actuator_velocity_limits_;
        std::vector<double> hw_actuator_effort_limits_;
        std::vector<double> hw_actuator_kp_limits_;
        std::vector<double> hw_actuator_kd_limits_;
        std::vector<double> hw_actuator_ki_limits_;

        // Actuator states
        std::vector<double> hw_state_positions_;
        std::vector<double> hw_state_velocities_;
        std::vector<double> hw_state_efforts_;

        // Actuator commands
        std::vector<double> hw_command_positions_;
        std::vector<double> hw_command_velocities_;
        std::vector<double> hw_command_efforts_;
        std::vector<double> hw_command_kps_;
        std::vector<double> hw_command_kds_;
        std::vector<double> hw_command_kis_;
    };

} // namespace pi3hat_hardware_interface

#endif // PI3HAT_HARDWARE_INTERFACE_PUBLIC__PI3HAT_HARDWARE_INTERFACE_HPP_