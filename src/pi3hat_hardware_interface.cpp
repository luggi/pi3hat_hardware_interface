#include "pi3hat_hardware_interface/pi3hat_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <unordered_set>
#include <sched.h>
#include <sys/mman.h>

#include <time.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#define DEG_TO_RAD 0.01745329251994329576923690768489

namespace pi3hat_hardware_interface
{
    Pi3HatHardwareInterface::~Pi3HatHardwareInterface()
    {
        on_deactivate(rclcpp_lifecycle::State());
    }

    /**
     * @brief on_init
     *
     * @param info defines the hardware interface configuration
     * @return hardware_interface::CallbackReturn
     */
    hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("Pi3HatHardwareInterface"), "Starting pi3hat_hardware_interface on_init()");

        /**
         * @brief Initialize state and command vectors
         * 
         *  Resize the vectors to the number of joints in the URDF file and set the values to NaN
         */
        hw_state_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_state_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_state_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        hw_command_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_command_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_command_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_command_kps_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_command_kds_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        // Determine which actuator types are in use and determine the span size required.
        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            if ("cheetah" == joint.parameters.at("can_protocol"))
            {
                // throw an error since this isnt supported yet
                hw_actuator_can_protocols_.push_back(CanProtocol::CHEETAH);
                tx_capacity_ += TxAllocation::CHEETAH_TX;
                rx_capacity_ += RxAllocation::CHEETAH_RX;
            }
            else if ("myactuator" == joint.parameters.at("can_protocol"))
            {
                // throw an error since this isnt supported yet
                hw_actuator_can_protocols_.push_back(CanProtocol::MYACTUATOR);
                tx_capacity_ += TxAllocation::MYACTUATOR_TX;
                rx_capacity_ += RxAllocation::MYACTUATOR_RX;
            }
            else if ("moteus" == joint.parameters.at("can_protocol"))
            {
                hw_actuator_can_protocols_.push_back(CanProtocol::MOTEUS);
                tx_capacity_ += TxAllocation::MOTEUS_TX;
                rx_capacity_ += RxAllocation::MOTEUS_RX;
            }
            else if ("odrive" == joint.parameters.at("can_protocol"))
            {
                hw_actuator_can_protocols_.push_back(CanProtocol::ODRIVE);
                tx_capacity_ += TxAllocation::ODRIVE_TX;
                rx_capacity_ += RxAllocation::ODRIVE_RX;
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("Pi3HatHardwareInterface"), "can_protocol parameter does not match a valid protocol");
                return hardware_interface::CallbackReturn::ERROR;
            }

            // Set params for each joint
            hw_actuator_can_ids_.push_back(std::stoi(joint.parameters.at("can_id")));
            hw_actuator_can_channels_.push_back(std::stoi(joint.parameters.at("can_channel")));
            hw_actuator_axis_directions_.push_back(std::stoi(joint.parameters.at("axis_direction")));
            hw_actuator_position_offsets_.push_back(std::stod(joint.parameters.at("position_offset")));
            hw_actuator_gear_ratios_.push_back(std::stod(joint.parameters.at("gear_ratio")));
            hw_actuator_torque_constants_.push_back(std::stod(joint.parameters.at("torque_constant")));
            hw_actuator_soft_start_durations_ms_.push_back(std::stoi(joint.parameters.at("soft_start_duration_ms")));

            // Set limits for each joint
            hw_actuator_position_mins_.push_back(std::stod(joint.parameters.at("position_min")));
            hw_actuator_position_maxs_.push_back(std::stod(joint.parameters.at("position_max")));
            hw_actuator_velocity_limits_.push_back(std::stod(joint.parameters.at("velocity_max")));
            hw_actuator_effort_limits_.push_back(std::stod(joint.parameters.at("effort_max")));
            hw_actuator_kp_limits_.push_back(std::stod(joint.parameters.at("kp_max")));
            hw_actuator_kd_limits_.push_back(std::stod(joint.parameters.at("kd_max")));
            hw_actuator_ki_limits_.push_back(std::stod(joint.parameters.at("ki_max")));
        }

        /**
         * @brief Ensure that all CAN IDs are unique
         * 
         * this is a more stringent requirement, since you could have the same IDs on different channels,
         * but it causes problems when reading the feedback from the actuators
         */
        std::unordered_set<int> seenIDs;
        for (int id : hw_actuator_can_ids_) {
            if (!seenIDs.insert(id).second) {
                RCLCPP_ERROR(rclcpp::get_logger("Pi3HatHardwareInterface"), "Failed to start: duplicate CAN IDs present");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        // TODO: To fully support moteus this should use the fd frame.
        // Configure the Pi3Hat CAN for non-FD mode without bitrate switching or automatic retranmission
        mjbots::pi3hat::Pi3Hat::CanConfiguration can_config;
        can_config.fast_bitrate = 1000000;
        can_config.slow_bitrate = 1000000;
        can_config.fdcan_frame = false;
        can_config.bitrate_switch = false;
        can_config.automatic_retransmission = false;

        // Configure the Pi3Hat for 1000hz IMU sampling
        mjbots::pi3hat::Pi3Hat::Configuration config;
        config.attitude_rate_hz = 1000;

        // Set the mounting orientation of the IMU
        config.mounting_deg.yaw = std::stod(info_.hardware_parameters.at("imu_mounting_deg.yaw"));
        config.mounting_deg.pitch = std::stod(info_.hardware_parameters.at("imu_mounting_deg.pitch"));
        config.mounting_deg.roll = std::stod(info_.hardware_parameters.at("imu_mounting_deg.roll"));

        // Initialize the Pi3Hat input object
        pi3hat_input_ = mjbots::pi3hat::Pi3Hat::Input();
        pi3hat_input_.attitude = &attitude_;
        // Resize the rx_can and tx_can spans to the number of actuators and protocols allocation
        tx_can_frames_.resize(tx_capacity_);
        rx_can_frames_.resize(rx_capacity_);

        // Create and assign Spans to the rx_can and tx_can fields of the Pi3Hat input object
        mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> rx_can_frames_span_(rx_can_frames_.data(), rx_capacity_);
        pi3hat_input_.rx_can = rx_can_frames_span_;
        mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> tx_can_frames_span_(tx_can_frames_.data(), tx_capacity_);
        pi3hat_input_.tx_can = tx_can_frames_span_;

        // rx_can and tx_can are Spans of CanFrames, which is a struct with the following fields:
        //      uint8_t bus;
        //      uint8_t id;
        //      uint8_t size;
        //      bool expect_reply;
        //      uint8_t data[8];

        // onInit() -> Initialize the CAN Protocol Interfaces
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            switch (hw_actuator_can_protocols_[i])
            {
            case CanProtocol::MOTEUS:
            {
                // TODO: implement
                /**
                 * -> push back a shared pointer to a new moteus actuator instance
                 * -> allocate the actuator's outgoing CAN Frame Span and assign it
                 */
                // Create new actuator and add shared pointer to vector
                hw_actuators_.push_back(
                    std::make_shared<MoteusActuator>(
                        hw_actuator_can_ids_[i],
                        hw_actuator_can_channels_[i],
                        info_.joints[i].name, // TODO: either add this parameter to this class and the URDF or create a nameless constructor in actuator
                        hw_actuator_axis_directions_[i],
                        hw_actuator_position_offsets_[i],
                        hw_actuator_gear_ratios_[i],
                        hw_actuator_torque_constants_[i],
                        hw_actuator_effort_limits_[i],
                        hw_actuator_position_mins_[i],
                        hw_actuator_position_maxs_[i],
                        hw_actuator_velocity_limits_[i],
                        hw_actuator_kp_limits_[i],
                        hw_actuator_kd_limits_[i],
                        hw_actuator_ki_limits_[i],

                        hw_actuator_soft_start_durations_ms_[i]
                    )
                );

                // allocate the actuator's outgoing CAN Frame Span and assign it
                hw_actuators_[i]->setTxSpan(allocateTxSpan(TxAllocation::MOTEUS_TX));
                hw_actuators_[i]->invalidateSpan();
                RCLCPP_INFO(rclcpp::get_logger("Pi3HatHardwareInterface"), "Created an Moteus Actuator at joint %d and allocated %i CAN Frames", i, TxAllocation::MOTEUS_TX);
            }
            case CanProtocol::ODRIVE:
            {
                RCLCPP_INFO(rclcpp::get_logger("Pi3HatHardwareInterface"), "Creating an ODrive Actuator at joint %d", i);
                // Create new actuator and add shared pointer to vector
                hw_actuators_.push_back(
                    std::make_shared<ODriveActuator>(
                        hw_actuator_can_ids_[i],
                        hw_actuator_can_channels_[i],
                        info_.joints[i].name, // TODO: either add this parameter to this class and the URDF or create a nameless constructor in actuator
                        hw_actuator_axis_directions_[i],
                        hw_actuator_position_offsets_[i],
                        hw_actuator_gear_ratios_[i],
                        hw_actuator_torque_constants_[i],
                        hw_actuator_effort_limits_[i],
                        hw_actuator_position_mins_[i],
                        hw_actuator_position_maxs_[i],
                        hw_actuator_velocity_limits_[i],
                        hw_actuator_kp_limits_[i],
                        hw_actuator_kd_limits_[i],
                        hw_actuator_ki_limits_[i],

                        hw_actuator_soft_start_durations_ms_[i]
                    )
                );

                // allocate the actuator's outgoing CAN Frame Span and assign it
                hw_actuators_[i]->setTxSpan(allocateTxSpan(TxAllocation::ODRIVE_TX));
                hw_actuators_[i]->invalidateSpan();
                RCLCPP_INFO(rclcpp::get_logger("Pi3HatHardwareInterface"), "Created an ODrive Actuator at joint %d and allocated %i CAN Frames", i, TxAllocation::ODRIVE_TX);
                break;
            }
            default:
                RCLCPP_ERROR(rclcpp::get_logger("Pi3HatHardwareInterface"), "Failed to start: unknown CAN protocol");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        // onInit() -> Set up the CAN configuration
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            config.can[hw_actuator_can_channels_[i] - 1] = can_config;
            pi3hat_input_.tx_can[i].id = hw_actuator_can_ids_[i];
            pi3hat_input_.tx_can[i].bus = hw_actuator_can_channels_[i];
            pi3hat_input_.tx_can[i].expect_reply = true;
            pi3hat_input_.tx_can[i].size = 8;

            if (hw_actuator_can_protocols_[i] == CanProtocol::ODRIVE) {
                // This forces the Pi Hat to check for CAN replies on the channels connected to ODrives,
                // since the ODrives send heartbeat messages automatically to the PiHat.
                pi3hat_input_.force_can_check |= (1U << hw_actuator_can_channels_[i]);
            }
        }

        // onInit() -> Initialize the Pi3Hat
        pi3hat_ = new mjbots::pi3hat::Pi3Hat(config);

        // onInit() -> Configure realtime scheduling
        {
            int realtime_cpu1 = 0; // first core
            int realtime_cpu2 = 1; // second core
            cpu_set_t cpuset = {};
            CPU_ZERO(&cpuset);
            CPU_SET(realtime_cpu1, &cpuset); // set first core
            CPU_SET(realtime_cpu2, &cpuset); // set second core

            const int r = ::sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);
            if (r < 0)
            {
                throw std::runtime_error("Error setting CPU affinity");
            }

            std::cout << "Affinity set to CPUs " << realtime_cpu1 << " and " << realtime_cpu2 << "\n";
        }
        {
            struct sched_param params = {};
            params.sched_priority = 10;
            const int r = ::sched_setscheduler(0, SCHED_RR, &params);
            if (r < 0)
            {
                throw std::runtime_error("Error setting realtime scheduler");
            }
        }
        {
            const int r = ::mlockall(MCL_CURRENT | MCL_FUTURE);
            if (r < 0)
            {
                throw std::runtime_error("Error locking memory");
            }
        }
        
        // TODO: refactor this function from here on -- has redundant code
        // onInit() -> Set all actuators to IDLE
        for (auto i = 0u; i < hw_state_positions_.size(); i++)
        {
            hw_actuators_[i]->setState(ActuatorState::DISARMED);
            RCLCPP_INFO(rclcpp::get_logger("Pi3HatHardwareInterface"), "Sending Actuator at Joint %d to idle...", i);
        }
        mjbots::pi3hat::Pi3Hat::Output result = pi3hat_->Cycle(pi3hat_input_);
        busy_wait_us(1000000); // wait for 1 second

        // onInit() -> Set the zero position for each actuator
        for (auto i = 0u; i < hw_state_positions_.size(); i++)
        {
            hw_actuators_[i]->setState(ActuatorState::DISARMED);
        }
        // cycle 3 times to ensure the zero position is set
        for (auto i = 0u; i < 3; i++)
        {
            result = pi3hat_->Cycle(pi3hat_input_);
            busy_wait_us(500000); // wait for 0.5 second
        }

        // give the input to the distribute_rx_input function
        distribute_rx_input(result);

        // Read the feedback from the actuators
        for (auto i = 0u; i < hw_state_positions_.size(); i++) 
        {
            hw_actuators_[i]->processRxFrames();
            RCLCPP_INFO(rclcpp::get_logger("Pi3HatHardwareInterface"), "Checking Connection to Actuator %d...", i);
        }

        RCLCPP_INFO(rclcpp::get_logger("Pi3HatHardwareInterface"), "Pi3HatHardwareInterface successfully initialized!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // TODO: Update the state interfaces . Do we want to add Temperature, Voltage?
    std::vector<hardware_interface::StateInterface> Pi3HatHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // Add joint state interfaces
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_state_positions_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocities_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_state_efforts_[i]));
        }

        // Add IMU state interfaces
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu_sensor", "orientation.x", &hw_state_imu_orientation_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu_sensor", "orientation.y", &hw_state_imu_orientation_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu_sensor", "orientation.z", &hw_state_imu_orientation_[2]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu_sensor", "orientation.w", &hw_state_imu_orientation_[3]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu_sensor", "angular_velocity.x", &hw_state_imu_angular_velocity_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu_sensor", "angular_velocity.y", &hw_state_imu_angular_velocity_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu_sensor", "angular_velocity.z", &hw_state_imu_angular_velocity_[2]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu_sensor", "linear_acceleration.x", &hw_state_imu_linear_acceleration_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu_sensor", "linear_acceleration.y", &hw_state_imu_linear_acceleration_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu_sensor", "linear_acceleration.z", &hw_state_imu_linear_acceleration_[2]));

        return state_interfaces;
    }

    hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // reset values always when configuring hardware
        for (uint i = 0; i < hw_state_positions_.size(); i++)
        {
            hw_state_positions_[i] = 0;
            hw_state_velocities_[i] = 0;
            hw_state_efforts_[i] = 0;
            hw_command_positions_[i] = 0;
            hw_command_velocities_[i] = 0;
            hw_command_efforts_[i] = 0;
            hw_command_kps_[i] = 0;
            hw_command_kds_[i] = 0;
        }

        RCLCPP_INFO(rclcpp::get_logger("Pi3HatHardwareInterface"), "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::CommandInterface> Pi3HatHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_command_positions_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_command_velocities_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_command_efforts_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, "kp", &hw_command_kps_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, "kd", &hw_command_kds_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, "ki", &hw_command_kis_[i]));
        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // onActivate() -> Enable all actuators
        for (auto i = 0u; i < hw_state_positions_.size(); i++)
        {
            RCLCPP_INFO(rclcpp::get_logger("Pi3HatHardwareInterface"), "Setting Joint %d state to ARMED", i);
            hw_actuators_[i]->setState(ActuatorState::ARMED);
        }
        mjbots::pi3hat::Pi3Hat::Output result = pi3hat_->Cycle(pi3hat_input_);
        busy_wait_us(1000000); // wait for 1 second

        /**
         * @brief Attempts 3 times to set all actuators to position mode.
         *      If all actuators are not in position mode after 3 attempts, return an error.
         */
        for (auto attempt = 0u; attempt < 3; attempt++)
        {
            // onActivate() -> Enable all actuators
            for (auto i = 0u; i < hw_state_positions_.size(); i++)
            {
                RCLCPP_INFO(rclcpp::get_logger("Pi3HatHardwareInterface"), "Setting Joint %d state to Position Mode", i);
                hw_actuators_[i]->setState(ActuatorState::POSITION_MODE);
            }
            result = pi3hat_->Cycle(pi3hat_input_);
            busy_wait_us(200000); // wait for 0.2 second

            if (result.error)
            {
                RCLCPP_ERROR(rclcpp::get_logger("Pi3HatHardwareInterface"), "Pi3Hat::Cycle() failed!");
                return hardware_interface::CallbackReturn::ERROR;
            } else {
                // give the input to the distribute_rx_input function
                distribute_rx_input(result);
            }

            int enabled_count = 0;

            // Read the feedback from the actuators
            for (auto i = 0u; i < hw_state_positions_.size(); i++) 
            {
                hw_actuators_[i]->processRxFrames();                
                // Check if the actuator is in position mode
                if (hw_actuators_[i]->getState() == ActuatorState::POSITION_MODE )
                {
                    RCLCPP_INFO(rclcpp::get_logger("Pi3HatHardwareInterface"), "Attempt %i: Actuator %i in position mode", attempt, i);
                    enabled_count++;
                } else {
                    RCLCPP_INFO(rclcpp::get_logger("Pi3HatHardwareInterface"), "Attempt %i: Actuator %i NOT in position mode", attempt, i);
                }
            }
            if (enabled_count == hw_state_positions_.size())
            {
                RCLCPP_INFO(rclcpp::get_logger("Pi3HatHardwareInterface"), "Successfully activated!");
                return hardware_interface::CallbackReturn::SUCCESS;
            }
        }

        RCLCPP_ERROR(rclcpp::get_logger("Pi3HatHardwareInterface"), "Failed to activate: Some actuators are not in position mode");
        return hardware_interface::CallbackReturn::ERROR;   
    }

    hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        for (auto i = 0u; i < hw_state_positions_.size(); i++)
        {
            hw_actuators_[i]->setState(ActuatorState::DISARMED);

            // switch (hw_actuator_can_protocols_[i])
            // {
            //         // TODO: Add support for other CAN protocols
            //     case CanProtocol::CHEETAH:
            //         // We send a command of all zeroes to the actuator before disabling it
            //         // This is to prevent the actuator from moving if we re-enable it later
            //         std::copy(std::begin(cheetahSetIdleCmdMsg), std::end(cheetahSetIdleCmdMsg), std::begin(pi3hat_input_.tx_can[i].data));
            //         break;
            //     case CanProtocol::ODRIVE:
            //     {
            //         hw_actuators_[i]->setState(ActuatorState::DISARMED);
            //         break;
            //     }
            //     default:
            //         RCLCPP_ERROR(rclcpp::get_logger("Pi3HatHardwareInterface"), "Failed to deactivate: unknown CAN protocol");
            //         return hardware_interface::CallbackReturn::ERROR;
            // }

        }
        // TODO: why do we do this for loop? can it be removed? Is the redundancy safer?
        pi3hat_->Cycle(pi3hat_input_);
        busy_wait_us(1000000); // wait for 1 second
        for (auto i = 0u; i < hw_state_positions_.size(); i++)
        {
            hw_actuators_[i]->setState(ActuatorState::DISARMED);
            // switch (hw_actuator_can_protocols_[i])
            // {
            //     // TODO: Add support for other CAN protocols
            // case CanProtocol::CHEETAH:
            //     std::copy(std::begin(cheetahDisableMsg), std::end(cheetahDisableMsg), std::begin(pi3hat_input_.tx_can[i].data));
            //     break;
            // case CanProtocol::ODRIVE:
            //     hw_actuators_[i]->setState(ActuatorState::DISARMED);
            //     break;
            // default:
            //     RCLCPP_ERROR(rclcpp::get_logger("Pi3HatHardwareInterface"), "Failed to deactivate: unknown CAN protocol");
            //     return hardware_interface::CallbackReturn::ERROR;
            // }
        }
        pi3hat_->Cycle(pi3hat_input_);
        busy_wait_us(1000000); // wait for 1 second

        RCLCPP_INFO(rclcpp::get_logger("Pi3HatHardwareInterface"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // TODO: implement
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_shutdown(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // TODO: implement
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_error(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        for (auto i = 0u; i < hw_state_positions_.size(); i++)
        {
            hw_actuators_[i]->setState(ActuatorState::ERROR);
            break;
        }
        pi3hat_->Cycle(pi3hat_input_);
        busy_wait_us(1000000); // wait for 1 second
        mjbots::pi3hat::Pi3Hat::Output result = pi3hat_->Cycle(pi3hat_input_);

        if (result.error)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Pi3HatHardwareInterface"), "Pi3Hat::Cycle() failed!");
            return hardware_interface::CallbackReturn::ERROR;
        } else {
            // give the input to the distribute_rx_input function
            distribute_rx_input(result);
        }
        
        // Read the feedback from the actuators
        for (auto i = 0u; i < hw_state_positions_.size(); i++) 
        {
            hw_actuators_[i]->processRxFrames();

            RCLCPP_INFO(rclcpp::get_logger("Pi3HatHardwareInterface"), "Actuator %i errors: %s", i, hw_actuators_[i]->printErrorMessage().c_str());
            
            // Check if the actuator is in position mode
            if (hw_actuators_[i]->getState() != ActuatorState::ERROR)
            {
                RCLCPP_ERROR(rclcpp::get_logger("Pi3HatHardwareInterface"), "Failed to send Error command: Actuator %d malfunctioning", i);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
    }

    hardware_interface::return_type Pi3HatHardwareInterface::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        // Reading is done in the write() method due to how the Pi3Hat Cycle() method works
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type Pi3HatHardwareInterface::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // write() -> Update the actuator states and assemble CAN frames
        for (auto i = 0u; i < hw_state_positions_.size(); i++)
        {
            if (std::isnan(hw_command_positions_[i]) || std::isnan(hw_command_velocities_[i]) || std::isnan(hw_command_efforts_[i]) || std::isnan(hw_command_kps_[i]) || std::isnan(hw_command_kds_[i]))
            {
                RCLCPP_WARN(rclcpp::get_logger("Pi3HatHardwareInterface"), "NaN command for actuator");
                continue;
            }

            switch (hw_actuator_can_protocols_[i])
            {
                case CanProtocol::ODRIVE:
                case CanProtocol::MOTEUS:
                {
                    hw_actuators_[i]->sendJointCommand(hw_command_positions_[i], hw_command_velocities_[i], hw_command_efforts_[i]);
                    break;
                }
                    // TODO: Add support for other CAN protocols
                case CanProtocol::CHEETAH:
                {
                    // constrain commands to the user-defined limits
                    double p_des = fminf(fmaxf(hw_actuator_position_mins_[i], hw_command_positions_[i]), hw_actuator_position_maxs_[i]);
                    double v_des = fminf(fmaxf(-hw_actuator_velocity_limits_[i], hw_command_velocities_[i]), hw_actuator_velocity_limits_[i]);
                    double tau_ff = fminf(fmaxf(-hw_actuator_effort_limits_[i], hw_command_efforts_[i]), hw_actuator_effort_limits_[i]);
                    double kp = fminf(fmaxf(0.0, hw_command_kps_[i]), hw_actuator_kp_limits_[i]);
                    double kd = fminf(fmaxf(0.0, hw_command_kds_[i]), hw_actuator_kd_limits_[i]);

                    // // compensate for axis directions and offsets
                    // p_des = (p_des - hw_actuator_position_offsets_[i]) * hw_actuator_axis_directions_[i];
                    // v_des = v_des * hw_actuator_axis_directions_[i];
                    // tau_ff = tau_ff * hw_actuator_axis_directions_[i];

                    // // pack ints into the can message
                    // pi3hat_input_.tx_can[i].data[0] = p_int >> 8;
                    // pi3hat_input_.tx_can[i].data[1] = p_int & 0xFF;
                    // pi3hat_input_.tx_can[i].data[2] = v_int >> 4;
                    // pi3hat_input_.tx_can[i].data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
                    // pi3hat_input_.tx_can[i].data[4] = kp_int & 0xFF;
                    // pi3hat_input_.tx_can[i].data[5] = kd_int >> 4;
                    // pi3hat_input_.tx_can[i].data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
                    // pi3hat_input_.tx_can[i].data[7] = t_int & 0xff;
                    break;
                }
                default:
                    RCLCPP_ERROR(rclcpp::get_logger("Pi3HatHardwareInterface"), "Failed to send: unknown CAN protocol");
                    break;
            }
        }

        pi3hat_input_.request_attitude = true;
        pi3hat_input_.wait_for_attitude = true;

        const auto result = pi3hat_->Cycle(pi3hat_input_);

        if (result.error)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Pi3HatHardwareInterface"), "Pi3Hat::Cycle() failed!");
            return hardware_interface::return_type::ERROR;
        }    

        // Update the IMU state if attitude data is available
        if (result.attitude_present)
        {
            /*
            The quaternion returned by the pi3hat is the rotation from the gravity frame to the IMU frame.
                Gravity frame:
                    +x and +y are parallel to the ground
                    +z points towards the ground
                IMU frame:
                    +x points towards the side of the Pi with the USB-C port
                    +y points towards the side of the Pi with the USB-A ports
                    +z points up from the Pi
            However, we want the rotation from the world frame to the IMU frame.
                World frame:
                    +x and +y are parallel to the ground
                    +z points towards the sky
            Therefore, we need to rotate the quaternion returned by the pi3hat by 180 degrees about the x-axis or y-axis. We choose to rotate about the x-axis.
            Let the quaternion returned by the pi3hat be (x, y, z, w).
            After applying a 180-degree rotation about the x-axis, the new quaternion is:
                (w, -z, y, -x)
            */
            hw_state_imu_orientation_[0] = pi3hat_input_.attitude->attitude.w;  // x-component of the new quaternion
            hw_state_imu_orientation_[1] = -pi3hat_input_.attitude->attitude.z; // y-component of the new quaternion
            hw_state_imu_orientation_[2] = pi3hat_input_.attitude->attitude.y;  // z-component of the new quaternion
            hw_state_imu_orientation_[3] = -pi3hat_input_.attitude->attitude.x; // w-component of the new quaternion
            hw_state_imu_angular_velocity_[0] = pi3hat_input_.attitude->rate_dps.x * DEG_TO_RAD;
            hw_state_imu_angular_velocity_[1] = pi3hat_input_.attitude->rate_dps.y * DEG_TO_RAD;
            hw_state_imu_angular_velocity_[2] = pi3hat_input_.attitude->rate_dps.z * DEG_TO_RAD;
            hw_state_imu_linear_acceleration_[0] = pi3hat_input_.attitude->accel_mps2.x;
            hw_state_imu_linear_acceleration_[1] = pi3hat_input_.attitude->accel_mps2.y;
            hw_state_imu_linear_acceleration_[2] = pi3hat_input_.attitude->accel_mps2.z;
        }

        // write() -> Read and update the actuator states if data is available
        if (result.rx_can_size > 0 && !result.error)
        {
            // give the input to the distribute_rx_input function
            distribute_rx_input(result);

            // TODO: refactor this loop to include all actuator types.
            for (auto i = 0u; i < hw_state_positions_.size(); i++)
            {
                // j is the index of the can frame in the pi3hat input
                switch (hw_actuator_can_protocols_[i])
                {
                //     // TODO: Add support for other CAN protocols
                // case CanProtocol::CHEETAH:
                // {
                //     int can_id = pi3hat_input_.rx_can[j].data[0];
                //     if (pi3hat_input_.rx_can[j].bus == hw_actuator_can_channels_[i] && can_id == hw_actuator_can_ids_[i])
                //     {
                //         // parse the can frame
                //         int p_int = (pi3hat_input_.rx_can[j].data[1] << 8) | pi3hat_input_.rx_can[j].data[2];
                //         int v_int = (pi3hat_input_.rx_can[j].data[3] << 4) | (pi3hat_input_.rx_can[j].data[4] >> 4);
                //         int i_int = ((pi3hat_input_.rx_can[j].data[4] & 0xF) << 8) | pi3hat_input_.rx_can[j].data[5];

                //     }
                //     break;
                // }
                case CanProtocol::MOTEUS:
                {
                    // Read the feedback from the actuators
                    hw_actuators_[i]->processRxFrames();

                    hw_state_positions_[i] = hw_actuators_[i]->getPosition();
                    hw_state_velocities_[i] = hw_actuators_[i]->getVelocity();
                    hw_state_efforts_[i] = hw_actuators_[i]->getEffort();
                    break;
                }

                case CanProtocol::ODRIVE:
                {
                    // Read the feedback from the actuators
                    hw_actuators_[i]->processRxFrames();

                    // Update the state interfaces
                    hw_state_positions_[i] = hw_actuators_[i]->getPosition();
                    hw_state_velocities_[i] = hw_actuators_[i]->getVelocity();
                    hw_state_efforts_[i] = hw_actuators_[i]->getEffort();
                    break;
                }
                default:
                    RCLCPP_ERROR(rclcpp::get_logger("Pi3HatHardwareInterface"), "Failed to receive: unknown CAN protocol");
                    break;
                }
            }
        }
        // else
        // {
        //     RCLCPP_INFO(rclcpp::get_logger("Pi3HatHardwareInterface"), "No CAN frames received");
        // }

        return hardware_interface::return_type::OK;
    }

    std::shared_ptr<mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>> Pi3HatHardwareInterface::allocateTxSpan(size_t size)
    {
        // Implementation to allocate a Span for Tx
        //* note: This could be more sophisticated to handle non-contiguous allocations
        size_t start = nextTxStart;
        if (start + size <= tx_can_frames_.size())
        {
            nextTxStart += size;
            return std::make_shared<mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>>(&tx_can_frames_[start], size);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("Pi3HatHardwareInterface"), "Something went wrong allocating the Tx Span....");
        }
    }

    int Pi3HatHardwareInterface::double_to_uint(double x, double x_min, double x_max,
                                                int bits)
    {
        /// Converts a double to an unsigned int, given range and number of bits ///
        double span = x_max - x_min;
        double offset = x_min;
        return (int)((x - offset) * ((double)((1 << bits) - 1)) / span);
    }

    double Pi3HatHardwareInterface::uint_to_double(int x_int, double x_min, double x_max,
                                                   int bits)
    {
        /// converts unsigned int to double, given range and number of bits ///
        double span = x_max - x_min;
        double offset = x_min;
        return ((double)x_int) * span / ((double)((1 << bits) - 1)) + offset;
    }

    double Pi3HatHardwareInterface::wrap_angle(double angle, double angle_min, double angle_max)
    {
        /// Wraps an angle to the range [angle_min, angle_max] ///
        double span = angle_max - angle_min;
        return angle - span * floor((angle - angle_min) / span);
    }

    double Pi3HatHardwareInterface::unwrap_angle(double angle, double prev_angle, double angle_min, double angle_max)
    {
        /// Deals with wrap-around for a continuously changing angle ///
        double span = angle_max - angle_min;
        double prev_angle_wrapped = wrap_angle(prev_angle, angle_min, angle_max);
        double d_angle = angle - prev_angle_wrapped;
        if (d_angle > span / 2)
        {
            d_angle -= span;
        }
        else if (d_angle < -span / 2)
        {
            d_angle += span;
        }
        return prev_angle + d_angle;
    }

    bool Pi3HatHardwareInterface::distribute_rx_input(mjbots::pi3hat::Pi3Hat::Output result)
    {
        bool any_rx = false;
        // loop through the input's rx_span and identify what bus the frame came from.
        for (auto i = 0u; i < result.rx_can_size; i++)
        {
            assign_frame(pi3hat_input_.rx_can[i]);
            any_rx = true;
        }
        return any_rx;
    }

    void Pi3HatHardwareInterface::assign_frame(mjbots::pi3hat::CanFrame frame)
    {
        uint32_t raw_id = frame.id;
        uint8_t bus = frame.bus;
        
        // First we assume that the raw_id represents a motor's ID (in the case of a Moteus)
        int index = Pi3HatHardwareInterface::findIndex(hw_actuator_can_ids_, raw_id);
        if (index >= 0 && index < hw_state_positions_.size() && hw_actuator_can_channels_[index] == bus) 
        {
            // if the index is valid, we add that frame to the actuator's rx_span
            hw_actuators_[index]->addRxFrame(frame);
        }

        // if we get here, it is likely that the raw_id isn't the node id. (in the case of an ODrive)
        uint32_t odrive_id = (raw_id >> 5);
        index = Pi3HatHardwareInterface::findIndex(hw_actuator_can_ids_, odrive_id);
        if (index >= 0 && index < hw_state_positions_.size() && 
                hw_actuator_can_protocols_[index] == CanProtocol::ODRIVE &&
                hw_actuator_can_channels_[index] == bus) 
        {
            // if the index is valid, we add that frame to the actuator's rx_span
            hw_actuators_[index]->addRxFrame(frame);
        } 
    }

    void Pi3HatHardwareInterface::update_state_interfaces()
    {
        // Update the state interfaces
        for (auto i = 0u; i < hw_state_positions_.size(); i++)
        {
            hw_state_positions_[i] = hw_actuators_[i]->getPosition();
            hw_state_velocities_[i] = hw_actuators_[i]->getVelocity();
            hw_state_efforts_[i] = hw_actuators_[i]->getEffort();
        }
    }

} // namespace pi3hat_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    pi3hat_hardware_interface::Pi3HatHardwareInterface, hardware_interface::SystemInterface)
