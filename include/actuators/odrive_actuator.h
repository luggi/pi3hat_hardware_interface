/**
 * @file odrive_actuator.h
 * @author John Bush (johncbus@usc.edu)
 * @brief Header file for ODriveActuator class (inherits from ActuatorBase)
 * @version 0.1
 * @date 2024-01-26
 * 
 * 
 */

#pragma once

#include <map>

#include "ODriveCAN/ODriveCAN.h"
#include "ODriveCAN/ODriveEnums.h"
#include "ODriveCAN/can_helpers.hpp"
#include "ODriveCAN/can_simple_messages.hpp"

#include "actuator_base.h"

class ODriveActuator : public ActuatorBase {
public:
    ODriveActuator(
        int can_id,
        int can_bus,
        const std::string name,
        const int direction,
        const float zero_offset,
        const float gear_ratio,
        const float torque_const,
        const float max_torque,
        const float pos_min,
        const float pos_max,
        const float vel_max,
        const float kp_max,
        const float kd_max,
        const float ki_max,
        const int soft_start_duration_ms)
        : ActuatorBase(
            can_id,
            can_bus,
            name,
            direction,
            zero_offset,
            gear_ratio,
            torque_const,
            max_torque,
            pos_min,
            pos_max,
            vel_max,
            kp_max,
            kd_max,
            ki_max,
            soft_start_duration_ms) 
    {
        odrive_can_ = ODriveCAN(can_id, can_bus);
    };

    // ****************************************************** 
    // Enumerations for state translation
    // ******************************************************
    bool on_init() override;
    void setState(ActuatorState state) override;
    void sendJointCommand(float position, float ff_velocity, float ff_torque) override;
    void setPosition(float position) override;
    void setVelocity(float velocity) override;
    void setTorque(float torque) override;  
    void set_kp(float kp) override;
    void set_kd(float kd) override;
    void set_ki(float ki) override;
    void sendQueryCommand() override;
    void setProperty() override;
    void ESTOP() override;
    void readCANFrame(mjbots::pi3hat::CanFrame frame) override;
    void clearErrors() override;

    void setTxSpan(std::shared_ptr<mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>> tx_frames) override
    {
        this->tx_frames_ = *tx_frames;

        // loop through span and set bus
        for (size_t i = 0; i < tx_frames_.size(); i++) {
            tx_frames_[i].bus = this->can_bus_;
        }
    };

    /**
     * @brief Class specific function to invalidate the tx span frames
     * 
     * Since the ODrive needs a multi-frame span to handle series of commands,
     * we mark the frames within the constant sized span valid or invalid. The
     * Pi3Hat only processes valid frames. this is to reduce bus usage.
     */
    void invalidateSpan();

protected:
    /**
     * ERROR           = 0,
        DISARMED        = 1,
        ARMED           = 2,
        POSITION_MODE   = 3,
        VELOCITY_MODE   = 4,
        TORQUE_MODE     = 5, 
     * 
     */
    std::map<ActuatorState, ODriveAxisState> actuatorToODriveMap = {
        {ERROR, AXIS_STATE_UNDEFINED},
        {DISARMED, AXIS_STATE_IDLE},
        {ARMED, AXIS_STATE_CLOSED_LOOP_CONTROL},
        {POSITION_MODE, AXIS_STATE_CLOSED_LOOP_CONTROL},
        {VELOCITY_MODE, AXIS_STATE_CLOSED_LOOP_CONTROL},
        {TORQUE_MODE, AXIS_STATE_CLOSED_LOOP_CONTROL},
        // ... continue mapping other states
    };

private:
    void validateFrame(int index);

    ODriveCAN odrive_can_;
    ODriveAxisState prev_axis_state_ = AXIS_STATE_UNDEFINED;
};
