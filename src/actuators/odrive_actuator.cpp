/**
 * @file odrive_actuator.cpp
 * @author John Bush (johncbus@usc.edu)
 * @brief Implements ODriveActuator class (inherits from ActuatorBase)
 * @version 0.1
 * @date 2024-01-26
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "actuators/odrive_actuator.h"


bool ODriveActuator::on_init() {
    // TODO: Implement
    return true;
}

void ODriveActuator::setZero() {
    // todo: implement
    return;
}

void ODriveActuator::setState(ActuatorState state) {
    // translate input state to an odrive state
    if (motor_state_.error) {
        // if the motor is in an error state, it must be cleared before any other state can be entered
        // If the desired state is not disarm or ERROR, then the state is invalid
        state = ActuatorState::ERROR;
    }
    
    switch(state) {
        case ActuatorState::ERROR:
            ODriveActuator::ESTOP();
            break;
        case ActuatorState::DISARMED: // Disarmed can be accessed from any state
            odrive_can_.setState(tx_frames_[0], ODriveAxisState::AXIS_STATE_IDLE);
            motor_command_.commanded_actuator_state_ = ActuatorState::DISARMED;
            validateFrame(0);
            break;
        case ActuatorState::ARMED: // armed cannot be accessed from error state without clearing errors
            odrive_can_.setState(tx_frames_[0], ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            motor_command_.commanded_actuator_state_ = ActuatorState::ARMED;
            validateFrame(0);
            break;
        case ActuatorState::POSITION_MODE: // must be armed to enter position mode
        {
            std::printf("Setting ODrive %i State to Position Mode\n", can_id_);
            // if the previous state was not closed loop control, set it to closed loop control
            int frame_idx = 0;
            if (prev_axis_state_ != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
                odrive_can_.setState(tx_frames_[frame_idx], ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
                validateFrame(frame_idx);
                frame_idx++;
            }
            motor_command_.commanded_actuator_state_ = ActuatorState::POSITION_MODE;
            odrive_can_.setControllerMode(tx_frames_[frame_idx], ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
            validateFrame(frame_idx);
            break;
        }
        case ActuatorState::VELOCITY_MODE:
        {
            // if the previous state was not closed loop control, set it to closed loop control
            int frame_idx = 0;
            if (prev_axis_state_ != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
                odrive_can_.setState(tx_frames_[frame_idx], ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
                validateFrame(frame_idx);
                frame_idx++;
            }
            motor_command_.commanded_actuator_state_ = ActuatorState::VELOCITY_MODE;
            odrive_can_.setControllerMode(tx_frames_[frame_idx], ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
            validateFrame(frame_idx);
            break;
        }
        case ActuatorState::TORQUE_MODE:
        {
            // if the previous state was not closed loop control, set it to closed loop control
            int frame_idx = 0;
            if (prev_axis_state_ != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
                odrive_can_.setState(tx_frames_[frame_idx], ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
                validateFrame(frame_idx);
                frame_idx++;
            }
            motor_command_.commanded_actuator_state_ = ActuatorState::TORQUE_MODE;
            odrive_can_.setControllerMode(tx_frames_[frame_idx], ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
            validateFrame(frame_idx);
            break;
        }
        default:
            // TODO: ROS logger error undefined state
            break;
    }
    return;
}

void ODriveActuator::sendJointCommand(float position, float ff_velocity, float ff_torque) {
    if (motor_state_.current_actuator_state_ == ActuatorState::POSITION_MODE) {
        // calculate the servo command raw values
        float position_command = gear_ratio_ * direction_ * (position + zero_offset_);
        float ff_velocity = gear_ratio_ * direction_ * ff_velocity;
        float ff_torque = direction_ * ff_torque / gear_ratio_;

        // clamp the values to the limits
        position_command = std::clamp(position_command, pos_min_, pos_max_);
        ff_velocity = std::clamp(ff_velocity, -vel_max_, vel_max_);
        ff_torque = std::clamp(ff_torque, -max_torque_, max_torque_);

        odrive_can_.setPosition(tx_frames_[0], position_command, ff_velocity, ff_torque);
        validateFrame(0);

        // update motor command state
        motor_command_.position_ = position_command;
        motor_command_.ff_velocity_ = ff_velocity;
        motor_command_.ff_torque_ = ff_torque;
        return;
    } else {
        // TODO: log error
        // TODO: decide if this should change the actuator state?
        return;
    }

}

void ODriveActuator::setPosition(float position) {
    if (motor_state_.current_actuator_state_ == ActuatorState::POSITION_MODE) {
        odrive_can_.setPosition(tx_frames_[0], position);
        validateFrame(0);
        return;
    } else {
        // TODO: log error
        return;
    }
}

void ODriveActuator::setVelocity(float velocity) {
    if (motor_state_.current_actuator_state_ == ActuatorState::VELOCITY_MODE) {
        odrive_can_.setVelocity(tx_frames_[0], velocity);
        validateFrame(0);
        return;
    } else {
        // TODO: log error
        return;
    }
}

void ODriveActuator::setTorque(float torque) {
    if (motor_state_.current_actuator_state_ == ActuatorState::TORQUE_MODE) {
        odrive_can_.setTorque(tx_frames_[0], torque);
        validateFrame(0);
        return;
    } else {
        // TODO: log a warning that the actuator is not in torque mode and throw an error.
        return;
    }
}

void ODriveActuator::set_kp(float kp) {
    motor_command_.kp_ = kp;
    odrive_can_.setPosGain(tx_frames_[0], kp);
    validateFrame(0);

}

void ODriveActuator::set_kd(float kd) {
    motor_command_.kd_ = kd;
    odrive_can_.setVelGain(tx_frames_[0], kd, motor_command_.ki_); // ! what if ki is not set?
    validateFrame(0);

}

void ODriveActuator::set_ki(float ki) {
    motor_command_.ki_ = ki;
    odrive_can_.setVelGain(tx_frames_[0], motor_command_.kd_, ki); //! similar issue as above
    validateFrame(0);
}

void ODriveActuator::ESTOP() {
    odrive_can_.estop(tx_frames_[0]);
    validateFrame(0);

    motor_command_.commanded_actuator_state_ = ActuatorState::ERROR;
    motor_state_.error = true;

    // invalidate all other frames
    for (int i = 1; i < tx_frames_.size(); i++) {
        tx_frames_[i].valid = false;
    }

    return;
}

/**
 *  ODRIVE_ERROR_WATCHDOG_TIMER_EXPIRED      = 0x01000000,
    ODRIVE_ERROR_ESTOP_REQUESTED             = 0x02000000,
    ODRIVE_ERROR_SPINOUT_DETECTED            = 0x04000000,
    ODRIVE_ERROR_BRAKE_RESISTOR_DISARMED     = 0x08000000,
    ODRIVE_ERROR_THERMISTOR_DISCONNECTED     = 0x10000000,
    ODRIVE_ERROR_CALIBRATION_ERROR           = 0x40000000,
 */
// defined in ODriveEnums.h ODriveError
std::string ODriveActuator::printErrorMessage() {
    uint32_t error_code = motor_state_.error_reason;
    switch(error_code) {
        case 0x00000000:
            return "No Error";
        case 0x00000001:
            return "ODrive Error: Error Initializing";
        case 0x00000002:
            return "ODrive Error: System Level";
        case 0x00000004:
            return "ODrive Error: Timing Error";
        case 0x00000008:
            return "ODrive Error: Missing Estimate";
        case 0x00000010:
            return "ODrive Error: Bad Configuration";
        case 0x00000020:
            return "ODrive Error: DRV Fault";
        case 0x00000040:
            return "ODrive Error: Missing Input";
        case 0x00000100:
            return "ODrive Error: DC Bus Over Voltage";
        case 0x00000200:
            return "ODrive Error: DC Bus Under Voltage";
        case 0x00000400:
            return "ODrive Error: DC Bus Over Current";
        case 0x00000800:
            return "ODrive Error: DC Bus Over Regen Current";
        case 0x00001000:
            return "ODrive Error: Current Limit Violation";
        case 0x00002000:
            return "ODrive Error: Motor Over Temp";
        case 0x00004000:
            return "ODrive Error: Inverter Over Temp";
        case 0x00008000:
            return "ODrive Error: Velocity Limit Violation";
        case 0x00010000:
            return "ODrive Error: Position Limit Violation";
        case 0x01000000:
            return "ODrive Error: Watchdog Timer Expired";
        case 0x02000000:
            return "ODrive Error: ESTOP Requested";
        case 0x04000000:
            return "ODrive Error: Spinout Detected";
        case 0x08000000:
            return "ODrive Error: Brake Resistor Disarmed";
        case 0x10000000:
            return "ODrive Error: Thermistor Disconnected";
        case 0x40000000:
            return "ODrive Error: Calibration Error";
        default:
            return "ODrive Error: Unknown Error";
    }
    
}

void ODriveActuator::processRxFrames() {
    for (auto& frame : rx_frames_) {
        odrive_can_.readFrame(frame);
        updateStateVars();
    }
    // clear the rx frames
    rx_frames_.clear();
    return;
}

void ODriveActuator::sendQueryCommand() {
    //TODO: Implement
    return;
}

void ODriveActuator::setProperty() {
    // TODO: Implement
    return;
}

void ODriveActuator::clearErrors() {
    odrive_can_.clearErrors(tx_frames_[0]);
    validateFrame(0);
    return;
}

void ODriveActuator::readCANFrame(mjbots::pi3hat::CanFrame frame) {
    odrive_can_.readFrame(frame);
}

void ODriveActuator::invalidateSpan() {
    for (auto& frame : tx_frames_) {
        frame.valid = false;
    }
}

void ODriveActuator::validateFrame(int frame_index) {
    tx_frames_[frame_index].valid = true;
}

ActuatorState ODriveActuator::getActuatorState() {
    // create copy of the ODrive CAN state object
    ODriveCAN::ODriveState odrive_state_ = odrive_can_.getState();
    return ODriveActuator::convertToActuatorState(odrive_state_);
}

ActuatorState ODriveActuator::convertToActuatorState(ODriveCAN::ODriveState odrive_state_) {
    ODriveAxisState axis_state = odrive_state_.axis_state;
    ODriveError error = odrive_state_.error;
    ODriveControlMode control_mode = odrive_state_.control_mode;

    if (axis_state <= 1) { // state is UNDEFINED (0) or IDLE (1)
        if (error != ODriveError::ODRIVE_ERROR_NONE) {
            return ActuatorState::DISARMED;
        } else {
            return ActuatorState::ERROR;
        }
    } else if (axis_state == ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
        switch (control_mode) {
            case ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL:
                return ActuatorState::TORQUE_MODE;
            case ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL:
                return ActuatorState::VELOCITY_MODE;
            case ODriveControlMode::CONTROL_MODE_POSITION_CONTROL:
                return ActuatorState::POSITION_MODE;
            default:
                return ActuatorState::ARMED;
        }
    } else {
        // this would indicate the motor is in some sort of calibration state or something bad happened.
        throw std::runtime_error("There was an error converting the ODrive State."); 
    }
}

bool ODriveActuator::updateStateVars() {
    // copy the current state
    MotorState new_motor_state_ = motor_state_;
    MotorCommand new_motor_command_ = motor_command_;

    // get the current states from the ODrive
    ODriveCAN::ODriveMotorState odrive_motor_state_ = odrive_can_.getMotorState();
    ODriveCAN::ODriveState odrive_state_ = odrive_can_.getState();
    
    new_motor_state_.current_actuator_state_ = convertToActuatorState(odrive_state_);
    
    new_motor_state_.error = odrive_state_.error;
    new_motor_state_.position_ = odrive_motor_state_.estimated_position;
    new_motor_state_.velocity_ = odrive_motor_state_.estimated_velocity;
    new_motor_state_.torque_ = odrive_motor_state_.estimated_current * torque_const_;
    new_motor_state_.temperature = odrive_state_.temperature;

    // update if something changed
    if (motor_state_ != new_motor_state_ || motor_command_ != new_motor_command_) {
        motor_state_ = new_motor_state_;
        motor_command_ = new_motor_command_;
        return true;
    } else {
        return false;
    }
}

