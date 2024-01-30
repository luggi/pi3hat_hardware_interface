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

ODriveAxisState ODriveActuator::translateState(ActuatorState state) {

}

bool ODriveActuator::on_init() {
    // TODO: Implement
}

void ODriveActuator::setState(ActuatorState state) {
    // translate input state to an odrive state
    if (motor_state_.error && state != ActuatorState::ERROR || state == ActuatorState::DISARMED) {
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
            motor_command_.actuator_state_ = ActuatorState::DISARMED;
            break;
        case ActuatorState::ARMED: // armed cannot be accessed from error state without clearing errors
            odrive_can_.setState(tx_frames_[0], ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            motor_command_.actuator_state_ = ActuatorState::ARMED;
            break;
        case ActuatorState::POSITION_MODE: // must be armed to enter position mode
        {
            // if the previous state was not closed loop control, set it to closed loop control
            int frame_idx = 0;
            if (prev_axis_state_ != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
                odrive_can_.setState(tx_frames_[frame_idx], ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
                validateFrame(frame_idx);
                frame_idx++;
            }
            motor_command_.actuator_state_ = ActuatorState::POSITION_MODE;
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
            motor_command_.actuator_state_ = ActuatorState::VELOCITY_MODE;
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
            motor_command_.actuator_state_ = ActuatorState::TORQUE_MODE;
            odrive_can_.setControllerMode(tx_frames_[frame_idx], ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
            validateFrame(frame_idx);
            break;
        }
        default:
            // TODO: ROS logger error undefined state
            break;
    }
}

void ODriveActuator::sendJointCommand(float position, float ff_velocity, float ff_torque) {
    odrive_can_.setPosition(tx_frames_[0], position, ff_velocity, ff_torque);
    validateFrame(0);
}

void ODriveActuator::setPosition(float position) {
    odrive_can_.setPosition(tx_frames_[0], position);
    validateFrame(0);

}

void ODriveActuator::setVelocity(float velocity) {
    odrive_can_.setVelocity(tx_frames_[0], velocity);
    validateFrame(0);

}

void ODriveActuator::set_kp(float kp) {
    motor_command_.kp_ = kp;
    odrive_can_.setPosGain(tx_frames_[0], kp);
    validateFrame(0);

}

void ODriveActuator::set_kd(float kd) {
    motor_command_.kd_ = kd;
    odrive_can_.setVelGain(tx_frames_[0], kd, 0.0f); //! this will make ki 0 which is not desired
    validateFrame(0);

}

void ODriveActuator::set_ki(float ki) {
    motor_command_.ki_ = ki;
    odrive_can_.setVelGain(tx_frames_[0], 0.0f, ki); //! same issue as above
    validateFrame(0);
}

void ODriveActuator::ESTOP() {
    odrive_can_.estop(tx_frames_[0]);
    validateFrame(0);

    motor_command_.actuator_state_ = ActuatorState::ERROR;
    motor_state_.error = true;

    // invalidate all other frames
    for (int i = 1; i < tx_frames_.size(); i++) {
        tx_frames_[i].valid = false;
    }
}

void ODriveActuator::readCANFrame(mjbots::pi3hat::CanFrame frame) {

}

void ODriveActuator::invalidateSpan() {
    for (auto& frame : tx_frames_) {
        frame.valid = false;
    }
}

void ODriveActuator::validateFrame(int frame_index) {
    tx_frames_[frame_index].valid = true;
}

