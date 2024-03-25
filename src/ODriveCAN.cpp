// Author: ODrive Robotics Inc.
// License: MIT
// Documentation: https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html

#include "ODriveCAN.h"
#include "pi3hat/pi3hat.h"

// #include <Arduino.h> // needed for debug printing //! will need to update this for the ROS2 Logger

void ODriveCAN::estop(mjbots::pi3hat::CanFrame& frame) {
    Estop_msg_t estop_msg;
    writeFrame(frame, estop_msg);
}

mjbots::pi3hat::CanFrame ODriveCAN::createEstopFrame() {
    Estop_msg_t estop_msg;
    return createFrame(estop_msg);
}

void ODriveCAN::reboot(mjbots::pi3hat::CanFrame& frame) {
    Reboot_msg_t reboot_msg;
    writeFrame(frame, reboot_msg);
}

mjbots::pi3hat::CanFrame ODriveCAN::createRebootFrame() {
    Reboot_msg_t reboot_msg;
    return createFrame(reboot_msg);
}

void ODriveCAN::setAbsPos(mjbots::pi3hat::CanFrame& frame, float position) {
    Set_Absolute_Position_msg_t abs_pos_msg;
    abs_pos_msg.Position = position;
    writeFrame(frame, abs_pos_msg);
}

mjbots::pi3hat::CanFrame ODriveCAN::createSetAbsPosFrame(float position) {
    Set_Absolute_Position_msg_t abs_pos_msg;
    abs_pos_msg.Position = position;
    return createFrame(abs_pos_msg);
}

void ODriveCAN::clearErrors(mjbots::pi3hat::CanFrame& frame) {
    Clear_Errors_msg_t clear_errors_msg;
    writeFrame(frame, clear_errors_msg);
}

mjbots::pi3hat::CanFrame ODriveCAN::createClearErrorsFrame() {
    Clear_Errors_msg_t clear_errors_msg;
    return createFrame(clear_errors_msg);
}

void ODriveCAN::setState(mjbots::pi3hat::CanFrame& frame, ODriveAxisState requested_state) {
    Set_Axis_State_msg_t set_axis_state_msg;
    set_axis_state_msg.Axis_Requested_State = requested_state;
    writeFrame(frame, set_axis_state_msg);
}

mjbots::pi3hat::CanFrame ODriveCAN::createSetStateFrame(ODriveAxisState requested_state) {
    Set_Axis_State_msg_t set_axis_state_msg;
    set_axis_state_msg.Axis_Requested_State = requested_state;
    return createFrame(set_axis_state_msg);
}

void ODriveCAN::setControllerMode(mjbots::pi3hat::CanFrame& frame, uint8_t control_mode, uint8_t input_mode) {
    Set_Controller_Mode_msg_t set_controller_mode_msg;
    set_controller_mode_msg.Control_Mode = control_mode;
    set_controller_mode_msg.Input_Mode = input_mode;
    writeFrame(frame, set_controller_mode_msg);
}

mjbots::pi3hat::CanFrame ODriveCAN::createSetControllerModeFrame(uint8_t control_mode, uint8_t input_mode) {
    Set_Controller_Mode_msg_t set_controller_mode_msg;
    set_controller_mode_msg.Control_Mode = control_mode;
    set_controller_mode_msg.Input_Mode = input_mode;
    return createFrame(set_controller_mode_msg);
}

void ODriveCAN::setPosition(mjbots::pi3hat::CanFrame& frame, float position, float velocity_feedforward, float torque_feedforward) {
    Set_Input_Pos_msg_t input_pos_msg;
    input_pos_msg.Input_Pos = position;
    input_pos_msg.Vel_FF = velocity_feedforward;
    input_pos_msg.Torque_FF = torque_feedforward;
    writeFrame(frame, input_pos_msg);
}

mjbots::pi3hat::CanFrame ODriveCAN::createSetPositionFrame(float position, float velocity_feedforward, float torque_feedforward) {
    Set_Input_Pos_msg_t input_pos_msg;
    input_pos_msg.Input_Pos = position;
    input_pos_msg.Vel_FF = velocity_feedforward;
    input_pos_msg.Torque_FF = torque_feedforward;
    return createFrame(input_pos_msg);
}

void ODriveCAN::setVelocity(mjbots::pi3hat::CanFrame& frame, float velocity, float torque_feedforward) {
    Set_Input_Vel_msg_t input_vel_msg;
    input_vel_msg.Input_Vel = velocity;
    input_vel_msg.Input_Torque_FF = torque_feedforward;
    writeFrame(frame, input_vel_msg);
}

mjbots::pi3hat::CanFrame ODriveCAN::createSetVelocityFrame(float velocity, float torque_feedforward) {
    Set_Input_Vel_msg_t input_vel_msg;
    input_vel_msg.Input_Vel = velocity;
    input_vel_msg.Input_Torque_FF = torque_feedforward;
    return createFrame(input_vel_msg);
}

void ODriveCAN::setTorque(mjbots::pi3hat::CanFrame& frame, float torque) {
    Set_Input_Torque_msg_t input_torque_msg;
    input_torque_msg.Input_Torque = torque;
    writeFrame(frame, input_torque_msg);
}

mjbots::pi3hat::CanFrame ODriveCAN::createSetTorqueFrame(float torque) {
    Set_Input_Torque_msg_t input_torque_msg;
    input_torque_msg.Input_Torque = torque;
    return createFrame(input_torque_msg);
}

void ODriveCAN::setPosGain(mjbots::pi3hat::CanFrame& frame, float pos_gain) {
    Set_Pos_Gain_msg_t set_pos_gain_msg;
    set_pos_gain_msg.Pos_Gain = pos_gain;
    writeFrame(frame, set_pos_gain_msg);
}

mjbots::pi3hat::CanFrame ODriveCAN::createSetPosGainFrame(float pos_gain) {
    Set_Pos_Gain_msg_t set_pos_gain_msg;
    set_pos_gain_msg.Pos_Gain = pos_gain;
    return createFrame(set_pos_gain_msg);
}

void ODriveCAN::setVelGain(mjbots::pi3hat::CanFrame& frame, float vel_gain, float vel_integrator_gain) {
    Set_Vel_Gains_msg_t set_vel_gain_msg;
    set_vel_gain_msg.Vel_Gain = vel_gain;
    set_vel_gain_msg.Vel_Integrator_Gain = vel_integrator_gain;
    writeFrame(frame, set_vel_gain_msg);
}

mjbots::pi3hat::CanFrame ODriveCAN::createSetVelGainFrame(float vel_gain, float vel_integrator_gain) {
    Set_Vel_Gains_msg_t set_vel_gain_msg;
    set_vel_gain_msg.Vel_Gain = vel_gain;
    set_vel_gain_msg.Vel_Integrator_Gain = vel_integrator_gain;
    return createFrame(set_vel_gain_msg);
}

void ODriveCAN::setLimits(mjbots::pi3hat::CanFrame& frame, float current_lim, float vel_lim) {
    Set_Limits_msg_t set_limits_msg;
    set_limits_msg.Current_Limit = current_lim;
    set_limits_msg.Velocity_Limit = vel_lim;
    writeFrame(frame, set_limits_msg);
}

mjbots::pi3hat::CanFrame ODriveCAN::createSetLimitsFrame(float current_lim, float vel_lim) {
    Set_Limits_msg_t set_limits_msg;
    set_limits_msg.Current_Limit = current_lim;
    set_limits_msg.Velocity_Limit = vel_lim;
    return createFrame(set_limits_msg);
}

void ODriveCAN::readFrame(mjbots::pi3hat::CanFrame frame) 
{
    uint32_t id = frame.id;
    uint8_t length = frame.size;
    uint8_t* data = frame.data;
    
    // guard to ensure we only process frames from the correct node
    if (node_id_ != (id >> ODriveCAN::kNodeIdShift)) return;

    switch (id & ODriveCAN::kCmdIdBits) 
    {
        // Read encoder estimates
        case Get_Encoder_Estimates_msg_t::cmd_id: 
        {
            Get_Encoder_Estimates_msg_t estimates;
            estimates.decode_buf(data);
            odrive_motor_state.estimated_position = estimates.Pos_Estimate;
            odrive_motor_state.estimated_velocity = estimates.Vel_Estimate;
            break;
        }
        // Read the heartbeat message from the ODrive
        case Heartbeat_msg_t::cmd_id: 
        {
            Heartbeat_msg_t status;
            status.decode_buf(data);
            odrive_state.axis_state = static_cast<ODriveAxisState>(status.Axis_State);
            odrive_state.error = static_cast<ODriveError>(status.Axis_Error);
            break;
        }
        // Read the temperature of the ODrive
        case Get_Temperature_msg_t::cmd_id: 
        {
            Get_Temperature_msg_t temperature;
            temperature.decode_buf(data);
            odrive_state.temperature = temperature.FET_Temperature;
            break;
        }
        // Read the current and setpoint current of the ODrive
        case Get_Iq_msg_t::cmd_id:
        {
            Get_Iq_msg_t iq;
            iq.decode_buf(data);
            odrive_motor_state.estimated_current = iq.Iq_Measured;
            odrive_motor_state.current_setpoint = iq.Iq_Setpoint;
            break;
        }
        default: {
            if (requested_msg_id_ == REQUEST_PENDING) return;
        }
    }


}


void ODriveCAN::onReceive(uint32_t id, uint8_t length, const uint8_t* data) {
#ifdef DEBUG
    int byte_index = length - 1;
    Serial.println("received:");
    Serial.print("  id: 0x");
    Serial.println(id, HEX);
    Serial.print("  data: 0x");
    while (byte_index >= 0) Serial.print(msg.data[byte_index--], HEX);
    Serial.println("");
#endif // DEBUG
    if (node_id_ != (id >> ODriveCAN::kNodeIdShift)) return;
    switch(id & ODriveCAN::kCmdIdBits) {
        case Get_Encoder_Estimates_msg_t::cmd_id: {
            Get_Encoder_Estimates_msg_t estimates;
            estimates.decode_buf(data);
            if (feedback_callback_) feedback_callback_(estimates, feedback_user_data_);
            break;
        }
        case Heartbeat_msg_t::cmd_id: {
            Heartbeat_msg_t status;
            status.decode_buf(data);
            if (axis_state_callback_ != nullptr) axis_state_callback_(status, axis_state_user_data_);
            // else Serial.println("missing callback");
            break;
        }
        default: {
            if (requested_msg_id_ == REQUEST_PENDING) return;
#ifdef DEBUG
            Serial.print("waiting for: 0x");
            Serial.println(requested_msg_id_, HEX);
#endif // DEBUG
            if ((id & ODriveCAN::kCmdIdBits) != requested_msg_id_) return;
            memcpy(buffer_, data, length);
            requested_msg_id_ = REQUEST_PENDING;
        }
    }
}

int64_t GetNow() {
  struct timespec ts = {};
  ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000ll +
      static_cast<int64_t>(ts.tv_nsec);
}

