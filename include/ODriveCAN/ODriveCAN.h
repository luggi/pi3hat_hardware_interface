/**
 * @file ODriveCAN.h
 * @author John Bush (johncbus@usc.edu)
 * @brief Frame constructors for ODrive CAN protocol. Adapted from ODrive Arduino Library
 * @version 0.1
 * @date 2024-01-26
 * 
 * 
 */
#pragma once

#include <time.h>

#include "ODriveEnums.h"
#include "can_helpers.hpp"
#include "can_simple_messages.hpp"
#include "pi3hat/pi3hat.h"


// #define DEBUG

#define REQUEST_PENDING 0xff

class ODriveCAN {
public:
    ODriveCAN() {
        bus_ = 0;
        node_id_ = 0;
    };
    
    ODriveCAN(uint32_t node_id, int bus)
        : node_id_(node_id), bus_(bus) {};
    
    // ****************************************************** 
    // Host -> ODrive Commands
    // ******************************************************
    /**
     * @brief Creates ESTOP frame to send to ODrive
     * 
     */
    void estop(mjbots::pi3hat::CanFrame& frame);
    [[nodiscard]] mjbots::pi3hat::CanFrame createEstopFrame();

    /**
     * @brief Creates Reboot frame command
     * 
     */
    void reboot(mjbots::pi3hat::CanFrame& frame);
    [[nodiscard]] mjbots::pi3hat::CanFrame createRebootFrame();

    /**
     * @brief Creates frame to set the absolute position of the motor
     * 
     */
    void setAbsPos(mjbots::pi3hat::CanFrame& frame, float position);
    [[nodiscard]] mjbots::pi3hat::CanFrame createSetAbsPosFrame(float position);

    /**
     * @brief Constructs frame to clear errors on ODrive
     * 
     */
    void clearErrors(mjbots::pi3hat::CanFrame& frame);
    [[nodiscard]] mjbots::pi3hat::CanFrame createClearErrorsFrame();

    /**
     * @brief Constructs frame to command ODrive to change its axis state.
     * 
     */
    void setState(mjbots::pi3hat::CanFrame& frame, ODriveAxisState requested_state);
    [[nodiscard]] mjbots::pi3hat::CanFrame createSetStateFrame(ODriveAxisState requested_state);

    /**
     * @brief Constructs frame to change control mode and input mode of the ODrive.
     * 
     */
    void setControllerMode(mjbots::pi3hat::CanFrame& frame, uint8_t control_mode, uint8_t input_mode);
    [[nodiscard]] mjbots::pi3hat::CanFrame createSetControllerModeFrame(uint8_t control_mode, uint8_t input_mode);

    /**
     * @brief Constructs CAN frame to send position setpoint with optional velocity and torque feedforward.
     * 
     */
    void setPosition(mjbots::pi3hat::CanFrame& frame, float position, float velocity_feedforward = 0.0f, float torque_feedforward = 0.0f);
    [[nodiscard]] mjbots::pi3hat::CanFrame createSetPositionFrame(float position, float velocity_feedforward = 0.0f, float torque_feedforward = 0.0f);
    
    /**
     * @brief Sends a velocity setpoint with optional torque feedforward.
     * 
     */
    void setVelocity(mjbots::pi3hat::CanFrame& frame, float velocity, float torque_feedforward = 0.0f);
    [[nodiscard]] mjbots::pi3hat::CanFrame createSetVelocityFrame(float velocity, float torque_feedforward = 0.0f);

    /**
     * @brief Sends a torque setpoint to the ODrive.
     * 
     */
    void setTorque(mjbots::pi3hat::CanFrame& frame, float torque);
    [[nodiscard]] mjbots::pi3hat::CanFrame createSetTorqueFrame(float torque);
    
    /**
     * @brief Sets the position gain of the ODrive.
     * 
     */
    void setPosGain(mjbots::pi3hat::CanFrame& frame, float pos_gain);
    [[nodiscard]] mjbots::pi3hat::CanFrame createSetPosGainFrame(float pos_gain);

    /**
     * @brief Sets the velocity gain of the ODrive.
     * 
     */
    void setVelGain(mjbots::pi3hat::CanFrame& frame, float vel_gain, float vel_integrator_gain);
    [[nodiscard]] mjbots::pi3hat::CanFrame createSetVelGainFrame(float vel_gain, float vel_integrator_gain);

    /**
     * @brief Controls the current and velocity limits of the ODrive
     * 
     */
    void setLimits(mjbots::pi3hat::CanFrame& frame, float current_lim, float vel_lim);
    [[nodiscard]] mjbots::pi3hat::CanFrame createSetLimitsFrame(float current_lim, float vel_lim);


    // ****************************************************** 
    // ODrive -> Host Frame Parsing Functions
    // ******************************************************
    struct ODriveMotorState {
        float estimated_position = 0.0f;
        float estimated_velocity = 0.0f;
        float estimated_current = 0.0f;
        float current_setpoint = 0.0f;
    };

    struct ODriveState {
        ODriveAxisState axis_state = AXIS_STATE_UNDEFINED;
        ODriveError error = ODRIVE_ERROR_NONE;
        ODriveControlMode control_mode = CONTROL_MODE_POSITION_CONTROL;
        ODriveInputMode input_mode = INPUT_MODE_PASSTHROUGH;
        float temperature = 0.0f;
    };

    /**
     * @brief Registers a callback for ODrive feedback processing.
     */
    void onFeedback(void (*callback)(Get_Encoder_Estimates_msg_t& feedback, void* user_data), void* user_data = nullptr) {
        feedback_callback_ = callback; 
        feedback_user_data_ = user_data;
    }

    /**
     * @brief Registers a callback for ODrive axis state feedback.
     */
    void onStatus(void (*callback)(Heartbeat_msg_t& feedback, void* user_data), void* user_data = nullptr) {
        axis_state_callback_ = callback; 
        axis_state_user_data_ = user_data;
    }

    /**
     * @brief Processes received CAN messages for the ODrive.
     */
    void onReceive(uint32_t id, uint8_t length, const uint8_t* data);

    /**
     * @brief Reads an incoming CAN frame and updates internal state.
     * 
     */
    void readFrame(mjbots::pi3hat::CanFrame frame);


    /**
     * @brief Get the class's Can Frame object
     * 
     * @return mjbots::pi3hat::CanFrame 
     */
    [[nodiscard]] mjbots::pi3hat::CanFrame getCanFrame() {return can_frame_;}

    /**
     * @brief Creates a CAN frame from a message
     */
    template<typename T>
    mjbots::pi3hat::CanFrame createFrame(T& msg) {
        mjbots::pi3hat::CanFrame out_frame;
        uint8_t data[8] = {};
        msg.encode_buf(out_frame.data);
        out_frame.size = msg.msg_length;
        out_frame.id = (node_id_ << ODriveCAN::kNodeIdShift) | msg.cmd_id;
        out_frame.bus = bus_;
        
        return out_frame;
    }

    template<typename T>
    void writeFrame(mjbots::pi3hat::CanFrame& frame, T& msg) {
        msg.encode_buf(frame.data);
        frame.size = msg.msg_length;
        frame.id = (node_id_ << ODriveCAN::kNodeIdShift) | msg.cmd_id;
        return;
    }
    

    // uint32_t id, uint8_t length, const uint8_t* data

    /*
    * CAN ID semantics (mirroring definition by linux/can.h):
    * |- Bit   31 : frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
    * |- Bit   30 : reserved (future remote transmission request flag)
    * |- Bit   29 : reserved (future error frame flag)
    * |- Bit 0-28 : CAN identifier (11/29 bit)
    */
        // uint32_t id;
        // uint8_t  data_length;
        // uint8_t  data[MAX_DATA_LENGTH];

    // CAN frame format constants
    static uint32_t constexpr CAN_EFF_FLAG    = 0x80000000U;
    static uint32_t constexpr CAN_SFF_MASK    = 0x000007FFU; /* standard frame format (SFF) */
    static uint32_t constexpr CAN_EFF_MASK    = 0x1FFFFFFFU; /* extended frame format (EFF) */

    inline bool isStandardId(uint32_t const id) {
        return ((id & CAN_EFF_FLAG) == 0);
    }
    inline bool isExtendedId(uint32_t const id) {
        return ((id & CAN_EFF_FLAG) == CAN_EFF_FLAG);
    }

    inline uint32_t CanStandardId(uint32_t const id) {
    return (id & CAN_SFF_MASK);
    }

    inline uint32_t CanExtendedId(uint32_t const id) {
    return (CAN_EFF_FLAG | (id & CAN_EFF_MASK));
    }

private:
    // internally maintained CAN frame
    mjbots::pi3hat::CanFrame can_frame_;

    uint32_t node_id_;
    int bus_ = 0;

    volatile uint8_t requested_msg_id_ = REQUEST_PENDING;

    uint8_t buffer_[8];

    static const uint8_t kNodeIdShift = 5;
    static const uint8_t kCmdIdBits = 0x1F;

    // internal state
    ODriveMotorState odrive_motor_state;
    ODriveState odrive_state;


    void* axis_state_user_data_;
    void* feedback_user_data_;
    
    void (*axis_state_callback_)(Heartbeat_msg_t& feedback, void* user_data) = nullptr;
    void (*feedback_callback_)(Get_Encoder_Estimates_msg_t& feedback, void* user_data) = nullptr;
};
