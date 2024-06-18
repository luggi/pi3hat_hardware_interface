#ifndef ACTUATOR_BASE_H
#define ACTUATOR_BASE_H

#include <stdint.h>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <limits>

#include "pi3hat/pi3hat.h"

#define NaN std::numeric_limits<double>::quiet_NaN();

// ******************************************************
// Structs and Enums
// ******************************************************
/**
 * @param can_id        /// the can id of this joint's motor (range depends on controller type)
 * @param can_bus       /// the can bus the motor communicates on [1-4]
 * @param name          /// The joint name
 * @param direction     /// 1 or -1, flips positive rotation direction (Default:1)
 * @param zero_offset   /// offset [rad] of joint zero position (Default:0) 
 * @param gear_ratio    /// Gear ratio joint to servo (ratio>1 means slower joint) (Default:1.0)
 * @param max_torque    /// Maximum torque of the joint [N m] (Default:inf)
 * @param torque_const  /// Torque Constant (kt) [N m / A] (Default:0.0)
 * @param pos_min       /// Minimum joint limit before taking protective measures such as torque limiting or shut off (Default:-inf)
 * @param pos_max       /// Maximum joint limit before taking protective measures such as torque limiting or shut off (Default:inf)
 * @param soft_start_duration_ms /// Duration of torque limit ramp (soft start) in ms
 * 
 */
struct MotorConfig {
    int can_id = 0;
    int can_bus = 0;
    std::string name = "";
    int direction = 1;
    float zero_offset = 0.0f;
    float gear_ratio = 1.0f;
    float torque_const = 0.0f;
    
    float max_torque = std::numeric_limits<float>::infinity();
    float pos_min = -std::numeric_limits<float>::infinity();
    float pos_max = std::numeric_limits<float>::infinity();
    float vel_max = std::numeric_limits<float>::infinity();
    float kp_max = std::numeric_limits<float>::infinity();
    float kd_max = std::numeric_limits<float>::infinity();
    float ki_max = std::numeric_limits<float>::infinity();
    int soft_start_duration_ms = 1;
};

/**
 * @brief Enumeration of basic motor states exposed to a user
 * 
 */
enum ActuatorState {
    ERROR           = 0,
    DISARMED        = 1,
    ARMED           = 2,
    POSITION_MODE   = 3,
    VELOCITY_MODE   = 4,
    TORQUE_MODE     = 5,
};

struct MotorState { 
    ActuatorState current_actuator_state_ = ActuatorState::DISARMED;
    uint8_t internal_mode_ = 0;
    bool connected = false;
    bool error = false;
    uint32_t error_reason = 0;
    float position_ = 0.0f;
    float velocity_ = 0.0f;
    float torque_ = 0.0f;
    // set to quiet_NaN because they are not always available
    float temperature_ = std::numeric_limits<float>::quiet_NaN();
    float voltage_ = std::numeric_limits<float>::quiet_NaN();

    bool operator==(const MotorState& other) const {
        return current_actuator_state_ == other.current_actuator_state_ &&
                internal_mode_ == other.internal_mode_ &&
                connected == other.connected &&
                error == other.error &&
                error_reason == other.error_reason &&
                position_ == other.position_ &&
                velocity_ == other.velocity_ &&
                torque_ == other.torque_ &&
                temperature_ == other.temperature_ &&
                voltage_ == other.voltage_;
    }

    bool operator!=(const MotorState& other) const {
        return !(*this == other);
    }
};

struct MotorCommand {
    ActuatorState commanded_actuator_state_ = ActuatorState::DISARMED;
    float position_ = 0.0f;
    float velocity_ = 0.0f;
    float torque_ = 0.0f;
    float ff_velocity_ = 0.0f;
    float ff_torque_ = 0.0f;
    float kp_ = 0.0f;
    float kd_ = 0.0f;
    float ki_ = 0.0f;

    bool operator==(const MotorCommand& other) const {
        return commanded_actuator_state_ == other.commanded_actuator_state_ &&
               position_ == other.position_ &&
               velocity_ == other.velocity_ &&
               torque_ == other.torque_ &&
               ff_velocity_ == other.ff_velocity_ &&
               ff_torque_ == other.ff_torque_ &&
               kp_ == other.kp_ &&
               kd_ == other.kd_ &&
               ki_ == other.ki_;
    }

    bool operator!=(const MotorCommand& other) const {
        return !(*this == other);
    }
};

// ******************************************************
// Actuator Base Class for CAN protocol strategies
// ******************************************************

class ActuatorBase {
public:
    // Constructor
    ActuatorBase(
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
        : can_id_(can_id), can_bus_(can_bus), name_(name), direction_(direction), zero_offset_(zero_offset),
      max_torque_(max_torque), gear_ratio_(gear_ratio), torque_const_(torque_const),
      pos_min_(pos_min), pos_max_(pos_max), vel_max_(vel_max), kp_max_(kp_max), kd_max_(kd_max),
      ki_max_(ki_max), soft_start_duration_ms_(soft_start_duration_ms)
    {};

    // Virtual destructor to ensure proper cleanup of derived classes
    virtual ~ActuatorBase() {};

    // Configuration function to initialize the PiHat for given CAN protocol and bus
    virtual bool configure(const MotorConfig config);

    // ****************************************************** 
    // Virtual functions to be implemented by derived classes
    // ****************************************************** 

    /**
     * @brief Performs startup routines and connectivity check for the motor controller
     * 
     * @return true if startup was successful (motor controller is connected and ready)
     * @return false if startup failed (motor controller is not connected or had errors)
     */
    virtual bool on_init() = 0;

    /**
     * @brief Set the State object
     * 
     * @param state 
     */
    virtual void setState(ActuatorState state) = 0;

    /**
     * @brief Get the current state of the motor controller
     * 
     * @return ActuatorState 
     */
    virtual ActuatorState getState() {return motor_state_.current_actuator_state_;};

    /**
     * @brief Get the torque of the motor 
     * 
     * @return double 
     */
    virtual double getEffort() {return motor_state_.torque_;};

    /**
     * @brief Get the Velocity of the motor
     * 
     * @return double 
     */
    virtual double getVelocity() {return motor_state_.velocity_;};

    /**
     * @brief Get the Position of the motor
     * 
     * @return double 
     */
    virtual double getPosition() {return motor_state_.position_;};

    /**
     * @brief Get the Temperature of the motor
     * @default NaN
     * 
     * @return double
     */
    virtual double getTemperature() {return motor_state_.temperature_;};

    /**
     * @brief Get the bus voltage of the motor controller
     * @default NaN
     * @return double 
     */
    virtual double getVoltage() {return motor_state_.voltage_;};

    /**
     * @brief Get the error state of the motor controller
     * 
     * @return 32 bit integer representing the error state specific to the motor controller
     */
    virtual uint32_t getErrorReason() {return motor_state_.error_reason;};

    /**
     * @brief Get the internal mode of the motor controller (READ ONLY)
     * 
     * @return uint8_t motor controller specific mode
     */
    virtual uint8_t getInternalMode() {return motor_state_.internal_mode_;};
    
    /**
     * @brief Sets the zero position of the joint
     * 
     */
    virtual void setZero() = 0;

    /**
     * @brief Forms a CAN frame to send a position command to the motor controller
     * 
     * @param position      /// position of the joint [rad]
     * @param ff_velocity   /// feedforward velocity of the joint [rad/s]
     * @param ff_torque     /// feedforward torque for the joint [N m]
     */
    virtual void sendJointCommand(float position, float ff_velocity, float ff_torque) = 0;

    /**
     * @brief Forms a CAN frame to send a position command to the motor controller
     * 
     * @param position      /// position of the joint [rad]
     */
    virtual void setPosition(float position) = 0;

    /**
     * @brief Forms a CAN frame to send a velocity command to the motor controller
     * 
     * @param velocity      /// velocity of the joint [rad/s]
     */
    virtual void setVelocity(float velocity) = 0;

    /**
     * @brief Commands the actuator with a torque command
     * 
     * @param torque (Nm)
     */
    virtual void setTorque(float torque) = 0;

    /**
     * @brief set the kp for pd loop
     * @param kp position gain in N m/rad
     */
    virtual void set_kp(float kp){kp_ = kp;}

    /**
     * @brief set the kd for pd loop
     * @param kd velocity gain in N m s/rad
     */
    virtual void set_kd(float kd){kd_ = kd;}

    /**
     * @brief set the ki for the pd loop
     * @param ki integral gain in N m s^2/rad
     * 
     */
    virtual void set_ki(float ki){ki_ = ki;}

    virtual void processRxFrames() = 0;

    // Virtual function for requesting data from the motor controller
    virtual void sendQueryCommand() = 0;

    // Virtual function for setting arbitrary properties of the motor controller
    virtual void setProperty() = 0;

    // Virtual function to stop motor (just a specific setState command and potentially set error flag)
    virtual void ESTOP() = 0;

    // Virtual function to read an arbitrary CAN frame
    virtual void readCANFrame(mjbots::pi3hat::CanFrame frame) = 0;

    /**
     * @brief Creates a CAN frame to clear errors on the motor controller
     */
    virtual void clearErrors() = 0;

    virtual MotorState getMotorState() {return motor_state_;}

    virtual void setTxSpan(std::shared_ptr<mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>> tx_frames) = 0;
    
    /**
     * @brief Marks the frames in the actuator's Tx span as invalid
     * 
     */
    virtual void invalidateSpan();

    /**
     * @brief Adds a received CAN frame to the actuator's rx_frames_ buffer
     * 
     * @param frame 
     */
    virtual void addRxFrame(mjbots::pi3hat::CanFrame frame);

    virtual bool updateStateVars() = 0;

    virtual std::string printErrorMessage() = 0;


protected:
    

    // Actuator Parameters
    int can_id_ = 0;                /// the can id of this joint's motor (range depends on controller type)
    int can_bus_ = 0;               /// the can bus the motor communicates on [1-4]
    std::string name_ = "";         /// The joint name
    int direction_ = 1;             /// 1 or -1, flips positive rotation direction
    float zero_offset_ = 0.0f;      /// offset [rad] of joint zero position
    float gear_ratio_ = 1.0f;       /// Gear ratio joint to servo (1 is direct-drive)
    float torque_const_ = 0.0f;     /// Torque Constant (kt) [N m / A]

    int soft_start_duration_ms_ = 1;

    // Actuator State (motor state configuration)
    MotorState motor_state_;
    MotorCommand motor_command_;
    
    // Joint State/Commands (includes params like gear ratio, direction and offset)
    float position_;                /// position of the joint [rad]
    float velocity_;                /// velocity of the joint [rad/s]
    float torque_ = 0;              /// Constrained torque for the joint [N m]
    float measured_torque_ = 0;     /// Measured joint torque [N m]

    // PD setpoints and gain scales
    float kp_ = 0;                  ///< Value of kp for joint [N m / rad]
    float kd_ = 0;                  ///< Value of kd for joint [N m s/ rad]
    float ki_ = 0;                  ///< Value of ki for joint [N m s^2 / rad]
    float position_target_ = 0;     ///< Target joint position [rad]
    float velocity_target_ = 0;     ///< Target joint velocity [rad/s]

    // Servo State/Commands (the raw motor values)
    float servo_position_;          /// position of the servo [rad]
    float servo_velocity_;          /// velocity of the servo [rad/s]
    float servo_torque_ = 0;        /// torque command for servo [N m]
    float measured_servo_torque_ = 0;  /// Measured servo torque [N m]

    // Limits
    float max_torque_ = std::numeric_limits<float>::infinity();
    float pos_min_ = -std::numeric_limits<float>::infinity();
    float pos_max_ = std::numeric_limits<float>::infinity();
    float vel_max_ = std::numeric_limits<float>::infinity();
    float kp_max_ = std::numeric_limits<float>::infinity();
    float kd_max_ = std::numeric_limits<float>::infinity();
    float ki_max_ = std::numeric_limits<float>::infinity();
 
// TODO: how are we handling this input object? It should be treated like a queue, Unless
// it gets fully flushed every cycle command? I think that it does get flushed by cycle, so we
// can instead fill it from index 0 every command.
    mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> tx_frames_;
    // rx_frames_ 

    std::vector<mjbots::pi3hat::CanFrame> rx_frames_;

};



#endif //ACTUATOR_BASE_H