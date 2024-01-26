#ifndef ACTUATOR_BASE_H
#define ACTUATOR_BASE_H

#include <stdint.h>
#include <string>
#include <vector>

#include "pi3hat/pi3hat.h"

// Abstract base class for CAN protocol strategies
class ActuatorBase {
public:
    // Constructor
    ActuatorBase(
        const std::string name,
        const int direction,
        const float zero_offset,
        const float gear_ratio,
        const float max_torque,
        const float torque_const,
        const float pos_min,
        const float pos_max,
        const int soft_start_duration_ms
    );

    // Virtual destructor to ensure proper cleanup of derived classes
    virtual ~ActuatorBase();

    // Configuration function to initialize the PiHat for given CAN protocol and bus
    virtual bool configure(const MotorConfig config) = 0;

    // ****************************************************** 
    // Virtual functions to be implemented by derived classes
    // ****************************************************** 

    // Virtual function for sending arbitrary CAN frame
    virtual void createCANFrame() = 0;

    /**
     * @brief Performs startup routines and connectivity check for the motor controller
     * 
     * @return true if startup was successful (motor controller is connected and ready)
     * @return false if startup failed (motor controller is not connected or had errors)
     */
    virtual bool on_init() = 0;

    // Virtual function to command controller state
    virtual void setState() = 0; 

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
     * @brief set the kp for pd loop
     * @param kp position gain in N m/rad
     */
    void set_kp(float kp){kp_ = kp;}

    /**
     * @brief set the kd for pd loop
     * @param kd velocity gain in N m s/rad
     */
    void set_kd(float kd){kd_ = kd;}

    /**
     * @brief set the ki for the pd loop
     * @param ki integral gain in N m s^2/rad
     * 
     */
    void set_ki(float ki){ki_ = ki;}

    // Virtual function for requesting data from the motor controller
    virtual void sendQueryCommand() = 0;

    // Virtual function for setting arbitrary properties of the motor controller
    virtual void setProperty() = 0;

    // Virtual function to stop motor (just a specific setState command and potentially set error flag)
    virtual void ESTOP() = 0;

    // Virtual function to read an arbitrary CAN frame
    virtual void readCANFrame() = 0;



    /**
     * @brief Creates a CAN frame to clear errors on the motor controller
     */
    virtual void clearErrors() = 0;

protected:
    // Actuator Parameters
    std::string name_ = "";         /// The joint name
    int direction_ = 1;             /// 1 or -1, flips positive rotation direction
    float zero_offset_ = 0.0f;      /// offset [rad] of joint zero position
    float gear_ratio_ = 1.0f;       /// Gear ratio joint to servo (1 is direct-drive)
    float torque_const_ = 0.0f;     /// Torque Constant (kt) [N m / A]

    int soft_start_duration_ms_ = 1;

    // Actuator State (motor state configuration)
    MotorState motor_state_;

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

    // Output CAN frame
    mjbots::pi3hat::CanFrame can_frame_;

private: 
    mjbots::pi3hat::Pi3Hat::Input pi3hat_input_;

};

/**
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
    std::string name = "";
    int direction = 1;
    float zero_offset = 0.0f;
    float gear_ratio = 1.0f;
    float max_torque = std::numeric_limits<float>::infinity();
    float torque_const = 0.0f;
    float pos_min = -std::numeric_limits<float>::infinity();
    float pos_max = std::numeric_limits<float>::infinity();
    int soft_start_duration_ms = 1;
};

struct MotorState {
    bool armed = false;
    bool error = false;
    bool calibration_ok = false;
    bool motor_ready = false;
    int current_state = 0;
    int control_mode = 0;
};

#endif // ACTUATOR_BASE_H