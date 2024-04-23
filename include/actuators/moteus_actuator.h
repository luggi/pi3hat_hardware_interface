/**
 * @file odrive_actuator.h
 * @author Josiah Hickman (jhickman@usc.edu)
 * @brief Header file for Moteus actuator class (inherits from ActuatorBase)
 * @version 0.1
 * @date 2024-02-05
 * 
 * 
 */


#pragma once

#include <map>

#include "moteus.h" 
#include "actuator_base.h"
#include "moteus/MoteusEnums.h"

class MoteusActuator : public ActuatorBase {
public:
    MoteusActuator(
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
        const int soft_start_duration_ms  
    ): ActuatorBase(
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
        
        options.id = can_id;
        options.bus = can_bus;
        // other necessary options

        moteus_controller_ = mjbots::moteus::Controller(options);

    };

    ~MoteusActuator();
    
    // Configuration function to initialize the PiHat for given CAN protocol and bus
    bool configure(const MotorConfig config);

    // ****************************************************** 
    // Implementations of base class functions
    // ****************************************************** 

    // Virtual function for sending arbitrary CAN frame
    void createCANFrame();

    // Virtual function to command controller state

    bool on_init() override;
    void setState(ActuatorState state) override;
    void sendJointCommand(float position, float ff_velocity, float ff_torque) override;
    void setPosition(float position) override;
    void setVelocity(float velocity) override;
    void set_kp(float kp) override;
    void set_kd(float kd) override;
    void set_ki(float ki) override;
    void processRxFrames() override;
    void sendQueryCommand() override;
    void setProperty() override;
    void ESTOP() override;
    void readCANFrame(mjbots::pi3hat::CanFrame frame) override;
    void clearErrors() override;

protected:
    /**
     * @brief Translates from the interface enumeration to the Moteus axis state enumeration
     * 
     * @param state generic interface state enumeration
     * @return MoteusAxisState - Moteus axis state enumeration 
     */
    MoteusAxisState translateState(ActuatorState state);
    
private:

    mjbots::pi3hat::CanFrame convert_frame(mjbots::moteus::CanFdFrame frame);
    mjbots::moteus::CanFdFrame convert_frame(mjbots::pi3hat::CanFrame frame);

    mjbots::moteus::Controller::Options options;
    mjbots::moteus::Controller moteus_controller_;


};