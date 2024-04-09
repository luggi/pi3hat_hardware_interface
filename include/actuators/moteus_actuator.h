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

// #include "moteus_protocol.h" 
#include "actuator_base.h"
#include "moteus/MoteusEnums.h"

class Moteus_Protocol : public ActuatorBase {
public:
    Moteus_Protocol(
        int can_id,
        int can_bus,
        const std::string name,
        const int direction,
        const float zero_offset,
        const float gear_ratio,
        const float max_torque,
        const float torque_const,
        const float pos_min,
        const float pos_max,
        const int soft_start_duration_ms  
    ): ActuatorBase(
        can_id,
            can_bus,
            name,
            direction,
            zero_offset,
            gear_ratio,
            max_torque,
            torque_const,
            pos_min,
            pos_max,
            soft_start_duration_ms)
    {
        
        options.id = can_id;
        options.bus = can_bus;
        // other necessary options

        moteus_controller_ = moteus::Controller(options);

    };

    ~Moteus_Protocol();
    
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
    /**
     * ERROR           = 0,
        DISARMED        = 1,
        ARMED           = 2,
        POSITION_MODE   = 3,
        VELOCITY_MODE   = 4, (Omitted)
        TORQUE_MODE     = 5, 
     * 
     */
    std::map<ActuatorState, MoteusAxisState> actuatorToMoteusMap = {
        {ERROR, kFault},
        {DISARMED, kstopped},
        {ARMED,kstopped},
        {POSITION_MODE,kPosition},
        {VELOCITY_MODE,kPosition},
        {TORQUE_MODE,kCurrent},
    
    };
private:

moteus::Controller::Options options;
moteus::Controller moteus_controller_;


};