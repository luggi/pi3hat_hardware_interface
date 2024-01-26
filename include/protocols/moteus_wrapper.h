#pragma once

// #include "moteus_protocol.h" 
#include "protocols/actuator_base.h"

class Moteus_Protocol : public ActuatorBase {
public:
    Moteus_Protocol();

    ~Moteus_Protocol();

    // Configuration function to initialize the PiHat for given CAN protocol and bus
    bool configure(const MotorConfig config);

    // ****************************************************** 
    // Implementations of base class functions
    // ****************************************************** 

    // Virtual function for sending arbitrary CAN frame
    void createCANFrame();

    // Virtual function to command controller state

};