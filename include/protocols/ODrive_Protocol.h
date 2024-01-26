#pragma once

#include "ODriveCAN/ODriveCAN.h"
#include "ODriveCAN/ODriveEnums.h"
#include "ODriveCAN/can_helpers.hpp"
#include "ODriveCAN/can_simple_messages.hpp"

#include "actuator_base.h"

class ODrive_Protocol : public ActuatorBase {
public:
    // TODO: ODrive_Protocol();

    // Configuration function to initialize the PiHat for given CAN protocol and bus
    virtual bool configure(const MotorConfig config);

    // ****************************************************** 
    // Implementations of base class functions
    // ****************************************************** 

};
