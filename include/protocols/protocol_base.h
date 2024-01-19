#ifndef PROTOCOL_BASE_H
#define PROTOCOL_BASE_H

#include <stdint.h>
#include <vector>

#include "pi3hat/pi3hat.h"

// Abstract base class for CAN protocol strategies
class CAN_Protocol {
public:
    // Virtual destructor to ensure proper cleanup of derived classes
    virtual ~CAN_Protocol() {}

    // Configuration function to initialize the PiHat for given CAN protocol and bus
    virtual bool configure(const std::vector<pi3hat::Configuration> & config) = 0;

    // ****************************************************** 
    // Virtual functions to be implemented by derived classes
    // ****************************************************** 

    // Virtual function for sending arbitrary CAN frame
    virtual void createCANFrame() = 0;

    // Virtual function to command controller state
    // TODO: acheive through 
    virtual void setState() = 0; 

    // Virtual function for commanding motor
    virtual void sendJointCommand() = 0;

    // Virtual function for requesting data from the motor controller
    virtual void sendQueryCommand() = 0;

    // Virtual function for setting arbitrary properties of the motor controller
    virtual void setProperty() = 0;

    // Virtual function to stop motor
    virtual void ESTOP() = 0;



};

#endif // PROTOCOL_BASE_H