/**
 * @file actuator_base.cpp
 * @author John Bush (johncbus@usc.edu)
 * @brief Class implementation for ActuatorBase.
 * @version 0.1
 * @date 2024-01-26
 * 
 * 
 */

#include <iostream>
#include "actuators/actuator_base.h"

bool ActuatorBase::configure(const MotorConfig config) {
    can_id_ = config.can_id;
    can_bus_ = config.can_bus;
    name_ = config.name;
    direction_ = config.direction;
    zero_offset_ = config.zero_offset;
    max_torque_ = config.max_torque;
    torque_const_ = config.torque_const;
    pos_min_ = config.pos_min;
    pos_max_ = config.pos_max;
    vel_max_ = config.vel_max;
    kp_max_ = config.kp_max;
    kd_max_ = config.kd_max;
    ki_max_ = config.ki_max;

    soft_start_duration_ms_ = config.soft_start_duration_ms;
    gear_ratio_ = config.gear_ratio;

    if (config.can_id < 1 || config.can_id > 127) {
        //TODO Replace with ROS2 Logger warn
        std::cout << "Warning: CAN ID must be between 1 and 127. \nCAN ID reset to 31" << std::endl;
        can_id_ = 31;
    } 
    if (config.can_bus < 1 || config.can_bus > 4) {
        //TODO Replace with ROS2 Logger warn
        std::cout << "Warning: CAN Bus must be between 1 and 4. \nCAN Bus reset to 1" << std::endl;
        can_bus_ = 1;
    }
    if (config.direction != 1 && config.direction != -1) {
        //TODO Replace with ROS2 Logger warn
        std::cout << "Warning: Direction must be 1 or -1. \nDirection reset to 1" << std::endl;
        direction_ = 1;
    }
    if (config.gear_ratio <= 0) {
        //TODO Replace with ROS2 Logger warn
        std::cout << "Warning: Gear ratio must be positive. \nGear ratio reset to 1.0" << std::endl;
        gear_ratio_ = 1.0;
    }
    if (config.kp_max <= 0) {
        //TODO Logger warn
        std::cout << "Warning: Kp Max should be greater than 0. \nKp Limit reset to infinity" << std::endl;
        ki_max_ = std::numeric_limits<float>::infinity();
    }

    return true;
}

void ActuatorBase::invalidateSpan() {
    for (auto& frame : tx_frames_) {
        frame.valid = false;
    }
}