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

ActuatorBase::ActuatorBase(
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
        const int soft_start_duration_ms)
    : can_id_(can_id), can_bus_(can_bus), name_(name), direction_(direction), zero_offset_(zero_offset),
      max_torque_(max_torque), gear_ratio_(gear_ratio), torque_const_(torque_const),
      pos_min_(pos_min), pos_max_(pos_max), soft_start_duration_ms_(soft_start_duration_ms)
{
    if (gear_ratio <= 0)
    {
        //TODO Replace with ROS2 Logger warn
        std::cout << "Warning: Gear ratio must be positive. \nGear ratio reset to 1.0" << std::endl;
        gear_ratio_ = 1.0;
    }
}

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

    return true;
}