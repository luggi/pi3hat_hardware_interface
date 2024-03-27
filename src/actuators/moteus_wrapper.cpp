/**
 * @file odrive_actuator.h
 * @author Josiah Hickman (jhickman@usc.edu)
 * @brief Header file for Moteus actuator class (inherits from ActuatorBase)
 * @version 0.1
 * @date 2024-02-05
 * 
 * 
 */

#include "actuators/moteus_wrapper.h"

//bool Moteus_Protocol::configure(const MotorConfig config) {


void Moteus_Protocol::setState(ActuatorState state) {

    if (motor_state_.error ) {
        state = ActuatorState::ERROR;  
    }

    switch (state) {
        case ActuatorState::ERROR:
            tx_frames_[0] = moteus_actuator_.MakeStop();
            motor_command_.commanded_actuator_state_ = ActuatorState::ERROR;
            break;
        // These cases fall through and do the same thing.
        case ActuatorState::POSITION_MODE:
        case ActuatorState::VELOCITY_MODE:
        case ActuatorState::TORQUE_MODE: 
        {
            // TODO: check if its okay to send a position command -> were we in an error state? etc.
            moteus::PositionMode::Command cmd;
            cmd.position = std::numeric_limits<double>::quiet_NaN();
            cmd.velocity = 0.0;

            tx_frames_[0] = moteus_actuator_.MakePosition(cmd);
        }
            
            

    }

}

void Moteus_Protocol::setPosition( float position) {

    moteus_actuator_.SetPosition(tx_frames_[0],position);
     // validateFrame(0); 
     //Void Function validate frame used in Odrive Actuator Set Position but has no moteus definition
     // Does an equivalence need to be written?



}

void Moteus_Protocol::setVelocity(float velocity) {

    moteus_actuator_

}

void Moteus_Protocol::set_kp(float kp ) {

    motor_command_.kp_ = kp;
    //OdriveCan has a setPosgain function but moteus doesn't so is the written code sufficient?
}

void Moteus_Protocol::set_kd(float kd ) {

    motor_command_.kd_ = kd;
    //OdriveCan has a setVelgain function but moteus doesn't so is the written code sufficient?
}

void Moteus_Protocol::set_ki(float ki ) {

    motor_command_.ki_ = ki;
    //OdriveCan has a setVelgain function but moteus doesn't so is the written code sufficient?
}

void Moteus_Protocol::ESTOP() {

    moteus_actuator.MakeStop()
    //Is make Stop or Make brake preferred?
    //moteus_actuator.MakeBrake()

    motor_command_.actuator_state_ = ActuatorState::ERROR;
    motor_state_.error = true;

    // invalidate all other frames
    for (int i = 1; i < tx_frames_.size(); i++) {
        tx_frames_[i].valid = false;
    }

}

void Moteus_Protocol::readCANFrame(mjbots::pi3hat::CanFrame frame) {

  //  moteu_actuator.DoRead( frame );
  



}