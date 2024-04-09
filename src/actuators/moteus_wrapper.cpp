/**
 * @file odrive_actuator.h
 * @author Josiah Hickman (jhickman@usc.edu)
 * @brief Header file for Moteus actuator class (inherits from ActuatorBase)
 * @version 0.1
 * @date 2024-02-05
 * 
 * 
 */

#include "actuators/moteus_actuator.h"
#include "moteus.h"

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

//To do Set position, Set state

void Moteus_Protocol::setPosition( float position) {

    moteus::PositionMode::Command cmd;

    cmd.position = position;
    cmd.velocity = 0.0;

    tx_frames[0] = moteus_actuator;




}

void Moteus_Protocol::setVelocity(float velocity) {

    moteus::PositionMode::Command cmd;

    cmd.velocity = velocity;
    cmd.position = std::numeric_limits<double>::quiet_NaN();

    tx_frames[0] = moteus_actuator;

}

void Moteus_Protocol::set_kp(float kp ) {

    moteus::PositionMode::Command cmd;

    cmd.kp_scale = kp;

    tx_frames[0] = moteus_actuator;
    
}

void Moteus_Protocol::set_kd(float kd ) {

    moteus::PositionMode::Command cmd;

    cmd.kd_scale = kd;

    tx_frames[0] = moteus_actuator;

}

//TO DO: Implement KI
/*void Moteus_Protocol::set_ki(float ki ) {

    moteus::PositionMode::Command cmd;

    cmd.feedforward_torque = ki;

    tx_frames[0] = moteus_actuator;
   
}

*/

void Moteus_Protocol::sendJointCommand(float position, float ff_velocity, float ff_torque) {

    moteus::PositionMode::Command cmd;

    cmd.feedforward_torque = ff_torque;

    cmd.position = position;

    cmd.velocity = ff_velocity;

    tx_frames[0] = moteus_actuator;


}

//TO DO: Implement Send Query Command, What is the function of this? Is Generic Query ok for this?
void Moteus_Protocol::sendQueryCommand() {

    //moteus::GenericQuery

}





void Moteus_Protocol::ESTOP() {

    moteus_actuator.MakeStop()
   

    motor_command_.actuator_state_ = ActuatorState::ERROR;
    motor_state_.error = true;

    // invalidate all other frames
    for (int i = 1; i < tx_frames_.size(); i++) {
        tx_frames_[i].valid = false;
    }

}

void Moteus_Protocol::readCANFrame(mjbots::pi3hat::CanFrame frame) {

    
  



}