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

//bool MoteusActuator::configure(const MotorConfig config) {


void MoteusActuator::setState(ActuatorState state) {

    if (motor_state_.error ) {
        state = ActuatorState::ERROR;  
    }

    switch (state) {
        case ActuatorState::ERROR:
            tx_frames_[0] = convert_frame(moteus_controller_.MakeStop());
            motor_command_.commanded_actuator_state_ = ActuatorState::ERROR;
            break;
        // These cases fall through and do the same thing.
        case ActuatorState::POSITION_MODE:
        case ActuatorState::VELOCITY_MODE:
        case ActuatorState::TORQUE_MODE: 
        {
            // TODO: check if its okay to send a position command -> were we in an error state? etc.
            mjbots::moteus::PositionMode::Command cmd;
            cmd.position = std::numeric_limits<double>::quiet_NaN();
            cmd.velocity = 0.0;

            tx_frames_[0] = convert_frame(moteus_controller_.MakePosition(cmd));
            break;
        }

        case ActuatorState::DISARMED:
        {
            // todo
            tx_frames_[0] = convert_frame(moteus_controller_.MakeStop());
            motor_command_.commanded_actuator_state_ = ActuatorState::DISARMED;
            break;
        }
            
            

    }

    return;

}

//To do Set position, Set state

void MoteusActuator::setPosition( float position) {

    mjbots::moteus::PositionMode::Command cmd;

    cmd.position = position;
    cmd.velocity = 0.0;

    // TODO:
     tx_frames_[0] = convert_frame(moteus_controller_.MakePosition(cmd));

}

void MoteusActuator::setVelocity(float velocity) {

    mjbots::moteus::PositionMode::Command cmd;

    cmd.velocity = velocity;
    cmd.position = std::numeric_limits<double>::quiet_NaN();

    // TODO: 
    tx_frames_[0] = convert_frame(moteus_controller_.MakePosition(cmd));
}

void MoteusActuator::set_kp(float kp ) {

    mjbots::moteus::PositionMode::Command cmd;

    cmd.kp_scale = kp;

    // TODO: 
    tx_frames_[0] = convert_frame(moteus_controller_.MakePosition(cmd));
    
}

void MoteusActuator::set_kd(float kd ) {

    mjbots::moteus::PositionMode::Command cmd;

    cmd.kd_scale = kd;

    // TODO: 
    tx_frames_[0] = convert_frame(moteus_controller_.MakePosition(cmd));

}

void MoteusActuator::set_ki(float ki ) {

   return;
   
}



void MoteusActuator::processRxFrames() {
    for (int i = 0; i < rx_frames_.size(); i++) {
        if (rx_frames_[i].valid) {
            readCANFrame(rx_frames_[i]);
        }
    }

    // clear all frames
    rx_frames_.clear(); 
    
}

void MoteusActuator::sendJointCommand(float position, float ff_velocity, float ff_torque) {

    mjbots::moteus::PositionMode::Command cmd;

    cmd.feedforward_torque = ff_torque;

    cmd.position = position;

    cmd.velocity = ff_velocity;

    // TODO: 
    tx_frames_[0] = convert_frame(moteus_controller_.MakePosition(cmd));


}

// Note: probably wont use this function
void MoteusActuator::sendQueryCommand() {
    return;
}

void MoteusActuator::ESTOP() {

    moteus_controller_.MakeStop();
   

    motor_command_.commanded_actuator_state_ = ActuatorState::ERROR;
    motor_state_.error = true;

    // invalidate all other frames
    for (int i = 1; i < tx_frames_.size(); i++) {
        tx_frames_[i].valid = false;
    }

}

void MoteusActuator::readCANFrame(mjbots::pi3hat::CanFrame frame) {

    // 1. convert the incoming pi3hat frame to a moteus frame
    mjbots::moteus::CanFdFrame moteus_frame = convert_frame(frame);

    // 2. create a result object and parse the frame
    mjbots::moteus::Controller::Result result;
    result.frame = moteus_frame;
    result.values = Query::Parse(moteus_frame->data, moteus_frame->size);
    
    // 3. update the internal motor state variables

    /*
        appropriately update these:

        ActuatorState current_actuator_state_ = ActuatorState::DISARMED;
        bool connected = false;
        bool error = false;
        int error_reason = 0;
        float position_ = 0.0f;
        float velocity_ = 0.0f;
        float torque_ = 0.0f;
        float temperature = 0.0f;
    */




   return;
}


[[nodiscard]] mjbots::moteus::CanFdFrame MoteusActuator::convert_frame(mjbots::pi3hat::CanFrame frame) {

    mjbots::moteus::CanFdFrame moteus_frame;

    moteus_frame.arbitration_id = frame.id;
    moteus_frame.size = frame.size;
    moteus_frame.bus = frame.bus;
    moteus_frame.reply_required = frame.expect_reply;
    moteus_frame.data[64] = frame.data[64];


    return moteus_frame;

}


[[nodiscard]] mjbots::pi3hat::CanFrame MoteusActuator::convert_frame(mjbots::moteus::CanFdFrame frame) {

    mjbots::pi3hat::CanFrame pi3hat_frame;

    pi3hat_frame.id = frame.arbitration_id;
    pi3hat_frame.size = frame.size;
    pi3hat_frame.bus = frame.bus;
    pi3hat_frame.expect_reply = frame.reply_required;
    pi3hat_frame.data[64] = frame.data[64];



}