#include "chassis_lib/steering_wheel.h"

SteeringWheel::SteeringWheel(const int &can_id,const std::shared_ptr<Can> &can_handle) 
{
    this->can_id = can_id;
    if (can_handle == nullptr) {
        this->can_handle = std::make_shared<Can>("can0");
        can_handle->can_start();
    } else {
        this->can_handle = can_handle;
    }
}

SteeringWheel::~SteeringWheel() {
    // Stop the motor
    // Stop the encoder
    // Stop the PID controller
}

void SteeringWheel::sendCommand(const double &vel,const double &angle)
{

}

