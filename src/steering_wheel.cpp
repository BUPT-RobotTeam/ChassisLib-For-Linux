#include "chassis_lib/steering_wheel.h"

SteeringWheel::SteeringWheel(const uint32_t &control_id,const std::shared_ptr<Can> &can_handle) 
{
    this->control_id = control_id;
    this->can_handle = can_handle;
}

SteeringWheel::~SteeringWheel() {
    sendCommand({0,0},{0,0});
}

bool SteeringWheel::sendCommand(const std::array<int16_t,2> &angle,const std::array<int16_t,2> &vel) {
    std::array<uint8_t,8> data;
    data[0] = angle[0] & 0xff;
    data[1] = (angle[0] >> 8) & 0xff;
    data[2] = vel[0] & 0xff;
    data[3] = (vel[0] >> 8) & 0xff;
    data[4] = angle[1] & 0xff;
    data[5] = (angle[1] >> 8) & 0xff;
    data[6] = vel[1] & 0xff;
    data[7] = (vel[1] >> 8) & 0xff;
    can_handle->send_can(control_id,Can::CAN_ID_STD,8,data);
    return true;
}

