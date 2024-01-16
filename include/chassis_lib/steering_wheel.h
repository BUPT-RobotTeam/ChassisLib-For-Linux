#ifndef STEERING_WHEEL_H
#define STEERING_WHEEL_H

#include "bupt_can/bupt_can.h"
#include <memory>
#include <array>
#include <chrono>

// 舵轮两个一组  左一组，右一组
class SteeringWheel
{
private:
    std::shared_ptr<Can> can_handle;
    uint32_t control_id; // 控制器ID
public:
    SteeringWheel(const uint32_t &control_id,const std::shared_ptr<Can> &can_handle);
    ~SteeringWheel();
    bool sendCommand(const std::array<int16_t,2> &angle,const std::array<int16_t,2> &vel);
};


#endif // STEERING_WHEEL_H