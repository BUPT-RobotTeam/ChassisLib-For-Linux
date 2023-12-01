#ifndef STEERING_WHEEL_H
#define STEERING_WHEEL_H

#include "bupt_can/bupt_can.h"
#include <memory>

class SteeringWheel
{
private:
    std::shared_ptr<Can> can_handle;
    int can_id;
public:
    SteeringWheel(const int &can_id,const std::shared_ptr<Can> &can_handle=nullptr);
    ~SteeringWheel();

    void sendCommand(const int &vel,const int &angle);
};


#endif // STEERING_WHEEL_H