#ifndef STEERING_CHASSIS_H
#define STEERING_CHASSIS_H

#include "chassis_lib/base_chassis.h"
#include "chassis_lib/steering_wheel.h"
#include <array>

// 4 steering wheel chassis

class SteeringChassis : public Chassis
{
private:
    std::array<std::unique_ptr<SteeringWheel>, 4> steering_wheels;
public:
    SteeringChassis(const std::array<int, 4> &can_ids, const std::shared_ptr<Can> &can_handle = nullptr);
    ~SteeringChassis();

    void initialize();
    void execute();
};

#endif // STEERING_CHASSIS_H