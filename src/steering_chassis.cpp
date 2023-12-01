#include "chassis_lib/steering_chassis.h"

SteeringChassis::SteeringChassis(const std::array<int, 4> &can_ids, const std::shared_ptr<Can> &can_handle)
{
    for (int i = 0; i < 4; i++) {
        steering_wheels[i] = std::make_unique<SteeringWheel>(can_ids[i], can_handle);
    }
}

SteeringChassis::~SteeringChassis()
{
    // Stop the motor
    // Stop the encoder
    // Stop the PID controller
}

void SteeringChassis::initialize()
{
    // Initialize the steering chassis
}

void SteeringChassis::execute()
{
    // Execute the steering chassis
}
