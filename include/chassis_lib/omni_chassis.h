#ifndef OMNI_CHASSIS_H
#define OMNI_CHASSIS_H

#include "base_chassis.h"
#include "motor_lib/dji_board.h"
#include "bupt_can/bupt_can.h"
#include <memory>
#include <cstdint>
#include <utility>
#include <array>

// Omni-Wheel Chassis, Driver: DJI RoboMaster M3508

class OmniChassis : public Chassis
{
private:
    std::shared_ptr<DJIBoard> dji_board;
    std::array<int16_t,4> wheel_speed;
public:
    OmniChassis(const std::string &pose_topic,const std::string &vel_topic,const uint32_t &board_id,const std::shared_ptr<Can> &can_handle = nullptr);
    ~OmniChassis();
    void initialize();
    void execute();
};


#endif // OMNI_CHASSIS_H
