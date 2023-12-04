#include "chassis_lib/omni_chassis.h"
#include "utils/utils.h"

OmniChassis::OmniChassis(const std::string &pose_topic,const std::string &vel_topic,
                        const uint32_t &board_id,const std::shared_ptr<Can> &can_handle)
                            : Chassis(pose_topic,vel_topic)
{
    limit_acc = 1.0;
    limit_vel = 1.0;
    dji_board = std::make_shared<DJIBoard>(board_id,can_handle);
}

OmniChassis::~OmniChassis()
{
    for (int i=1;i<=4;i++)
    {
        dji_board->MotorOff(i);
    }
}

void OmniChassis::initialize()
{
    for (int i=1;i<=4;i++)
    {
        dji_board->MotorOn(i);
        dji_board->VelCfg(i);
    }
}

void OmniChassis::execute()
{
    // 限幅
    clamp(target_vel.linear.x,-limit_vel,limit_vel); 
    clamp(target_vel.linear.y,-limit_vel,limit_vel);
    clamp(target_vel.angular.z,-limit_vel,limit_vel);

    // 解算轮速
    wheel_speed[0] = - target_vel.linear.y - target_vel.angular.z * (width / 2);
    wheel_speed[1] = target_vel.linear.x - target_vel.angular.z * (length / 2);
    wheel_speed[2] = target_vel.linear.y + target_vel.angular.z * (width / 2);
    wheel_speed[3] = - target_vel.linear.x + target_vel.angular.z * (length / 2);

    // 发送控制报文
    for (int i=1;i<=4;i++)
    {
        dji_board->VelCtrl(i,wheel_speed[i-1]);
    }
}