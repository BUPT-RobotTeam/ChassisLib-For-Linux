#include "chassis_lib/steering_chassis.h"
#include "utils/utils.h"
#include <eigen3/Eigen/Dense>

SteeringChassis::SteeringChassis(const std::string &pose_topic,const std::string &vel_topic,
                                const std::array<int, 4> &can_ids, const std::shared_ptr<Can> &can_handle)
                                    : Chassis(pose_topic,vel_topic)
{
    limit_vel = 1.0;
    limit_acc = 1.0;
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

void SteeringChassis::setParameter(const double &limit_vel, const double &limit_acc, const double &wheel_radius, const double &width, const double &length)
{
    this->limit_vel = limit_vel;
    this->limit_acc = limit_acc;
    this->wheel_radius = wheel_radius;
    this->width = width;
    this->length = length;

    this->mat[0] << 1, 0, cos(M_PI_2 + atan(width / length)),
              0, 1, sin(M_PI_2 + atan(width / length));
    this->mat[1] << 1, 0, cos(-M_PI_2 - atan(width / length)),
              0, 1, sin(-M_PI_2 - atan(width / length));
    this->mat[2] << 1, 0, cos(-atan(length / width)),
              0, 1, sin(-atan(length / width));
    this->mat[3] << 1, 0, cos(atan(length / width)),
              0, 1, sin(atan(length / width));
}


void SteeringChassis::initialize()
{
    // Initialize the steering chassis
}

void SteeringChassis::execute()
{
    clamp(target_vel.linear.x, -limit_vel, limit_vel);
    clamp(target_vel.linear.y, -limit_vel, limit_vel);
    clamp(target_vel.angular.z, -limit_vel, limit_vel);

    // 解算角速度
    Eigen::Vector3<double> cmd_vel;
    Eigen::Vector2<double> vel[4];

    cmd_vel << target_vel.linear.x,target_vel.linear.y,target_vel.angular.z;

    for (int i = 0 ; i < 4 ; i++)
    {
        vel[i] = mat[i] * cmd_vel;
        steering_wheels[i]->sendCommand(vel[i].norm(),atan2(vel[i](1),vel[i](0)));
    }
}
