#ifndef STEERING_CHASSIS_H
#define STEERING_CHASSIS_H

#include "chassis_lib/base_chassis.h"
#include "chassis_lib/steering_wheel.h"
#include <array>
#include <eigen3/Eigen/Dense>

// 4 steering wheel chassis

class SteeringChassis : public Chassis
{
private:
    std::array<std::unique_ptr<SteeringWheel>, 4> steering_wheels;
    std::array<int, 4> steering_wheel_yaw;
    std::array<int, 4> steering_wheel_vel;
    double limit_vel;
    double limit_acc;
    double wheel_radius;
    double width, length;
    Eigen::Matrix<double, 2, 3> mat[4];
public:
    SteeringChassis(const std::string &pose_topic,const std::string &vel_topic,const std::array<int, 4> &can_ids, const std::shared_ptr<Can> &can_handle = nullptr);
    ~SteeringChassis();
    void setParameter(const double &limit_vel,const double &limit_acc,const double &wheel_radius,const double &width,const double &length);
    void initialize();
    void execute();
};

#endif // STEERING_CHASSIS_H