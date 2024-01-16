#ifndef STEERING_CHASSIS_H
#define STEERING_CHASSIS_H

#include "chassis_lib/base_chassis.h"
#include "chassis_lib/steering_wheel.h"
#include <array>
#include <mutex>
#include <atomic>
#include <eigen3/Eigen/Dense>

// 4 steering wheel chassis

class SteeringChassis : public Chassis
{
public:
    static const uint32_t CONTROL_L;
    static const uint32_t CONTROL_R;
    static const uint32_t HEART_BEAT_ID;
private:
    std::array<std::unique_ptr<SteeringWheel>, 2> steering_wheels;

    std::array<std::atomic_bool, 4> sw_state; 
    std::condition_variable sw_init_cv;
    std::atomic_bool sw_init_flag;
    double limit_vel;
    double limit_acc;
    double wheel_radius;
    double width, length;
    Eigen::Matrix<double, 2, 3> mat[4];
    
public:
    SteeringChassis(const std::string &pose_topic,const std::string &vel_topic, const std::shared_ptr<Can> &can_handle);
    ~SteeringChassis();
    void setParameter(const double &limit_vel,const double &limit_acc,const double &wheel_radius,const double &width,const double &length);
    void initialize();
    void execute();
};

#endif // STEERING_CHASSIS_H