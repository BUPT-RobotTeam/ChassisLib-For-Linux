#ifndef CHASSIS_H
#define CHASSIS_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>

class Chassis : public rclcpp::Node
{
protected:
    geometry_msgs::msg::Twist target_vel;
    geometry_msgs::msg::PoseStamped current_pose;

    double limit_vel,limit_acc; // 速度限制、加速度限制
    double width,length; // 车体宽度、长度
    double wheel_radius; // 驱动轮半径
    double ratio; // 减速比

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    rclcpp::TimerBase::SharedPtr timer;
public:
    Chassis(const std::string &pose_topic,const std::string &vel_topic);
    ~Chassis() = default;
    void setVelocity(const geometry_msgs::msg::Twist & cmd_vel);
    void updatePose(const geometry_msgs::msg::PoseStamped & pose);
    
    virtual void setParameter(const double &limit_vel, const double &limit_acc,
                              const double &width, const double &length,
                              const double &wheel_radius, const double &ratio);
    virtual void initialize()=0;
    virtual void execute()=0;
};

#endif // CHASSIS_H