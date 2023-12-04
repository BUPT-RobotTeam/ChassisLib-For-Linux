#include "chassis_lib/base_chassis.h"
#include <string>

Chassis::Chassis(const std::string &pose_topic,const std::string &vel_topic) : Node("chassis_node")
{
    vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(vel_topic, 10, std::bind(&Chassis::setVelocity,this,std::placeholders::_1));
    pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic, 10, std::bind(&Chassis::updatePose,this,std::placeholders::_1));
    // Initialize the chassis
    // Initialize the motors
    // Initialize the encoders
    // Initialize the PID controllers
}

Chassis::~Chassis() {
    // Stop the motors
}

void Chassis::setVelocity(const geometry_msgs::msg::Twist & cmd_vel) {
    target_vel = cmd_vel;
}

void Chassis::updatePose(const geometry_msgs::msg::PoseStamped & pose) {
    current_pose = pose;
}

void Chassis::setParameter(const double &limit_vel, const double &limit_acc, const double &wheel_radius, const double &width, const double &length)
{
    this->limit_vel = limit_vel;
    this->limit_acc = limit_acc;
    this->wheel_radius = wheel_radius;
    this->width = width;
    this->length = length;
}
