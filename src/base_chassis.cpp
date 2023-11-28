#include "chassis_lib/base_chassis.h"

Chassis::Chassis() {
    // Initialize the chassis
    // Initialize the motors
    // Initialize the encoders
    // Initialize the PID controllers
}

Chassis::~Chassis() {
    // Stop the motors
}

void Chassis::setVelocity(const geometry_msgs::msg::Twist::SharedPtr & cmd_vel) {
    target_vel = *cmd_vel;
}

void Chassis::updatePose(const geometry_msgs::msg::PoseStamped::SharedPtr & pose) {
    current_pose = *pose;
}


