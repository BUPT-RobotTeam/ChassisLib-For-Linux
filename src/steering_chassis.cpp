#include "chassis_lib/steering_chassis.h"
#include "utils/utils.h"
#include <eigen3/Eigen/Dense>

const uint32_t SteeringChassis::CONTROL_L = 0x300;
const uint32_t SteeringChassis::CONTROL_R = 0x301;
const uint32_t SteeringChassis::HEART_BEAT_ID = 0x302;
const std::array<int16_t,4> SteeringChassis::bias = {-88,-120,120,30};

SteeringChassis::SteeringChassis(const std::string &pose_topic,const std::string &vel_topic,
                                 const std::shared_ptr<Can> &can_handle)
                                 : Chassis(pose_topic,vel_topic)
{
    // 逆时针顺序编号
    steering_wheels[0] = std::make_unique<SteeringWheel>(CONTROL_L,can_handle);  //左前左后
    steering_wheels[1] = std::make_unique<SteeringWheel>(CONTROL_R,can_handle);  //右后右前
    last_angle = bias;
    sw_init_flag=false;

    can_handle->register_msg(HEART_BEAT_ID,Can::CAN_ID_STD,
        [this](const std::shared_ptr<can_frame> &frame)
        {
            auto feature = frame->data[0];
            RCLCPP_INFO(this->get_logger(),"Processing heartbeat Packet. The Packet Feature is %d",feature);
            switch (feature)
            {
                case 1:
                    sw_state[0] = true;
                    break;
                case 2:
                    sw_state[1] = true;
                    break;
                case 4:
                    sw_state[2] = true;
                    break;
                case 8:
                    sw_state[3] = true;
            }
            if (!sw_init_flag.load())
                if (sw_state[0] && sw_state[1] && sw_state[2] && sw_state[3])
                {
                    sw_init_cv.notify_all();
                    sw_init_flag.store(true);
                }
            //else
            //TODO: Update Timestamp
                
        }
    );
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

    double theta = atan(length/width);
    double r=sqrt(width*width+length*length)/2;
    mat[0]<< 1, 0, r*cos(M_PI-theta),
             0, 1, r*sin(M_PI-theta);
    mat[1]<< 1, 0, r*cos(theta-M_PI),
             0, 1, r*sin(theta-M_PI);
    mat[2]<< 1, 0, r*cos(-theta),
             0, 1, r*sin(-theta);
    mat[3]<< 1, 0, r*cos(theta),
             0, 1, r*sin(theta);      
                        
}


void SteeringChassis::initialize()
{
    std::mutex mtx;
    std::unique_lock<std::mutex> lock(mtx);
    sw_init_cv.wait(lock,[this](){return sw_init_flag.load();});
    RCLCPP_INFO(this->get_logger(),"Steering chassis initialized!");
}

void SteeringChassis::execute()
{
    clamp(target_vel.linear.x, -limit_vel, limit_vel);
    clamp(target_vel.linear.y, -limit_vel, limit_vel);
    clamp(target_vel.angular.z, -limit_vel, limit_vel);

    // 解算角速度
    Eigen::Vector3<double> cmd_vel;
    Eigen::Vector2<double> vel[4];

    if (target_vel.linear.x == 0 && target_vel.linear.y == 0 && target_vel.angular.z == 0)
    {
        for (int i = 0 ; i < 4 ; i++)
        {
            vel[i] << 0,0;
        }
        std::array<std::array<int16_t,2>,2> send_vel_buffer;
        std::array<std::array<int16_t,2>,2> send_angle_buffer;
        for (int i = 0 ; i < 4 ; i++)
        {
            send_vel_buffer[i/2][i%2]=0;
            send_angle_buffer[i/2][i%2]=last_angle[i]*10;
        }
        steering_wheels[0]->sendCommand(send_angle_buffer[0],send_vel_buffer[0]);
        steering_wheels[1]->sendCommand(send_angle_buffer[1],send_vel_buffer[1]);
        return;
    }

    cmd_vel << target_vel.linear.x,target_vel.linear.y,target_vel.angular.z;

    std::array<std::array<double,2>,2> send_vel;
    std::array<std::array<double,2>,2> send_angle;
    std::array<std::array<int16_t,2>,2> send_vel_buffer;
    std::array<std::array<int16_t,2>,2> send_angle_buffer;
    for (int i = 0 ; i < 4 ; i++)
    {
        vel[i] = mat[i] * cmd_vel;

        send_vel[i/2][i%2]=vel[i].norm();
        send_vel[i/2][i%2]=send_vel[i/2][i%2]/(2*M_PI*wheel_radius)*60;

        send_angle[i/2][i%2]=-(radToDeg(atan2(vel[i](1),vel[i](0))))+bias[i];
        send_angle[i/2][i%2]=normalizeDeg(send_angle[i/2][i%2]);

        if (abs(normalizeDeg(send_angle[i/2][i%2]-last_angle[i])) > abs(normalizeDeg(send_angle[i/2][i%2]+180-last_angle[i])))
        {
            send_angle[i/2][i%2]=normalizeDeg(send_angle[i/2][i%2]+180),send_vel[i/2][i%2]=-send_vel[i/2][i%2];
            RCLCPP_INFO(this->get_logger(),"Steering wheel %d reverse! %f",i,send_angle[i/2][i%2]+bias[i]);
        }
        
        last_angle[i]=send_angle[i/2][i%2];

        send_vel_buffer[i/2][i%2]=send_vel[i/2][i%2]*6;
        send_angle_buffer[i/2][i%2]=send_angle[i/2][i%2]*10;
    }
        

    steering_wheels[0]->sendCommand(send_angle_buffer[0],send_vel_buffer[0]);
    steering_wheels[1]->sendCommand(send_angle_buffer[1],send_vel_buffer[1]);
}
