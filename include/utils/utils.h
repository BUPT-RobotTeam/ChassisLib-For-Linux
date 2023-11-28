#ifndef UTILS_H_
#define UTILS_H_

#include <cmath>
#include <type_traits>

template<typename T>
typename std::enable_if<std::is_fundamental<T>::value, T>::type
clamp(const T &val, const T &min, const T &max)
{
    if(val < min)
        return min;
    else if(val > max)
        return max;
    else
        return val;
}

template<typename T>
typename std::enable_if<std::is_fundamental<T>::value, T>::type
sign(const T &val)
{
    if(val > 0)
        return 1;
    else if(val < 0)
        return -1;
    else
        return 0;
}

template<typename T>
typename std::enable_if<std::is_fundamental<T>::value, T>::type
normalizeRad(const T &rad)
{
    T normalized_rad = rad;
    while(normalized_rad > M_PI)
        normalized_rad -= 2 * M_PI;
    while(normalized_rad < -M_PI)
        normalized_rad += 2 * M_PI;
    return normalized_rad;
}

template<typename T>
typename std::enable_if<std::is_fundamental<T>::value, T>::type
normalizeDeg(const T &deg)
{
    T normalized_deg = deg;
    while(normalized_deg > 180.0)
        normalized_deg -= 360.0;
    while(normalized_deg < -180.0)
        normalized_deg += 360.0;
    return normalized_deg;
}

template<typename T>
typename std::enable_if<std::is_fundamental<T>::value, T>::type
radToDeg(const T &rad)
{
    return rad * 180.0 / M_PI;
}

template<typename T>
typename std::enable_if<std::is_fundamental<T>::value, T>::type
degToRad(const T &deg)
{
    return deg * M_PI / 180.0;
}

class PID
{
private:
    double kp, ki, kd;
    double error, last_error, integral;
    double limit_output;
public:
    PID(const double &kp, const double &ki, const double &kd, const double &limit_output)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->limit_output = limit_output;
    };
    ~PID() = default;
    double update(const double &target, const double &current, const double &dt)
    {
        error = target - current;
        integral += error * dt;
        double derivative = (error - last_error) / dt;
        last_error = error;
        double output = kp * error + ki * integral + kd * derivative;
        return clamp(output, -limit_output, limit_output);
    };
    void set_pid(const double &kp, const double &ki, const double &kd)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    };
    void reset()
    {
        integral = 0;
        last_error = 0;
        error = 0;
    };
};

#endif