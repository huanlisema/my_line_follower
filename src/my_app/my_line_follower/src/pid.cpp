#include "my_line_follower/pid.hpp"
PID::PID(double kp, double ki, double kd)
{
    Kp = kp;
    Ki = ki;
    Kd = kd;
    SetPoint = 0.0;
    PTerm = 0.0;
    ITerm = 0.0;
    DTerm = 0.0;
    last_error = 0.0;
    windup_guard = 20.0;
    output = 0.0;
    last_time = std::chrono::steady_clock::now();
}
void PID::clear()
{
    SetPoint = 0.0;
    PTerm = 0.0;
    ITerm = 0.0;
    DTerm = 0.0;
    last_error = 0.0;
    output = 0.0;
    last_time = std::chrono::steady_clock::now();
}
void PID::update(double feedback_value)
{
    double error = SetPoint - feedback_value;//误差
    auto current_time = std::chrono::steady_clock::now();//获取当前时间戳
    double delta_time = std::chrono::duration<double>(current_time - last_time).count();//时间差，单位秒
    double delta_error = error - last_error;
    PTerm = Kp * error;
    ITerm += error * delta_time;
    if (ITerm < -windup_guard)//防止积分饱和，小车猛转头
    {
        ITerm = -windup_guard;
    }
    else if (ITerm > windup_guard)
    {
        ITerm = windup_guard;
    }
    DTerm = 0.0;
    if (delta_time > 0)
    {
        DTerm = delta_error / delta_time;
    }
    last_time = current_time;
    last_error = error;
    output = PTerm + Ki * ITerm + Kd * DTerm;
}
void PID::setKp(double kp)
{
    Kp = kp;
}

void PID::setKi(double ki)
{
    Ki = ki;
}

void PID::setKd(double kd)
{
    Kd = kd;
}

void PID::setWindup(double windup)
{
    windup_guard = windup;
}

double PID::getOutput() const
{
    return output;
}