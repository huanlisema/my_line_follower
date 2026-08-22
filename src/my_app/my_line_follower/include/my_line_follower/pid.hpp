#ifndef PID_HPP
#define PID_HPP

#include <chrono>

class PID
{
public:
    // 构造函数
    PID(double kp, double ki, double kd);
    // 更新PID
    void update(double feedback_value);
    // 清空PID状态
    void clear();
    // 设置参数
    void setKp(double kp);
    void setKi(double ki);
    void setKd(double kd);
    // 设置积分限幅
    void setWindup(double windup);
    // 获取输出
    double getOutput() const;
private:
    double Kp;
    double Ki;
    double Kd;
    double SetPoint;//目标值
    double PTerm;//P项计算结果,PTerm = Kp × error
    double ITerm;//积分项累计值,ITerm += error × delta_time
    double DTerm;//微分项,DTerm = Δerror / Δtime
    double last_error;//上一帧的误差
    double windup_guard;//积分项的最大允许范围,上下限，防止积分饱和
    double output;
    std::chrono::steady_clock::time_point last_time;//记录上一帧PID计算的时间
};
#endif