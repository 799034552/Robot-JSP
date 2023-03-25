#ifndef __ROBOT_H_
#define __ROBOT_H_
#include <utility>

enum Action
{
    None,
    buy,
    sell,
    wait
};
typedef struct Speed_pid
{
    double previous_error = 0;       // 上一次偏差
    double integral = 0;             // 积分和
    double previous_error_inner = 0; // 内环PID上一次偏差
    double integral_inner = 0;       // 内环PID积分和
    double last_speed = 0;
}Speed_pid;

class Robot
{
public:
    int id;
    std::pair<double, double> pos;
    int workbrench_id;
    int buy_frame;
    int carry_id;
    int time_conf;
    int hit_conf;
    double rotate_speed;
    std::pair<double, double> linear_speed;
    double face = 0;

    int forward_id = -1; // 目标工作台，-1表示没有
    int action = None;
    Robot(){};
    std::pair<double, double> Robot_control(double distance, double angle); // 给定距离目标的距离和角度，输出前进速度和旋转速度
    std::pair<double, double> Robot_control(std::pair<double,double>target_pos);    // 机器人控制
    std::pair<double, double> Robot_pid_control(std::pair<double,double>target_pos, double target_theta); //给定目标点的位置和机器人在目标位置的姿态，输出前进速度和旋转速度
    double robot_speed_pid(double distance);
    Speed_pid speed_pid;

    std::pair<double, double> power_field();
};

extern double power_k1;
extern double power_k2;
extern double power_p0;

#endif