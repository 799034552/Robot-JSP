#include "robot.h"
#include "controler.h"
#include <iostream>
#include <cmath>
using namespace std;

std::pair<double, double> Robot::Robot_controle(double distance, double angle)
{
    if (this->forward_id == -1) return {0, 0};
    double forward_speed = 0, rotate_speed = 0;
    auto &wb = wb_list[this->forward_id];

    // 速度控制参数, 位置式pid
    static double kp_speed = 6;
    static double kp_speed_wall = 4;
    static double ki_speed = 1;
    static double kd_speed = 0;
    // 转动控制参数, 位置式pid
    static double kp_rotate = 30;
    static double ki_rotate = 1;
    static double kd_rotate = 0;
    // 如果目标不同了，清除累计误差等参数
    if (this->last_forward_id != this->forward_id) {
        this->last_dis_diff = -1;
        this->last_rotate_diff = -1;
    }
    // 计算距离误差与角度误差
    auto dis_diff = cal_distance(this->pos, wb.pos);
    if (this->last_dis_diff == -1) this->last_dis_diff = dis_diff;
    
    auto target_rotate = cal_angle(this->pos, wb.pos);
    auto rotate_diff = cal_rotate_diff(target_rotate, this->face);
    if (this->last_rotate_diff == -1) this->last_rotate_diff = rotate_diff;

    double speed_p = kp_speed;
    if (cal_wall_dis(wb.pos) < 4) {
        speed_p = kp_speed_wall;
    }
    if (abs(rotate_diff) - abs(last_rotate_diff) > -0.2 && rotate_diff > PI/180*30) { // 如果角度变小误差,并且距离
        forward_speed = 1;
    } else if (abs(rotate_diff) < PI/4) {
        forward_speed = speed_p * dis_diff + kd_speed * (dis_diff - last_dis_diff);
    }
    else if (abs(rotate_diff) < PI/2) {
        if (cal_wall_dis(pos) < 5) {
            forward_speed = 1;
        } else {
            forward_speed = speed_p * dis_diff + kd_speed * (dis_diff - last_dis_diff);
        }
    } else {
        forward_speed = 1;
    }

    // 转动控制
    rotate_speed = kp_rotate * rotate_diff + kd_rotate * (rotate_diff - last_rotate_diff);

    return {forward_speed, rotate_speed};
}

double cal_rotate_diff(double target, double face) {
    auto dif = target - face;
    if (target >= face) {
        if (dif <= PI)
            return dif;
        else
            return -(2*PI - dif);
    }
    else {
        dif = -dif;
        if (dif <= PI)
            return -dif;
        else
            return 2*PI - dif;
    }
    return 0;
}

double cal_wall_dis(pair<double, double> pos) {
    return min(min(50 - pos.first, pos.second), min(50 - pos.first, pos.second));
}