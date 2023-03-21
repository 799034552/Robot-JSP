#include "robot.h"
#include <iostream>
#include <cmath>
#define PI 3.14159265359
#define MAX_NUM 5000000
int k = 0;

using namespace std;

/// @brief 计算机器人离哪面墙最近
/// @param pos 机器人坐标
/// @param face 机器人朝向
/// @return 机器人距离墙的距离；机器人是否正在接近墙
std::pair<double, double> robot_wall_relation(std::pair<double, double> pos, double face)
{
    double distance_to_wall = MAX_NUM;
    double angle;
    // 右边的墙
    if (50 - pos.first < distance_to_wall)
    {
        distance_to_wall = 50 - pos.first;
        angle = face;
    }
    // 左边的墙
    if (pos.first < distance_to_wall)
    {
        distance_to_wall = pos.first;
        angle = face - PI;
    }
    // 上边的墙
    if (50 - pos.second < distance_to_wall)
    {
        distance_to_wall = 50 - pos.second;
        angle = face - PI / 2;
    }
    // 下边的墙
    if (pos.second < distance_to_wall)
    {
        distance_to_wall = pos.second;
        angle = face + PI / 2;
    }
    return std::pair<double, double>(distance_to_wall, angle);
}

std::pair<double, double> Robot::Robot_controle(double distance, double angle)
{
    static double kp = 20;                       //  转弯p参数
    static double kd = 0.01;                     //  转弯d参数
    static double k1 = 1.8;                      //  前进速度参数，用转弯半径控制速度
    static double k2 = 2;                        //  前进速度参数，用距离工作台的距离控制速度
    static double k3 = 1;                        //  前进速度参数,用离墙的距离控制速度
    static double rotate_radius = 2.18;          // 最大转弯半径
    static double rotate_angle_threshold = 0.4;  // 角度大于此阈值，考虑减速
    static double wall_angle_threshold = PI / 2; // 机器人与墙壁的角度小于此阈值，考虑减速

    double forward_speed = 0;
    double rotate_speed = 0;

    double angle_diff = angle - this->face;
    if (angle_diff > PI)
        angle_diff -= 2 * PI;
    else if (angle_diff < -PI)
        angle_diff += 2 * PI;

    // 负数表示顺时针旋转
    rotate_speed = kp * angle_diff - kd * this->rotate_speed;

    // 计算离墙壁的情况
    std::pair<double, double> robot_wall_re = robot_wall_relation(this->pos, this->face);

    bool a = distance < 2 * rotate_radius && abs(angle_diff) > rotate_angle_threshold;                      // 防转圈
    bool b = robot_wall_re.first < 0.3 * rotate_radius && abs(robot_wall_re.second) < wall_angle_threshold; // 防撞墙
    if (b)
    {
        forward_speed = k3 * robot_wall_re.first;
    }
    else if (a)
    {
        forward_speed = -k1 * abs(angle_diff) + k2 * distance;
        // forward_speed = -k1 * abs(angle_diff) + k2 * distance;
        forward_speed = max(3.0, forward_speed);
    }
    else
    {
        forward_speed = 6.0;
    }

    std::pair<double, double> result(forward_speed, rotate_speed);
    return result;
}