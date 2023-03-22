#include "robot.h"
#include "controler.h"
#include <iostream>
#include <cmath>

using namespace std;

/// @brief 计算机器人离哪面墙最近
/// @param pos 机器人坐标
/// @param face 机器人朝向
/// @return 机器人距离墙的距离；机器人是否正在接近墙
std::pair<double, double> robot_wall_relation(std::pair<double, double> pos, double face)
{
    double distance_to_wall = MAX_NUMBER;
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

/// @brief 返回一个点离墙面的最近距离
/// @param pos
/// @return
double distance_pos_wall(std::pair<double, double> pos)
{
    double distance_to_wall = MAX_NUMBER;
    // 右边的墙
    if (50 - pos.first < distance_to_wall)
    {
        distance_to_wall = 50 - pos.first;
    }
    // 左边的墙
    if (pos.first < distance_to_wall)
    {
        distance_to_wall = pos.first;
    }
    // 上边的墙
    if (50 - pos.second < distance_to_wall)
    {
        distance_to_wall = 50 - pos.second;
    }
    // 下边的墙
    if (pos.second < distance_to_wall)
    {
        distance_to_wall = pos.second;
    }
    return distance_to_wall;
}

/// @brief 计算向量模长
/// @param vec
/// @return
double length(std::pair<double, double> vec)
{
    return sqrt(vec.first * vec.first + vec.second + vec.second);
}

std::pair<double, double> Robot::Robot_controle(double distance, double angle)
{
    static double kp = 20;                           //  转弯p参数
    static double kd = 0.01;                         //  转弯d参数
    static double k1 = 1.8;                          //  前进速度参数，用转弯半径控制速度
    static double k2 = 2;                            //  前进速度参数，用距离工作台的距离控制速度
    static double k3 = 0.2;                          //  前进速度参数,用离墙的距离控制速度
    static double rotate_radius = 2.18;              // 最大转弯半径
    static double rotate_angle_threshold = 0.4;      // 角度大于此阈值，考虑减速
    static double wall_angle_threshold = 3 * PI / 5; // 机器人与墙壁的角度小于此阈值，考虑减速

    double forward_speed = 0;
    double rotate_speed = 0;

    double angle_diff = angle - this->face;
    if (angle_diff > PI)
        angle_diff -= 2 * PI;
    else if (angle_diff < -PI)
        angle_diff += 2 * PI;

    // 负数表示顺时针旋转
    rotate_speed = kp * angle_diff - kd * this->rotate_speed;

    // 计算转弯的圆心坐标
    std::pair<double, double> po((-this->rotate_speed) * (this->linear_speed.second),
                                 (this->rotate_speed) * (this->linear_speed.first));       // 机器人指向圆心的半径向量，叉乘运算
    std::pair<double, double> o(this->pos.first + po.first, this->pos.second + po.second); // 圆心坐标
    double r = length(po);                                                                 // 半径长度
    double dis = distance_pos_wall(o);                                                     // 圆心与墙面距离

    std::pair<double, double> robot_wall_re = robot_wall_relation(this->pos, this->face);   // 计算离墙壁的情况
    bool a = distance < 2 * rotate_radius && abs(angle_diff) > rotate_angle_threshold;      // 防转圈
    bool b = robot_wall_re.first < 0.8 && abs(robot_wall_re.second) < wall_angle_threshold; // 防撞墙

    bool test = false;
    if ((abs(robot_wall_re.second) < wall_angle_threshold) && (dis <= r + 2.5))
    {
        forward_speed = 0.3 * dis;
        test = true;
    }
    else if (a)
    {
        forward_speed = -k1 * abs(angle_diff) + k2 * distance;
        forward_speed = max(3.0, forward_speed);
    }
    else
    {
        forward_speed = 6.0;
    }

    if (this->id == 1 && frame_id > 620 && frame_id < 650)
    {
        cerr << frame_id << " " << test << endl;
        cerr << "set forward speed: " << forward_speed << " set rotate speed: " << rotate_speed << endl;
        cerr << "actual forward speed: " << length(this->linear_speed) << " actual rotate speed: " << this->rotate_speed << endl;
        cerr << "distance o to wall:" << dis << " radius:" << r;
        cerr << this->linear_speed.first << ", " << this->linear_speed.second;
        cerr << distance << " " << angle_diff << endl;
        cerr << robot_wall_re.first << " " << robot_wall_re.second << endl;
        cerr << this->face << " " << this->pos.first << " " << this->pos.second << endl;
        cerr << "----------------------------------->" << endl;
    }

    std::pair<double, double> result(forward_speed, rotate_speed);
    return result;
}

double Robot::robot_speed_pid(double distance)
{
    static double kp = 0, kd = 0, ki = 0;   // 外环pid
    static double kip = 0, kid = 0, kii = 0;    // 内环pid

    double error = distance; // 与目标点偏差
    this->speed_pid.integral = this->speed_pid.integral + ki*error;
    this->speed_pid.previous_error
}
