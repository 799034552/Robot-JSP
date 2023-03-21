#include "robot.h"
#include <iostream>
#include <cmath>
#define PI 3.14159265359
int k = 0;

using namespace std;

/// @brief 计算机器人离哪面墙最近
/// @param pos 机器人坐标
/// @param face 机器人朝向
/// @return 机器人距离墙的距离；机器人与最近的墙的角度
std::pair<double, double> robot_wall_relation(std::pair<double,double> pos, double face){
    double distance_to_wall = INT_MAX;
    double angle = 0;
    if(50-pos.first<distance_to_wall){
        distance_to_wall = 50-pos.first;

    }
    else if (pos.first<distance_to_wall)
    {
        distance_to_wall = pos.first;
    }
    else if(50-pos.second<distance_to_wall){
        distance_to_wall = 50-pos.second;
    }
    else if (pos.second<distance_to_wall)
    {
        distance_to_wall = pos.second;
    }
    return std::pair<double, double>(distance_to_wall, angle);
}

std::pair<double, double> Robot::Robot_controle(double distance, double angle)
{
    static double kp = 20;              //  转弯p参数
    static double kd = 0.01;            //  转弯d参数
    static double k1 = 1.8;               //  前进速度参数
    static double k2 = 2;               //  前进速度参数
    static double rotate_radius = 2.18; // 最大转弯半径
    static double angle_threshold = 0.4;          // 角度大于此阈值，考虑减速

    double forward_speed = 0;
    double rotate_speed = 0;

    double angle_diff = angle - this->face;
    if (angle_diff > PI)
        angle_diff -= 2 * PI;
    else if (angle_diff < -PI)
        angle_diff += 2 * PI;

    // 负数表示顺时针旋转
    rotate_speed = kp * angle_diff - kd * this->rotate_speed;

    // 计算离墙壁距离
    std::pair<double, double> robot_wall_re = robot_wall_relation(this->pos, this->face);

    if (distance < 2 * rotate_radius && abs(angle_diff) > angle_threshold)
    {
        forward_speed = -k1 * abs(angle_diff) + k2 * distance;
        forward_speed = max(3.0, forward_speed);
    }
    // else if (robot_wall_re.first<rotate_radius &&)
    // {
    //     /* code */
    // }
    
    else
    {
        forward_speed = 6.0;
    }

    // if (this->id == 0)
    // {
    //     k++;
    //     if(k%400==1){
    //     cerr <<forward_speed<<" " <<distance << " " << angle_diff << endl;
    //     cerr <<this->pos.first<<" "<<this->pos.second<<endl;
    //     cerr<<"----------------------------------->"<<endl;
    //     }
    // }

    std::pair<double, double> result(forward_speed, rotate_speed);
    return result;
}