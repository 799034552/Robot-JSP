#include "robot.h"
#include <iostream>
#define PI 3.14159265359

using namespace std;
std::pair<double, double> Robot::Robot_controle(double distance, double angle)
{
    static double kp = 20;
    static double kd = 1;
    double forward_speed = 0;
    double rotate_speed = 0;

    double angle_diff = angle - this->face;
    if (angle_diff > PI)
        angle_diff -= 2 * PI;
    else if (angle_diff < -PI)
        angle_diff += 2 * PI;

    // 负数表示顺时针旋转
    rotate_speed = kp * angle_diff + kd * rotate_speed;
    // if(this->id==0){
    //     cerr<<rotate_speed<<" "<<angle_diff<<endl;
    // }
    if (distance < 4)
    {
        // 如果距离目标3米内
        forward_speed = 3.0;
    }
    else
    {
        // 全力加速
        forward_speed = 100.0;
    }

    this->last_face_diff = angle_diff;
    std::pair<double, double> result(forward_speed, rotate_speed);
    return result;
}