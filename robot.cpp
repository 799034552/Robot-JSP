#include "robot.h"
#include "controler.h"
#include <iostream>
#include <cmath>

using namespace std;
int tt = 0;
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
    return sqrt(vec.first * vec.first + vec.second * vec.second);
}

// 计算单位向量
std::pair<double, double> unit_vector(std::pair<double, double> vec)
{
    std::pair<double, double> result;
    double len = length(vec)+0.00001;
    result.first = vec.first / len;
    result.second = vec.second / len;
    return result;
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

    // 如果合力方向不与速度方向平行，则施加转弯
    std::pair<double, double> net_force = this->power_field();
    angle = atan2(net_force.second, net_force.first);
    double angle_diff = angle - this->face;
    if (angle_diff > PI)
        angle_diff -= 2 * PI;
    else if (angle_diff < -PI)
        angle_diff += 2 * PI;

    // 负数表示顺时针旋转
    rotate_speed = kp * angle_diff - kd * this->rotate_speed;

    std::pair<double,double> forward_direction(cos(this->face),sin(this->face));
    double traction = (forward_direction.first*net_force.first+forward_direction.second*net_force.second);//前进方向的力
    forward_speed = k3*traction;


    // 计算转弯的圆心坐标
    // std::pair<double, double> po((-this->rotate_speed) * (this->linear_speed.second),
    //                              (this->rotate_speed) * (this->linear_speed.first));       // 机器人指向圆心的半径向量，叉乘运算
    // std::pair<double, double> o(this->pos.first + po.first, this->pos.second + po.second); // 圆心坐标
    // double r = length(po);                                                                 // 半径长度
    // double dis = distance_pos_wall(o);                                                     // 圆心与墙面距离

    // std::pair<double, double> robot_wall_re = robot_wall_relation(this->pos, this->face);                   // 计算离墙壁的情况
    // bool a = distance < 2 * rotate_radius && abs(angle_diff) > rotate_angle_threshold;                      // 防转圈
    // bool b = robot_wall_re.first < 0.3 * rotate_radius && abs(robot_wall_re.second) < wall_angle_threshold; // 防撞墙
    // if (b)
    // {
    //     forward_speed = k3 * robot_wall_re.first;
    // }
    // else if (a)
    // {
    //     forward_speed = -k1 * abs(angle_diff) + k2 * distance;
    //     // forward_speed = -k1 * abs(angle_diff) + k2 * distance;
    //     forward_speed = max(2.5, forward_speed);
    // }
    // else
    // {
    //     forward_speed = 6.0;
    // }

    if (this->id == 1 && frame_id > 680 && frame_id < 700)
    {
        // cerr << frame_id << " " << test << endl;
        cerr << "set forward speed: " << forward_speed << ", "<<traction<<" set rotate speed: " << rotate_speed << endl;
        cerr << "actual forward speed: " << length(this->linear_speed) << " actual rotate speed: " << this->rotate_speed << endl;
        // cerr << "distance o to wall:" << dis << " radius:" << r;
        // cerr << this->linear_speed.first << ", " << this->linear_speed.second;
        cerr << distance << ", " << angle_diff <<endl;
        // cerr << robot_wall_re.first << " " << robot_wall_re.second << endl;
        // cerr << this->face << " " << this->pos.first << " " << this->pos.second << endl;
        cerr << "----------------------------------->" << endl;
    }

    std::pair<double, double> result(forward_speed, rotate_speed);
    return result;
}

double Robot::robot_speed_pid(double distance)
{
    static double kp = 1, kd = 0, ki = 0;                        // 外环pid
    static double Kp_inner = 15, Ki_inner = 0.1, Kd_inner = 1.7; // 内环pid
    double dt = 0.015;                                           // 时间参数

    /***
     * 外环控制
     */
    double error = distance; // 与目标点偏差
    this->speed_pid.integral = this->speed_pid.integral + dt * error;
    double derivative = (error - this->speed_pid.previous_error) / dt;
    double output = kp * error + ki * this->speed_pid.integral + kd * derivative;
    this->speed_pid.previous_error = error;

    double setpoint_inner = output; // 外环的PID输出赋值给内环的PID输入

    /***
     * 内环控制
     */
    double measured_value_inner = length(this->linear_speed);
    double error_inner = setpoint_inner - measured_value_inner;
    this->speed_pid.integral_inner = this->speed_pid.integral_inner + dt * error_inner;  // 计算得到积分累加和
    double derivative_inner = (error_inner - this->speed_pid.previous_error_inner) / dt; // 计算得到微分
    // 计算得到PID输出
    double output_inner = Kp_inner * error_inner + Ki_inner * this->speed_pid.integral_inner + Kd_inner * derivative_inner;
    this->speed_pid.previous_error_inner = error_inner; // 保存当前偏差为下一次采样时所需要的历史偏差
    // tt++;
    // if (this->id == 0 && tt % 100 == 0)
    //     cerr << frame_id << " " << this->speed_pid.integral_inner << " " << this->speed_pid.integral << endl;
    return output_inner;
}

/// @brief 计算吸引力
std::pair<double, double> cal_attraction(std::pair<double, double> current_pos, std::pair<double, double> target_pos, double k)
{
    std::pair<double, double> result;
    result.first = k * (target_pos.first - current_pos.first);
    result.second = k * (target_pos.second - current_pos.second);
    return result;
}

/// @brief 计算斥力
std::pair<double, double> cal_repulsion(std::pair<double, double> current_pos, std::pair<double, double> obstacle_pos, double k, double p0)
{
    std::pair<double, double> result;
    double distance = cal_distance(current_pos, obstacle_pos);
    double scale = k * (1 / distance - 1 / p0);
    scale /= distance * distance;
    result.first = scale * (current_pos.first - obstacle_pos.first) / distance;
    result.second = scale * (current_pos.second - obstacle_pos.second) / distance;
    return result;
}

std::pair<double, double> Robot::power_field()
{
    static double k1 = 4;                                                 // 引力场参数
    static double k2 = 60;                                                // 斥力场参数
    static double p0 = 2;                                                // 斥力场产生作用距离
    static double wk = 20;                                                // 墙壁斥力场scale
    static double wp = 1;                                                 // 墙壁斥力场产生作用距离
    std::pair<double, double> target_pos = wb_list[this->forward_id].pos; // 目标位置
    std::pair<double, double> current_pos = this->pos;                    // 当前位置

    vector<std::pair<double, double>> force;   // 储存所有的力矢量
    std::pair<double, double> net_force(0, 0); // 合力

    // 计算吸引力
    force.push_back(cal_attraction(current_pos, target_pos, k1));
    // 计算机器人斥力
    for (int i = 0; i < robot_list.size(); ++i)
    {
        if (this->id == i)
            continue;
        force.push_back(cal_repulsion(current_pos, robot_list[i].pos, k2, p0));
    }
    // 计算墙壁斥力
    force.push_back(cal_repulsion(current_pos, {current_pos.first, 0}, wk, wp));
    force.push_back(cal_repulsion(current_pos, {current_pos.first, 50}, wk, wp));
    force.push_back(cal_repulsion(current_pos, {0, current_pos.second}, wk, wp));
    force.push_back(cal_repulsion(current_pos, {50, current_pos.second}, wk, wp));
    // 计算合力
    for (int i = 0; i < force.size(); ++i)
    {
        net_force.first += force[i].first;
        net_force.second += force[i].second;
    }

    // if ((this->id == 1) && frame_id > 680 && frame_id < 700)
    // {
    //     cerr << frame_id << " " << this->id << endl;
    //     for (int i = 0; i < force.size(); ++i)
    //     {
    //         cerr << i << " " << force[i].first << ", " << force[i].second << ", " << length(force[i]) << endl;
    //     }
    //     cerr << "net_force: " << net_force.first << ", " << net_force.second << ", " << length(net_force) << endl;
    //     cerr << "pos: " << this->pos.first << ", " << this->pos.second << endl;
    //     // cerr << "----------------------------------->" << endl;
    // }

    return net_force;
}