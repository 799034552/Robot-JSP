#include "robot.h"
#include "controler.h"
#include <iostream>
#include <cmath>
#include "bspline.h"
using namespace std;
double power_k1 = 4;
double power_k2 = 15;
double power_p0 = 5;

vector<int> Avoid_Collision_Direction(4); // 存储4个机器人避让碰撞旋转的方向

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
    double len = length(vec) + 0.00001;
    result.first = vec.first / len;
    result.second = vec.second / len;
    return result;
}

// 将向量逆时针旋转角度 a
std::pair<double, double> rotate_vector(std::pair<double, double> vec, double a)
{
    std::pair<double, double> result;
    result.first = vec.first * cos(a) - vec.second * sin(a);
    result.second = vec.first * sin(a) + vec.second * cos(a);
    return result;
}

std::pair<double, double> Robot::Robot_control(double distance, double angle)
{
    static double kp = 20;    //  转弯p参数
    static double kd = 0.01;  //  转弯d参数
    static double k1 = 1.8;   //  前进速度参数，用转弯半径控制速度
    static double k2 = 2;     //  前进速度参数，用距离工作台的距离控制速度
    static double k3 = 1;     //  前进速度参数,用离墙的距离控制速度
    static double ksp = 2;    //  势力场控制速度p参数
    static double ksp2 = 1;   //  目标工作台离墙很近时势力场控制速度p参数
    static double ksd = 0.04; //  势力场控制速度d参数

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

    std::pair<double, double> forward_direction(cos(this->face), sin(this->face));
    double traction = (forward_direction.first * net_force.first + forward_direction.second * net_force.second); // 前进方向的力
    if (distance_pos_wall(wb_list[this->forward_id].pos) < 1.5 &&
        Pos(wb_list[this->forward_id].pos).distance(this->pos) < 5)
        forward_speed = ksp2 * traction + ksd * (length(this->linear_speed) - this->speed_pid.last_speed);
    else
        forward_speed = ksp * traction + ksd * (length(this->linear_speed) - this->speed_pid.last_speed);

    double RT_distance = Pos(wb_list[this->forward_id].pos).distance(this->pos);
    double face_target_angel_diff = abs(Vec(this->face).angle_diff(Vec(Pos(this->pos), Pos(wb_list[this->forward_id].pos))));
    if (RT_distance < 2 && face_target_angel_diff > rotate_angle_threshold)
    {
        forward_speed = -k1 * abs(face_target_angel_diff) + k2 * RT_distance;
        forward_speed = max(1.5, forward_speed);
    }

    this->speed_pid.last_speed = length(this->linear_speed);

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
    // tt++;
    if ((this->id == 0) && frame_id > 1544 && frame_id < 1580)
    {
        // fout.setf(ios::fixed, ios::floatfield); // 设定为 fixed 模式，以小数点表示浮点数
        // fout.precision(2);                      // 设置精度 2
        // fout << frame_id << "_" << this->id << ", ";
        // fout << Vec(this->linear_speed).angle_diff(this->face) << ", ";
        // fout << length(this->linear_speed) << ", " << this->pos.first << ", " << this->pos.second << endl;
        // cerr << frame_id << ", "<<this->id<<endl;
        // cerr << "set forward speed: " << forward_speed << " set rotate speed: " << rotate_speed << endl;
        // cerr << "current forward speed: " << length(this->linear_speed) << " current rotate speed: " << this->rotate_speed << endl;
        // cerr << "face: "<<this->face << " pos: " << this->pos.first << ", " << this->pos.second << endl;
        // cerr << "__________________________________________________________" << endl;
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
    // https://blog.csdn.net/junshen1314/article/details/50472410
    std::pair<double, double> result;
    result.first = k * (target_pos.first - current_pos.first);
    result.second = k * (target_pos.second - current_pos.second);
    double distance = cal_distance(current_pos, target_pos);
    if (distance > 14)
    {
        result.first = result.first / distance * 14;
        result.second = result.second / distance * 14;
    }
    return result;
}

/// @brief 计算斥力
std::pair<double, double> cal_repulsion(std::pair<double, double> current_pos, std::pair<double, double> obstacle_pos, std::pair<double, double> target_pos, double k, double p0)
{
    // https://zhuanlan.zhihu.com/p/548241778
    std::pair<double, double> f1;
    std::pair<double, double> f2;

    double distance = cal_distance(current_pos, obstacle_pos);
    double CT_distance = cal_distance(current_pos, obstacle_pos);
    if (distance > p0)
        return {0, 0};

    double scale = k * (1 / distance - 1 / p0);
    scale /= distance * distance;
    f1.first = scale * (current_pos.first - obstacle_pos.first) / distance;
    f1.second = scale * (current_pos.second - obstacle_pos.second) / distance;

    // f1.first *= CT_distance * CT_distance;
    // f1.second *= CT_distance * CT_distance;
    // scale = k * (1 / distance - 1 / p0) * (1 / distance - 1 / p0);
    // f2.first = scale * (target_pos.first - current_pos.first);
    // f2.second = scale * (target_pos.second - current_pos.second);

    return {f1.first, f1.second};
    // return {f1.first + f2.first, f1.second + f2.second};
}

std::pair<double, double> Robot::power_field()
{
    Avoid_Collision_Direction[this->id] = 0;
    static double k1 = 4;  // 引力场参数
    static double k2 = 10; // 斥力场参数
    static double p0 = 6;  // 斥力场产生作用距离
    // static double k1 = power_k1;
    // static double k2 = power_k2;
    // static double p0 = power_p0;
    static double wk = 40;                                                // 墙壁斥力场scale
    static double wp = 0.7;                                               // 墙壁斥力场产生作用距离
    std::pair<double, double> target_pos = wb_list[this->forward_id].pos; // 目标位置
    std::pair<double, double> current_pos = this->pos;                    // 当前位置

    vector<std::pair<double, double>> force;   // 储存所有的力矢量
    std::pair<double, double> net_force(0, 0); // 合力
    vector<int> temp_robot_id_list;
    temp_robot_id_list.push_back(this->id);

    // 计算吸引力
    force.push_back(cal_attraction(current_pos, target_pos, k1));
    // 计算机器人斥力
    bool test = false;
    for (int i = 0; i < robot_list.size(); ++i)
    {
        if (this->id == i)
            continue;
        else
            temp_robot_id_list.push_back(i);

        std::pair<double, double> repulsion = cal_repulsion(current_pos, robot_list[i].pos, target_pos, k2, p0);
        // 相同目的地，id大的机器人避让id小的
        if (this->forward_id == robot_list[i].forward_id && this->id < robot_list[i].id)
        {
            repulsion.first *= 0;
            repulsion.second *= 0;
        }

        force.push_back(repulsion);
    }
    // 计算墙壁斥力
    force.push_back(cal_repulsion(current_pos, {current_pos.first, 0}, target_pos, wk, wp));
    force.push_back(cal_repulsion(current_pos, {current_pos.first, 50}, target_pos, wk, wp));
    force.push_back(cal_repulsion(current_pos, {0, current_pos.second}, target_pos, wk, wp));
    force.push_back(cal_repulsion(current_pos, {50, current_pos.second}, target_pos, wk, wp));
    // 计算合力
    for (int i = 0; i < force.size(); ++i)
    {
        net_force.first += force[i].first;
        net_force.second += force[i].second;
    }

    double same_direction = 0; // 斥力方向与前进方向相反,即两个机器人相向而行时,控制他们避让
    for (int i = 1; i < 4; ++i)
    {
        double angle1 = atan2(force[i].second, force[i].first);
        double angle2 = atan2(net_force.second, net_force.first);
        double angle_sum = abs(angle1) + abs(angle2);
        // 检测到会迎面相撞
        if (angle_sum >= PI / 180 * 160 && angle_sum <= PI / 180 * 200)
        {
            // 查询id比它小的是否做出的碰撞决策，与它采取相同决策
            if (temp_robot_id_list[0] > temp_robot_id_list[i])
                same_direction = Avoid_Collision_Direction[temp_robot_id_list[i]];
            else
            {
                double obstacle_direction = Vec(this->face).cross_product(Vec(this->pos, robot_list[temp_robot_id_list[i]].pos)); // 判断障碍物在前进方向的左手边还是右手边
                if (obstacle_direction > 0)
                    same_direction = -1;
                else
                    same_direction = 1;
            }
            Avoid_Collision_Direction[this->id] = same_direction;
        }
    }
    if (same_direction)
        net_force = rotate_vector(net_force, same_direction * PI / 5);

    if ((this->id == 1 || this->id == 3) && frame_id > 1100 && frame_id < 1140)
    {
        // cerr << frame_id << " " << this->id << ", " << same_direction << endl;
        // cerr << Vec(this->face) << endl;
        // if (this->id == 1)
        // {
        //     cerr << Vec(this->pos, robot_list[3].pos) << endl;
        //     cerr << "cross product: " << Vec(this->face).cross_product(Vec(this->pos, robot_list[3].pos)) << endl;
        // }
        // else
        // {
        //     cerr << Vec(this->pos, robot_list[1].pos) << endl;
        //     cerr << "cross product: " << Vec(this->face).cross_product(Vec(this->pos, robot_list[1].pos)) << endl;
        // }

        
        // for (int i = 0; i < force.size(); ++i)
        // {
        //     cerr << i << " " << force[i].first << ", " << force[i].second << ", " << length(force[i]) << " angle: " << atan2(force[i].second, force[i].first) << endl;
        // }
        // cerr << "net_force: " << net_force.first << ", " << net_force.second << ", " << length(net_force) << " angle: " << atan2(net_force.second, net_force.first) << endl;
        // cerr << "----------------------------------->" << endl;
    }

    return net_force;
}
// sqrt((u(1)-u(4))^2+(u(2)-u(5))^2)*cos(atan2(u(5)-u(2),u(4)-u(1))-pi/2-u(3))

// -sqrt((u(1)-u(4))^2+(u(2)-u(5))^2)*sin(atan2(u(5)-u(2),u(4)-u(1))-pi/2-u(3))
std::pair<double, double> Robot::Robot_pid_control(Pos target_pos, double target_theta)
{
    static double kp_r = -40;
    static double kd_r = -2.5;
    static double kp_f = -80;

    Pos current_pos(this->pos);
    double current_theta = this->face;
    double distance = sqrt(pow(target_pos.x - current_pos.x, 2) + pow(target_pos.y - current_pos.y, 2));

    double theta_p = distance * cos(atan2(current_pos.y - target_pos.y, current_pos.x - target_pos.x) - PI / 2 - target_theta);
    double rotate_rate = kp_r * theta_p + kd_r * (current_theta - target_theta); // 法向 pd 控制
    double speed_p = -distance * sin(atan2(current_pos.y - target_pos.y, current_pos.x - target_pos.x) - PI / 2 - target_theta);
    double speed = kp_f * speed_p; // 前向 p 控制

    if ((this->id == 1) && frame_id > 50 && frame_id < 60)
    {
        // cerr << frame_id << " " << this->id << endl;
        // cerr << "target pos: " << target_pos << endl;
        // cerr << "current pos: " << current_pos << endl;
        // cerr << "current theta: " << this->face << ", target theta: " << target_theta << endl;
        // cerr << "speed: " << speed << ", rotate rate: " << rotate_rate << endl;
        // cerr << distance << ", " << cos(atan2(current_pos.y - target_pos.y, current_pos.x - target_pos.x) - PI / 2 - target_theta) << ", " << sin(atan2(current_pos.y - target_pos.y, current_pos.x - target_pos.x) - PI / 2 - target_theta) << endl;
        // cerr<<"set speed: "<<speed<<" rotate speed: "<<rotate_rate<<endl;
        // cerr << "----------------------------------->" << endl;
    }

    return {speed, rotate_rate};
}

std::pair<double, double> Robot::Robot_control()
{
    // 路径规划
    double half_car_lenb = 0.53; // 持有物品的半径
    if (this->carry_id == 0)
        half_car_lenb = 0.45;

    Pos current_pos(this->pos);
    Pos target_pos(wb_list[this->forward_id].pos);

    vector<Pos> control_list{current_pos, (2 * current_pos + target_pos) / 3, (current_pos + 2 * target_pos) / 3, target_pos};
    Bspline bspline(control_list);
    bspline.augement_ctrl_point(half_car_lenb, 0, Vec(this->face));
    bspline.augement_ctrl_point(half_car_lenb, bspline.control_point.size() - 1, Vec(0.0));

    vector<Pos> path_list = bspline.create_b_spline();

    // pid 控制
    Vec forward_vector(path_list[0], path_list[20]);
    pair<double, double> result = this->Robot_pid_control(path_list[20], forward_vector.angle());
    if ((this->id == 1) && frame_id > 50 && frame_id < 60)
    {
        // cerr << frame_id << " " << this->id << endl;
        // cerr << "target pos: " << target_pos << endl;
        //     cerr << "current pos: " << current_pos << endl;
        // cerr << "next pos: " << path_list[20] << endl;
        // cerr << "control point: " << endl;
        // for (auto &p : control_list)
        // {
        //     cerr << p << "; " << endl;
        // }
        // cerr << endl;
        // cerr << "extra control point: " << endl;
        // for (auto &p : bspline.control_point)
        // {
        //     cerr << p << "; " << endl;
        // }
        // cerr << endl;
        // cerr << "path point: " << endl;
        // for (int i = 0; i < 10; i++)
        // {
        //     cerr << path_list[i] << "; " << endl;
        // }
        //     cerr << endl;
        // cerr << "*******************" << endl;
    }

    return result;
}