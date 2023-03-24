#include "robot.h"
#include "controler.h"
using namespace std;
#define MAX_GRAIN 1e10

const double NORMAL_R = 0.45; //正常半径
const double CARRY_A = 0.53; //带东西半径
const double NORMAL_L_A = 250.0 / NORMAL_R / NORMAL_R / PI / 20; //正常线加速度
const double CARRY_L_A = 250.0 / CARRY_A / CARRY_A / PI / 20; //带东西线加速度
const double NORMAL_R_A = 50.0 / (1.0/2 * PI * NORMAL_R*NORMAL_R*NORMAL_R*NORMAL_R* 20); //正常角加速度
const double CARRY_R_A = 50.0 / (1.0/2 * PI * CARRY_A*CARRY_A*CARRY_A*CARRY_A* 20); //带东西角加速度
const double MAX_L_SPEED = 6.0;  // 最大线速度
const double MIN_L_SPEED = -2.0;  //最小线速度
const double MAX_R_SPEED = PI;  // 最大角速度

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
// 将向量逆时针旋转角度 a
std::pair<double, double> rotate_vector(std::pair<double, double> vec, double a)
{
    std::pair<double, double> result;
    result.first = vec.first * cos(a) - vec.second * sin(a);
    result.second = vec.first * sin(a) + vec.second * cos(a);
    return result;
}


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
    if (cal_wall_dis(wb.pos) < 4 && carry_id != -1) {
        speed_p = kp_speed_wall;
    }
    if (abs(rotate_diff) - abs(last_rotate_diff) > -0.3 && rotate_diff > PI/180*30) { // 如果角度变小误差,并且距离
        forward_speed = 1;
    } else if (abs(rotate_diff) < PI/4) {
        forward_speed = speed_p * dis_diff + kd_speed * (dis_diff - last_dis_diff);
    }
    else if (abs(rotate_diff) < PI/2) {
        if (cal_wall_dis(pos) < 5 && carry_id != -1) {
            forward_speed = 1;
        } else {
            forward_speed = speed_p * dis_diff + kd_speed * (dis_diff - last_dis_diff);
        }
    } else {
        forward_speed = 1;
    }

    // 转动控制
    rotate_speed = kp_rotate * rotate_diff + kd_rotate * (rotate_diff - last_rotate_diff);
    

    // 碰撞控制
    for(auto &rb: robot_list) {
        if (rb.id != id && cal_distance(pos, rb.pos) < 5) {
            auto t = cal_hit();
            auto rotate_diff = cal_rotate_diff(t.second, this->face);
            forward_speed = t.first * 2;
            rotate_speed = rotate_diff * 20;
            break;
        }
    }
    auto t = cal_hit();
    rotate_diff = cal_rotate_diff(t.second, this->face);
    forward_speed = t.first * 4;
    rotate_speed = rotate_diff * 20;

    // 碰撞控制
    // for(auto &rb: robot_list) {
    //     if (rb.id != id && cal_distance(pos, rb.pos) < 5) {
    //         auto t = avoid_hit();
    //         forward_speed = t.first;
    //         rotate_speed = t.second;
    //         break;
    //     }
    // }

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

pair<double,double> Robot::avoid_hit() {
    // 一些超参数
    static double linear_d = 0.3;
    static double rotate_d = 0.3;
    static double dt = 0.1;
    static double predict_time = 1.0;
    static double goal_gain = 1;
    static double speed_gain = 1.05;
    static double obstacle_gain = 1;
    double r, linear_a, rotate_a;
    if (carry_id == -1) { //没有背东西
        r = NORMAL_R;
        linear_a =  NORMAL_L_A;
        rotate_a = NORMAL_R_A;
    } else {
        r = CARRY_A;
        linear_a =  CARRY_L_A;
        rotate_a = CARRY_R_A;
    }
    auto goal_pos = wb_list[this->forward_id].pos;
    vector<pair<double, double>> obstacle_pos;
    for(auto &robot: robot_list) {
        if (robot.id != this->id)
            obstacle_pos.emplace_back(robot.pos);
    }
    // 初始状态：[x, y, 朝向, 线速度， 角速度]
    // vector<double> origin_status = {pos.first, pos.second, face, get_true_linear(this->linear_speed, face), rotate_speed};
    RobotStatus origin_status(pos.first, pos.second, get_true_linear(this->linear_speed, face), rotate_speed, face);
    auto speed_range = cal_speed_range(origin_status, linear_a, rotate_a, dt); // 计算速度范围
    double min_cost = MAX_GRAIN;
    double best_v = 0.01, best_u = 0.01;
    for(double v = speed_range[1]; v <= speed_range[0]; v += linear_d) {
        for(double u = speed_range[3]; u <= speed_range[2]; u += rotate_d) {
            // 算出在线速度与角速度下的一段时间内的所有路径
            auto trajectory = cal_trajectory(origin_status, v, u, dt, predict_time);
            // for(auto &a: trajectory) {
            //     cerr<<a.x<<" "<<a.y<<" "<<a.linear_speed<< " "<< a.rotate_speed<<" "<< a.face<<endl;
            // }
            // exit(0);
            auto goal_cost = goal_gain * calc_goal_cost(trajectory, goal_pos);
            auto speed_cost = speed_gain * (MAX_L_SPEED - trajectory.back().linear_speed);
            auto obstacle_cost = obstacle_gain * calc_obstacle_cost(trajectory, obstacle_pos, r);
            auto final_cost = goal_cost+ speed_cost + obstacle_cost;

            if (min_cost >= final_cost) {
                min_cost = final_cost;
                best_v = v;
                best_u = u;
            }

        }
    }
    return {best_v,best_u};
}

double get_true_linear(pair<double, double> l_speed, double face) {
    auto angle = atan2(l_speed.second, l_speed.first);
    auto angle_diff = angle - face;
    return cos(angle_diff)*sqrt(l_speed.first*l_speed.first + l_speed.second*l_speed.second);
}

vector<double> cal_speed_range(const RobotStatus &origin_status, const double &linear_a,const double &rotate_a,const double &dt) {
    // 初始状态：[x, y, 朝向, 线速度， 角速度]
    double max_l_speed = origin_status.linear_speed ; //origin_status.linear_speed + dt*linear_a;
    double min_l_speed = origin_status.linear_speed ; //origin_status.linear_speed - dt*linear_a;
    double max_r_speed = origin_status.rotate_speed + dt*rotate_a;
    double min_r_speed = origin_status.rotate_speed - dt*rotate_a;
    
    return {
        min(max_l_speed, MAX_L_SPEED),
        max(min_l_speed, MIN_L_SPEED),
        min(max_r_speed, MAX_R_SPEED),
        max(min_r_speed, -MAX_R_SPEED)
    };
}

vector<RobotStatus> cal_trajectory(RobotStatus origin_status, const double &v, const double &u, const double &dt, const double &predict_time) {
    vector<RobotStatus> trajectory = {origin_status};
    double time = 0;
    origin_status.linear_speed = v;
    origin_status.rotate_speed = u;
    while (time <= predict_time) {
        origin_status.x += cos(origin_status.face)*v*dt;
        origin_status.y += sin(origin_status.face)*v*dt;
        origin_status.face += u*dt;
        trajectory.push_back(origin_status);
        time += dt;
    }
    return trajectory;
}

double calc_goal_cost(const vector<RobotStatus> &trajectory, pair<double,double> goal) {
    double target_face = atan2(goal.second - trajectory.back().y, goal.first - trajectory.back().x);
    double err_angle = target_face - trajectory.back().face;
    return abs(atan2(sin(err_angle), cos(err_angle)));
}

double calc_obstacle_cost(const vector<RobotStatus> &trajectory, const vector<pair<double, double>> &obstacle, const double &r) {
    double mid_dis = MAX_GRAIN;
    for(const auto & robot_status: trajectory) {
        for(const auto & obs_item: obstacle) {
            auto distance = sqrt((robot_status.x - obs_item.first) * (robot_status.x - obs_item.first) + (robot_status.y - obs_item.second) * (robot_status.y - obs_item.second));
            if (distance <= r)
                return MAX_GRAIN;
            else if (distance < mid_dis) {
                mid_dis = distance;
            }
        }
    }
    return 1 / mid_dis;
}

pair<double, double> Robot::cal_hit() {
    static double k1 = 4;                                                 // 引力场参数
    static double k2 = 15;                                                // 斥力场参数
    static double p0 = 5;                                                 // 斥力场产生作用距离
    vector<std::pair<double, double>> force;   // 储存所有的力矢量
    std::pair<double, double> net_force(0, 0); // 合力
    auto &wb = wb_list[this->forward_id];

    // 计算吸引力
    force.push_back(cal_attraction(pos, wb.pos, k1));
    // 计算机器人斥力
    bool test = false;
    for (int i = 0; i < robot_list.size(); ++i)
    {
        if (this->id == i)
            continue;

        std::pair<double, double> repulsion = cal_repulsion(pos, robot_list[i].pos, wb.pos, k2, p0);
        // id大的机器人避让id小的
        if (this->forward_id == robot_list[i].forward_id && this->id < robot_list[i].id)
        {
            repulsion.first *= 0;
            repulsion.second *= 0;
        }

        // pair<double, double> robot(current_pos.first - robot_list[i].pos.first,current_pos.second - robot_list[i].pos.second)

        // if (abs(this->face) + abs(robot_list[i].face) >= PI / 180 * 160 && abs(this->face) + abs(robot_list[i].face) <= PI / 180 * 200)
        // {
        //     test = true;
        //     repulsion.first = repulsion.first * cos(PI / 6) - repulsion.second * sin(PI / 6);
        //     repulsion.second = repulsion.first * sin(PI / 6) + repulsion.second * cos(PI / 6);
        // }
        force.push_back(repulsion);
    }
    // 计算墙壁斥力
    // force.push_back(cal_repulsion(current_pos, {current_pos.first, 0}, wk, wp));
    // force.push_back(cal_repulsion(current_pos, {current_pos.first, 50}, wk, wp));
    // force.push_back(cal_repulsion(current_pos, {0, current_pos.second}, wk, wp));
    // force.push_back(cal_repulsion(current_pos, {50, current_pos.second}, wk, wp));
    // 计算合力
    for (int i = 0; i < force.size(); ++i)
    {
        net_force.first += force[i].first;
        net_force.second += force[i].second;
    }

    double same_direction = 0; // 斥力方向与前进方向相反,即两个机器人相向而行时,控制他们避让
    for (int i = 1; i < force.size(); ++i)
    {
        double angle1 = atan2(force[i].second, force[i].first);
        double angle2 = atan2(net_force.second, net_force.first);
        double angle_sum = abs(angle1) + abs(angle2);
        if (angle_sum >= PI / 180 * 160 && angle_sum <= PI / 180 * 180)
            same_direction = 1;
        if (angle_sum >= PI / 180 * 180 && angle_sum <= PI / 180 * 200)
            same_direction = -1;
    }
    if (same_direction)
        net_force = rotate_vector(net_force, same_direction * PI / 5);
    std::pair<double, double> forward_direction(cos(this->face), sin(this->face));
    double traction = (forward_direction.first * net_force.first + forward_direction.second * net_force.second); // 前进方向的力


    double angle = atan2(net_force.second, net_force.first);
    return {traction, angle};
}