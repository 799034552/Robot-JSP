#ifndef __ROBOT_H_
#define __ROBOT_H_
#include <utility>
#include <iostream>
#include <cmath>
#include <vector>
using std::vector;
using std::pair;

enum Action {None, buy, sell, wait};


class Robot {
    public:
    int id;
    std::pair<double,double> pos;
    int workbrench_id;
    int carry_id;
    int time_conf;
    int hit_conf;
    double rotate_speed;
    std::pair<double,double> linear_speed;
    double face = 0;

    int forward_id = -1; //目标工作台，-1表示没有
    int action = None; 
    bool target_can_change = true;
    double dis_err = -1;
    double last_dis_diff = -1;
    double last_rotate_diff = -1;
    int last_forward_id = -1;
    Robot() {};
    std::pair<double,double> Robot_controle(double distance, double angle); //给定距离目标的距离和角度，输出前进速度和旋转速度
    std::pair<double,double> avoid_hit(); //防碰撞
    pair<double, double> cal_hit();
};

struct RobotStatus {
    double x;
    double y;
    double linear_speed;
    double rotate_speed;
    double face;
    RobotStatus() {}
    RobotStatus(double _x, double _y, double _linear_speed, double _rotate_speed, double _face):
        x(_x),
        y(_y),
        linear_speed(_linear_speed),
        rotate_speed(_rotate_speed),
        face(_face)
    {
    }
};

double cal_rotate_diff(double target, double face);
double cal_wall_dis(std::pair<double, double> pos);
double get_true_linear(std::pair<double, double> l_speed, double face);
vector<double> cal_speed_range(const RobotStatus &origin_status, const double &linear_a,const double &rotate_a,const double &dt);
vector<RobotStatus> cal_trajectory(RobotStatus origin_status, const double &v, const double &u, const double &dt, const double &predict_time);
double calc_goal_cost(const vector<RobotStatus> &trajectory, pair<double,double> goal);
double calc_obstacle_cost(const vector<RobotStatus> &trajectory, const vector<pair<double, double>> &obstacle, const double &r);
#endif