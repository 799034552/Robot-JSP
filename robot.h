#ifndef __ROBOT_H_
#define __ROBOT_H_
#include <utility>

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
};

double cal_rotate_diff(double target, double face);
double cal_wall_dis(std::pair<double, double> pos);

#endif