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
    double face;

    int forward_id = -1; //目标工作台，-1表示没有
    int action = None; 
    Robot() {};
};
#endif