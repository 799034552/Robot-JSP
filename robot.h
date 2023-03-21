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

    double last_face_diff = 0; //上一帧与目标朝向差
    int forward_id = -1; //目标工作台，-1表示没有
    int action = None; 
    Robot() {};
    std::pair<double,double> Robot_controle(double distance, double angle); //给定距离目标的距离和角度，输出前进速度和旋转速度
};
#endif