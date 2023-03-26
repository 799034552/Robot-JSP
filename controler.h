#ifndef __CONTROLER_H_
#define __CONTROLER_H_
#include <iostream>
#include <string.h>
#include <utility>
#include <vector>
#include <math.h>
#include "robot.h"
#include <memory>
#include <algorithm>
#include <unordered_map>
using namespace std;

#define PI 3.14159265359
#define MAX_NUMBER 50000000

struct Urgent_task;
class Workbench {
    public:
    int id;
    int type; // 类型
    std::pair<double,double> pos; // 位置
    int left_time; //剩余生产时间
    int input_box;
    int last_input_box;
    int output_box;
    int out_occupy_by_second = -1;
    int input_frame = -1;
    int output_occupy_by = -1; //拿东西是否被占用
    vector<int> input_occupy_by; //放东西是否被占用
    Urgent_task* urgent_task;
    Workbench(int _type):type(_type),input_occupy_by(vector<int>(10, -1)),urgent_task(nullptr) {

    };
    bool get_input_box_item(int n); // 获取对应的输入格子是否被占用
    double reduce_distance();
};

class WorldStatus {
    public:
    vector<Workbench> wb_list;
    vector<Robot> robot_list;
    int frame_id;
    int money;
    WorldStatus() {}
    WorldStatus(vector<Workbench> _wb_list, vector<Robot> _robot_list, int _frame_id, int _money)
        :wb_list(_wb_list),robot_list(_robot_list), frame_id(_frame_id), money(_money) {}
    shared_ptr<WorldStatus> duplicate();
    void show();
};

class DecisionTreeNode {
    public:
    shared_ptr<DecisionTreeNode> parent;
    vector<shared_ptr<DecisionTreeNode>> child;
    shared_ptr<WorldStatus> one_world; //这个世界
    vector<pair<int,int>> choose; //在这个世界中做出的选择
};

// 紧急任务列表
struct Urgent_task {
    vector<pair<int, int>> task_list;
    vector<int> task_status; // 任务的抢占情况，-1 没有人抢任务 -2 该任务已完成
};

extern char tmp_txt[1024]; //临时数据存储
extern vector<Robot> robot_list;
extern vector<Workbench> wb_list;
extern long frame_id;
extern int money;
extern vector<vector<int>> type_to_wb;
extern unordered_map<int, vector<int>> wb_can_put;
extern unordered_map<int, vector<int>> product_to_sell;
extern unordered_map<int, vector<int>> favourite_map;
extern bool debug;
extern vector<Urgent_task*> urgent_list;
extern int map_type;
extern unordered_map<int, int> wb_product_count;


void cal_map_type();
bool readUntilOK(char*);
void read_map();
//用空格切割输入并转换为double
vector<double> split_line_text(char s[]);
vector<double> get_split_line_text(bool is_debug=false);
// 删除一行
void delete_line(int n=1);
// 获取一帧的数据
void get_frame(bool is_debug=false);
// 获取距离
inline double cal_distance(const pair<double, double> &a,const pair<double, double> &b) {
    return sqrt((a.first - b.first) * (a.first - b.first) + (a.second - b.second) * (a.second - b.second));
}
// 获取方向
double cal_angle(pair<double, double> start, pair<double, double> end);
// 查看他是否能够被消化
bool can_somebody_put(int pr_type, pair<double, double> robot_pos, pair<double, double> wb_pos);
// 初始化
void init();
void create_urgent();
double add_distance(int wb_i);
//假设这个状态中所有机器人的目标都不变，计算dt时间后整个世界的状态
shared_ptr<WorldStatus> cal_next_statue(const shared_ptr<WorldStatus> &cur_status, int dt);
int cal_reach_time(const WorldStatus & cur_status, const int &robot_id);
// 改变生产台的输入
void change_wb_statue(Workbench & wb, int dt);
// 生成决策树
shared_ptr<DecisionTreeNode> build_decision_tree(shared_ptr<DecisionTreeNode> &root, int deep=1);
// 对于选择遍历结果

void world_choose(shared_ptr<WorldStatus> cur_status, vector<int> &no_target_robot, vector<vector<int>> &can_choose, shared_ptr<DecisionTreeNode> &root);
void true_world_choose(shared_ptr<WorldStatus> cur_status, vector<int> &no_target_robot, vector<vector<int>> &can_choose, vector<int> &choose, shared_ptr<DecisionTreeNode> &root);
#endif

