#ifndef __CONTROLER_H_
#define __CONTROLER_H_
#include <iostream>
#include <string.h>
#include <utility>
#include <vector>
#include <math.h>
#include "robot.h"
#include <unordered_map>
using namespace std;

#define PI 3.14159265359

class Workbench {
    public:
    int id;
    int type; // 类型
    std::pair<double,double> pos; // 位置
    int left_time; //剩余生产时间
    int input_box;
    int output_box;

    int output_occupy_by = -1; //拿东西是否被占用
    vector<int> input_occupy_by; //放东西是否被占用
    Workbench(int _type):type(_type),input_occupy_by(vector<int>(10, -1)) {

    };
    bool get_input_box_item(int n); // 获取对应的输入格子是否被占用
};

extern char tmp_txt[1024]; //临时数据存储
extern vector<Robot> robot_list;
extern vector<Workbench> wb_list;
extern long frame_id;
extern int money;
extern vector<vector<int>> type_to_wb;
extern unordered_map<int, vector<int>> wb_can_put;
extern unordered_map<int, vector<int>> product_to_sell;



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
inline double cal_distance(pair<double, double> a, pair<double, double> b) {
    return sqrt((a.first - b.first) * (a.first - b.first) + (a.second - b.second) * (a.second - b.second));
}
// 获取方向
double cal_angle(pair<double, double> start, pair<double, double> end);
// 查看他是否能够被笑话
bool can_somebody_put(int pr_type);

#endif


