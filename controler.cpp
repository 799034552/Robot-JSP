#include "controler.h"
using namespace std;

char tmp_txt[1024];
vector<Robot> robot_list;
vector<Workbench> wb_list;
// 通过工作台类型读取工作台列表
vector<vector<Workbench*>> type_to_wb(10, vector<Workbench*>());
long frame_id;
int money;
// 产品能放到什么工作台
unordered_map<int, vector<int>> product_to_sell {
    {1, vector<int>{4, 5, 9}},
    {2, vector<int>{4, 6, 9}},
    {3, vector<int>{5, 6, 9}},
    {4, vector<int>{7, 9}},
    {5, vector<int>{7, 9}},
    {6, vector<int>{7, 9}},
    {7, vector<int>{8,9}},
};
// 工作台能放什么产品
unordered_map<int, vector<int>> wb_can_put {
    {1, vector<int>{}},
    {2, vector<int>{}},
    {3, vector<int>{}},
    {4, vector<int>{1, 2}},
    {5, vector<int>{1, 3}},
    {6, vector<int>{2, 3}},
    {7, vector<int>{4, 5, 6}},
    {8, vector<int>{7}},
    {9, vector<int>{1,2,3,4,5,6,7}},
};


bool readUntilOK(char* t) {
    while (fgets(t, sizeof t, stdin)) {
        if (t[0] == 'O' && t[1] == 'K') {
            return true;
        }
    }
    return false;
}

//获取地图
void read_map() {
    for (int i = 0; i < 100; ++i) {
        fgets(tmp_txt, sizeof tmp_txt, stdin);
        for(int j = 0; j < 100; ++j) {
            switch (tmp_txt[j])
            {
            case '.':
                break;
            case 'A':
                robot_list.emplace_back();
                robot_list.back().id = robot_list.size() - 1;
                robot_list.back().pos.first = 0.25 + 0.5 * j ;
                robot_list.back().pos.second = 50 - ( i*0.5 + 0.25);
                break;
            default:
                //工作台
                wb_list.emplace_back(tmp_txt[j] - '0');
                wb_list.back().id = wb_list.size() - 1;
                wb_list.back().pos.first = 0.25 + 0.5 * j ;
                wb_list.back().pos.second = 50 - ( i*0.5 + 0.25);
                type_to_wb[tmp_txt[j] - '0'].push_back(&wb_list.back());
                break;
            }
        }
    }
}

vector<double> split_line_text(char s[])
{
    vector<double> res;
    int l = strlen(s);
    int start = 0;
    for(int i = 0; i < l; ++i) {
        if (s[i] == ' ') {
            s[i] == '\0';
            res.emplace_back(strtod(s+start, nullptr));
            start = i + 1;
        }
    }
    if (start < l) {
        res.emplace_back(strtod(s+start, nullptr));
    }
    return res;
}

vector<double> get_split_line_text(bool is_debug) {
    fgets(tmp_txt, sizeof tmp_txt, stdin);
    if (is_debug)
        cerr<<"----------orgin line:"<<tmp_txt<<endl;
    return split_line_text(tmp_txt);
}

void delete_line(int n) {
    while(n--)
        fgets(tmp_txt, sizeof tmp_txt, stdin);
}

void get_frame(bool is_debug) {
    auto line_info = get_split_line_text(is_debug);
    frame_id = line_info[0];
    money = line_info[1];
    delete_line();
    for(int i = 0; i < wb_list.size(); ++i) {
        line_info = get_split_line_text(is_debug);
        wb_list[i].left_time = line_info[3];
        wb_list[i].input_box = line_info[4];
        wb_list[i].output_box = line_info[5];
    }
    for(int i = 0; i < robot_list.size(); ++i) {
        line_info = get_split_line_text(is_debug);
        robot_list[i].workbrench_id = line_info[0];
        robot_list[i].carry_id = line_info[1];
        robot_list[i].time_conf = line_info[2];
        robot_list[i].hit_conf = line_info[3];
        robot_list[i].rotate_speed = line_info[4];
        robot_list[i].linear_speed.first = line_info[5];
        robot_list[i].linear_speed.second = line_info[6];
        robot_list[i].face = line_info[7];
        robot_list[i].pos.first = line_info[8];
        robot_list[i].pos.second = line_info[9];
    }
    delete_line();
}

bool Workbench::get_input_box_item(int n) {
    int a = 1 << n;
    return a & this->input_box;
}

double cal_angle(pair<double, double> start, pair<double, double> end) {
    auto x = end.first - start.first;
    auto y = end.second - start.first;
    return atan2(y, x);
}

