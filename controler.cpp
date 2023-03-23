#include "controler.h"
using namespace std;

char tmp_txt[1024];
vector<Robot> robot_list;
vector<Workbench> wb_list;
unordered_map<int, vector<int>> favourite_map;
bool debug = false;
// 通过工作台类型读取工作台列表
vector<vector<int>> type_to_wb(10, vector<int>());
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
// 紧急通知列表，快来取我列表
vector<Urgent_task*> urgent_list;



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
                type_to_wb[tmp_txt[j] - '0'].emplace_back(wb_list.back().id);
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
    auto y = end.second - start.second;
    return atan2(y, x);
}

bool can_somebody_put(int pr_type, pair<double, double> robot_pos, pair<double, double> wb_pos) {
    for (auto &wb_type: product_to_sell[pr_type]) {
        for(auto &wb_i : type_to_wb[wb_type]) {
            if (!wb_list[wb_i].get_input_box_item(pr_type) && wb_list[wb_i].input_occupy_by[pr_type] == -1) {
                // 计算需要消耗的时间
                auto d1 = cal_distance(robot_pos, wb_pos);
                auto d2 = cal_distance(wb_pos, wb_list[wb_i].pos);
                auto total_frame = (d1 + d2) / 5.5 * 50;
                if (total_frame < 9000 - frame_id)
                    return true;
            }
                
            // if (!wb_list[wb_i].get_input_box_item(pr_type))
            //     return true;
        }
    }
    return false;
}

void init() {
    // 对7号工作台的位置进行排序
    vector<pair<double, int>> dis_total; // 7号工作台最近的4、5、6号工作台距离之和
    for(auto &outer_wb_i: type_to_wb[7]) {
        dis_total.emplace_back(0, wb_list[outer_wb_i].id);
        int a[3] = {4,5,6};
        for(const auto &i : a) {
            double min_dis = MAX_NUMBER;
            for(auto &inner_wb_i : type_to_wb[4]) {
                auto tmp = cal_distance(wb_list[outer_wb_i].pos, wb_list[inner_wb_i].pos);
                if (tmp < min_dis)
                    min_dis = tmp;
            }
            dis_total.back().first += min_dis;
        }
    }
    sort(dis_total.begin(),dis_total.end(),[](pair<double, int> &a, pair<double, int> &b) -> bool {return a.first < b.first;});
    for(auto & tmp: dis_total) {
        favourite_map[7].emplace_back(tmp.second);
    }
}

//链式找到离他最近的能发任务的工作台
bool sub_create_urgent(Urgent_task* task, int wb_i) {
    // 如果他已经有任务
    if (wb_list[wb_i].urgent_task != nullptr) {
        delete task;
        return false;
    }
    auto &wb = wb_list[wb_i];
    // 如果他是最后一级
    switch (wb.type)
    {
    case 1:
    case 2:
    case 3:
        return true;
    default:
        break;
    }
    // 他自己是否有东西输出
    if (wb.output_box) {
        return true;
    }
    
    // 先找到他会缺什么
    for(auto &need_type: wb_can_put[wb.type]) {
        // 如果他真的缺
        if (!wb.get_input_box_item(need_type)) {
            double distance = MAX_NUMBER;
            int forward_id = -1;
            // 找到离他最近的那个工作台
            for(auto &wb_i_2: type_to_wb[need_type]) {
                auto tmp = cal_distance(wb.pos, wb_list[wb_i_2].pos);
                if (tmp < distance) {
                    distance = tmp;
                    forward_id = wb_i_2;
                }
            }
            if (sub_create_urgent(task,  forward_id)) {
                task->task_list.emplace_back(forward_id, wb_i);
            }
        }
    }
    return true;
}

void create_urgent() {
    for(auto &wb_i: type_to_wb[7]) {
        auto & wb = wb_list[wb_i];
        // 工作台输入有2个以上东西并且没有输入列表并且没有在生成的时候
        if ((wb.get_input_box_item(4) + wb.get_input_box_item(5) + wb.get_input_box_item(6)) > 1 && wb.urgent_task == nullptr && wb.left_time < 200) {
            // 发布紧急任务
            auto task = new Urgent_task();
            if (sub_create_urgent(task, wb.id)) {
                urgent_list.push_back(task);
                for(int i = 0; i < task->task_list.size(); ++i) 
                    task->task_status.emplace_back(-1);
            }
        }
    }
}

double add_distance(int wb_i) {
    return 0;
    // return 0;
    // if (frame_id > 8282)
    //     return 0;
    auto x = frame_id;
    auto &wb = wb_list[wb_i];
    switch (wb.type)
    {
    case 1:
    case 2:
    case 3:
        if (frame_id > 7500) {
            // http://tools.jb51.net/jisuanqi/create_fun
            // 7500, 2
            // 7800, 8
            // 8000, 12
            // 8500, 20
            // 8700, 40
            // 9000, 70
            return  -2.2504409233607092e-13*x*x*x*x*x+9.235449761589254e-9*x*x*x*x-0.00015139988756175988*x*x*x+1.2393479242755483*x*x-5066.0809834156535*x+8272977.739510031;
        }
        break;
    case 4:
    case 5:
    case 6:
        if (frame_id > 7500) {
            // 7500, 0
            // 8000, 5
            // 8500, 15
            // 9000, 30
            return  -3.6271869860450264e-22*x*x*x+0.000010000000000008898*x*x-0.1450000000000725*x+525.0000000001965;
        }
        break;
    }
    return 0;
}
