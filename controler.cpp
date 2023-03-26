#include "controler.h"
using namespace std;

int map_type = 0;
const double normal_speed = 5.2;
char tmp_txt[1024];
vector<Robot> robot_list;
vector<Workbench> wb_list;
unordered_map<int, vector<int>> favourite_map;
bool debug = false;
// 通过工作台类型读取工作台列表
vector<vector<int>> type_to_wb(10, vector<int>());
long frame_id;
int money;
// 4、5、6号工作台吃到的数量
unordered_map<int, vector<int>> eat_product {
    {4, vector<int>(10,0)},
    {5, vector<int>(10,0)},
    {6, vector<int>(10,0)},
    
};
// 工件正在生产或者已经生成的数量
unordered_map<int, int> wb_product_count {
    {1, 0},
    {2, 0},
    {3, 0},
    {4, 0},
    {5, 0},
    {6, 0},
    {7, 0}
};

// 工作台生成时间
unordered_map<int, int> wb_to_time {
    {1, 50},
    {2, 50},
    {3, 50},
    {4, 500},
    {5, 500},
    {6, 500},
    {7, 1000},
    {8, 1},
    {9, 1},
};
// 产品购买与售出价格
unordered_map<int, pair<int, int>> product_money {
    {1, pair<int, int>{3000, 6000}},
    {2, pair<int, int>{4400, 7600}},
    {3, pair<int, int>{5800, 9200}},
    {4, pair<int, int>{15400, 22500}},
    {5, pair<int, int>{17200, 25000}},
    {6, pair<int, int>{19200, 27500}},
    {7, pair<int, int>{76000, 105000}},
};
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
        switch (wb_list[i].type)
        {
        case 1:
        case 2:
        case 3:
            if (wb_list[i].left_time == -1)
                wb_list[i].left_time = 50;
            break;
        }
        // if (wb_list[i].left_time == wb_to_time[wb_list[i].type] - 1) {
        //     wb_product_count[wb_list[i].type]++;
        // }
        
        wb_list[i].last_input_box = wb_list[i].input_box;
        wb_list[i].input_box = line_info[4];
        wb_list[i].output_box = line_info[5];
        // 检测喂进去了多少东西
        if (wb_list[i].last_input_box != wb_list[i].input_box && wb_list[i].input_box != 0) {
            wb_product_count[wb_list[i].type]++;
        }
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
                auto total_frame = (d1 + d2) / 5.2 * 50;
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

shared_ptr<WorldStatus> cal_next_statue(const shared_ptr<WorldStatus> &cur_status, int dt) {
    shared_ptr<WorldStatus> next_status = cur_status->duplicate();
    next_status->frame_id = cur_status->frame_id + dt;
    unordered_map<int, vector<pair<int, int>>> reach_robot; // 工作台: [[机器人id，到达时间]]
    for(const auto &rb: cur_status->robot_list) {
        // 计算到达时间
        auto reach_time = cal_reach_time(*cur_status, rb.id);
        if (reach_time == -1) {

        }else if (reach_time > dt) { // 无法到达
            auto angle = cal_angle(rb.pos,  cur_status->wb_list[rb.forward_id].pos);
            // 更新下一时刻机器人的位置状态
            next_status->robot_list[rb.id].pos.first += normal_speed*(dt / 50.0)* cos(angle);
            next_status->robot_list[rb.id].pos.second += normal_speed*(dt / 50.0)* sin(angle);
        } else { // 可以到达
            next_status->robot_list[rb.id].pos = cur_status->wb_list[rb.forward_id].pos;
            reach_robot[rb.forward_id].emplace_back(rb.id, reach_time);
        }
    }
    for(auto &wb: next_status->wb_list) {
        if (reach_robot.count(wb.id) == 0) { //当前工作台没有机器人到达
            change_wb_statue(wb, dt);
        } 
        if (reach_robot.count(wb.id) != 0) { //当前工作台有机器人到达
            int last_reach_time = 0;
            for(auto & reach_r :reach_robot[wb.id]) {
                change_wb_statue(wb, reach_r.second - last_reach_time);
                last_reach_time = reach_r.second;
                auto & rb = next_status->robot_list[reach_r.first];
                if ( rb.action == buy) { // 机器人想买东西
                    if (wb.output_box) { // 有东西
                        wb.output_box = 0;
                        rb.action = None;
                        rb.carry_id = wb.type;
                        rb.forward_id = -1;
                        rb.buy_frame = cur_status->frame_id + reach_r.second;
                        next_status->money -= product_money[wb.type].first;
                    }
                } else if (rb.action == sell) {
                    if (!wb.get_input_box_item(rb.carry_id)) { // 输入格没有东西
                        wb.input_box &= (1 << rb.carry_id);
                        rb.action = None;
                        int x = cur_status->frame_id + reach_r.second - rb.buy_frame;
                        next_status->money += (1 - sqrt(1 - (1-x/9000.0)*(1-x/9000.0)))*(1-0.8)+0.8;
                        rb.carry_id = 0;
                        rb.forward_id = -1;
                    }
                }
            }
            change_wb_statue(wb, dt - last_reach_time);
        }
    }
    return next_status;
}
int cal_reach_time(const WorldStatus & cur_status, const int &robot_id) {
    const auto &rb = cur_status.robot_list[robot_id];
    if (rb.forward_id == -1) return -1;
    const auto &wb = cur_status.wb_list[rb.forward_id];
    auto distance = cal_distance(rb.pos, wb.pos);
    return distance / normal_speed * 50;
}

shared_ptr<WorldStatus> WorldStatus::duplicate() {
    return make_shared<WorldStatus>(*this);
}

// @brief 改变工作台的状态
void change_wb_statue(Workbench & wb, int dt) {
    if (wb.left_time >= 0) { //在生产中
        if (wb.left_time < dt) { //可以生产完
            if (wb.output_box) { //如果输出中有东西
                wb.left_time = 0;
                return;
            }
            wb.output_box = 1;
            bool flag = true; // 是否有足够的输入
            for(auto &input: wb_can_put[wb.type]) {
                if (!wb.get_input_box_item(input))
                    flag = false;
            }
            if (flag) { // 消耗输入
                wb.left_time = max(wb_to_time[wb.type] - (dt - wb.left_time), 0);
                wb.input_box = 0;
            } else { // 没有足够的输入
                wb.left_time = -1;
            }
        } else { //不能生成完
            wb.left_time -= dt;
        }
    }
}

void WorldStatus::show() {
    cerr<<endl<<"---------------------------------\n";
    cerr<<"frame_id:"<<frame_id<<endl;
    cerr<<"money:"<<money<<endl;
    cerr<<"robot_id  forward_id  action   carry_id    x    y\n";
    
    for(auto&rb:robot_list) {
        sprintf(tmp_txt, "%5d %5d %5d %5d %6.2f %6.2f\n", rb.id, rb.forward_id, rb.action, rb.carry_id, rb.pos.first, rb.pos.second);
        cerr<<tmp_txt;
    }
    cerr<<"wb_id  wb_type wb_left_time   wb_output    wb_input   x   y\n";
    for(auto &wb:wb_list) {
        sprintf(tmp_txt, "%5d %5d %5d %5d %5d %6.2f %6.2f\n", wb.id, wb.type,wb.left_time, wb.output_box, wb.input_box, wb.pos.first, wb.pos.second);
        cerr<<tmp_txt;
    }
    cerr<<endl<<"---------------------------------\n";
}

shared_ptr<DecisionTreeNode> build_decision_tree(shared_ptr<DecisionTreeNode> &root, int deep) {
    shared_ptr<WorldStatus> next_status = nullptr;
    int min_reach_time = MAX_NUMBER;
    for(auto &rb: root->one_world->robot_list) {
        min_reach_time = min(cal_reach_time(*root->one_world, rb.id), min_reach_time);
    }
    if(min_reach_time == -1) //有人没有目标
        next_status = root->one_world;
    else
        next_status = cal_next_statue(root->one_world, min_reach_time);

    //创建第一层决策树
    vector<int> no_target_robot;
    vector<vector<int>> can_choose;
    // 找到所有没有目标的机器人
    for(auto &rb: next_status->robot_list) {
        if (rb.forward_id == -1) { // 这个机器人没有目标
            no_target_robot.push_back(rb.id);
            can_choose.emplace_back();
            for(auto&wb: next_status->wb_list) {
                if (rb.carry_id == 0) { //这个机器人没有拿东西
                    if(wb.output_box || wb.left_time != -1) //可以去有输出的地方或者是正在生产的地方
                        can_choose.back().push_back(wb.id);
                        
                } else { // 如果这个机器人拿了东西
                    if (!wb.get_input_box_item(rb.carry_id))
                        can_choose.back().push_back(wb.id);
                    // if (wb.id != 1 && wb.id != 2 && wb.id != 3) //可以去所有能够放的地方放
                    //     can_choose.back().push_back(wb.id);
                }
            }
        }
    }
    if (debug) {
        cerr<<can_choose[0].size()<<endl;
    }
    //遍历所有可能的去向，添加所有世界
    world_choose(next_status, no_target_robot, can_choose, root);
    deep -= 1;
    if (deep > 0) {
        for(auto & node:root->child) {
            build_decision_tree(node, deep);
        }
    }
    return root;
}
// 对于选择遍历结果
void world_choose(shared_ptr<WorldStatus> cur_status, vector<int> &no_target_robot, vector<vector<int>> &can_choose, shared_ptr<DecisionTreeNode> &root) {
    vector<int> choose;
    true_world_choose(cur_status, no_target_robot, can_choose, choose, root);
}

void true_world_choose(shared_ptr<WorldStatus> cur_status, vector<int> &no_target_robot, vector<vector<int>> &can_choose, vector<int> &choose, shared_ptr<DecisionTreeNode> &root) {
    if (choose.size() == no_target_robot.size()) { //遍历结束
        shared_ptr<DecisionTreeNode> child_node = make_shared<DecisionTreeNode>();
        root->child.push_back(child_node);
        // child_node->parent = root;
        auto next_status = cur_status->duplicate();
        child_node->one_world = next_status;
        for(int i = 0; i < choose.size(); ++i) {
            auto &rb = next_status->robot_list[no_target_robot[i]];
            rb.forward_id = choose[i];
            if (rb.carry_id == 0) { //身上没东西
                rb.action = buy;
            } else {
                rb.action = sell;
            }
            child_node->choose.emplace_back(rb.id, rb.forward_id);
        }
        return;
    }
    int i = choose.size();
    for(int j = 0; j < can_choose[i].size(); ++j) {
        choose.emplace_back(j);
        true_world_choose(cur_status,no_target_robot, can_choose,choose, root);
        choose.pop_back();
    }

}

//拿东西的
double add_distance(int wb_i) {

    // return 0;
    auto &wb = wb_list[wb_i];
    double left_time_cost = 0;
    if (!wb.output_box && wb.left_time>= 0) {
        left_time_cost = wb.left_time / 50.0 * 5.2 + 100;
    }
    // 对于4号图减少1的距离
    // if (map_type == 4 && wb.type == 1) {
    //     return left_time_cost - 20;
    // }
    double end_cost = 0;
    auto x = frame_id;
    // switch (wb.type)
    // {
    // case 1:
    // case 2:
    // case 3:
    //     if (frame_id > 8000) {
    //         // http://tools.jb51.net/jisuanqi/create_fun
    //         // 7500, 2
    //         // 7800, 8
    //         // 8000, 12
    //         // 8500, 20
    //         // 8700, 40
    //         // 9000, 70
    //         end_cost =  -2.2504409233607092e-13*x*x*x*x*x+9.235449761589254e-9*x*x*x*x-0.00015139988756175988*x*x*x+1.2393479242755483*x*x-5066.0809834156535*x+8272977.739510031;
    //     }
    //     break;
    // case 4:
    // case 5:
    // case 6:
    //     if (frame_id > 8000) {
    //         // 7500, 0
    //         // 8000, 5
    //         // 8500, 15
    //         // 9000, 30
    //         end_cost =  -3.6271869860450264e-22*x*x*x+0.000010000000000008898*x*x-0.1450000000000725*x+525.0000000001965;
    //     }
    //     break;
    // }
    return left_time_cost + end_cost;
    return 0;
}

// 放东西的
double Workbench::reduce_distance(const int &carry_id) {
    double avg_c = (wb_product_count[4] + wb_product_count[5] + wb_product_count[6]) / 3.0;
    switch (type)
    {
    case 1:
    case 2:
    case 3:
    case 8:
        return 0;
    case 9:
        return -30;
        break;
    case 4:
    case 5:
    case 6:
        if (map_type == 1) {
            if (wb_product_count[type] > avg_c) {
                // return -10;
                // cerr<<wb_product_count[4]<<endl
                // <<wb_product_count[5]<<endl
                // <<wb_product_count[6]<<endl;
                // // exit(0);

                // return -20;
            }
        }
        // if (map_type == 3) {
        //     return 10;
        // }
        if (map_type == 1) {
            int sum = 0;
            // for(auto &wb_i: wb_can_put[carry_id]) {
            //     auto &wb = wb_list[wb_i];
            //     if (wb.type == 9) continue;
            //     sum += eat_product[wb.type][carry_id];
            // }
            // if (eat_product[type][carry_id] > sum / 2.0 + 3) {
            //     return -10;
            // }
            wb_product_count[4] = wb_product_count[5] = wb_product_count[6] = 0;
            for(auto & wb_i: type_to_wb[7]) {
                if (wb_list[wb_i].get_input_box_item(4))
                    wb_product_count[4]++;
                if (wb_list[wb_i].get_input_box_item(5))
                    wb_product_count[5]++;
                if (wb_list[wb_i].get_input_box_item(6))
                    wb_product_count[6]++;
            }
            avg_c = (wb_product_count[4] + wb_product_count[5] + wb_product_count[6]) / 3.0;
            if (wb_product_count[type] > avg_c + 1)
                return -10;
            //     // cerr<<wb_product_count[4]<<endl
            //     // <<wb_product_count[5]<<endl
            //     // <<wb_product_count[6]<<endl;
            //     // // exit(0);

            //     // return -20;
            // }
        }
        // if (map_type == 3) {
        //     return 10;
        // }
        break;
    default:
        break;

    }
    int count = 0;
    for(auto &can_input_type: wb_can_put[type]) {
        if (get_input_box_item(can_input_type))
            ++count;
    }
    switch (map_type)
    {
    case 1:
        if (wb_can_put[type].size() - count == 1) {
            return (frame_id - input_frame) / 50.0 * 1;
        }
        break;
    case 2:
        // if (type == 4) {
        //     return (frame_id - input_frame) / 50.0 * 3;
        // }
        if (wb_can_put[type].size() - count == 1) {
            return (frame_id - input_frame) / 50.0 * 1;
        }
        break;
    case 3:
        // if (wb_can_put[type].size() - count == 1) {
        //     return (frame_id - input_frame) / 50.0 * 1;
        // }
        // if (type == 6)
        //     return 20;
        return 10;
        break;
    case 4:
        if (type == 4) {
            // return 1000;
            return max((frame_id - input_frame) / 50.0 * 4, 25.0);
        }
        if (wb_can_put[type].size() - count == 1) {
            // return (frame_id - input_frame) / 50.0 * 0.9;
            return (frame_id - input_frame) / 50.0 * 0.9;
        }
        break;
    }
    // if (type_to_wb[4].size() == 1 && type == 4 && wb_list[type_to_wb[4][0]].id == 17) { // 图4
    //     return (frame_id - input_frame) / 50.0 * 3;
    // }
    // if (type_to_wb[4].size() == 1 && type == 4 && wb_list[type_to_wb[4][0]].id == 12) { // 图2
    //     return (frame_id - input_frame) / 50.0 * 4;
    // }
    if (wb_can_put[type].size() - count == 1) {
        return (frame_id - input_frame) / 50.0 * 3;
    }
    return 0;
}

void cal_map_type() {
    if (wb_list.size() == 43)
        map_type = 1;
    if (wb_list.size() == 25)
        map_type = 2;
    if (wb_list.size() == 50)
        map_type = 3;
    if (wb_list.size() == 18)
        map_type = 4;
}

int cal_input_total(int wb_id) {
    auto &wb = wb_list[wb_id];
    int count = 0;
    for(auto &t: wb_can_put[wb.type]) {
        if (wb.get_input_box_item(t)) 
            count++;
    }
    return count;
}
