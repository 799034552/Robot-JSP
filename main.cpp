#include <iostream>
#include <string.h>
#include <limits.h>
#include "controler.h"
using namespace std;

int main(int argc, char *argv[])
{
    // 用于调参
    if (argc > 1)
    {
        // double val = strtod(argv[1], nullptr);
        power_k1 = strtod(argv[1], nullptr);
        power_k2 = strtod(argv[2], nullptr);
        power_p0 = strtod(argv[3], nullptr);
        // XXX = val;
    }
    else
    { // 默认值
        // XXX = DEFAULT_VAL;
    }
    // 清除
    setbuf(stdout, nullptr);
    setbuf(stderr, nullptr);
    read_map();
    delete_line();
    printf("OK\n");
    init();
    fflush(stdout);
    while (1)
    {
        get_frame(debug);
        printf("%d\n", frame_id);
        create_urgent(); // 创建紧急任务

        for (int i = 0; i < 4; ++i)
        {
            auto &this_robot = robot_list[i];
            double distance = MAX_NUMBER;
            int forward_id = -1;      // 当前机器人目的地id，-1表示没有
            int favourite_type = -1;  // 当前机器人偏好去的工作台类型
            int back_forward_id = -1; // 暂时保存机器人的前往的目的地
            // 如果一个机器人没事干
            // if (this_robot.action == None) {
            if (this_robot.action == None || this_robot.action == sell || this_robot.action == buy)
            {
                // if (frame_id <= 51 && (this_robot.action == sell || this_robot.action == buy))
                //     continue;
                // 先解除当前机器人的所有占用
                if (this_robot.action == buy)
                {
                    wb_list[this_robot.forward_id].output_occupy_by = -1;
                    back_forward_id = this_robot.forward_id;
                }
                else if (this_robot.action == sell)
                {
                    wb_list[this_robot.forward_id].input_occupy_by[this_robot.carry_id] = -1;
                    back_forward_id = this_robot.forward_id;
                }

                // 如果机器人没拿东西
                if (this_robot.carry_id == 0)
                {
                    // 找出能够拿东西的最近工作台
                    for (int j = 0; j < wb_list.size(); ++j)
                    {
                        // cerr<< frame_id<<endl;
                        if (frame_id < 0)
                        {
                            if (wb_list[j].output_occupy_by != -1 || wb_list[j].left_time == -1)
                                continue; // 被占用或者没有在生产就下一个
                        }
                        else
                        {
                            if (wb_list[j].output_occupy_by != -1 || !wb_list[j].output_box)
                                continue; // 被占用或者生产格没东西就下一个
                        }
                        // if (wb_list[j].output_occupy_by != -1 || wb_list[j].left_time == -1) continue; // 被占用或者没有在生产就下一个
                        if (!can_somebody_put(wb_list[j].type, this_robot.pos, wb_list[j].pos))
                            continue; // 要拿的东西是否能被消化
                        double tmp = cal_distance(this_robot.pos, wb_list[j].pos) + add_distance(j);

                        if (distance > tmp)
                        {
                            forward_id = j;
                            distance = tmp;
                        }
                    }
                    // 如果找到了
                    if (forward_id != -1)
                    {
                        this_robot.action = buy;
                        this_robot.forward_id = forward_id;
                        wb_list[forward_id].output_occupy_by = i;
                    }
                    else
                    { // 如果没有找到
                    }
                }
                // 如果机器人拿了东西
                else
                {
                    // cerr<<product_to_sell[this_robot.carry_id][0]<<endl;
                    // 找出能够放东西的最近工作台
                    // 遍历能够接受该物品的工作台类别
                    for (auto wb_type : product_to_sell[this_robot.carry_id])
                    {
                        // 在特定类别中寻找
                        for (auto wb_i : type_to_wb[wb_type])
                        {
                            // bool is_force = false; // 是否强迫输出，用于偏好判断
                            auto &wb = wb_list[wb_i];
                            if ((wb.input_occupy_by[this_robot.carry_id] != -1) || wb.get_input_box_item(this_robot.carry_id))
                                continue; // 被占用或者输入格满就下一个
                            double tmp = cal_distance(this_robot.pos, wb.pos);
                            // 如果他是7号并且在生产中，尽可能不要给他
                            if (wb.type == 7 && wb.left_time > 300)
                            {
                                tmp *= 3;
                            }
                            if (distance > tmp)
                            {
                                forward_id = wb.id;
                                distance = tmp;
                            }
                        }
                    }
                    // 如果找到了
                    if (forward_id != -1)
                    {
                        // 偏好选择
                        for (auto wb_i : favourite_map[wb_list[forward_id].type])
                        {
                            auto &wb = wb_list[wb_i];
                            if ((wb.input_occupy_by[this_robot.carry_id] != -1) || wb.get_input_box_item(this_robot.carry_id))
                                continue; // 被占用或者输入格满就下一个
                            // 出现同类型的没有在生产的7号工作台，并且工作台与当前目标距离差值小
                            if (wb.left_time < 300 && (wb.get_input_box_item(4) + wb.get_input_box_item(5) + wb.get_input_box_item(6) > 1) && cal_distance(wb_list[wb.id].pos, wb_list[forward_id].pos) < 10)
                            {
                                forward_id = wb.id;
                            }
                            break;
                        }
                        this_robot.action = sell;
                        this_robot.forward_id = forward_id;
                        wb_list[forward_id].input_occupy_by[this_robot.carry_id] = i;
                    }
                    else
                    { // 如果没有找到
                    }
                }
                // 本来是有任务的
                if (back_forward_id != -1)
                {
                    // 现在也有任务
                    if (forward_id != -1)
                    {
                        // 如果任务不相同
                        if (back_forward_id != forward_id)
                        {
                            auto diff_dis = cal_distance(wb_list[back_forward_id].pos, wb_list[forward_id].pos);
                            // if (frame_id > 6000)
                            //     diff_dis = 100;
                            // 以前任务有人能消化就恢复 并且距离相差太近就恢复
                            if (can_somebody_put(wb_list[back_forward_id].type, this_robot.pos, wb_list[back_forward_id].pos))
                            {
                                if (this_robot.action == buy)
                                {
                                    wb_list[this_robot.forward_id].output_occupy_by = -1;
                                    wb_list[back_forward_id].output_occupy_by = this_robot.id;
                                    this_robot.forward_id = back_forward_id;
                                }
                                else if (this_robot.action == sell)
                                {
                                    wb_list[this_robot.forward_id].input_occupy_by[this_robot.carry_id] = -1;
                                    wb_list[back_forward_id].input_occupy_by[this_robot.carry_id] = this_robot.id;
                                    this_robot.forward_id = back_forward_id;
                                }
                            }
                        }
                    }
                    // 现在没任务
                    else
                    {
                        this_robot.forward_id = -1;
                        this_robot.action = None;
                    }
                }
            }
        }
        for (int i = 0; i < 4; ++i)
        {
            auto &this_robot = robot_list[i];
            if (this_robot.action == buy || this_robot.action == sell)
            {
                // 还没有到达目的地
                if (this_robot.workbrench_id != this_robot.forward_id)
                {
                    auto distance = cal_distance(this_robot.pos, wb_list[this_robot.forward_id].pos);

                    // 判断机器人旋转方向
                    auto angle = cal_angle(this_robot.pos, wb_list[this_robot.forward_id].pos);
                    std::pair<double, double> control = this_robot.Robot_controle(distance, angle);
                    // cerr<<this_robot.id<<"  "<<frame_id<<endl;
                    // exit(0);
                    printf("forward %d %f\n", this_robot.id, control.first);
                    printf("rotate %d %f\n", this_robot.id, control.second);
                }
                // 是否已经到达目的地
                else
                {
                    // 到达目的地
                    if (this_robot.action == buy)
                    {
                        printf("buy %d\n", this_robot.id);
                        wb_list[this_robot.forward_id].output_occupy_by = -1;
                    }

                    else if (this_robot.action == sell)
                    {
                        printf("sell %d\n", this_robot.id);
                        wb_list[this_robot.forward_id].input_occupy_by[this_robot.carry_id] = -1;
                    }
                    // for(auto &urgent_task: urgent_list) {
                    //     for(int k = 0; k < urg)
                    // }

                    // 恢复自身状态与目标工作台占用
                    this_robot.action = None;
                    this_robot.forward_id = -1;
                }
            }
            // 如果机器人在分配任务后还是没事干
            else
            {
                // cerr<<"wait\n";
                printf("forward %d %f\n", this_robot.id, 0.0);
                printf("rotate %d %f\n", this_robot.id, 0.0);
            }
        }
        printf("OK\n");
        if (frame_id == 9000)
            break;
    }
    return 0;
}