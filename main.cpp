#include <iostream>
#include <string.h>
#include "controler.h"
using namespace std;

int main() {
    // 清除
    setbuf(stdout, nullptr);
    setbuf(stderr, nullptr);
    read_map();
    delete_line();
    printf("OK\n");
    fflush(stdout);
    // for(int i = 0; i < wb_list.size(); ++i) {
    //     cerr<<wb_list[i].id<<endl;
    // }
    // exit(0);
    while(1) {
        get_frame(false);
        printf("%d\n", frame_id);
        for(int i = 0; i < 4; ++i) {
            auto & this_robot = robot_list[i];
            double distance = INT_MAX;
            int forward_id = -1;
            // 如果一个机器人没事干
            if (this_robot.action == None) {
                // cerr<<this_robot.forward_id<<endl;
                // cerr<<"sfddasf\n";
                // 如果机器人没拿东西
                if (this_robot.carry_id == 0) {
                    // 找出能够拿东西的最近工作台
                    for(int j = 0; j < wb_list.size(); ++j) {
                        if (wb_list[j].output_occupy_by != -1 || !wb_list[j].output_box) continue; // 被占用或者生产格没东西就下一个
                        if (!can_somebody_put(wb_list[j].type)) continue; // 要拿的东西是否能被消化
                        double tmp = cal_distance(this_robot.pos, wb_list[j].pos);
                        if (distance > tmp) {
                            forward_id = j;
                            distance = tmp;
                        }
                    }
                    //如果找到了
                    if (forward_id != -1) {
                        this_robot.action = buy;
                        this_robot.forward_id = forward_id;
                        wb_list[forward_id].output_occupy_by = i;
                    } else { // 如果没有找到
                    }
                }
                // 如果机器人拿了东西
                else {
                     // cerr<<product_to_sell[this_robot.carry_id][0]<<endl;
                    // 找出能够放东西的最近工作台
                    // 遍历能够接受该物品的工作台类别
                    for(auto wb_type : product_to_sell[this_robot.carry_id]) {
                        // 在特定类别中寻找
                        for (auto wb_i: type_to_wb[wb_type]) {
                            auto & wb = wb_list[wb_i];
                            if((wb.input_occupy_by[this_robot.carry_id] != -1) || wb.get_input_box_item(this_robot.carry_id)) continue; //被占用或者输入格满就下一个
                            
                            double tmp = cal_distance(this_robot.pos, wb.pos);
                            if (distance > tmp) {
                                forward_id = wb.id;
                                distance = tmp;
                            }
                        }
                    }
                    //如果找到了
                    if (forward_id != -1) {
                        this_robot.action = sell;
                        this_robot.forward_id = forward_id;
                        wb_list[forward_id].input_occupy_by[this_robot.carry_id] = i;
                    } else { // 如果没有找到
                       
                    }
                }
            }
            // 机器人有事干
            if (this_robot.action == buy || this_robot.action == sell)
            {
                // if (this_robot.id == 0) {
                //     cerr<<this_robot.forward_id<<" "<<wb_list[this_robot.forward_id].type<<endl;
                // }
                // 还没有到达目的地
                if (this_robot.workbrench_id != this_robot.forward_id){
                    // cerr<<"get111"<<endl;
                    auto distance = cal_distance(this_robot.pos, wb_list[this_robot.forward_id].pos);
                    if (distance < 4) {
                        //如果距离目标3米内
                        printf("forward %d %f\n", this_robot.id, 3.0);

                    }
                    else {
                        //全力加速
                        printf("forward %d %f\n", this_robot.id, 100.0);
                    }
                    
                    // 判断机器人旋转方向
                    auto angle = cal_angle(this_robot.pos, wb_list[this_robot.forward_id].pos);
                    auto diss = this_robot.face - angle;
                    double roate_speed = 0;

                    // 与预期方向一致
                    if (abs(diss) < 0.00001) {
                        roate_speed = 0.0;
                    }
                    else if (this_robot.face > angle) {
                        diss = this_robot.face - angle;
                        if (diss <= PI) {
                            roate_speed = -PI;
                        } else {
                            roate_speed = PI;
                        }
                    } else {
                        diss = angle - this_robot.face;
                        if (diss <= PI) {
                            roate_speed = PI;
                        } else {
                            roate_speed = -PI;
                        }
                    }
                    printf("rotate %d %f\n", this_robot.id, roate_speed);
                    
                }
                //是否已经到达目的地
                else  {
                    // cerr<<"get"<<endl;
                    //到达目的地
                    // cerr<< "errrrrrrrrrrrrr\n";
                    if (this_robot.action == buy) {
                        printf("buy %d\n", this_robot.id);
                        wb_list[this_robot.forward_id].output_occupy_by = -1;
                    }
                        
                    else if (this_robot.action == sell) {
                        printf("sell %d\n", this_robot.id);
                        wb_list[this_robot.forward_id].input_occupy_by[this_robot.carry_id] = -1;
                    }
                        
                    // 恢复自身状态与目标工作台占用
                    this_robot.action = None;
                    this_robot.forward_id = -1;
                }
            }
            // 如果机器人在分配任务后还是没事干
            else {
                // cerr<<"wait\n";
                printf("forward %d %f\n", this_robot.id, 0.0);
                printf("rotate %d %f\n", this_robot.id, 0.0);
            }
        }
        printf("OK\n");
    }
    return 0;
}
