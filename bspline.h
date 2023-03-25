#ifndef __BSPLINE_H_
#define __BSPLINE_H_
#include <vector>
#include "basic.h"

// B 样条曲线
class Bspline
{
private:
    double f1s(double s);
    double f2s(double s);
    double f3s(double s);
    double f4s(double s);
public:
    std::vector<Pos> control_point;

    Bspline(std::vector<Pos> pos_list);
    ~Bspline();
    void augement_ctrl_point(double half_car_len, int index, Vec speed);
    void create_b_spline();
};





#endif