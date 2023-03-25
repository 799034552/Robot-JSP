#include "bspline.h"
#include <algorithm>
#include <cmath>

using namespace std;

Bspline::Bspline(std::vector<Pos> pos_list)
{
    // 至少4个控制点
    this->control_point = pos_list;
}

Bspline::~Bspline()
{
}

double Bspline::f1s(double s)
{
    return pow(1 - s, 3.0) / 6;
}
double Bspline::f2s(double s)
{
    return (3 * s * s * s - 6 * s * s + 4) / 6.0;
}
double Bspline::f3s(double s)
{
    return (-3 * s * s * s + 3 * s * s + 3 * s + 1) / 6.0;
}
double Bspline::f4s(double s)
{
    return s * s * s / 6.0;
}

/// @brief 增广控制点，使曲线经过点control_point[index]
/// @param half_car_len 小车车长一半
/// @param speed control_point[index] 小车速度方向
void Bspline::augement_ctrl_point(double half_car_len, int index, Vec speed)
{
    Pos c = this->control_point[index];
    Vec temp = speed.unit_vector() * half_car_len;
    Pos cn_2(c.x - temp.x, c.y - temp.y);
    Pos cn(c.x + temp.x, c.y + temp.y);

    auto it = this->control_point.emplace(this->control_point.begin() + index, cn_2);
    this->control_point.emplace(it + 2, cn);
    return;
}

void Bspline::create_b_spline()
{
    vector<double> s(101);
    generate(s.begin(),s.end(),[n = -0.01] () mutable { return n+=0.01; });
}