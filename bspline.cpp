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

double f1(double &s)
{
    return pow(1 - s, 3.0) / 6;
}
double f2(double &s)
{
    return (3 * s * s * s - 6 * s * s + 4) / 6.0;
}
double f3(double &s)
{
    return (-3 * s * s * s + 3 * s * s + 3 * s + 1) / 6.0;
}
double f4(double &s)
{
    return s * s * s / 6.0;
}

/// @brief 增广控制点，使曲线经过点control_point[index]
/// @param half_car_len 小车车长一半
/// @param speed control_point[index] 期望小车速度方向
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

vector<Pos> Bspline::create_b_spline()
{
    static int sample_size = 101; // 采样点步长为0.01，共101个

    int N = this->control_point.size();
    vector<Pos> path((N - 3) * sample_size);

    vector<double> s(sample_size);
    generate(s.begin(), s.end(), [n = -0.01]() mutable
             { return n += 0.01; });
    vector<double> f1s(s);
    vector<double> f2s(s);
    vector<double> f3s(s);
    vector<double> f4s(s);

    for_each(f1s.begin(), f1s.end(), &f1);
    for_each(f2s.begin(), f2s.end(), &f2);
    for_each(f3s.begin(), f3s.end(), &f3);
    for_each(f4s.begin(), f4s.end(), &f4);

    auto it = path.begin();
    for (int i = 0; i < N - 3; ++i)
    {
        for (int j = 0; j < sample_size; ++j, ++it)
            *it = this->control_point[i] * f1s[j] + this->control_point[i + 1] * f2s[j] + this->control_point[i + 2] * f3s[j] + this->control_point[i + 3] * f4s[j];
    }
    return path;
}