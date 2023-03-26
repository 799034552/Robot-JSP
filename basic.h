#ifndef __BASIC_H_
#define __BASIC_H_
#include <cmath>
#include <utility>
#include <iostream>
#include <fstream>
#define PI 3.14159265359

class Pos
{
private:
    /* data */
public:
    double x;
    double y;

    Pos(){};
    Pos(double x, double y);
    Pos(std::pair<double, double> pos);
    ~Pos();

    friend Pos operator*(const Pos &A, double scale);
    friend Pos operator*(double scale, const Pos &A);
    friend Pos operator/(const Pos &A, double scale);
    friend Pos operator/(double scale, const Pos &A);
    friend std::ostream &operator<<(std::ostream &output, const Pos &p)
    {
        output << p.x << ", " << p.y;
        return output;
    }

    Pos operator+(const Pos &b)
    {
        Pos p;
        p.x = this->x + b.x;
        p.y = this->y + b.y;
        return p;
    }

    double distance(Pos b);
};

class Vec
{
private:
    /* data */
public:
    double x;
    double y;

    Vec(){};
    Vec(double x, double y);
    Vec(std::pair<double, double> v);
    Vec(double angle); // 生成单位向量
    Vec(Pos start, Pos end);
    ~Vec();

    Vec operator+(const Vec &b)
    {
        Vec p;
        p.x = this->x + b.x;
        p.y = this->y + b.y;
        return p;
    }
    Vec operator-(const Vec &b)
    {
        Vec p;
        p.x = this->x - b.x;
        p.y = this->y - b.y;
        return p;
    }
    friend Vec operator*(const Vec &A, double scale);
    friend Vec operator*(double scale, const Vec &A);
    friend Vec operator/(const Vec &A, double scale);
    friend Vec operator/(double scale, const Vec &A);
    friend std::ostream &operator<<(std::ostream &output, const Vec &v)
    {
        output << v.x << ", " << v.y;
        return output;
    }

    double length();
    Vec unit_vector();
    double angle();
    double angle_diff(Vec b);   // 计算向量角度差, this->angle - b.angle
    double cross_product(Vec b); // 计算平面两个矢量的叉乘, oa X ob, 返回 z 轴方向的矢量
};

extern std::ofstream fout;
#endif