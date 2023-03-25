#ifndef __BASIC_H_
#define __BASIC_H_
#include <cmath>

class Pos
{
private:
    /* data */
public:
    double x;
    double y;

    Pos(){};
    Pos(double x, double y);
    ~Pos();

    friend Pos operator*(const Pos &A, double scale);
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
    Vec(double angle); // 生成单位向量
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

    double length();
    Vec unit_vector();
};

#endif