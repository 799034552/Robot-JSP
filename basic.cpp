#include "basic.h"
Pos::Pos(double x, double y)
{
    this->x = x;
    this->y = y;
}
Pos::Pos(std::pair<double, double> pos)
{
    this->x = pos.first;
    this->y = pos.second;
}
Pos::~Pos()
{
}
Pos operator*(const Pos &A, double scale)
<<<<<<< HEAD
=======
{
    return {A.x * scale, A.y * scale};
}
Pos operator*(double scale, const Pos &A)
>>>>>>> 0abbbc7dc911e81cee9d2b23f407536834cd8db5
{
    return {A.x * scale, A.y * scale};
}
Pos operator/(const Pos &A, double scale){
    return {A.x / scale, A.y / scale};
}
Pos operator/(double scale, const Pos &A)
{
    return {A.x / scale, A.y / scale};
}

Vec::Vec(double x, double y)
{
    this->x = x;
    this->y = y;
}
Vec::Vec(double angle)
{
    this->x = cos(angle);
    this->y = sin(angle);
}
/// @brief 生成 start 指向 end 的矢量
Vec::Vec(Pos start,Pos end)
{
    this->x = end.x - start.x;
    this->y = end.y - start.y;
}
Vec::~Vec()
{
}

Vec operator*(const Vec &A, double scale)
{
    return Vec{A.x * scale, A.y * scale};
}
Vec operator*(double scale, const Vec &A)
{
    return Vec{A.x * scale, A.y * scale};
}
Vec operator/(const Vec &A, double scale)
{
    return Vec{A.x / scale, A.y / scale};
}
Vec operator/(double scale, const Vec &A)
{
    return Vec{A.x / scale, A.y / scale};
}

double Vec::length()
{
    return sqrt(this->x * this->x + this->y * this->y);
}

Vec Vec::unit_vector()
{
    double len = this->length();
    if (len == 0)
        return {0, 0};
    else
        return {this->x / len, this->y / len};
}

double Vec::angle()
{
    return atan2(this->y,this->x);
}