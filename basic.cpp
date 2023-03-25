#include "basic.h"
Pos::Pos(double x, double y)
{
    this->x = x;
    this->y = y;
}

Pos::~Pos()
{
}
friend Pos operator*(const Pos &A, double scale)
{
    return {A.x * scale, A.y * scale};
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
Vec::~Vec()
{
}

Vec operator*(const Vec &A, double scale)
{
    return Vec{A.x * scale, A.y * scale};
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