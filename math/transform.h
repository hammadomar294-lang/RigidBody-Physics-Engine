#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <cmath>

#include "vector2.h"

class Transform2D
{
private:
    float PositionX;
    float PositionY;
    float Sin;
    float Cos;

public:
    static Transform2D Zero;
    Transform2D(Vec2 v , float angle);
    Transform2D(float x , float y , float angle);

    Vec2 transformPoint(const Vec2 & original);
};




#endif