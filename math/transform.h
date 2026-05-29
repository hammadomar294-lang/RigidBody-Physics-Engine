#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <cmath>

#include "vector2.h"

class Transform
{
private:
    const float PositionX;
    const float PositionY;
    const float Sin;
    const float cos;

public:
    static Transform Zero;
    Transform(Vec2 v , float angle);
    Transform(float x , float y , float angle);

    Vec2 TransformPoint(const Vec2 & original , Transform transform);
};




#endif