#include "math.h"

float math::Distance(const Vec2 &v1, const Vec2 &v2) 
{
    return sqrt(pow((v1.X - v2.X) , 2)  +  pow((v1.Y - v2.Y) , 2));
}

float math::Dot(const Vec2 &v1, const Vec2 &v2) 
{
    return v1.X * v2.X + v1.Y * v2.Y;
}

float math::Cross(const Vec2 &v1, const Vec2 &v2) 
{
    return v1.X * v2.Y - v1.Y * v2.X;
}

