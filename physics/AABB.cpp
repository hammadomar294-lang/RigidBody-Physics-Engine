#include "AABB.h"

AABB::AABB(const Vec2 &min, const Vec2 &max)
{
    this->Min = min;
    this->Max = max;
}

AABB::AABB(float minX, float minY, float maxX, float maxY)
{
    this->Min.X = minX;
    this->Min.Y = minY;
    this->Max.X = maxX;
    this->Max.Y = maxY;
}

AABB::AABB()
{
    this->Min = {0.0,0.0};
    this->Max = {0.0,0.0};
}
