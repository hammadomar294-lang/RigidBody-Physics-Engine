#ifndef AABB_H
#define AABB_H

#include "../math/vector2.h"

class AABB
{
public:

    Vec2 Min;
    Vec2 Max;

    AABB(const Vec2 &min , const Vec2 &max);
    AABB(float minX , float minY , float maxX , float maxY);
    AABB();
    
};

#endif