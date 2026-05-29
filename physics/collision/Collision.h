#ifndef COLLISION_H
#define COLLISION_H

#include "../../math/vector2.h"
#include "../../math/math.h"
class Collision
{
public:
    struct CollisionResult
    {
        bool IsIntersect = false;
        float Depth = 0.0f;
        Vec2 NormalCollisionDirection = {0.0f,0.0f};
    };


    static CollisionResult IsIntersectCircle(const Vec2 & circleA , float radiusA , const Vec2 & circleB , float radiusB);
};

#endif