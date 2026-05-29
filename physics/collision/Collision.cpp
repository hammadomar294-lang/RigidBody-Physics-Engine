#include "Collision.h"

Collision::CollisionResult Collision::IsIntersectCircle(const Vec2 &circleA, float radiusA, const Vec2 &circleB, float radiusB)
{
    CollisionResult result; // is intersect is false

    float radii = radiusA + radiusB;
    float distance = math::Distance(circleA , circleB);

    if (distance >= radii)
        return result;

    float depth = radii - distance;

    Vec2 collisionDirection = circleA - circleB;
    Vec2 normal = collisionDirection.Normalize();

    result.NormalCollisionDirection = normal;
    result.Depth = depth;
    result.IsIntersect = true;

    return result;
}