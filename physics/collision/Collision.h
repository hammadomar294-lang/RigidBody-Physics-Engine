#ifndef COLLISION_H
#define COLLISION_H


#include <limits>
#include <utility>
#include <vector>
#include <algorithm>
#include <iostream>

#include "../../math/math.h"
#include "../../math/vector2.h"
#include "../../math/math.h"
#include "../RigidBody.h"
#include "../Manifold.h"

using namespace std;
class Collision
{
public:
    struct CollisionResult
    {
        bool IsIntersect = false;
        float Depth = numeric_limits<float>::max();
        Vec2 NormalCollisionDirection = {0.0f,0.0f};
    };

    struct PointSegmentResult
    {
        Vec2 ClosestPoint = {0.0,0.0};
        float Distance = numeric_limits<float>::max();
    };

    static CollisionResult IsIntersectCircle(const Vec2 & circleA , float radiusA , const Vec2 & circleB , float radiusB);
    static CollisionResult IsPolygonSIntersect(const vector<Vec2> & verticesA , const vector<Vec2> & verticesB);
    static CollisionResult IsPolygonCircleIntersect(const vector<Vec2> & vertices , const Vec2 & circleCenter , float radius);

    static CollisionResult Collide(RigidBody* bodyA, RigidBody* bodyB);

    static void FindContactPoints(Manifold& manifold);


private:
    
    static pair<float , float> ProjectVertices(const vector<Vec2> & vertices , const Vec2 & axis);
    static Vec2 CalculateCenter(const vector<Vec2> & vertices);

    static pair<float , float> ProjectCircle(const Vec2 & circleCenter , float radius , const Vec2 & axis);
    // returns the index of which vertex is closest
    static int FindClosestVertexToCircle(const Vec2 & circleCenter , const vector<Vec2> & vertices);

    static void FindCircleCircleContact(Manifold& manifold);

    static PointSegmentResult FindPointSegmentResult(const Vec2 & c , const Vec2 & pointA, const Vec2 & pointB);
    static void FindCirclePolygonContact(Manifold& manifold);

    static void FindPolygonPolygonContact(Manifold& manifold);


    
};

#endif