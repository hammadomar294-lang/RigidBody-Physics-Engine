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

using namespace std;
class Collision
{
public:
    struct CircleCollisionResult
    {
        bool IsIntersect = false;
        float Depth = 0.0f;
        Vec2 NormalCollisionDirection = {0.0f,0.0f};
    };

    struct PolygonCollisionResult
    {
        bool IsIntersect = true;
        float Depth = numeric_limits<float>::max();
        Vec2 NormalCollisionDirection = {0.0f,0.0f};
    };
    struct PolygonCircleCollisionResult
    {
        bool IsIntersect = true;
        float Depth = numeric_limits<float>::max();
        Vec2 NormalCollisionDirection = {0.0f,0.0f};
    };


    static CircleCollisionResult IsIntersectCircle(const Vec2 & circleA , float radiusA , const Vec2 & circleB , float radiusB);
    static PolygonCollisionResult IsPolygonSIntersect(const vector<Vec2> & verticesA , const vector<Vec2> & verticesB);
    static PolygonCircleCollisionResult IsPolygonCircleIntersect(const vector<Vec2> & vertices , const Vec2 & circleCenter , float radius);

private:
    // polygon helper
    static pair<float , float> ProjectVertices(const vector<Vec2> & vertices , const Vec2 & axis);
    static Vec2 CalculateCenter(const vector<Vec2> & vertices);

    static pair<float , float> ProjectCircle(const Vec2 & circleCenter , float radius , const Vec2 & axis);
    // returns the index of which vertex is closest
    static int FindClosestVertexToCircle(const Vec2 & circleCenter , const vector<Vec2> & vertices);

    
};

#endif