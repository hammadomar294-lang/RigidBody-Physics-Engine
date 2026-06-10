#include "Collision.h"


Collision::CollisionResult Collision::IsIntersectCircle(const Vec2 &circleA, float radiusA, const Vec2 &circleB, float radiusB)
{
    CollisionResult result; // default to false

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


Collision::CollisionResult Collision::IsPolygonSIntersect(const vector<Vec2> &verticesA, const vector<Vec2> &verticesB)
{
    CollisionResult result; 

    pair<float,float> FirstPair;
    pair<float,float> SecondPair;

    for (int i = 0 ; i < verticesA.size() ; i++)
    {
        // start from upper left corner and make the side vector by point - next and then go back to the original
        Vec2 v1 = verticesA[i];
        Vec2 v2 = verticesA[ (i+1) % verticesA.size() ];

        Vec2 side = v2-v1;
        // asis equal perpendicular to the side of square
        Vec2 axis(-side.Y , side.X);
        axis = axis.Normalize();
    
        FirstPair = ProjectVertices(verticesA , axis);
        SecondPair = ProjectVertices(verticesB , axis);

        if (FirstPair.first > SecondPair.second || SecondPair.first >FirstPair.second)
        {
            result.IsIntersect = false;
            return result;
        }
        // calculate depth and normal
        result.IsIntersect = true;
        float axisDepth = min(FirstPair.second - SecondPair.first , SecondPair.second - FirstPair.first);
        if (axisDepth < result.Depth)
        {
            result.Depth = axisDepth;
            result.NormalCollisionDirection = axis;
        }   
        
    }
    for (int i = 0 ; i < verticesB.size() ; i++)
    {
        // start from upper left corner and make the side vector by point - next and then go back to the original
        Vec2 v1 = verticesB[i];
        Vec2 v2 = verticesB[ (i+1) % verticesB.size() ];

        Vec2 side = v2-v1;
        // axis equal perpendicular to the side of square
        Vec2 axis(-side.Y , side.X);
        axis = axis.Normalize();
        
        FirstPair = ProjectVertices(verticesA , axis);
        SecondPair = ProjectVertices(verticesB , axis);

        if (FirstPair.first > SecondPair.second || SecondPair.first >FirstPair.second)
        {
            result.IsIntersect = false;
            return result;
        }
        result.IsIntersect = true;
        // calculate depth and normal
        float axisDepth = min(FirstPair.second - SecondPair.first , SecondPair.second - FirstPair.first);
        if (axisDepth < result.Depth)
        {
            result.Depth = axisDepth;
            result.NormalCollisionDirection = axis;
        }   
    }
    // another test to figure out if the normal we found is pointing in the right direction
    Vec2 centerA = CalculateCenter(verticesA);
    Vec2 centerB = CalculateCenter(verticesB);

    // direction from A to B
    Vec2 direction = centerB - centerA;
    if (math::Dot(direction , result.NormalCollisionDirection) < 0)
    {
        result.NormalCollisionDirection = result.NormalCollisionDirection * -1;
    }
    
    return result;
}


Collision::CollisionResult Collision::IsPolygonCircleIntersect(const vector<Vec2> &vertices, const Vec2 &circleCenter, float radius)
{
    CollisionResult result;
    pair<float,float> FirstPair;
    pair<float,float> SecondPair;

    for (int i = 0 ; i < vertices.size() ; i++)
    {
        // start from upper left corner and make the side vector by point - next and then go back to the original
        Vec2 v1 = vertices[i];
        Vec2 v2 = vertices[ (i+1) % vertices.size() ];

        Vec2 side = v2-v1;
        // asis equal perpendicular to the side of square
        Vec2 axis(-side.Y , side.X);
        axis = axis.Normalize();
    
        FirstPair = ProjectVertices(vertices , axis);
        SecondPair = ProjectCircle(circleCenter , radius , axis);

        if (FirstPair.first > SecondPair.second || SecondPair.first >FirstPair.second)
        {
            result.IsIntersect = false;
            return result;
        }
        result.IsIntersect = true;
        // calculate depth and normal
        float axisDepth = min(FirstPair.second - SecondPair.first , SecondPair.second - FirstPair.first);
        
        if (axisDepth < result.Depth)
        {
            result.Depth = axisDepth;
            result.NormalCollisionDirection = axis;
        }   
        
    }
    int closestIndex = FindClosestVertexToCircle(circleCenter , vertices);
    Vec2 closestPoint = vertices[closestIndex];

    //from center to closest vertex
    Vec2 axis = closestPoint - circleCenter;
    axis = axis.Normalize();

    FirstPair = ProjectVertices(vertices , axis);
    SecondPair = ProjectCircle(circleCenter , radius , axis);

    if (FirstPair.first > SecondPair.second || SecondPair.first > FirstPair.second)
    {
        result.IsIntersect = false;
        return result;
    }
    result.IsIntersect = true;
    // calculate depth and normal
    float axisDepth = min(FirstPair.second - SecondPair.first , SecondPair.second - FirstPair.first);
    if (axisDepth < result.Depth)
    {
        result.Depth = axisDepth;
        result.NormalCollisionDirection = axis;
    }   

    Vec2 polyCenter = CalculateCenter(vertices);

    Vec2 direction = polyCenter  -  circleCenter;

    if (math::Dot(direction , result.NormalCollisionDirection) < 0)
    {
        result.NormalCollisionDirection = result.NormalCollisionDirection * -1;
    }

    return result;
}

#pragma region helpers
//    min   max
pair<float,float> Collision::ProjectVertices(const vector<Vec2> &vertices, const Vec2 &axis)
{
    float Min = numeric_limits<float>::max();
    float Max = numeric_limits<float>::lowest();

    for (const Vec2 & vertex : vertices)
    {
        float projection = math::Dot(vertex , axis);

        if (projection <= Min) Min = projection;
        if (projection > Max) Max = projection;
    }
    return {Min, Max};
}

Vec2 Collision::CalculateCenter(const vector<Vec2> &vertices)
{
    float sumX = 0.0f;
    float sumY = 0.0f;

    for (const auto & v : vertices)
    {
        sumX += v.X;
        sumY += v.Y;
    }
    return {sumX / (float)vertices.size() , sumY / (float)vertices.size()};
}
//    min     max
pair<float , float> Collision::ProjectCircle(const Vec2 &circleCenter, float radius, const Vec2 &axis)
{
    float Min = numeric_limits<float>::max();
    float Max = numeric_limits<float>::lowest();

    // we need 2 point on the circle in the direction of the axis so center + or - the axis * radius
    Vec2 direction = axis * radius;
    Vec2 p1 = circleCenter + direction;
    Vec2 p2 = circleCenter - direction;
    
    float projection1 = math::Dot(p1,axis);
    float projection2 = math::Dot(p2,axis);

    if (projection1 > projection2)
    {
        Max = projection1;
        Min = projection2;
    }
    else if (projection1 < projection2)
    {
        Max = projection2;
        Min = projection1;
    }

    return {Min,Max};
}

int Collision::FindClosestVertexToCircle(const Vec2 &circleCenter, const vector<Vec2> &vertices)
{
    float Min = numeric_limits<float>::max();
    float distance = 0.0f;
    int index = -1;
    for (int i = 0 ; i<vertices.size() ; i++)
    {
        distance = math::Distance(vertices[i] , circleCenter);
        if (distance < Min)
        {
            Min = distance;
            index = i;
        }
    }
    return index;
}

#pragma endregion