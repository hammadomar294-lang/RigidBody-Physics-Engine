#ifndef MANIFOLD_H
#define MANIFOLD_H

#include "../math/vector2.h"
#include "RigidBody.h"


class Manifold
{
public:

    RigidBody *BodyA;
    RigidBody *BodyB;

    float Depth;
    Vec2 Normal;

    int ContactCount;
    Vec2 ContactPoints[2];

    Manifold(RigidBody * bodyA , RigidBody * bodyB , 
        float depth ,const Vec2 & normal ,const Vec2 & contact1 
        ,const Vec2 & contact2 , int contactCount);
    
   
};

#endif