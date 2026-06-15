#include "Manifold.h"

Manifold::Manifold(RigidBody & bodyA , RigidBody & bodyB , 
        float depth ,const Vec2 & normal ,const Vec2 & contact1 
        ,const Vec2 & contact2 , int contactCount)
         : BodyA(bodyA) , BodyB(bodyB)
    {
        Depth = depth;
        Normal = normal;
        ContactPoints[0] = contact1;
        ContactPoints[1] = contact2;
        ContactCount = contactCount;
    }