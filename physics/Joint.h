#ifndef JOINT_H
#define JOINT_H

#include "RigidBody.h"

enum JointType
{
    Spring = 0 , Rope = 1
};

class Joint
{
public:

    RigidBody * BodyA;
    RigidBody * BodyB;

    float RestLength;
    float Damping;
    float Hz;
    float Stiffness;
    float MinLength;
    float MaxLength;

    JointType jointType;

    // static Joint MakeRopeJoint(RigidBody *bodyA ,  RigidBody *bodyB , float restLength , float damping = 0 , float hz = 0);
    // static Joint MakeSpringJoint(RigidBody *bodyA ,  RigidBody *bodyB , float restLength , float damping = 0 , float hz = 0);

    Joint( RigidBody *bodyA ,  RigidBody *bodyB  , JointType type , float restLength , float stiffness = 0 , float damping = 0 , float hz = 0);
    
    void Solve();
    void SolveSpring();
};

#endif