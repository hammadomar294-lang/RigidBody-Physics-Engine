#include "Joint.h"

// Joint Joint::MakeRopeJoint(RigidBody *bodyA, RigidBody *bodyB, float restLength, float damping, float hz)
// {
//     BodyA = bodyA;
//     BodyB = bodyB;
//     RestLength = restLength;
//     Damping = damping;
//     Hz = hz;
//     jointType = JointType::Rope;

//     return Joint(bodyA , bodyB , restLength , damping , hz);
// }

Joint::Joint(RigidBody *bodyA, RigidBody *bodyB , JointType type , float restLength , float stiffness , float damping, float hz )

{
    BodyA = bodyA;
    BodyB = bodyB;
    RestLength = restLength;
    Stiffness = stiffness;
    Damping = damping;
    Hz = hz;
    jointType = type;
    if (jointType == JointType::Spring)
    {
        MinLength = restLength * 0.35;
        MaxLength = restLength * 1.35;
    }
}

void Joint::Solve() 
{
    float dt = GetFrameTime();

    Vec2 direction = BodyB->Position - BodyA->Position;

    float distance = math::Distance(BodyA->Position, BodyB->Position);

    if(math::NearlyEqual(distance , 0.0) || dt <= 0.0f)
        return;

    float error = distance - RestLength;
    direction = direction.Normalize();

    
    float totalInvMass = BodyA->InvMass + BodyB->InvMass;

    if(totalInvMass == 0.0f) return;

    Vec2 correction = direction * (error / totalInvMass);

    if(!BodyA->IsStatic)
    {
        Vec2 moveStep = correction * BodyA->InvMass;
        BodyA->MoveBy(moveStep);
        BodyA->LinearVelocity += moveStep / dt; 
    }
    
    if(!BodyB->IsStatic)
    {
        Vec2 moveStep = correction * -BodyB->InvMass;
        BodyB->MoveBy(moveStep);
        BodyB->LinearVelocity += moveStep / dt;
    }
}

void Joint::SolveSpring()
{
    Vec2 direction = BodyB->Position - BodyA->Position;

    direction = direction.Normalize();

    Vec2 relativeVelocity = BodyB->LinearVelocity - BodyA->LinearVelocity;

    float velocityAlongDirection = math::Dot(relativeVelocity , direction);

    float distance = math::Distance(BodyA->Position , BodyB->Position);
    
    float deltaDistance = RestLength - distance;

    if (deltaDistance == 0) return;

    float force = -(Stiffness * deltaDistance) + (velocityAlongDirection * Damping);

    

    Vec2 impulse = direction * force * GetFrameTime();

    if (!BodyA->IsStatic)
        BodyA->LinearVelocity += impulse * BodyA->InvMass;
    if (!BodyB->IsStatic)
        BodyB->LinearVelocity += impulse * -BodyB->InvMass;

    if (distance >= MaxLength)
    {   if (BodyA->IsStatic)
            BodyB->MoveBy(direction * -(distance - MaxLength + 1.0));

        else if (BodyB->IsStatic)
            BodyA->MoveBy(direction * (distance - MaxLength - 1.0));
        else 
        {
            BodyB->MoveBy(direction * -(distance - MaxLength) * 0.5);
            BodyA->MoveBy(direction * (distance - MaxLength) * 0.5);
        }
        BodyA->LinearVelocity = {0.0,0.0};
        BodyB->LinearVelocity = {0.0,0.0};
    }
    else if (distance <= MinLength)
    {
        if (BodyA->IsStatic)
            BodyB->MoveBy(direction * -(MinLength - distance + 1.0) );

        else if (BodyB->IsStatic)
            BodyA->MoveBy(direction * (MinLength - distance - 1.0) );
        else 
        {
            BodyB->MoveBy(direction * -(MinLength - distance) * 0.5);
            BodyA->MoveBy(direction * (MinLength - distance) * 0.5);
        }
        BodyA->LinearVelocity = {0.0,0.0};
        BodyB->LinearVelocity = {0.0,0.0};
    }
}
