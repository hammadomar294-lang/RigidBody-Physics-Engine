#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <unordered_map>
#include <unordered_set>

#include "raylib.h"

#include "physics/RigidBody.h"
#include "math/math.h"
#include "physics/collision/Collision.h"
#include "physics/constants.h"
#include "physics/Manifold.h"
#include "physics/AABB.h"
#include "physics/Joint.h"

using namespace std;

void MakePendulum(int numOfBalls , float length , Camera2D & camera);
void BounceScreen(vector<RigidBody>& bodies , Camera2D & camera);
struct debugIfo
{
    double TotalTimeToUpdatePhysics = 0.0;
    int TotalStep = 0;
    double averagePhysicsTime = 0.0;
    double timer = 0.0;

    int totalPairs = 0;
    int aabbRejected = 0;
    int SATCalls = 0;

    int totalPairsAccum =0;
    int aabbRejectedAccum = 0;
    int satCallsAccum = 0;

    int averagePairs = 0;
    int averageRejected = 0;
    int averageSATCalls = 0;
};

struct Cell
{
    int X;
    int Y;

    Cell(int x , int y)
    {
        X = x;
        Y = y;
    }
};

int CellSize = 100;

Cell MakeCell (const Vec2 & position)
{
    return {(int)floor(position.X / CellSize) , (int)floor(position.Y / CellSize)};
}

string CellToString(const Cell & cell)
{
    return "{" + to_string(cell.X) + "," + to_string(cell.Y) + "}";
}


vector <Manifold> manifolds;
vector <RigidBody*> Bodies;
vector <Joint> Joints;


// key is the string that holds the cell and the value is the index in Bodies
unordered_map< string , vector<int> > Grid;

debugIfo inf;

int iterations = 20;


void BuildGrid(vector<RigidBody*>& bodies)
{
    Grid.clear();
    for (int i = 0 ; i < bodies.size() ; i++)
    {
        if (bodies[i]->IsStatic)
            continue;
        AABB aabb = bodies[i]->GetAABB();

        int minCellX = floor(aabb.Min.X / CellSize);
        int maxCellX = floor(aabb.Max.X / CellSize);

        int minCellY = floor(aabb.Min.Y / CellSize);
        int maxCellY = floor(aabb.Max.Y / CellSize);

       for(int x = minCellX; x <= maxCellX; x++)
        {
            for(int y = minCellY; y <= maxCellY; y++)
            {
                Cell cell(x, y);

                Grid[CellToString(cell)].push_back(i);
            }
        }
    }
}

void RotateBodys(vector<RigidBody*> & Bodies , float angle)
{
    for (int i = 0 ; i < Bodies.size() - 1 ; i++)
    {
        if (Bodies[i]->shapeType == ShapeType::Box)
        {
            Bodies[i]->RotateBy(angle);
        }
    }

}

bool IsAABBIntersect(RigidBody* bodyA , RigidBody* bodyB)
{
    AABB a = bodyA->GetAABB();
    AABB b = bodyB->GetAABB();

    return !(a.Max.X < b.Min.X ||
             a.Min.X > b.Max.X ||
             a.Max.Y < b.Min.Y ||
             a.Min.Y > b.Max.Y);
}

void ResolveCollisionBasic(const Manifold & manifold)
{
    RigidBody * bodyA = manifold.BodyA; 
    RigidBody * bodyB = manifold.BodyB; 
    Vec2 normal = manifold.Normal;

    Vec2 relativeVelocity = bodyB->LinearVelocity - bodyA->LinearVelocity;

    if (math::Dot(normal , relativeVelocity) > 0.0)
        return;

    float minRestitution = min(bodyA->Restitution , bodyB->Restitution);
    // impulse = -(1 + minRestitution) * Dot(relativeVelocity , nomal) / 1/massA + 1/massB
    float impulse = -(1 + minRestitution) * math::Dot(relativeVelocity , normal) / (bodyA->InvMass + bodyB->InvMass);

    bodyA->LinearVelocity += normal * (-impulse * bodyA->InvMass);
    bodyB->LinearVelocity += normal * (impulse * bodyB->InvMass);
}

void ResolveCollisionRotation(const Manifold & manifold)
{
    RigidBody * bodyA = manifold.BodyA; 
    RigidBody * bodyB = manifold.BodyB;
    
    Vec2 normal = manifold.Normal;
    
    Vec2 contactArray[2] = {manifold.ContactPoints[0] , manifold.ContactPoints[1]};
    int contactCount = manifold.ContactCount;

    Vec2 bodyA_levers[2];
    Vec2 bodyB_levers[2];

    Vec2 impulseArray[2];
    Vec2 impulseFrictionArray[2];

    Vec2 relativeVelocity = bodyB->LinearVelocity - bodyA->LinearVelocity;

    float minRestitution = min(bodyA->Restitution , bodyB->Restitution);

    for (int i = 0 ; i < contactCount ; i++)
    {
        Vec2 leverA = contactArray[i] - bodyA->Position; 
        bodyA_levers[i] = leverA;

        Vec2 leverB = contactArray[i] - bodyB->Position;
        bodyB_levers[i] = leverB;

        
        Vec2 leverA_Perpendicular = {-leverA.Y , leverA.X};
        Vec2 leverB_Perpendicular = {-leverB.Y , leverB.X};
        
        // angular velocity at this instance of time is linear 
        Vec2 angularLinearVelocity_A = leverA_Perpendicular * bodyA->RotationVelocity;
        Vec2 angularLinearVelocity_B = leverB_Perpendicular * bodyB->RotationVelocity;
        
        // we increment linear velocity by the angular velocity at that instance
        Vec2 velocityA = bodyA->LinearVelocity + angularLinearVelocity_A;
        Vec2 velocityB = bodyB->LinearVelocity + angularLinearVelocity_B;

        Vec2 relativeVelocity = velocityB - velocityA;

        float relativeNormalDot = math::Dot(relativeVelocity , normal);

        if (relativeNormalDot > 0.0)
            continue;

        float leverA_DotNormal = math::Dot(leverA_Perpendicular , normal);
        float leverB_DotNormal = math::Dot(leverB_Perpendicular , normal);

        float numerator = -(1 + minRestitution) * relativeNormalDot;

        float denominator = (bodyA->InvMass + bodyB->InvMass) + 
        (leverA_DotNormal * leverA_DotNormal * bodyA->InvInertia) +
        (leverB_DotNormal * leverB_DotNormal * bodyB->InvInertia);

        Vec2 impulse = normal * (numerator / denominator / contactCount);
        
        impulseArray[i] = impulse;
    }

    for (int i = 0 ; i < contactCount ; i++)
    {
        bodyA->LinearVelocity += (impulseArray[i] * -bodyA->InvMass);
        bodyA->RotationVelocity += math::Cross(bodyA_levers[i] , impulseArray[i]) * -bodyA->InvInertia;

        bodyB->LinearVelocity += (impulseArray[i] * bodyB->InvMass);
        bodyB->RotationVelocity += math::Cross(bodyB_levers[i] , impulseArray[i]) * bodyB->InvInertia;
    }


}

void ResolveCollisionRotationWithFriction(const Manifold & manifold)
{
    RigidBody * bodyA = manifold.BodyA; 
    RigidBody * bodyB = manifold.BodyB;
    
    Vec2 normal = manifold.Normal;
    
    Vec2 contactArray[2] = {manifold.ContactPoints[0] , manifold.ContactPoints[1]};
    int contactCount = manifold.ContactCount;

    Vec2 bodyA_levers[2] = {};
    Vec2 bodyB_levers[2] = {};

    Vec2 impulseArray[2] = {};
    Vec2 impulseFrictionArray[2] = {};
    float scalerImpulseArray[2] = {};

    Vec2 relativeVelocity = bodyB->LinearVelocity - bodyA->LinearVelocity;

    float minRestitution = min(bodyA->Restitution , bodyB->Restitution);

    float avgStaticFriction = (bodyA->StaticFriction + bodyB->StaticFriction) * 0.5f;
    float avgDynamicFriction = (bodyA->DynamicFriction + bodyB->DynamicFriction) * 0.5f;

    for (int i = 0 ; i < contactCount ; i++)
    {
        Vec2 leverA = contactArray[i] - bodyA->Position; 
        bodyA_levers[i] = leverA;

        Vec2 leverB = contactArray[i] - bodyB->Position;
        bodyB_levers[i] = leverB;

        
        Vec2 leverA_Perpendicular = {-leverA.Y , leverA.X};
        Vec2 leverB_Perpendicular = {-leverB.Y , leverB.X};
        
        // angular velocity at this instance of time is linear 
        Vec2 angularLinearVelocity_A = leverA_Perpendicular * bodyA->RotationVelocity;
        Vec2 angularLinearVelocity_B = leverB_Perpendicular * bodyB->RotationVelocity;
        
        // we increment linear velocity by the angular velocity at that instance
        Vec2 velocityA = bodyA->LinearVelocity + angularLinearVelocity_A;
        Vec2 velocityB = bodyB->LinearVelocity + angularLinearVelocity_B;

        Vec2 relativeVelocity = velocityB - velocityA;

        float relativeNormalDot = math::Dot(relativeVelocity , normal);
        
        Vec2 result = normal * relativeNormalDot;

        Vec2 tangent ={-result.Y, result.X};

        if (relativeNormalDot > 0.0)
            continue;

        float leverA_DotNormal = math::Dot(leverA_Perpendicular , normal);
        float leverB_DotNormal = math::Dot(leverB_Perpendicular , normal);

        float numerator = -(1 + minRestitution) * relativeNormalDot;

        float denominator = (bodyA->InvMass + bodyB->InvMass) + 
        (leverA_DotNormal * leverA_DotNormal * bodyA->InvInertia) +
        (leverB_DotNormal * leverB_DotNormal * bodyB->InvInertia);

        scalerImpulseArray[i] = (numerator / denominator / contactCount);
        Vec2 impulse = normal * (numerator / denominator / contactCount);
        
        impulseArray[i] = impulse;
    }

    for (int i = 0 ; i < contactCount ; i++)
    {
        bodyA->LinearVelocity += (impulseArray[i] * -bodyA->InvMass);
        bodyA->RotationVelocity += math::Cross(bodyA_levers[i] , impulseArray[i]) * -bodyA->InvInertia;

        bodyB->LinearVelocity += (impulseArray[i] * bodyB->InvMass);
        bodyB->RotationVelocity += math::Cross(bodyB_levers[i] , impulseArray[i]) * bodyB->InvInertia;
    }

    // tangent friction resolution 
    for (int i = 0 ; i < contactCount ; i++)
    {
        Vec2 leverA = contactArray[i] - bodyA->Position; 
        bodyA_levers[i] = leverA;

        Vec2 leverB = contactArray[i] - bodyB->Position;
        bodyB_levers[i] = leverB;

        
        Vec2 leverA_Perpendicular = {-leverA.Y , leverA.X};
        Vec2 leverB_Perpendicular = {-leverB.Y , leverB.X};
        
        // angular velocity at this instance of time is linear 
        Vec2 angularLinearVelocity_A = leverA_Perpendicular * bodyA->RotationVelocity;
        Vec2 angularLinearVelocity_B = leverB_Perpendicular * bodyB->RotationVelocity;
        
        // we increment linear velocity by the angular velocity at that instance
        Vec2 velocityA = bodyA->LinearVelocity + angularLinearVelocity_A;
        Vec2 velocityB = bodyB->LinearVelocity + angularLinearVelocity_B;

        Vec2 relativeVelocity = velocityB - velocityA;

        float relativeNormalDot = math::Dot(relativeVelocity , normal);
        
        // tangent = relativeVelocity - the vertical component = the horizontal component
        Vec2 tangent = relativeVelocity - normal * relativeNormalDot;

        if (math::NearlyEqual(tangent , {0.0 , 0.0}))
            continue;

        tangent = tangent.Normalize();

        float leverA_Perpendicular_DotTangent = math::Dot(leverA_Perpendicular , tangent);
        float leverB_Perpendicular_DotTangent = math::Dot(leverB_Perpendicular , tangent);

        float numerator = - math::Dot(relativeVelocity , tangent);

        float denominator = (bodyA->InvMass + bodyB->InvMass) + 
        (leverA_Perpendicular_DotTangent * leverA_Perpendicular_DotTangent * bodyA->InvInertia) +
        (leverB_Perpendicular_DotTangent * leverB_Perpendicular_DotTangent * bodyB->InvInertia);

        float Impulse = (numerator / denominator / contactCount);

        Vec2 frictionImpulse;

        if (abs(Impulse) <= scalerImpulseArray[i] * avgStaticFriction)
        {
            frictionImpulse = tangent * Impulse;
        }
        else
        {
            frictionImpulse = tangent * scalerImpulseArray[i] * -avgDynamicFriction;
        }
        
        impulseFrictionArray[i] = frictionImpulse;
    }

    for (int i = 0 ; i < contactCount ; i++)
    {
        bodyA->LinearVelocity += (impulseFrictionArray[i] * -bodyA->InvMass);
        bodyA->RotationVelocity += math::Cross(bodyA_levers[i] , impulseFrictionArray[i]) * -bodyA->InvInertia;

        bodyB->LinearVelocity += (impulseFrictionArray[i] * bodyB->InvMass);
        bodyB->RotationVelocity += math::Cross(bodyB_levers[i] , impulseFrictionArray[i]) * bodyB->InvInertia;
    }
}

void UpdateBodyPhysics(vector<RigidBody*> & Bodies , int iterations)
{
    for(auto & body : Bodies)
    {
        body->UpdatePhysics(iterations);
    }
}

void SeparateBodies(RigidBody * bodyA, RigidBody * bodyB , Vec2 MTV)
{
    if (bodyA->IsStatic)
    {
        bodyB->MoveBy(MTV);
    }
    else if (bodyB->IsStatic)
    {
        bodyA->MoveBy(MTV * -1);
    }
    else
    {
        bodyA->MoveBy(MTV * -0.5);

        bodyB->MoveBy(MTV * 0.5);
    }
}

vector<Manifold> DetectFrameCollisionsOld(vector<RigidBody*> & Bodies)
{

    vector<Manifold> tempManifolds;
    unordered_set<long long> checkedPairs;
    for (int i = 0; i < Bodies.size() ; i++)
    {
        Cell cell = MakeCell(Bodies[i]->Position);
        
        RigidBody* bodyA = Bodies[i];

        for (int x = -1 ; x <= 1 ; x++)
        {
            for (int y = -1 ; y <= 1 ; y++)
            {
                int cellX = cell.X + x;
                int cellY = cell.Y + y;

                Cell neighbour(cellX , cellY);

                auto it = Grid.find(CellToString(neighbour));
                if (it == Grid.end()) 
                    continue;

                vector <int> & NeighbourBodiesIndices = it->second;

                for (int j = 0 ; j < NeighbourBodiesIndices.size() ; j++)
                {
                    int targetIndex = NeighbourBodiesIndices[j];
                    if (i == targetIndex)
                        continue;

                    int minId = std::min(i, targetIndex);
                    int maxId = std::max(i, targetIndex);
                    long long pairKey = ((long long)minId << 32) | (maxId & 0xFFFFFFFFLL);

                    if (checkedPairs.count(pairKey))
                        continue;

                    checkedPairs.insert(pairKey);

                    RigidBody* bodyB = Bodies[NeighbourBodiesIndices[j]];

                    inf.totalPairs++;

                    if (bodyA->IsStatic && bodyB->IsStatic)
                    {
                        continue;
                    }

                    if (!IsAABBIntersect(bodyA, bodyB))
                    {
                        inf.aabbRejected++;
                        continue;
                    }

                    inf.SATCalls++;

                    Collision::CollisionResult result = Collision::Collide(bodyA, bodyB);

                    if (!result.IsIntersect)
                        continue;

                    Manifold manifold = Manifold(bodyA , bodyB , result.Depth 
                        , result.NormalCollisionDirection , {0.0,0.0} , {0.0,0.0} , 0);
                    
                    tempManifolds.push_back(manifold);
                }
            }
        }
    }

    return tempManifolds;

    // for (int i = 0; i < Bodies.size() - 1; i++)
    // {
    //     RigidBody& bodyA = Bodies[i];

    //     for (int j = i + 1; j < Bodies.size(); j++)
    //     {
    //         inf.totalPairs++;
    //         RigidBody& bodyB = Bodies[j];

    //         if (bodyA.IsStatic && bodyB.IsStatic)
    //         {
    //             continue;
    //         }

    //         if (!IsAABBIntersect(bodyA, bodyB))
    //         {
    //             inf.aabbRejected++;
    //             continue;
    //         }

    //         inf.SATCalls++;

    //         Collision::CollisionResult result = Collision::Collide(bodyA, bodyB);

    //         if (!result.IsIntersect)
    //             continue;

    //         Manifold manifold = Manifold(bodyA , bodyB , result.Depth 
    //             , result.NormalCollisionDirection , {0.0,0.0} , {0.0,0.0} , 0);
            
    //         tempManifolds.push_back(manifold);
    //     }
    // }
    // return tempManifolds;

}


vector<Manifold> DetectFrameCollisions(vector<RigidBody*> & Bodies)
{
    vector<Manifold> tempManifolds;
    
    // CRITICAL: Tracks pairs we already checked this frame to prevent double-resolution bugs
    unordered_set<long long> checkedPairs; 

    for (int i = 0; i < Bodies.size(); i++)
    {
        RigidBody* bodyA = Bodies[i];
        AABB aabbA = bodyA->GetAABB();

        // Get the span of cells for bodyA
        int minCellX = floor(aabbA.Min.X / CellSize);
        int maxCellX = floor(aabbA.Max.X / CellSize);
        int minCellY = floor(aabbA.Min.Y / CellSize);
        int maxCellY = floor(aabbA.Max.Y / CellSize);

        // Loop ONLY through the cells bodyA actually touches
        for (int x = minCellX; x <= maxCellX; x++)
        {
            for (int y = minCellY; y <= maxCellY; y++)
            {
                Cell currentCell(x, y);
                
                // Use .find() to avoid creating empty vectors in your map accidentally
                auto it = Grid.find(CellToString(currentCell));
                if (it == Grid.end()) 
                    continue;

                vector<int>& CellBodiesIndices = it->second;

                for (int j = 0; j < CellBodiesIndices.size(); j++)
                {
                    int targetIndex = CellBodiesIndices[j];

                    // 2. Ensure unique pairs (i < targetIndex) to avoid evaluating A vs B AND B vs A
                    if (i >= targetIndex)
                        continue;

                    // 3. Skip if this pair was already handled in a previous overlapping cell
                    long long pairKey = ((long long)i << 32) | (targetIndex & 0xFFFFFFFFLL);
                    if (checkedPairs.count(pairKey))
                        continue;

                    // Mark pair as evaluated
                    checkedPairs.insert(pairKey);

                    RigidBody* bodyB = Bodies[targetIndex];

                    inf.totalPairs++;

                    // if (bodyA->IsStatic && bodyB->IsStatic)
                    //     continue;

                    if (!IsAABBIntersect(bodyA, bodyB))
                    {
                        inf.aabbRejected++;
                        continue;
                    }

                    inf.SATCalls++;

                    Collision::CollisionResult result = Collision::Collide(bodyA, bodyB);

                    if (!result.IsIntersect)
                        continue;

                    Manifold manifold = Manifold(bodyA, bodyB, result.Depth,
                        result.NormalCollisionDirection, {0.0,0.0}, {0.0,0.0}, 0);
                    
                    tempManifolds.push_back(manifold);
                }
            }
        }
    }

    return tempManifolds;
}



void BroadPhase(vector<RigidBody*> & Bodies)
{
    BuildGrid(Bodies);
    manifolds.clear();
    // manifolds = DetectFrameCollisionsOld(Bodies); 
    manifolds = DetectFrameCollisions(Bodies); 
    
}

void NarrowPhase(vector <Manifold> & manifolds , int iterations , int currentIteration) 
{
    for(auto & manifold : manifolds)
    {
       
        Collision::FindContactPoints(manifold);

        Vec2 MTV = manifold.Normal * manifold.Depth;
        SeparateBodies(manifold.BodyA , manifold.BodyB , MTV);
        
        // ResolveCollisionBasic(manifold);
        // ResolveCollisionRotation(manifold);
        ResolveCollisionRotationWithFriction(manifold);

        if (manifold.BodyA->IsSleeping && !manifold.BodyB->IsSleeping && !manifold.BodyB->IsStatic) {
            manifold.BodyA->IsSleeping = false;
            manifold.BodyA->SleepTimer = 0.0f;
        }
        if (manifold.BodyB->IsSleeping && !manifold.BodyA->IsSleeping && !manifold.BodyA->IsStatic) {
            manifold.BodyB->IsSleeping = false;
            manifold.BodyB->SleepTimer = 0.0f;
        }
    }
}

void SetForcesToZero(vector<RigidBody*> & Bodies)
{
    for (auto & body : Bodies)
    {
        body->Force = {0.0,0.0};
    }
}


void SolveJoints(vector <Joint> & joints)
{
    for (int i = 0 ; i < 20 ; i++)
    {
        for (auto & joint : joints)
        {
            if (joint.jointType == JointType::Rope)
            {
                joint.Solve();
            }
            else if (joint.jointType == JointType::Spring)
            {
                joint.SolveSpring();
                
            }
        }
    }
    
}

void UpdatePhysics(vector<RigidBody*> & Bodies , int iterations , vector <Joint> & joints)
{
    iterations = clamp(iterations , constants::MinSteps , constants::MaxSteps);

    for (int it = 0 ; it < iterations ; it++)
    {
        
        UpdateBodyPhysics(Bodies , iterations);
        BroadPhase(Bodies);
        NarrowPhase(manifolds , iterations , it);
        SolveJoints(joints);
    }
    SetForcesToZero(Bodies); 
}


void MakeBodies(int num)
{
    for (int i = 0; i < num; i++)
    {
        bool makeCircle = rand() % 2;

        float x = 200.0f + rand() % 1200;
        float y = 50.0f + rand() % 500;

        float density =
            0.5f + ((float)rand() / RAND_MAX) * 4.5f;

        float restitution =
            0.1f + ((float)rand() / RAND_MAX) * 0.8f;

        if (makeCircle)
        {
            float radius =
                10.0f + rand() % 40;

            Bodies.push_back(
                new RigidBody(
                    RigidBody::CreateCircle(
                        {x, y},
                        density,
                        restitution,
                        false,
                        radius)));
        }
        else
        {
            float width =
                20.0f + rand() % 80;

            float height =
                20.0f + rand() % 80;

            Bodies.push_back(
                new RigidBody(
                    RigidBody::CreateBox(
                        {x, y},
                        density,
                        restitution,
                        false,
                        width,
                        height)));
        }
    }
}

void MakeRope(int numOfBalls , const Vec2 & startPos)
{
    RigidBody * previous = nullptr;
    float radius = 5.0;
    float restLength = 2.0 * radius + 1.0;
    Vec2 temp = {0.0 , restLength};
    for (int i = 0 ; i < numOfBalls ; i++)
    {
        RigidBody * node = new RigidBody(RigidBody::CreateCircle(startPos + temp * i , 1.0 , 0.5 , false , radius));

        node->BodyColor = RED;
        Bodies.push_back(node);
        if (i == 0)
            node->IsStatic = true;

        
        if (previous != nullptr)
        {
            Joint joint(previous , node , JointType::Rope ,restLength);
            Joints.push_back(joint);
        }

        
        previous = node;
    }
}

void MakeSpring(Vec2 & startPos)
{
    RigidBody * previous = nullptr;

    float restLength = 200.0 , width = 80.0 , height = 80.0;

    Vec2 temp = {restLength + width , 0.0};


    // im making the surface of the ground box is at y = 680 in initializeWorld function 
    if (startPos.Y >= 640.0)
    {
        startPos.Y = 679 - height / 2.0;
    }

    for (int i = 0 ; i < 2 ; i++)
    {
        RigidBody * node = new RigidBody(RigidBody::CreateBox(startPos + temp * i, 1.0 , 0.5 , false , width , height));
        if (i == 0)
            node->IsStatic = true;
        Bodies.push_back(node);

        if (previous != nullptr)
        {
            Joint joint(previous , node , JointType::Spring , restLength , 1000.0 , 7.0);
            Joints.push_back(joint);
        }
        previous = node;
    }
}


int FindClosestBody(const Vec2 & clickPos)
{
    int closestIndex = -1;
    float minDis = numeric_limits<float>::max();

    Cell cell = MakeCell(clickPos);

    auto it = Grid.find(CellToString(cell));
    
    if (it == Grid.end()) 
        return -1;

    auto & indices = it->second;

    for (auto index : indices)
    {
        float tempDis = math::Distance(clickPos , Bodies[index]->Position);
        if (tempDis < minDis)
        {
            minDis = tempDis;
            closestIndex = index;
        }
    }
    return closestIndex;
}

void UpdateDragControls(Camera2D& camera , vector<RigidBody*> & Bodies) 
{
    float forceMagnitude = 5000000.0f;
    static int bodyIndex = -1;

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
    {
        Vector2 mouseScreen = GetMousePosition();
        Vector2 mouseWorld = GetScreenToWorld2D(mouseScreen, camera);

        Vec2 clickPos(mouseWorld.x, mouseWorld.y);

        int index = FindClosestBody(clickPos);
        if (index != -1)
        {
            bodyIndex = index;
        }

    }

    Vector2 mouseScreen = GetMousePosition();
    Vector2 mouseWorldRay = GetScreenToWorld2D(mouseScreen, camera);

    Vec2 mouseWorld(mouseWorldRay.x, mouseWorldRay.y);


    if(IsMouseButtonDown(MOUSE_BUTTON_LEFT))
    {
        static Vec2 previousMouseWorld;
        static bool firstFrame = true;

        
        if (firstFrame)
        {
            previousMouseWorld = mouseWorld;
            firstFrame = false;
        }

        Vec2 mouseVelocity =
            (mouseWorld - previousMouseWorld) /
            constants::Physics_dt;
        
        previousMouseWorld = mouseWorld;

        if (bodyIndex != -1)
        {
            Bodies[bodyIndex]->IsHeld = true;
            Bodies[bodyIndex]->IsSleeping = false;
            forceMagnitude += 1000.0;
            Bodies[bodyIndex]->Position = mouseWorld;
        }
    }

    if(IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
    {   
        if (bodyIndex != -1)
        {
            Vec2 delta = mouseWorld - Bodies[bodyIndex]->Position;
        
            Bodies[bodyIndex]->AddForce(delta * forceMagnitude);

            Bodies[bodyIndex]->IsHeld = false;
        }
        
        bodyIndex = -1;
    }

    if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))
    {
        MakeSpring(mouseWorld);
    }
    if (IsKeyPressed(KEY_P))
    {
        MakePendulum(2 , 300.0 , camera);
    }
}


Vec2 GetShootingDir(Camera2D& camera) 
{
    Vector2 mouseScreen = GetMousePosition();
    Vector2 mouseWorld = GetScreenToWorld2D(mouseScreen, camera);

    Vec2 hoveringPos(mouseWorld.x, mouseWorld.y);

    Vec2 start = {100.0 , 650.0};
    Vec2 direction = hoveringPos - start;
    direction = direction.Normalize();

    return direction;
} 

void UpdateShootingControls(Camera2D& camera , vector<RigidBody*> & Bodies)  
{
    Vec2 direction = GetShootingDir(camera);

    Vec2 start = {100.0 , 650.0};
    static float speed = 0.0;
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
    {
        speed += 120.0;
        speed = clamp(speed , constants::MinShootingForce , constants::MaxShootingForce);
    }

    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
    {
        RigidBody * body = new RigidBody(RigidBody::CreateCircle(start , 1.0 , 0.5 , false , 30.0));
        body->LinearVelocity = direction * speed;
        Bodies.push_back(body);
        speed = 0.0;
    }
}


void MakePendulum(int numOfBalls , float length , Camera2D & camera)
{
    Vector2 mouseScreen = GetMousePosition();
    Vector2 mouseWorld = GetScreenToWorld2D(mouseScreen, camera);

    Vec2 startPos(mouseWorld.x, mouseWorld.y);

    Vec2 temp = {0.0 , length};

    RigidBody * previous = nullptr;
    float radius = 30.0;

    for (int i = 0 ; i < numOfBalls ; i++)
    {
        RigidBody * node = new RigidBody(RigidBody::CreateCircle(startPos + temp * i , 1.0 , 0.5 , false , radius));

        node->BodyColor = RED;
        Bodies.push_back(node);
        if (i == 0)
            node->IsStatic = true;

        
        if (previous != nullptr)
        {
            Joint joint(previous , node , JointType::Rope ,length);
            Joints.push_back(joint);
        }
        previous = node;
    }
}

void MakeTower()
{
    float x = 900.0f;
    float y = 640.0f;

    float pillarW = 20.0f;
    float pillarH = 80.0f;

    float beamW = 100.0f;
    float beamH = 20.0f;

    for (int level = 0; level < 4; level++)
    {
        float currentY = y - level * 90.0f;

        Bodies.push_back(
            new RigidBody(
                RigidBody::CreateBox(
                    {x - 35.0f, currentY},
                    1.0f, 0.2f, false,
                    pillarW, pillarH)));

        Bodies.push_back(
            new RigidBody(
                RigidBody::CreateBox(
                    {x + 35.0f, currentY},
                    1.0f, 0.2f, false,
                    pillarW, pillarH)));

        Bodies.push_back(
            new RigidBody(
                RigidBody::CreateBox(
                    {x, currentY - 45.0f},
                    1.0f, 0.2f, false,
                    beamW, beamH)));
    }

    Bodies.push_back(
        new RigidBody(
            RigidBody::CreateCircle(
                {x, y - 360.0f},
                1.0f, 0.2f, false,
                25.0f)));
}

// void UpdateControls(Camera2D& camera , vector<RigidBody*> & Bodies)  
// {
//     float dx = 0.0f;
//     float dy = 0.0f;
    
//     float rotation = constants::pi * 0.8f* constants::Physics_dt;
    
    
    
//     if (bodyIndex == -1)
//     {
//         for (int i = 0 ; i < Bodies.size() ; i++)
//         {
            
//             if (Bodies.size() == 0)
//                 break;
//             if (Bodies[i]->IsStatic)
//                 continue;
//             else
//             {
//                 bodyIndex = i;
//                 break;
//             }
//         }
//     }

//     if (IsKeyDown(KEY_RIGHT))
//     {
//         dx ++;
//     }

//     if (IsKeyDown(KEY_LEFT))
//     {
//         dx --;
//     }

//     if (IsKeyDown(KEY_UP))
//     {
//         dy --;
//     }

//     if (IsKeyDown(KEY_DOWN))
//     {
//         dy ++;
//     }
//     if (dx != 0 || dy != 0)
//     {
//         Vec2 input = {dx , dy};
//         Vec2 forceDirection = input.Normalize();
//         Vec2 force = forceDirection * forceMagnitude;
//         Bodies[bodyIndex]->IsSleeping = false;
//         Bodies[bodyIndex]->AddForce(force); 
//     }

//     if (IsKeyPressed(KEY_X))
//     {
//         camera.zoom += 0.2f;
//     }

//     if (IsKeyPressed(KEY_Z))
//     {
//         camera.zoom -= 0.2f;
//     }
//     camera.zoom = clamp(camera.zoom, 0.1f, 10.0f);

//     if (IsKeyDown(KEY_A))
//     {
//         Bodies[bodyIndex]->RotateBy(-rotation);
//     }
//     if (IsKeyDown(KEY_D))
//     {
//         Bodies[bodyIndex]->RotateBy(rotation);
//     }

//     if (IsKeyDown(KEY_P))
//     {
//         Bodies[1]->RotateBy(rotation);
//     }
//     if (IsKeyDown(KEY_O))
//     {
//         Bodies[1]->RotateBy(-rotation);
//     }

//     if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
//     {
//         Vector2 mouseScreen = GetMousePosition();
//         Vector2 mouseWorld = GetScreenToWorld2D(mouseScreen, camera);

//         Vec2 clickPos(mouseWorld.x, mouseWorld.y);

//         int index = FindClosestBody(clickPos);
//         if (index != -1)
//         {
//             bodyIndex = index;
//         }

//     }

//     if(IsMouseButtonDown(MOUSE_BUTTON_LEFT))
//     {
//         static Vec2 previousMouseWorld;
//         static bool firstFrame = true;

//         Vector2 mouseScreen = GetMousePosition();
//         Vector2 mouseWorldRay = GetScreenToWorld2D(mouseScreen, camera);

//         Vec2 mouseWorld(mouseWorldRay.x, mouseWorldRay.y);

//         if (firstFrame)
//         {
//             previousMouseWorld = mouseWorld;
//             firstFrame = false;
//         }

//         Vec2 mouseVelocity =
//             (mouseWorld - previousMouseWorld) /
//             constants::Physics_dt;

//         Vec2 maxDrag = mouseVelocity.Normalize() * constants::MaxDragVelocity;
//         mouseVelocity.Clamp(maxDrag);
        
//         previousMouseWorld = mouseWorld;

//         if (bodyIndex != -1)
//         {
//             Vec2 delta = mouseWorld - Bodies[bodyIndex]->Position;
//             Bodies[bodyIndex]->IsSleeping = false;
//             forceMagnitude += 1000.0;
//             Bodies[bodyIndex]->Position = mouseWorld;
//         }
            

//         // MakeSpring(clickPos);

//         // float width  = 20 + rand() % 80;
//         // float height = 20 + rand() % 80;
//         // float restitution = 0.2f + ((float)rand() / RAND_MAX) * (0.8f);
//         // RigidBody * body = new RigidBody(RigidBody::CreateBox(clickPos , 1.0f ,restitution , false , width , height)) ;
//         // Bodies.push_back(body);
//     }

//     if(IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
//     {
//         Vector2 mouseScreen = GetMousePosition();
//         Vector2 mouseWorldRay = GetScreenToWorld2D(mouseScreen, camera);

//         Vec2 mouseWorld(mouseWorldRay.x, mouseWorldRay.y);

//         Vec2 delta = mouseWorld - Bodies[bodyIndex]->Position;
//         Bodies[bodyIndex]->AddForce(delta * forceMagnitude);
//         bodyIndex = -1;
//     }
//     if(IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))
//     {
//         Vector2 mouseScreen = GetMousePosition();
//         Vector2 mouseWorld = GetScreenToWorld2D(mouseScreen, camera);
//         Vec2 clickPos(mouseWorld.x, mouseWorld.y);

        
//         // MakeRope(40 , clickPos);

//         float restitution = 0.2f + ((float)rand() / RAND_MAX) * (0.8f);
//         float radius = 10 + rand() % 40;
//         RigidBody * body = new RigidBody(RigidBody::CreateCircle(clickPos , 1.0f ,restitution , false , radius)); 
//         Bodies.push_back(body);
//     }
// }


void DrawBodies(vector<RigidBody*>& Bodies , Camera2D& camera) 
{
    if (Bodies.size() == 0)
        return;
    for(auto body : Bodies)
    {
        
        if (body->IsStatic)
        {
            body->BodyColor = BLACK;
        }
        if(body->shapeType == ShapeType::Circle)
        {
            DrawCircle(
                body->Position.X,
                body->Position.Y,
                body->Radius,
                body->DisplayColor
            );
        }
        else if(body->shapeType == ShapeType::Box)
        {
            
            vector<Vec2> verts = body->GetTransformedVertices();

            for(int i = 0; i < body->Triangles.size(); i += 3)
            {
                Vec2 p0 = verts[body->Triangles[i]];
                Vec2 p1 = verts[body->Triangles[i + 1]];
                Vec2 p2 = verts[body->Triangles[i + 2]];

                DrawTriangle(
                    {p0.X, p0.Y},
                    {p1.X, p1.Y},
                    {p2.X, p2.Y},
                    body->DisplayColor
                );

            }  
        }
    }

        DrawText(
            TextFormat("Bodies: %d | Step time: %.2f ms | total pairs %d ", 
                (int)Bodies.size() , inf.averagePhysicsTime , (int)inf.averageSATCalls) , 
            20,
            20,
            20,
            WHITE
        );

        for (auto & joint : Joints)
        {
            if (joint.jointType == JointType::Spring)
            {
                float x1 = joint.BodyA->Position.X + joint.BodyA->Width * 0.5;
                float x2 = joint.BodyB->Position.X -  joint.BodyB->Width * 0.5;
                Vector2 start = {x1 , joint.BodyA->Position.Y} ;
                Vector2 end = {x2, joint.BodyB->Position.Y};

                DrawLineEx(start , end , 1.0 , ORANGE);
            }
            else if (joint.jointType == JointType::Rope)
            {
                Vector2 start = {joint.BodyA->Position.X , joint.BodyA->Position.Y} ;
                Vector2 end = {joint.BodyB->Position.X, joint.BodyB->Position.Y};
                DrawLineEx(start , end , 2.0 , ORANGE);
            }
            
        }
    }


void RemoveOffScreen(vector<RigidBody*>& Bodies , Camera2D & camera)
{
    Vector2 rayCamMin = GetScreenToWorld2D({0, 0}, camera);
    Vector2 rayCamMax = GetScreenToWorld2D(
    {
        (float)GetScreenWidth(),
        (float)GetScreenHeight()
    }, camera);

    Vec2 camMin(rayCamMin.x, rayCamMin.y);
    Vec2 camMax(rayCamMax.x, rayCamMax.y);

    for (int i = Bodies.size() - 1; i >= 0; --i)
    {
        if (Bodies.size() == 0)
            break;
        if (Bodies[i]->IsStatic)
            continue;
        AABB aabb = Bodies[i]->GetAABB();

        if (aabb.Min.X > camMax.X ||
            aabb.Max.X < camMin.X ||
            aabb.Min.Y > camMax.Y ||
            aabb.Max.Y < camMin.Y)
        {
            Bodies.erase(Bodies.begin() + i);
        }
    }
}


void BounceScreen(vector<RigidBody*>& bodies , Camera2D & camera )
{
    Vector2 rayCamMin = GetScreenToWorld2D({0, 0}, camera);
    Vector2 rayCamMax = GetScreenToWorld2D(
    {
        (float)GetScreenWidth(),
        (float)GetScreenHeight()
    }, camera);

    Vec2 camMin(rayCamMin.x, rayCamMin.y);
    Vec2 camMax(rayCamMax.x, rayCamMax.y);
    for (auto body : bodies)
    {
        if (body->shapeType == ShapeType::Circle)
        {
            if (body->Position.X - body->Radius - 1.0 < camMin.X)
            {
                body->Position.X = camMin.X + body->Radius + 1.0;
                body->LinearVelocity.X *= -body->Restitution;
            }

            if (body->Position.X + body->Radius+ 1.0 > camMax.X)
            {
                body->Position.X = camMax.X - body->Radius - 1.0;
                body->LinearVelocity.X *= -body->Restitution;
            }

            if (body->Position.Y - body->Radius - 1.0< camMin.Y)
            {
                body->Position.Y = camMin.Y + body->Radius + 1.0;
                body->LinearVelocity.Y *= -body->Restitution;
            }

            if (body->Position.Y + body->Radius + 1.0> camMax.Y)
            {
                body->Position.Y = camMax.Y - body->Radius - 1.0;
                body->LinearVelocity.Y *= -body->Restitution;
            }
        }
        else if(body->shapeType == ShapeType::Box)
        {
            float halfWidth = body->Width * 0.5f;
            float halfHeight = body->Height * 0.5f;

            if (body->Position.X - halfWidth - 1.0< camMin.X)
            {
                body->Position.X = camMin.X + halfWidth + 1.0;
                body->LinearVelocity.X *= -body->Restitution;
            }

            if (body->Position.X + halfWidth + 1.0 > camMax.X)
            {
                body->Position.X = camMax.X - halfWidth - 1.0;
                body->LinearVelocity.X *= -body->Restitution;
            }

            if (body->Position.Y - halfHeight - 1.0< camMin.Y)
            {
                body->Position.Y = camMin.Y + halfHeight + 1.0;
                body->LinearVelocity.Y *= -body->Restitution;
            }

            if (body->Position.Y + halfHeight + 1.0> camMax.Y)
            {
                body->Position.Y = camMax.Y - halfHeight -1.0;
                body->LinearVelocity.Y *= -body->Restitution;
            }
        }
    }
}

vector<RigidBody*> InitializeWorld()
{
    vector<RigidBody*> bodies;

    RigidBody *ground = new RigidBody(RigidBody::CreateBox(
        {640.0f, 700.0f}, 
        1.0f,             
        0.5f,             
        true,             
        1280.0f,          
        40.0f));

    ground->BodyColor = GREEN;
    bodies.push_back(ground);

    // RigidBody * slope1 = new RigidBody(RigidBody::CreateBox(
    //     {240.0f, 350.0f}, 
    //     1.0f,             
    //     0.5f,             
    //     true,             
    //     400.0f,          
    //     20.0f             
    // ) );

    // slope1->RotateBy(constants::pi * 0.325);
    // bodies.push_back(slope1);

    // RigidBody * slope2 = new RigidBody(RigidBody::CreateBox(
    //     {550.0f, 250.0f}, 
    //     1.0f,             
    //     0.5f,             
    //     true,             
    //     300.0f,          
    //     20.0f             
    // ) ) ;

    // slope2->RotateBy(-constants::pi * 0.300);
    // bodies.push_back(slope2);

    return bodies;
}

void CleanupWorld(vector<RigidBody*>& bodies) {
    for (auto body : bodies) {
        delete body; 
    }
    bodies.clear();
}

int main()
{
   
    

    InitWindow(1280, 720, "Physics Engine");

    SetTargetFPS(60);

    srand(time(nullptr));

    Bodies = InitializeWorld();
    MakeBodies(30);

    Bodies.reserve(1000);
    Joints.reserve(1000);

    Camera2D camera = {0};

    camera.target = {640, 360};
    camera.offset = {640.0f, 360.0f};

    camera.rotation = 0.0f;
    camera.zoom = 1.0f;

    while(!WindowShouldClose())
    {
        // UpdateControls(camera , Bodies);
        // UpdateShootingControls(camera , Bodies);
        UpdateDragControls(camera , Bodies);
        

        BounceScreen(Bodies , camera);

        auto start = std::chrono::high_resolution_clock::now();
        UpdatePhysics(Bodies , iterations , Joints);
        auto end = std::chrono::high_resolution_clock::now();

        double physicsStep = std::chrono::duration<double, std::milli>(end - start).count();
        inf.TotalTimeToUpdatePhysics += physicsStep;
        inf.TotalStep++;
        inf.timer += constants::Physics_dt;

        inf.totalPairsAccum += inf.totalPairs;
        inf.aabbRejectedAccum += inf.aabbRejected;
        inf.satCallsAccum += inf.SATCalls;

        inf.totalPairs = 0;
        inf.aabbRejected = 0;
        inf.SATCalls = 0;

        if (inf.timer >= 1)
        {
            inf.timer = 0.0;

            inf.averagePhysicsTime = inf.TotalTimeToUpdatePhysics / inf.TotalStep;
            inf.TotalTimeToUpdatePhysics = 0.0;
            
            inf.averagePairs = inf.totalPairsAccum / inf.TotalStep / iterations;
            inf.averageRejected = inf.aabbRejectedAccum / inf.TotalStep / iterations;
            inf.averageSATCalls = inf.satCallsAccum / inf.TotalStep / iterations;

            inf.totalPairsAccum = 0;
            inf.aabbRejectedAccum = 0;
            inf.satCallsAccum = 0;

            inf.TotalStep = 0;
        }


        // RemoveOffScreen(Bodies , camera);

        BeginDrawing();

        ClearBackground(DARKGRAY);

        BeginMode2D(camera);

        DrawBodies(Bodies , camera);

        EndMode2D();

        EndDrawing();
    }

    CleanupWorld(Bodies);
    CloseWindow();

    return 0;
}