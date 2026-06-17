#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <chrono>

#include "raylib.h"

#include "physics/RigidBody.h"
#include "math/math.h"
#include "physics/collision/Collision.h"
#include "physics/constants.h"
#include "physics/Manifold.h"
#include "physics/AABB.h"

using namespace std;

vector <Manifold> manifolds;
vector <RigidBody> Bodies;
vector <Vec2> contactPoints;

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

int iterations = 20;

void BounceScreen(vector<RigidBody>& bodies , Camera2D & camera);
vector<RigidBody> MakeBodies(int num);

void RotateBodys(vector<RigidBody> & Bodies , float angle)
{
    for (int i = 0 ; i < Bodies.size() - 1 ; i++)
    {
        if (Bodies[i].shapeType == ShapeType::Box)
        {
            Bodies[i].RotateBy(angle);
        }
    }

}

bool IsAABBIntersect(RigidBody& bodyA , RigidBody& bodyB)
{
    AABB a = bodyA.GetAABB();
    AABB b = bodyB.GetAABB();

    return !(a.Max.X < b.Min.X ||
             a.Min.X > b.Max.X ||
             a.Max.Y < b.Min.Y ||
             a.Min.Y > b.Max.Y);
}

void ResolveCollision(const Manifold & manifold)
{
    RigidBody & bodyA = manifold.BodyA; 
    RigidBody & bodyB = manifold.BodyB; 
    Vec2 normal = manifold.Normal;

    Vec2 relativeVelocity = bodyB.LinearVelocity - bodyA.LinearVelocity;

    if (math::Dot(normal , relativeVelocity) > 0.0)
        return;

    float minRestitution = min(bodyA.Restitution , bodyB.Restitution);
    // impulse = -(1 + minRestitution) * Dot(relativeVelocity , nomal) / 1/massA + 1/massB
    float impulse = -(1 + minRestitution) * math::Dot(relativeVelocity , normal) / (bodyA.InvMass + bodyB.InvMass);
    bodyA.LinearVelocity -= normal * (impulse * bodyA.InvMass);
    bodyB.LinearVelocity += normal * (impulse * bodyB.InvMass);
}

void UpdateBodyPhysics(vector<RigidBody> & Bodies , int iterations)
{
    for(auto & body : Bodies)
    {
        body.UpdatePhysics(iterations);

    }
}

void SeparateBodies(RigidBody & bodyA, RigidBody & bodyB , Vec2 MTV)
{
    if (bodyA.IsStatic)
    {
        bodyB.MoveBy(MTV);
    }
    else if (bodyB.IsStatic)
    {
        bodyA.MoveBy(MTV * -1);
    }
    else
    {
        bodyA.MoveBy(MTV * -0.5);

        bodyB.MoveBy(MTV * 0.5);
    }
}

vector<Manifold> DetectFrameCollisions(vector<RigidBody> & Bodies)
{
    vector<Manifold> tempManifolds;
    for (int i = 0; i < Bodies.size() - 1; i++)
    {
        RigidBody& bodyA = Bodies[i];

        for (int j = i + 1; j < Bodies.size(); j++)
        {
             totalPairs++;
            RigidBody& bodyB = Bodies[j];

            if (bodyA.IsStatic && bodyB.IsStatic)
            {
                continue;
            }

            if (!IsAABBIntersect(bodyA, bodyB))
            {
                aabbRejected++;
                continue;
            }

            SATCalls++;

            Collision::CollisionResult result = Collision::Collide(bodyA, bodyB);

            if (!result.IsIntersect)
                continue;

            Manifold manifold = Manifold(bodyA , bodyB , result.Depth 
                , result.NormalCollisionDirection , {0.0,0.0} , {0.0,0.0} , 0);
            
            tempManifolds.push_back(manifold);
        }
    }
    return tempManifolds;
}

void BroadPhase(vector<RigidBody> & Bodies)
{
    manifolds.clear();
    manifolds = DetectFrameCollisions(Bodies); 
}

void NarrowPhase(vector <Manifold> & manifolds) 
{
    for(auto & manifold : manifolds)
    {
        Vec2 MTV = manifold.Normal * manifold.Depth;
        SeparateBodies(manifold.BodyA , manifold.BodyB , MTV);

        Collision::FindContactPoints(manifold);
        ResolveCollision(manifold);
    }
}

void SetForcesToZero(vector<RigidBody> & Bodies)
{
    for (auto & body : Bodies)
    {
        body.Force = {0.0,0.0};
    }
}

void UpdatePhysics(vector<RigidBody> & Bodies , Camera2D & camera , int iterations)
{
    iterations = clamp(iterations , constants::MinSteps , constants::MaxSteps);

    for (int it = 0 ; it < iterations ; it++)
    {

        UpdateBodyPhysics(Bodies , iterations);
        BroadPhase(Bodies);
        NarrowPhase(manifolds);
    }
    SetForcesToZero(Bodies);
}

void UpdateControls(Camera2D& camera , vector<RigidBody> & Bodies)  
{
    float dx = 0.0f;
    float dy = 0.0f;
    float forceMagnitude = 5000000.0f;
    float rotation = constants::pi * 0.8f* constants::Physics_dt;
    
    static int bodyIndex = 0;
    for (int i = 0 ; i < Bodies.size() ; i++)
    {
        if (Bodies[i].IsStatic)
            continue;
        else
        {
            bodyIndex = i;
            break;
        }
    }

    if (IsKeyDown(KEY_RIGHT))
    {
        dx ++;
    }

    if (IsKeyDown(KEY_LEFT))
    {
        dx --;
    }

    if (IsKeyDown(KEY_UP))
    {
        dy --;
    }

    if (IsKeyDown(KEY_DOWN))
    {
        dy ++;
    }
    if (dx != 0 || dy != 0)
    {
        Vec2 input = {dx , dy};
        Vec2 forceDirection = input.Normalize();
        Vec2 force = forceDirection * forceMagnitude;

        Bodies[bodyIndex].AddForce(force);
        Bodies[bodyIndex].BodyColor = BLUE;
    }

    if (IsKeyPressed(KEY_X))
    {
        camera.zoom += 0.2f;
    }

    if (IsKeyPressed(KEY_Z))
    {
        camera.zoom -= 0.2f;
    }
    camera.zoom = clamp(camera.zoom, 0.1f, 10.0f);

    if (IsKeyDown(KEY_A))
    {
        Bodies[bodyIndex].RotateBy(-rotation);
    }
    if (IsKeyDown(KEY_D))
    {
        Bodies[bodyIndex].RotateBy(rotation);
    }

    if (IsKeyDown(KEY_P))
    {
        Bodies[1].RotateBy(rotation);
    }
    if (IsKeyDown(KEY_O))
    {
        Bodies[1].RotateBy(-rotation);
    }

    if(IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
    {
        Vector2 mouseScreen = GetMousePosition();
        Vector2 mouseWorld = GetScreenToWorld2D(mouseScreen, camera);
        Vec2 clickPos(mouseWorld.x, mouseWorld.y);

        float width  = 20 + rand() % 80;
        float height = 20 + rand() % 80;
        float restitution = 0.2f + ((float)rand() / RAND_MAX) * (0.8f);
        RigidBody body = RigidBody::CreateBox(clickPos , 1.0f ,restitution , false , width , height);
        Bodies.push_back(body);
    }
    if(IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))
    {
        Vector2 mouseScreen = GetMousePosition();
        Vector2 mouseWorld = GetScreenToWorld2D(mouseScreen, camera);
        Vec2 clickPos(mouseWorld.x, mouseWorld.y);

        float restitution = 0.2f + ((float)rand() / RAND_MAX) * (0.8f);
        float radius = 10 + rand() % 40;
        RigidBody body = RigidBody::CreateCircle(clickPos , 1.0f ,restitution , false , radius);
        Bodies.push_back(body);
    }
}

void DrawBodies(vector<RigidBody>& Bodies)
{
    
    for(RigidBody &body : Bodies)
    {
        if (body.IsStatic)
        {
            body.BodyColor = BLACK;
        }
        if(body.shapeType == ShapeType::Circle)
        {
            DrawCircle(
                body.Position.X,
                body.Position.Y,
                body.Radius,
                body.BodyColor
            );
        }
        else if(body.shapeType == ShapeType::Box)
        {
            
            vector<Vec2> verts = body.GetTransformedVertices();

            for(int i = 0; i < body.Triangles.size(); i += 3)
            {
                Vec2 p0 = verts[body.Triangles[i]];
                Vec2 p1 = verts[body.Triangles[i + 1]];
                Vec2 p2 = verts[body.Triangles[i + 2]];

                DrawTriangle(
                    {p0.X, p0.Y},
                    {p1.X, p1.Y},
                    {p2.X, p2.Y},
                    body.BodyColor
                );

            }           
        }
    }
        DrawText(
            TextFormat("Bodies: %d | contact points: %d | total pairs: %d | aabb rejected: %d | SAT calls: %d | Step time: %.2f ms", 
                (int)Bodies.size() , (int)contactPoints.size() , (int)averagePairs , (int)averageRejected
                 , (int)averageSATCalls ,averagePhysicsTime ),   
            20,
            20,
            20,
            WHITE
        );
}

void RemoveOffScreen(vector<RigidBody>& Bodies , Camera2D & camera)
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
    AABB aabb = Bodies[i].GetAABB();

    if (aabb.Min.X > camMax.X ||
        aabb.Max.X < camMin.X ||
        aabb.Min.Y > camMax.Y ||
        aabb.Max.Y < camMin.Y)
    {
        Bodies.erase(Bodies.begin() + i);
    }
}
}

vector<RigidBody> MakeBodies(int num)
{
    vector<RigidBody> Bodies;
    int padding = 100;
    int staticBodys = 5;
    for(int i = 0; i < num; i++)
    {
        int type = rand() % 2;

        staticBodys--;
        bool isStatic = true;
        
        if (staticBodys < 0)
            isStatic = false;
        else 
            isStatic = true;

        float restitution = (float)rand() / RAND_MAX;

        float density = 0.5f + ((float)rand() / RAND_MAX) * 1.0f;

        Vec2 position(
            rand() % (1200 - padding) ,
            rand() % (700 - padding)
        );

        if(type == 0)
        {
            float radius = 10 + rand() % 40;

            Bodies.push_back(
                RigidBody::CreateCircle(
                    position,
                    density,
                    restitution,
                    isStatic,
                    radius
                )
            );
        }
        else
        {
            float width  = 20 + rand() % 80;
            float height = 20 + rand() % 80;

            Bodies.push_back(
                RigidBody::CreateBox(
                    position,
                    density,
                    restitution,
                    isStatic,
                    width,
                    height
                )
            );
        }
    }

    return Bodies;
}

void BounceScreen(vector<RigidBody>& bodies , Camera2D & camera )
{
    Vector2 rayCamMin = GetScreenToWorld2D({0, 0}, camera);
    Vector2 rayCamMax = GetScreenToWorld2D(
    {
        (float)GetScreenWidth(),
        (float)GetScreenHeight()
    }, camera);

    Vec2 camMin(rayCamMin.x, rayCamMin.y);
    Vec2 camMax(rayCamMax.x, rayCamMax.y);
    for (RigidBody& body : bodies)
    {
        if (body.shapeType == ShapeType::Circle)
        {
            if (body.Position.X - body.Radius - 1.0 < camMin.X)
            {
                body.Position.X = camMin.X + body.Radius + 1.0;
                body.LinearVelocity.X *= -0.9;
            }

            if (body.Position.X + body.Radius+ 1.0 > camMax.X)
            {
                body.Position.X = camMax.X - body.Radius - 1.0;
                body.LinearVelocity.X *= -0.9;
            }

            if (body.Position.Y - body.Radius - 1.0< camMin.Y)
            {
                body.Position.Y = camMin.Y + body.Radius + 1.0;
                body.LinearVelocity.Y *= -0.9;
            }

            if (body.Position.Y + body.Radius + 1.0> camMax.Y)
            {
                body.Position.Y = camMax.Y - body.Radius - 1.0;
                body.LinearVelocity.Y *= -0.9;
            }
        }
        else if(body.shapeType == ShapeType::Box)
        {
            float halfWidth = body.Width * 0.5f;
            float halfHeight = body.Height * 0.5f;

            if (body.Position.X - halfWidth - 1.0< camMin.X)
            {
                body.Position.X = camMin.X + halfWidth + 1.0;
                body.LinearVelocity.X *= -0.9f;
            }

            if (body.Position.X + halfWidth + 1.0 > camMax.X)
            {
                body.Position.X = camMax.X - halfWidth - 1.0;
                body.LinearVelocity.X *= -0.9f;
            }

            if (body.Position.Y - halfHeight - 1.0< camMin.Y)
            {
                body.Position.Y = camMin.Y + halfHeight + 1.0;
                body.LinearVelocity.Y *= -0.9f;
            }

            if (body.Position.Y + halfHeight + 1.0> camMax.Y)
            {
                body.Position.Y = camMax.Y - halfHeight -1.0;
                body.LinearVelocity.Y *= -0.9f;
            }
        }
    }
}

vector<RigidBody> InitializeWorld()
{
    vector<RigidBody> bodies;

    RigidBody ground = RigidBody::CreateBox(
        {640.0f, 700.0f}, 
        1.0f,             
        0.5f,             
        true,             
        1280.0f,          
        40.0f             
    );

    ground.BodyColor = GREEN;
    bodies.push_back(ground);

    // RigidBody slope1 = RigidBody::CreateBox(
    //     {240.0f, 350.0f}, 
    //     1.0f,             
    //     0.5f,             
    //     true,             
    //     400.0f,          
    //     40.0f             
    // );

    // slope1.RotateBy(constants::pi * 0.325);
    // bodies.push_back(slope1);

    // RigidBody slope2 = RigidBody::CreateBox(
    //     {550.0f, 250.0f}, 
    //     1.0f,             
    //     0.5f,             
    //     true,             
    //     400.0f,          
    //     40.0f             
    // );

    // slope2.RotateBy(-constants::pi * 0.325);
    // bodies.push_back(slope2);

    return bodies;
}

int main()
{
    InitWindow(1280, 720, "Physics Engine");

    SetTargetFPS(60);

    srand(time(nullptr));

    Bodies = InitializeWorld();

    Camera2D camera = {0};

    camera.target = {640, 360};
    camera.offset = {640.0f, 360.0f};

    camera.rotation = 0.0f;
    camera.zoom = 1.0f;

    while(!WindowShouldClose())
    {
        UpdateControls(camera , Bodies);

        auto start = std::chrono::high_resolution_clock::now();
        UpdatePhysics(Bodies, camera, iterations);
        auto end = std::chrono::high_resolution_clock::now();

        double physicsStep = std::chrono::duration<double, std::milli>(end - start).count();
        TotalTimeToUpdatePhysics += physicsStep;
        TotalStep++;
        timer += constants::Physics_dt;

        totalPairsAccum += totalPairs;
        aabbRejectedAccum += aabbRejected;
        satCallsAccum += SATCalls;

        totalPairs = 0;
        aabbRejected = 0;
        SATCalls = 0;

        if (timer >= 1)
        {
            timer = 0.0;

            averagePhysicsTime = TotalTimeToUpdatePhysics / TotalStep;
            TotalTimeToUpdatePhysics = 0.0;
            
            averagePairs = totalPairsAccum / TotalStep / iterations;
            averageRejected = aabbRejectedAccum / TotalStep / iterations;
            averageSATCalls = satCallsAccum / TotalStep / iterations;

            totalPairsAccum = 0;
            aabbRejectedAccum = 0;
            satCallsAccum = 0;

            TotalStep = 0;
        }


        RemoveOffScreen(Bodies , camera);

        BeginDrawing();

        ClearBackground(DARKGRAY);

        BeginMode2D(camera);

        DrawBodies(Bodies);

        EndMode2D();

        EndDrawing();
    }

    CloseWindow();

    return 0;
}