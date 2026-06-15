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

vector<Manifold> DetectFrameCollisions(vector<RigidBody> & Bodies)
{
    vector<Manifold> tempManifolds;
    for (int i = 0; i < Bodies.size() - 1; i++)
    {
        RigidBody& bodyA = Bodies[i];

        for (int j = i + 1; j < Bodies.size(); j++)
        {
            RigidBody& bodyB = Bodies[j];
            if (!IsAABBIntersect(bodyA, bodyB))
                continue;

            Collision::CollisionResult result =
                Collision::Collide(bodyA, bodyB);

            if (!result.IsIntersect)
                continue;

            if (bodyA.IsStatic && bodyB.IsStatic)
            {
                continue;
            }

            if (bodyA.IsStatic)
            {
                bodyB.MoveBy(
                    result.NormalCollisionDirection *
                    result.Depth
                );
            }
            else if (bodyB.IsStatic)
            {
                bodyA.MoveBy(
                    result.NormalCollisionDirection *
                    (-result.Depth)
                );
            }
            else
            {
                bodyA.MoveBy(
                    result.NormalCollisionDirection *
                    (-result.Depth * 0.5f)
                );

                bodyB.MoveBy(
                    result.NormalCollisionDirection *
                    (result.Depth * 0.5f)
                );
            }
            Manifold manifold = Manifold(bodyA , bodyB , result.Depth 
                , result.NormalCollisionDirection , {0.0,0.0} , {0.0,0.0} , 0);
            
            tempManifolds.push_back(manifold);
        }
    }
    return tempManifolds;
}

void UpdatePhysics(vector<RigidBody> & Bodies , Camera2D & camera , int iterations)
{
    iterations = clamp(iterations , constants::MinSteps , constants::MaxSteps);

    contactPoints.clear();

    for (int it = 0 ; it < iterations ; it++)
    {
        // BounceScreen(Bodies , camera);

        UpdateBodyPhysics(Bodies , iterations);

        manifolds.clear();

        manifolds = DetectFrameCollisions(Bodies); 

        
        for(auto & manifold : manifolds)
        {
            Collision::FindContactPoints(manifold);

            if (it == iterations - 1)
            {
                if (manifold.ContactCount > 0)
                {
                    contactPoints.push_back(manifold.ContactPoints[0]);
                    if (manifold.ContactCount > 1)
                    {
                        contactPoints.push_back(manifold.ContactPoints[1]);
                    }
                }
            }
            
            ResolveCollision(manifold);
        }
    }
    
    for (auto & body : Bodies)
    {
        body.Force = {0.0,0.0};
    }
}

void UpdateControls(Camera2D& camera , vector<RigidBody> & Bodies)  
{
    float dx = 0.0f;
    float dy = 0.0f;
    float forceMagnitude = 5000000.0f;
    float rotation = constants::pi * 0.8f* GetFrameTime();
    
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

    

    if(IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
    {
        Vector2 mouseScreen = GetMousePosition();
        Vector2 mouseWorld = GetScreenToWorld2D(mouseScreen, camera);
        Vec2 clickPos(mouseWorld.x, mouseWorld.y);

        float width  = 20 + rand() % 80;
        float height = 20 + rand() % 80;
        RigidBody body = RigidBody::CreateBox(clickPos , 1.0f , 0.2f , false , width , height);
        Bodies.push_back(body);
    }
    if(IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))
    {
        Vector2 mouseScreen = GetMousePosition();
        Vector2 mouseWorld = GetScreenToWorld2D(mouseScreen, camera);
        Vec2 clickPos(mouseWorld.x, mouseWorld.y);

        float radius = 10 + rand() % 40;
        RigidBody body = RigidBody::CreateCircle(clickPos , 1.0f , 0.2f , false , radius);
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
    for (auto & point : contactPoints)
    {
        DrawRectangle(point.X - 5 , point.Y - 5 , 10 , 10 , YELLOW);
    }
    
        DrawText(
            TextFormat("Bodies: %d | Manifolds: %d | contact points: %d ", 
                (int)Bodies.size() , (int)manifolds.size() , (int)contactPoints.size() ),   
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
        1.0f,             
        true,             
        1280.0f,          
        40.0f             
    );

    ground.BodyColor = GREEN;

    bodies.push_back(ground);

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

        UpdatePhysics(Bodies, camera, 20);

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