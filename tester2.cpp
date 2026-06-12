#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>

#include "raylib.h"

#include "physics/RigidBody.h"
#include "math/math.h"
#include "physics/collision/Collision.h"
#include "physics/constants.h"



using namespace std;

void BounceScreen(vector<RigidBody>& bodies , Camera2D & camera);

void RotateBodys(vector<RigidBody> & bodysVector , float angle)
{
    for (int i = 0 ; i < bodysVector.size() - 1 ; i++)
    {
        if (bodysVector[i].shapeType == ShapeType::Box)
        {
            bodysVector[i].RotateBy(angle);
        }
    }

}

Collision::CollisionResult Collide(RigidBody& bodyA, RigidBody& bodyB)
{
    if (bodyA.shapeType == ShapeType::Circle)
    {
        if (bodyB.shapeType == ShapeType::Circle)
        {
            return Collision::IsIntersectCircle(
                bodyA.Position,
                bodyA.Radius,
                bodyB.Position,
                bodyB.Radius
            );
        }
        else if (bodyB.shapeType == ShapeType::Box)
        {
            Collision::CollisionResult res = Collision::IsPolygonCircleIntersect(bodyB.GetTransformedVertices(),bodyA.Position,bodyA.Radius);
            res.NormalCollisionDirection = res.NormalCollisionDirection * -1;
            return res;
        }
    }
    else if (bodyA.shapeType == ShapeType::Box)
    {
        if (bodyB.shapeType == ShapeType::Box)
        {
            return Collision::IsPolygonSIntersect(
                bodyA.GetTransformedVertices(),
                bodyB.GetTransformedVertices()
            );
        }
        else if (bodyB.shapeType == ShapeType::Circle)
        {
            return Collision::IsPolygonCircleIntersect(
                bodyA.GetTransformedVertices(),
                bodyB.Position,
                bodyB.Radius
            );
        }
    }

    Collision::CollisionResult result;
    return result;
}

void ResolveCollision(RigidBody & bodyA , RigidBody & bodyB , Vec2 normal)
{
    Vec2 relativeVelocity = bodyB.LinearVelocity - bodyA.LinearVelocity;

    if (math::Dot(normal , relativeVelocity) > 0.0)
        return;

    float minRestitution = min(bodyA.Restitution , bodyB.Restitution);
    // impulse = -(1 + minRestitution) * Dot(relativeVelocity , nomal) / 1/massA + 1/massB
    float impulse = -(1 + minRestitution) * math::Dot(relativeVelocity , normal) / (bodyA.InvMass + bodyB.InvMass);
    bodyA.LinearVelocity -= normal * (impulse * bodyA.InvMass);
    bodyB.LinearVelocity += normal * (impulse * bodyB.InvMass);
}

void DetectCircleCollision(vector <RigidBody> & bodysVector)
{
    for (int i = 0 ; i < bodysVector.size() - 1 ; i++)
    {
        if (bodysVector[i].shapeType != ShapeType::Circle)
            continue;
        
        RigidBody & circleA = bodysVector[i];
        for (int j = i + 1 ; j < bodysVector.size() ; j++)
        {
            if(bodysVector[j].shapeType != ShapeType::Circle)
                continue;

            RigidBody & circleB = bodysVector[j];
            Collision::CollisionResult result = Collision::IsIntersectCircle(circleA.Position , circleA.Radius , circleB.Position , circleB.Radius);
            if (result.IsIntersect)
            {
                circleA.MoveBy( result.NormalCollisionDirection * (-result.Depth * 0.5f));
                circleB.MoveBy( result.NormalCollisionDirection * (result.Depth * 0.5f));
                ResolveCollision(circleA , circleB , result.NormalCollisionDirection);
            }
        }
    }
}

void DetectPolygonCollision(vector <RigidBody> & bodysVector)
{
    for (int i = 0 ; i < bodysVector.size() - 1 ; i++)
    {
        if (bodysVector[i].shapeType != ShapeType::Box)
            continue;

        vector<Vec2> verticesA = bodysVector[i].GetTransformedVertices();

        for (int j = i + 1 ; j < bodysVector.size() ; j++)
        {
            if(bodysVector[j].shapeType != ShapeType::Box)
                continue;

            vector<Vec2> verticesB = bodysVector[j].GetTransformedVertices();
            Collision::CollisionResult result = Collision::IsPolygonSIntersect(verticesA , verticesB);
            if (result.IsIntersect)
            {
                bodysVector[i].MoveBy(result.NormalCollisionDirection * (-result.Depth * 0.5f) );
                bodysVector[j].MoveBy(result.NormalCollisionDirection * (result.Depth * 0.5f) );
                ResolveCollision(bodysVector[i] , bodysVector[j] , result.NormalCollisionDirection);
            }
        }
    }
}

void DetectPolygonCircleCollision(vector <RigidBody> & bodysVector)
{
    for (int i = 0 ; i < bodysVector.size() - 1 ; i++)
    {
        RigidBody& bodyA = bodysVector[i];

        for (int j = i + 1 ; j < bodysVector.size() ; j++)
        {
            RigidBody& bodyB = bodysVector[j];

            if(bodyA.shapeType == ShapeType::Box && bodyB.shapeType == ShapeType::Circle)
            {
                auto verticesA = bodyA.GetTransformedVertices();
                Collision::CollisionResult result = Collision::IsPolygonCircleIntersect(verticesA , bodyB.Position , bodyB.Radius);
                if (result.IsIntersect)
                {
                    bodysVector[i].MoveBy(result.NormalCollisionDirection * (result.Depth * 0.5f) );
                    bodysVector[j].MoveBy(result.NormalCollisionDirection * (-result.Depth * 0.5f) );
                    ResolveCollision(bodyB, bodyA , result.NormalCollisionDirection);
                }
            }
            else if (bodyA.shapeType == ShapeType::Circle && bodyB.shapeType == ShapeType::Box)
            {
                auto verticesB = bodyB.GetTransformedVertices();
                Collision::CollisionResult result = Collision::IsPolygonCircleIntersect(verticesB , bodyA.Position , bodyA.Radius);
                if (result.IsIntersect)
                {
                    bodysVector[i].MoveBy(result.NormalCollisionDirection * (-result.Depth * 0.5f) );
                    bodysVector[j].MoveBy(result.NormalCollisionDirection * (result.Depth * 0.5f) );
                    ResolveCollision(bodyA, bodyB , result.NormalCollisionDirection);
                }
            }
        }
    }
}

void UpdateBodyPhysics(vector<RigidBody> & bodysVector)
{
    for(auto & body : bodysVector)
    {
        body.UpdatePhysics();
    }
}

void UpdatePhysics(vector<RigidBody> & bodysVector , Camera2D & camera)
{
    BounceScreen(bodysVector , camera);
    // update movement
    UpdateBodyPhysics(bodysVector);

    // update collisions
    for (int i = 0; i < bodysVector.size() - 1; i++)
    {
        RigidBody& bodyA = bodysVector[i];

        for (int j = i + 1; j < bodysVector.size(); j++)
        {
            RigidBody& bodyB = bodysVector[j];

            Collision::CollisionResult result =
                Collide(bodyA, bodyB);

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

            ResolveCollision(
                bodyA,
                bodyB,
                result.NormalCollisionDirection
            );
        }
    }


    // DetectPolygonCollision(bodysVector);

    // DetectCircleCollision(bodysVector);

    // DetectPolygonCircleCollision(bodysVector);
}

void UpdateControls(Camera2D& camera , vector<RigidBody> & bodysVector)
{
    float dx = 0.0f;
    float dy = 0.0f;
    float forceMagnitude = 1800000.0f;
    float rotation = constants::pi * 0.8f* GetFrameTime();
    
    int bodyIndex = -1;
    for (int i = 0 ; i < bodysVector.size() ; i++)
    {
        if (bodysVector[i].IsStatic)
            continue;
        else
        {
            bodyIndex = i;
            break;
        }
    }

    if (IsKeyDown(KEY_RIGHT))
    {
        dx += 2;
    }

    if (IsKeyDown(KEY_LEFT))
    {
        dx -= 2;
    }

    if (IsKeyDown(KEY_UP))
    {
        dy -= 2;
    }

    if (IsKeyDown(KEY_DOWN))
    {
        dy += 2;
    }

    if (IsKeyPressed(KEY_X))
    {
        camera.zoom += 0.2f;
    }

    if (IsKeyPressed(KEY_Z))
    {
        camera.zoom -= 0.2f;
    }
    if (IsKeyDown(KEY_A))
    {
        bodysVector[bodyIndex].RotateBy(-rotation);
    }
    if (IsKeyDown(KEY_D))
    {
        bodysVector[bodyIndex].RotateBy(rotation);
    }
    // if (IsKeyPressed(KEY_SPACE))
    // {
    //     forceMagnitude = 0;
    //     cout<<forceMagnitude<<endl;
    // }

    camera.zoom = clamp(camera.zoom, 0.1f, 10.0f);

    if (dx != 0 || dy != 0)
    {
        Vec2 input = {dx , dy};
        Vec2 forceDirection = input.Normalize();
        Vec2 force = forceDirection * forceMagnitude;

        bodysVector[bodyIndex].AddForce(force);
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
        0.2f,             
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

    vector <RigidBody> Bodies = InitializeWorld();

    Camera2D camera = {0};

    camera.target = {640, 360};
    camera.offset = {640.0f, 360.0f};

    camera.rotation = 0.0f;
    camera.zoom = 1.0f;

    while(!WindowShouldClose())
    {
        UpdatePhysics(Bodies , camera);

        UpdateControls(camera , Bodies);

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