#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>

#include "raylib.h"

#include "physics/RigidBody.h"
#include "math/math.h"
#include "physics/collision/Collision.h"

#include "physics/world.h"
using namespace std;

void ResetAllToGray(vector<RigidBody> & bodysVector)
{
    for(RigidBody& body : bodysVector)
    {
        body.BodyColor = GRAY;
    }
}

// --------------------------------------------------
// CAMERA
// --------------------------------------------------

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
                circleA.MoveBy( result.NormalCollisionDirection * (result.Depth * 0.5f));
                circleB.MoveBy( result.NormalCollisionDirection * (-result.Depth * 0.5f));
                circleA.BodyColor = RED;
                circleB.BodyColor = RED;
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

                bodysVector[i].BodyColor = RED;
                bodysVector[j].BodyColor = RED;
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

                    bodysVector[i].BodyColor = RED;
                    bodysVector[j].BodyColor = RED;
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

                    bodysVector[i].BodyColor = RED;
                    bodysVector[j].BodyColor = RED;
                }
            }
        }
    }
}

void UpdatePhysics(vector<RigidBody> & bodysVector)
{
    ResetAllToGray(bodysVector);
    DetectPolygonCollision(bodysVector);
    DetectCircleCollision(bodysVector);
    DetectPolygonCircleCollision(bodysVector);
}

void UpdateControls(Camera2D& camera , vector<RigidBody> & bodysVector)
{
    float dx = 0.0f;
    float dy = 0.0f;
    float speed = 450.0f;
    float rotation = world::pi * 0.8f* GetFrameTime();

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
        bodysVector[1].RotateBy(-rotation);
    }
    if (IsKeyDown(KEY_D))
    {
        bodysVector[1].RotateBy(rotation);
    }

    camera.zoom = clamp(camera.zoom, 0.1f, 10.0f);

    if (dx != 0 || dy != 0)
    {
        Vec2 input = {dx , dy};
        Vec2 direction = input.Normalize();
        Vec2 displacement = direction * speed * GetFrameTime();

        bodysVector[1].MoveBy(displacement);
    }
}

vector<RigidBody> MakeBodies(int num)
{
    vector<RigidBody> Bodies;
    int padding = 100;
    for(int i = 0; i < num; i++)
    {
        int type = rand() % 2;

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
                    1.0f,
                    0.5f,
                    false,
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
                    1.0f,
                    0.5f,
                    false,
                    width,
                    height
                )
            );
        }
    }

    return Bodies;
}

// void ApplyGravity(vector<RigidBody>& Bodies)
// {   
//     for(RigidBody &body : Bodies)
//     {
//         if (body.Position.Y + body.Radius < world::ground)
//         {
//             float depth = (body.Position.Y + body.Radius) -  world::ground;
//             body.Position.Y -=depth;
//         }
//         else
//         {
//             body.LinearVelocity.Y += 9.81f * GetFrameTime();
//             body.Position += body.LinearVelocity * GetFrameTime();
//         }
        
//     }
// }

void DrawBodies(vector<RigidBody>& Bodies)
{
    
    for(RigidBody &body : Bodies)
    {
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

int main()
{
    InitWindow(1280, 720, "Physics Engine");

    SetTargetFPS(60);

    srand(time(nullptr));

    world world1;

    vector<RigidBody> Bodies = MakeBodies(20);
    world1.bodysVector = Bodies;

    Camera2D camera = {0};

    camera.target = {640, 360};
    camera.offset = {640.0f, 360.0f};

    camera.rotation = 0.0f;
    camera.zoom = 1.0f;

    while(!WindowShouldClose())
    {

        UpdateControls(camera , Bodies);

        world1.UpdateWorld();

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