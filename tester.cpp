#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>

#include "raylib.h"

#include "physics/RigidBody.h"
#include "math/math.h"
#include "physics/collision/Collision.h"
using namespace std;

// --------------------------------------------------
// CAMERA
// --------------------------------------------------

void UpdateCamera(Camera2D& camera , vector<RigidBody> & bodysVector)
{
    float dx = 0.0f;
    float dy = 0.0f;
    float speed = 450.0f;

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

    if (IsKeyPressed(KEY_A))
    {
        camera.zoom += 0.2f;
    }

    if (IsKeyPressed(KEY_Z))
    {
        camera.zoom -= 0.2f;
    }

    camera.zoom = clamp(camera.zoom, 0.1f, 10.0f);

    if (dx != 0 || dy != 0)
    {
        Vec2 input = {dx , dy};
        Vec2 direction = input.Normalize();
        Vec2 displacement = direction * speed * GetFrameTime();

        bodysVector[1].MoveBy(displacement);
    }

    for (int i = 0 ; i < bodysVector.size() - 1 ; i++)
    {
        if (bodysVector[i].shapeType == ShapeType::Box)
        {
            bodysVector[i].RotateBy(world::pi * 0.5f * GetFrameTime());
        }
    }

    for (int i = 0 ; i < bodysVector.size() - 1 ; i++)
    {
        RigidBody & circleA = bodysVector[i];
        for (int j = i + 1 ; j < bodysVector.size() ; j++)
        {
            RigidBody & circleB = bodysVector[j];
            Collision::CollisionResult result = Collision::IsIntersectCircle(circleA.Position , circleA.Radius , circleB.Position , circleB.Radius);
            if (result.IsIntersect)
            {
                circleA.MoveBy( result.NormalCollisionDirection * (result.Depth * 0.5f));
                circleB.MoveBy( result.NormalCollisionDirection * (-result.Depth * 0.5f));
                // cout<<result.Depth<<endl;
            }
        }
    }
}

// --------------------------------------------------
// MAKING BODYS
// --------------------------------------------------

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

// --------------------------------------------------
// DRAWING
// --------------------------------------------------

void DrawBodies(const vector<RigidBody>& Bodies)
{
    
    for(RigidBody body : Bodies)
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

// --------------------------------------------------
// MAIN
// --------------------------------------------------

int main()
{
    InitWindow(1280, 720, "Physics Engine");

    SetTargetFPS(60);

    srand(time(nullptr));

    vector<RigidBody> Bodies = MakeBodies(10);

    // -------- CREATE RANDOM BODIES --------
    // your body creation code here

    Camera2D camera = {0};

    camera.target = {640, 360};
    camera.offset = {640.0f, 360.0f};

    camera.rotation = 0.0f;
    camera.zoom = 1.0f;

    // --------------------------------------------------
    // GAME LOOP
    // --------------------------------------------------

    while(!WindowShouldClose())
    {
        UpdateCamera(camera , Bodies);

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