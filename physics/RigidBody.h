#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <numbers>
#include <algorithm>
#include <raylib.h>
#include <vector>
#include <iostream>

#include "../math/math.h"
#include "../math/vector2.h"
#include "../math/transform.h"

#include "constants.h"

using namespace std;

enum ShapeType
{
    Circle = 0 , Box = 1
};

class RigidBody
{
    public:
    Vec2 Position;
    // Velocity Rotation RotationVelocity will be set to 0 cus we want them to be created in one state to velocity and no rotation
    Vec2 LinearVelocity;
    float Rotation; // the angle from the positive x 
    float RotationVelocity;

    Vec2 Force;
    float Mass;
    float InvMass;
    float Density;
    float Restitution; // bouncy or not 
    float Area; // will not be passed but calculated

    bool IsStatic; // if it can move or not

    float Radius;
    float Width;
    float Height;
    
    ShapeType shapeType;

    Color BodyColor;
    

    static RigidBody CreateCircle(const Vec2 & position , float density , float restitution , bool isStatic , float radius =1);

    static RigidBody CreateBox(const Vec2 & position , float density , float restitution , bool isStatic , float width, float height);

    

    void MoveBy(Vec2 amount);
    void MoveTo(Vec2 position);
    void RotateBy(float amount);

    void UpdatePhysics();

    void AddForce(const Vec2 &amount);
    void ApplyImpulse(const Vec2 &impulse);

    RigidBody(const Vec2 & position , float mass , float density , float restitution , float area ,
                              bool isStatic , ShapeType shapetype , Color color , float radius =1, float width =1, float height =1);
    
    vector<Vec2> Vertices;
    vector<Vec2> TransformedVertices;
    vector<int> Triangles; // holds the indices which means this vertex is number 1 in Arrangement of the vertices
                        // which mean starting from top left and moving clockwise

    static vector<Vec2> MakeBoxVertices(float width , float height);
    vector<Vec2> GetTransformedVertices();
    bool VerticesNeedsUpdate;
    static vector<int> TriangulateBox();

};

#endif