#include "RigidBody.h"


RigidBody::RigidBody(const Vec2 & position , float mass , float density , float restitution , float area ,
                              bool isStatic , ShapeType shapetype , Color color  , float radius , float width , float height ) : Position(position) , LinearVelocity(0,0)
{
    this->LinearVelocity = {0,0};
    this->Mass = mass;
    this->Density = density;
    this->Restitution = restitution;
    this->IsStatic = isStatic;

    this->Area = area;
    this->Radius = radius;
    this->Width = width;
    this->Height = height;

    this->shapeType = shapetype;
    this->BodyColor = color;

    this->Rotation = 0;
    this->RotationVelocity = 0; 
    if(shapeType == ShapeType::Box)
    {
        this->Vertices = MakeBoxVertices(width , height);
        this->Triangles = TriangulateBox();
    }
    this->VerticesNeedsUpdate = true;
}

vector<Vec2> RigidBody::MakeBoxVertices(float width, float height)
{
    float left = -width*0.5f;
    float right = width*0.5f;
    float top = height*0.5f;
    float bottom = -height*0.5f;

    vector<Vec2> vertices(4); 
    // we start from top left and clockwise
    vertices[0] = {left,top};
    vertices[1] = {right,top};
    vertices[2] = {right,bottom};
    vertices[3] = {left,bottom};

    return vertices;
}

vector<int> RigidBody::TriangulateBox() // we start from top left and clockwise
{
    vector<int> triangleIndices(6);
    triangleIndices[0] = 0;
    triangleIndices[1] = 1;
    triangleIndices[2] = 2;
    triangleIndices[3] = 0;
    triangleIndices[4] = 2;
    triangleIndices[5] = 3;
    return triangleIndices;
}

RigidBody RigidBody::CreateCircle(const Vec2 & position , float density , float restitution , bool isStatic , float radius )
{
    Color color = GRAY;
    
    density = clamp(density , world::MinDensity , world::MaxDensity);
    restitution = clamp(restitution , 0.0f , 1.0f);

    float area = radius * radius * world::pi;
    area = clamp(area , world::MinSize , world::MaxSize);  

    float mass = area * density;

    return RigidBody(position , mass , density , restitution , area , isStatic , ShapeType::Circle , color ,radius , 0.0f , 0.0f);
}

RigidBody RigidBody::CreateBox(const Vec2 &position, float density, float restitution, bool isStatic ,float width, float height)
{
    Color color = GRAY;

    density = clamp(density , world::MinDensity , world::MaxDensity);
    restitution = clamp(restitution , 0.0f , 1.0f);

    float area = width * height;
    area = clamp(area , world::MinSize , world::MaxSize);  

    float mass = area * density;

    return RigidBody(position , mass , density , restitution , area , isStatic , ShapeType::Box , color ,0.0f ,width , height);
}

vector<Vec2> RigidBody::GetTransformedVertices()
{
    if(VerticesNeedsUpdate)
    {
        vector<Vec2> tempVertices(Vertices.size());
        Transform2D trans(Position,Rotation);

        for (int i = 0 ; i < Vertices.size() ; i++)
        {
            Vec2 original = Vertices[i];
            tempVertices[i] = trans.transformPoint(original);
        }
        TransformedVertices = tempVertices;
    }

    VerticesNeedsUpdate = false;
    return TransformedVertices;
    
}

void RigidBody::MoveBy(Vec2 amount)
{
   Position = Position + amount;
   VerticesNeedsUpdate = true;
}

void RigidBody::MoveTo(Vec2 position)
{
    Position = position;
    VerticesNeedsUpdate = true; 
}

void RigidBody::RotateBy(float amount)
{
    Rotation += amount;
    VerticesNeedsUpdate = true; 
}

void RigidBody::UpdatePhysics()
{
    Position += LinearVelocity * GetFrameTime();
    Rotation += RotationVelocity * GetFrameTime();
}
