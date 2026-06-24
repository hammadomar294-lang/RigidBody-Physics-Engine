#include "RigidBody.h"


RigidBody::RigidBody(const Vec2 & position , float mass , float density , float restitution , float area ,
                              bool isStatic , ShapeType shapetype , Color color  , float radius , float width , float height )
                               : Position(position) , LinearVelocity(0,0) , Force(0.0f,0.0f)
{
    
    Mass = mass;
    Density = density;
    Restitution = restitution;
    IsStatic = isStatic;

    Area = area;
    Radius = radius;
    Width = width;
    Height = height;

    shapeType = shapetype;
    BodyColor = color;
    DisplayColor = color;

    Rotation = 0;
    RotationVelocity = 0; 

    IsHeld = false;

    if(shapeType == ShapeType::Box)
    {
        Vertices = MakeBoxVertices(width , height);
        Triangles = TriangulateBox();
        VerticesNeedsUpdate = true;
        Inertia = (1.0f / 12.0f) * mass * (height * height + width * width);
    }
    else if (shapeType == ShapeType::Circle)
    {
        localStart = {0.0f, 0.0f};
        localEnd = {radius, 0.0f};
        
        Inertia = 0.5f * mass * radius * radius;
    }

    if (!IsStatic)
    {
        InvMass = 1.0/mass;
        InvInertia = 1.0/Inertia;
    }
        
    else
    {
        InvMass = 0.0;
        InvInertia = 0.0;
    }
    
    AABBNeedUpdate = true;
    aabb = GetAABB();

    DynamicFriction = 0.1f + static_cast<float>(rand()) / RAND_MAX * 0.8f;

    StaticFriction = DynamicFriction + static_cast<float>(rand()) / RAND_MAX * (1.0f - DynamicFriction);

    IsSleeping = false;
    SleepTimer = 0.0f;
    
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

AABB RigidBody::GetAABB()
{
    float minX = numeric_limits<float>::max();
    float minY = numeric_limits<float>::max();
    float maxX = numeric_limits<float>::lowest();
    float maxY = numeric_limits<float>::lowest();

    if (shapeType == ShapeType::Circle)
    {
        minX = Position.X - Radius;
        minY = Position.Y - Radius;
        maxX = Position.X + Radius;
        maxY = Position.Y + Radius;
    }
    else
    {
        vector<Vec2> tempVertices = GetTransformedVertices();
        for (const auto & vertex : tempVertices)
        {
            if (vertex.X < minX) minX = vertex.X;
            if (vertex.X > maxX) maxX = vertex.X;
            if (vertex.Y < minY) minY = vertex.Y;
            if (vertex.Y > maxY) maxY = vertex.Y;
        }
    }

    AABBNeedUpdate = false;
    return AABB(minX , minY , maxX , maxY);
}

RigidBody RigidBody::CreateCircle(const Vec2 & position , float density , float restitution , bool isStatic , float radius )
{
    Color color = {
    (unsigned char)(rand() % 256), 
    (unsigned char)(rand() % 256), 
    (unsigned char)(rand() % 256), 
    255                            
    };
    
    density = clamp(density , constants::MinDensity , constants::MaxDensity);
    restitution = clamp(restitution , 0.0f , 1.0f);

    float area = radius * radius * constants::pi;
    area = clamp(area , constants::MinSize , constants::MaxSize);  

    float mass = area * density;

    return RigidBody(position , mass , density , restitution , area , isStatic , ShapeType::Circle , color ,radius , 0.0f , 0.0f);
}

RigidBody RigidBody::CreateBox(const Vec2 &position, float density, float restitution, bool isStatic ,float width, float height)
{
    Color color = {
    (unsigned char)(rand() % 256), 
    (unsigned char)(rand() % 256), 
    (unsigned char)(rand() % 256), 
    255                            
};

    density = clamp(density , constants::MinDensity , constants::MaxDensity);
    restitution = clamp(restitution , 0.0f , 1.0f);

    float area = width * height;
    area = clamp(area , constants::MinSize , constants::MaxSize);  

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
   AABBNeedUpdate = true;
}

void RigidBody::MoveTo(Vec2 position)
{
    Position = position;
    VerticesNeedsUpdate = true; 
    AABBNeedUpdate = true;
}

void RigidBody::RotateBy(float amount)
{
    Rotation += amount;
    VerticesNeedsUpdate = true; 
    AABBNeedUpdate = true;
}

void RigidBody::UpdatePhysics(int iterations)
{
    if (shapeType == ShapeType::Box)
    {
        VerticesNeedsUpdate = true;
        AABBNeedUpdate = true;
    }
    if (IsStatic)
    {   RotationVelocity = 0.0f;
        LinearVelocity = {0.0,0.0};
        IsSleeping = false;
        return;
    }
    if (IsSleeping)
    {
        
        LinearVelocity = {0.0,0.0};
        RotationVelocity = 0.0;
        return;
    }
    else 
    {
        Vec2 acceleration = constants::gravity +  Force * InvMass;

        if (IsHeld)
        {
            acceleration = Force * InvMass;
        }
        LinearVelocity += acceleration * constants::Physics_dt / iterations;

        Position += LinearVelocity * constants::Physics_dt / iterations;

        Rotation += RotationVelocity * constants::Physics_dt / iterations;

        if (LinearVelocity.LengthSq() <= constants::SLEEP_LINEAR_THRESHOLD_SQ && abs(RotationVelocity) <= constants::SLEEP_ROTATION_THRESHOLD)
        {
            
            SleepTimer += (constants::Physics_dt / iterations);
            if (SleepTimer >= constants::TIME_TO_SLEEP)
            {
                LinearVelocity = {0.0,0.0};
                RotationVelocity = 0.0;
                IsSleeping = true;
            }
        }
        else 
        {
            SleepTimer = 0.0;
            IsSleeping = false;
        }
    }
   
    
}

void RigidBody::AddForce(const Vec2 &amount)
{
    Force += amount;
}
