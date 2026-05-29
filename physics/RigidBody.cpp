#include "RigidBody.h"


RigidBody::RigidBody(const Vec2 & position , float mass , float density , float restitution , float area ,
                              bool isStatic , ShapeType shapetype , Color randomColor  , float radius , float width , float height ) : Position(position) , LinearVelocity(0,0)
{
    
    this->Mass = mass;
    this->Density = density;
    this->Restitution = restitution;
    this->IsStatic = isStatic;

    this->Area = area;
    this->Radius = radius;
    this->Width = width;
    this->Height = height;

    this->shapeType = shapetype;
    this->BodyColor = randomColor;

    this->Rotation = 0;
    this->RotationVelocity = 0;
}

RigidBody RigidBody::CreateCircle(const Vec2 & position , float density , float restitution , bool isStatic , float radius )
{
    Color randomColor =
    {
        (unsigned char)(rand() % 256),
        (unsigned char)(rand() % 256),
        (unsigned char)(rand() % 256),
        255
    };

    density = clamp(density , world::MinDensity , world::MaxDensity);
    restitution = clamp(restitution , 0.0f , 1.0f);

    float area = radius * radius * world::pi;
    area = clamp(area , world::MinSize , world::MaxSize);  

    float mass = area * density;

    return RigidBody(position , mass , density , restitution , area , isStatic , ShapeType::Circle , randomColor ,radius , 0.0f , 0.0f);
}

RigidBody RigidBody::CreateBox(const Vec2 &position, float density, float restitution, bool isStatic ,float width, float height)
{
    Color randomColor =
    {
        (unsigned char)(rand() % 256),
        (unsigned char)(rand() % 256),
        (unsigned char)(rand() % 256),
        255
    };

    density = clamp(density , world::MinDensity , world::MaxDensity);
    restitution = clamp(restitution , 0.0f , 1.0f);

    float area = width * height;
    area = clamp(area , world::MinSize , world::MaxSize);  

    float mass = area * density;

    return RigidBody(position , mass , density , restitution , area , isStatic , ShapeType::Box , randomColor ,0.0f ,width , height);
}

void RigidBody::MoveBy(Vec2 amount)
{
   Position = Position + amount;
}

void RigidBody::MoveTo(Vec2 position)
{
    Position = position;
}
