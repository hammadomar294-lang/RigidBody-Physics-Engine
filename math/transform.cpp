#include "transform.h"

Transform2D Transform2D::Zero = Transform2D(0.0f , 0.0f , 0.0f);

Transform2D::Transform2D(Vec2 v, float angle)
{
    this->PositionX = v.X;
    this->PositionY = v.Y;
    this->Sin = sin(angle);
    this->Cos = cos(angle);
}

Transform2D::Transform2D(float x, float y, float angle)
{
    this->PositionX = x;
    this->PositionY = y;
    this->Sin = sin(angle);
    this->Cos = cos(angle);
}

Vec2 Transform2D::transformPoint(const Vec2 &original)
{
    return Vec2{
                    original.X * Cos - original.Y * Sin + PositionX 
                    , original.X * Sin + original.Y * Cos + PositionY
               };
}
