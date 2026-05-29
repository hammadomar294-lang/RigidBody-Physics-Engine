#include "transform.h"

Transform Transform::Zero = Transform(0.0f , 0.0f , 0.0f);

Transform::Transform(Vec2 v, float angle)
{
    this->PositionX = v.X;
    this->PositionY = v.Y;
    this->Sin = sin(angle);
    this->cos = cos(angle);
}

Transform::Transform(float x, float y, float angle)
{
    this->PositionX = x;
    this->PositionY = y;
    this->Sin = sin(angle);
    this->cos = cos(angle);
}

Vec2 Transform::TransformPoint(const Vec2 &original, Transform transform)
{
    return Vec2{
                    original.X * transform.cos - original.Y * transform.Sin + transform.PositionX 
                    , original.X * transform.sin + original.Y * transform.cos + transform.PositionY
               };
}
