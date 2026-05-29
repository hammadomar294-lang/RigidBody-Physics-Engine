#include "vector2.h"

Vec2::Vec2(float x, float y)
{
    this->X = x;
    this->Y = y;
}
Vec2 Vec2::operator+(const Vec2 &other) const
{
    return {X + other.X , Y + other.Y};
}

Vec2 Vec2::operator-(const Vec2 &other) const
{
    return {X - other.X , Y - other.Y};
}

Vec2 Vec2::operator*(float scaler) const
{
    return {X * scaler , Y * scaler};
}

Vec2 Vec2::operator/(float scaler) const
{
    return {X / scaler , Y / scaler};
}

bool Vec2::operator==(const Vec2 &other) const
{
    float epsilon = 0.0001f;
    return std::abs(X - other.X) < epsilon && std::abs(Y - other.Y) < epsilon;
}

bool Vec2::operator!=(const Vec2 &other) const
{
    return !(*this == other);
}

float Vec2::Length() const
{
    return sqrt( X * X + Y * Y);
}

Vec2 Vec2::Normalize() const
{
    float length = Length();
    if (length == 0) return {0,0};

    return {X/length , Y/length};
}
