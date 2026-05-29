#ifndef VEC2_H
#define VEC2_H

#include <cmath>
using namespace std;

class Vec2
{
public:
    float X;
    float Y;

    Vec2(float x , float y);

    Vec2 operator+(const Vec2 & other) const;
    Vec2 operator-(const Vec2 & other) const;
    Vec2 operator+=(const Vec2 & other) const;
    Vec2 operator-=(const Vec2 & other) const;
    Vec2 operator*(float scaler) const;
    Vec2 operator/(float scaler) const;

    bool operator==(const Vec2 & other) const;
    bool operator!=(const Vec2 & other) const;

    float Length() const;

    Vec2 Normalize() const;
    


    
};

#endif