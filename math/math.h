#ifndef MATH_H
#define MATH_H

#include "vector2.h"
#include <cmath>

class math
{
public:
    static float Distance(const Vec2 & v1 , const Vec2 & v2) ;
    static float Dot(const Vec2 & v1 , const Vec2 & v2) ;
    static float Cross(const Vec2 & v1 , const Vec2 & v2) ;

    static bool NearlyEqual(float a , float b);
    static bool NearlyEqual(const Vec2 & a , const Vec2 & b);

    
};

#endif