#ifndef MATH_H
#define MATH_H

#include "vector2.h"

class math
{
public:
    static float Distance(const Vec2 & v1 , const Vec2 & v2) ;
    static float Dot(const Vec2 & v1 , const Vec2 & v2) ;
    static float Cross(const Vec2 & v1 , const Vec2 & v2) ;

    
};

#endif