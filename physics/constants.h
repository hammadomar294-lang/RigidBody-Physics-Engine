#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "../math/vector2.h"
class constants
{

public:
    static constexpr float MinSize = 0.01f * 0.01f;
    static constexpr float MaxSize = 64.0f * 64.0f;

    static constexpr float MinDensity = 0.2f; // g/cm^3
    static constexpr float MaxDensity = 22.6f; // g/cm^3

    static constexpr float MinMass = 1.0f; //g
    static constexpr float MaxMass = 1000.0 * 10000.0f; 

    static constexpr int MinSteps = 1;
    static constexpr int MaxSteps = 64;

    static constexpr float pi = 3.14159265359f;
    inline static Vec2 gravity = {0.0f ,500.0f};
    static constexpr float Physics_dt = 1.0f / 60.0f; 

    static constexpr float SLEEP_LINEAR_THRESHOLD_SQ = 2.0f * 2.0f;  
    static constexpr float SLEEP_ROTATION_THRESHOLD = 2.0f; 
    static constexpr float TIME_TO_SLEEP = 0.5f;    
    
    static constexpr float MaxDragVelocity = 20.0f;  

    static constexpr float MaxShootingForce = 1000.0f;          
    static constexpr float MinShootingForce = 1.0f;          
};

#endif
