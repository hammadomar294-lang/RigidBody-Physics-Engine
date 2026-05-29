#ifndef WORLD_H
#define WORLD_H

class world
{
public:

    static constexpr float MinSize = 0.01f * 0.01f;
    static constexpr float MaxSize = 64.0f * 64.0f;

    static constexpr float MinDensity = 0.2f; // g/cm^3
    static constexpr float MaxDensity = 22.6f; // g/cm^3

    static constexpr float MinMass = 1.0f; //g
    static constexpr float MaxMass = 1000.0 * 10000.0f; 

    static constexpr float pi = 3.14159265359f;
};

#endif