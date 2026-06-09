#ifndef WORLD_H
#define WORLD_H

#include "../math/vector2.h"
#include "RigidBody.h"
#include "collision/Collision.h"

#include <vector>
#include <cstdlib>
#include <ctime>

#include "raylib.h"

using namespace std;
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
    static Vec2 gravity = {0.0f ,9.81f};

    vector<RigidBody> bodysVector;

    void AddBody(const RigidBody & body);
    bool RemoveBody(RigidBody & body);
    int IsBodyExist(const RigidBody & body) const;
    RigidBody GetBody(int index) const;


    void UpdateWorld();
   
};

#endif