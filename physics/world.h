#ifndef WORLD_H
#define WORLD_H

#include "../math/vector2.h"
#include "collision/Collision.h"
#include "RigidBody.h"

#include <vector>
#include <cstdlib>
#include <ctime>

#include "raylib.h"

using namespace std;
class world
{
public:

    world();

    vector<RigidBody> bodysVector;

    void AddBody(const RigidBody & body);
    bool RemoveBody(RigidBody & body);
    int IsBodyExist(const RigidBody & body) const;
    RigidBody GetBody(int index) const;

    void ResetAllToGray();

    Collision::CollisionResult Collide(RigidBody &bodyA , RigidBody &bodyB);

    vector<RigidBody> MakeBodies(int num);

    void UpdateWorld();
   
};

#endif