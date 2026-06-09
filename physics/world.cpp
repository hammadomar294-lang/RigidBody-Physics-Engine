#include <world.h>

void world::AddBody(const RigidBody &body)
{
    bodysVector.push_back(body);
}

bool world::RemoveBody(RigidBody& body)
{
    int index = IsBodyExist(body);
    if(index == -1) 
        return false;

    bodysVector.erase(Bodies.begin() + index);

    return true;
}

int world::IsBodyExist(const RigidBody &body) const
{
    for (int i = 0; i < bodysVector.size(); i++)
    {
        if (&bodysVector[i] == &body)
            return i;
    }
    return -1;
}

RigidBody world::GetBody(int index) const
{
    return bodysVector[index];
}

void ResetAllToGray(vector<RigidBody> & bodysVector)
{
    for(RigidBody& body : bodysVector)
    {
        body.BodyColor = GRAY;
    }
}

Collision::CollisionResult Collide(RigidBody bodyA , RigidBody bodyB)
{
    if(bodyA.shapeType == ShapeType::Box)
    {
        vector<Vec2> verticesA = bodyA.GetTransformedVertices();

        if (bodyB.shapeType == ShapeType::Box)
        {
            vector<Vec2> verticesB = bodyB.GetTransformedVertices();
            Collision::CollisionResult result = Collision::IsPolygonSIntersect(verticesA , verticesB);
            return result;
        }
        else if(bodyB.shapeType == ShapeType::Circle)
        {
            Collision::CollisionResult result = Collision::IsPolygonCircleIntersect(verticesA , bodyB.Position , bodyB.Radius);
            return result;
        }
    }
    else if (bodyA.shapeType == ShapeType::Circle)
    {
        if (bodyB.shapeType == ShapeType::Box)
        {
            auto verticesB = bodyB.GetTransformedVertices();
            Collision::CollisionResult result = Collision::IsPolygonCircleIntersect(verticesB , bodyA.Position , bodyA.Radius);
            return result;
        }
        else if(bodyB.shapeType == ShapeType::Circle)
        {
            Collision::CollisionResult result = Collision::IsIntersectCircle(bodyA.Position , bodyA.Radius , bodyB.Position , bodyB.Radius);
            return result;
        }
    }
}

void world::UpdateWorld()
{
    // updates movements
    for(auto & body : bodysVector)
    {
        body.UpdatePhysics();
    }

    // updates collisions
    ResetAllToGray(bodysVector);
    for (int i = 0 ; i < bodysVector.size() - 1 ; i++)
    {
        RigidBody bodyA = bodysVector[i];
        for (int j = 0 ; j < bodysVector.size() - 1 ; j++)
        {
            RigidBody bodyB = bodysVector[j];
            Collision::CollisionResult result = Collide(bodyA,bodyB);
            if (result.IsIntersect)
            {
                bodyA.MoveBy(result.NormalCollisionDirection * -result.Depth * 0.5f);
                bodyB.MoveBy(result.NormalCollisionDirection * result.Depth * 0.5f);

                bodyA.BodyColor = RED;
                bodyB.BodyColor = RED;
            }
        }
    }
}
