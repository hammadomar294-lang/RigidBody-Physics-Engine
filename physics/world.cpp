#include "world.h"

world::world()
{
    int num = 6;
    vector<RigidBody> temp = (num);
    this->bodysVector = temp;
}

void world::AddBody(const RigidBody &body)
{
    bodysVector.push_back(body);
}

bool world::RemoveBody(RigidBody& body)
{
    int index = IsBodyExist(body);
    if(index == -1) 
        return false;

    bodysVector.erase(bodysVector.begin() + index);

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

void world::ResetAllToGray()
{
    for(RigidBody& body : bodysVector)
    {
        body.BodyColor = GRAY;
    }
}

Collision::CollisionResult world::Collide(RigidBody &bodyA , RigidBody &bodyB)
{
    Collision::CollisionResult result;
    if(bodyA.shapeType == ShapeType::Box)
    {
        vector<Vec2> verticesA = bodyA.GetTransformedVertices();

        if (bodyB.shapeType == ShapeType::Box)
        {
            vector<Vec2> verticesB = bodyB.GetTransformedVertices();
            result = Collision::IsPolygonSIntersect(verticesA , verticesB);
           
            return result;
            
        }
        else if(bodyB.shapeType == ShapeType::Circle)
        {
            result = Collision::IsPolygonCircleIntersect(verticesA , bodyB.Position , bodyB.Radius);
            if (result.IsIntersect) cout<<" box circle true"<<endl;
            return result;
            
        }
    }
    else if (bodyA.shapeType == ShapeType::Circle)
    {
        if (bodyB.shapeType == ShapeType::Box)
        {
            auto verticesB = bodyB.GetTransformedVertices();
            result = Collision::IsPolygonCircleIntersect(verticesB , bodyA.Position , bodyA.Radius);
            if (result.IsIntersect) cout<<" circle box true"<<endl;
            return result;
        }
        else if(bodyB.shapeType == ShapeType::Circle)
        {
            result.IsIntersect = false;
            result = Collision::IsIntersectCircle(bodyA.Position , bodyA.Radius , bodyB.Position , bodyB.Radius);
            if (result.IsIntersect) cout<<" circle circle true"<<endl;
            return result;
            
        }
    }
    return result;
}

void world::UpdateWorld()
{
    // updates movements
    for(auto & body : bodysVector)
    {
        body.UpdatePhysics();
    }

    // updates collisions
    
    for (int i = 0; i < bodysVector.size() - 1; i++)
    {   
        RigidBody & bodyA = bodysVector[i];
        for (int j = i + 1; j < bodysVector.size(); j++)
        {
            RigidBody &bodyB = bodysVector[j];
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

vector<RigidBody> world::MakeBodies(int num)
{
    vector<RigidBody> Bodies;
    int padding = 100;
    for(int i = 0; i < num; i++)
    {
        int type = rand() % 2;

        Vec2 position(
            rand() % (1200 - padding) ,
            rand() % (700 - padding)
        );

        if(type == 0)
        {
            float radius = 10 + rand() % 40;

            Bodies.push_back(
                RigidBody::CreateCircle(
                    position,
                    1.0f,
                    0.5f,
                    false,
                    radius
                )
            );
        }
        else
        {
            float width  = 20 + rand() % 80;
            float height = 20 + rand() % 80;

            Bodies.push_back(
                RigidBody::CreateBox(
                    position,
                    1.0f,
                    0.5f,
                    false,
                    width,
                    height
                )
            );
        }
    }

    return Bodies;
}