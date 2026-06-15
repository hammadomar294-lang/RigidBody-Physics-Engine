void DetectCircleCollision(vector <RigidBody> & Bodies)
{
    for (int i = 0 ; i < Bodies.size() - 1 ; i++)
    {
        if (Bodies[i].shapeType != ShapeType::Circle)
            continue;
        
        RigidBody & circleA = Bodies[i];
        for (int j = i + 1 ; j < Bodies.size() ; j++)
        {
            if(Bodies[j].shapeType != ShapeType::Circle)
                continue;

            RigidBody & circleB = Bodies[j];
            Collision::CollisionResult result = Collision::IsIntersectCircle(circleA.Position , circleA.Radius , circleB.Position , circleB.Radius);
            if (result.IsIntersect)
            {
                circleA.MoveBy( result.NormalCollisionDirection * (-result.Depth * 0.5f));
                circleB.MoveBy( result.NormalCollisionDirection * (result.Depth * 0.5f));
                ResolveCollision(circleA , circleB , result.NormalCollisionDirection);
            }
        }
    }
}

void DetectPolygonCollision(vector <RigidBody> & Bodies)
{
    for (int i = 0 ; i < Bodies.size() - 1 ; i++)
    {
        if (Bodies[i].shapeType != ShapeType::Box)
            continue;

        vector<Vec2> verticesA = Bodies[i].GetTransformedVertices();

        for (int j = i + 1 ; j < Bodies.size() ; j++)
        {
            if(Bodies[j].shapeType != ShapeType::Box)
                continue;

            vector<Vec2> verticesB = Bodies[j].GetTransformedVertices();
            Collision::CollisionResult result = Collision::IsPolygonSIntersect(verticesA , verticesB);
            if (result.IsIntersect)
            {
                Bodies[i].MoveBy(result.NormalCollisionDirection * (-result.Depth * 0.5f) );
                Bodies[j].MoveBy(result.NormalCollisionDirection * (result.Depth * 0.5f) );
                ResolveCollision(Bodies[i] , Bodies[j] , result.NormalCollisionDirection);
            }
        }
    }
}

void DetectPolygonCircleCollision(vector <RigidBody> & Bodies)
{
    for (int i = 0 ; i < Bodies.size() - 1 ; i++)
    {
        RigidBody& bodyA = Bodies[i];

        for (int j = i + 1 ; j < Bodies.size() ; j++)
        {
            RigidBody& bodyB = Bodies[j];

            if(bodyA.shapeType == ShapeType::Box && bodyB.shapeType == ShapeType::Circle)
            {
                auto verticesA = bodyA.GetTransformedVertices();
                Collision::CollisionResult result = Collision::IsPolygonCircleIntersect(verticesA , bodyB.Position , bodyB.Radius);
                if (result.IsIntersect)
                {
                    Bodies[i].MoveBy(result.NormalCollisionDirection * (result.Depth * 0.5f) );
                    Bodies[j].MoveBy(result.NormalCollisionDirection * (-result.Depth * 0.5f) );
                    ResolveCollision(bodyB, bodyA , result.NormalCollisionDirection);
                }
            }
            else if (bodyA.shapeType == ShapeType::Circle && bodyB.shapeType == ShapeType::Box)
            {
                auto verticesB = bodyB.GetTransformedVertices();
                Collision::CollisionResult result = Collision::IsPolygonCircleIntersect(verticesB , bodyA.Position , bodyA.Radius);
                if (result.IsIntersect)
                {
                    Bodies[i].MoveBy(result.NormalCollisionDirection * (-result.Depth * 0.5f) );
                    Bodies[j].MoveBy(result.NormalCollisionDirection * (result.Depth * 0.5f) );
                    ResolveCollision(bodyA, bodyB , result.NormalCollisionDirection);
                }
            }
        }
    }
}
