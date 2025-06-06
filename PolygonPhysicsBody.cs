using HowlEngine.Collections.Shapes;
using System.Numerics;

namespace HowlEngine.Physics;

public struct PolygonPhysicsBody{
    
    /// <summary>
    /// Gets the collider of this rigid body.
    /// </summary>

    public Polygon Shape;

    /// <summary>
    /// Gets the body data of this physics body.
    /// </summary>

    public PhysicsBody PhysicsBody;

    public Vector2 Position {
        get => PhysicsBody.Position;
        set{
            PhysicsBody.Position = value;
            Shape.Position = value;
        }
    }

    public float Rotation {
        get => Shape.Rotation;
        set => Shape.Rotation = value; 
    }

    /// <summary>
    /// Creates a new PolygonRigidBody instance.
    /// </summary>
    /// <param name="vertices">The points that make up the polygon.</param>
    /// <param name="position">The starting position of the polygon.</param>
    /// <param name="area">The area of the polygon.</param>
    /// <param name="density">The density of the polygon.</param>
    /// <param name="restitution">The restiturion ,between 1 and 0, of the polygon.</param>

    public PolygonPhysicsBody(Vector2[] vertices, Vector2 position, float area, float density, float restitution){
        Shape = new Polygon(vertices, position, 0);
        PhysicsBody = new PhysicsBody(position, area * density, density, restitution);
    }
}