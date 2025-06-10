using System.Numerics;
using HowlEngine.Collections;
using HowlEngine.Collections.Shapes;
using HowlEngine.Physics;

public struct CirclePhysicsBody{
    
    
    /// <summary>
    /// Gets the collider of this physics body
    /// </summary>
    
    public Circle Shape;


    /// <summary>
    /// Gets the body data of this physics body.
    /// </summary>

    public PhysicsBody PhysicsBody;

    public List<int> SpatialHashIndices;

    public Vector2 Position {
        get => PhysicsBody.Position;
        set{
            PhysicsBody.Position = value;
            Shape.Position = value;
        }
    }


    /// <summary>
    /// Creates a new CirclePhysicsBody instance.
    /// </summary>
    /// <param name="position"></param>
    /// <param name="radius">the length from the center of this circle to its edge.</param>
    /// <param name="density">The density of this physics body.</param>
    /// <param name="restitution">The restitution (between 0 and 1) of this physics body.</param>

    public CirclePhysicsBody(Vector2 position, float radius, float density, float restitution){
        Shape = new Circle(position, radius);
        PhysicsBody = new PhysicsBody(position, Shape.Area * density, density, restitution);
        SpatialHashIndices = new List<int>();
    }
}