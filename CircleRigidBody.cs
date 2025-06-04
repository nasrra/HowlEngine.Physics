using HowlEngine.Math;
using HowlEngine.Collections.Shapes;
using HowlEngine.Physics;

public struct CircleRigidBody{
    
    
    /// <summary>
    /// Gets the collider of this physics body
    /// </summary>
    
    public Circle collider;


    /// <summary>
    /// Gets the body data of this physics body.
    /// </summary>

    public PhysicsBody body;

    public Vector2 Position {
        get => collider.Position;
        set {
            collider.Position = value;
            body.Position = value;
        }
    }


    /// <summary>
    /// Creates a new CirclePhysicsBody instance.
    /// </summary>
    /// <param name="position"></param>
    /// <param name="radius">the length from the center of this circle to its edge.</param>
    /// <param name="density">The density of this physics body.</param>
    /// <param name="restitution">The restitution (between 0 and 1) of this physics body.</param>

    public CircleRigidBody(Vector2 position, float radius, float density, float restitution){
        collider = new Circle(position.X, position.Y, radius);
        body = new PhysicsBody(position, collider.Area * density, density, restitution);
    }
}