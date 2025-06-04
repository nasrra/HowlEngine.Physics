using HowlEngine.Math;
using HowlEngine.Collections.Shapes;

namespace HowlEngine.Physics;

public struct BoxRigidBody{
    
    
    /// <summary>
    /// Gets the collider of this physics body
    /// </summary>
    
    public readonly Rectangle collider;


    /// <summary>
    /// Gets the body data of this physics body.
    /// </summary>

    public readonly PhysicsBody body;


    /// <summary>
    /// Creates a new BoxPhysicsBody instance.
    /// </summary>
    /// <param name="position"></param>
    /// <param name="width">The width, in pixels, of this physics body.</param>
    /// <param name="height">The height, in pixels, of this physics body.</param>
    /// <param name="density">The density of this physics body.</param>
    /// <param name="restitution">The restitution (between 0 and 1) of this physics body.</param>

    public BoxRigidBody(Vector2 position, float width, float height, float density, float restitution){
        collider = new Rectangle(position.X, position.Y, width, height);
        body = new PhysicsBody(position, width * height * density, density, restitution);
    }
}