using System.Numerics;
using HowlEngine.Collections.Shapes;

namespace HowlEngine.Physics;

public struct BoxRigidBody{


    /// <summary>
    /// Gets the collider of this rigid body.
    /// </summary>

    public Polygon Shape;


    /// <summary>
    /// Gets the body data of this physics body.
    /// </summary>

    public PhysicsBody PhysicsBody;


    /// <summary>
    /// Gets and sets the position of this rigid body.
    /// </summary>

    public Vector2 Position {
        get => Shape.Position;
        set{
            Shape.Position = value;
            PhysicsBody.Position = value;
        }
    }


    /// <summary>
    /// Gets and set the rotation of this rigidbody.
    /// </summary>

    public float Rotation {
        get => Shape.Rotation;
        set => Shape.Rotation = value;
    }


    /// <summary>
    /// Creates a new BoxRigidBody instance 
    /// </summary>
    /// <param name="position"></param>
    /// <param name="width">The width, in pixels, of this physics body.</param>
    /// <param name="height">The height, in pixels, of this physics body.</param>
    /// <param name="density">The density of this physics body.</param>
    /// <param name="restitution">The restitution (between 0 and 1) of this physics body.</param>

    public BoxRigidBody(Vector2 position, float width, float height, float density, float restitution){
        
        // calculate relative positions.
        
        float halfWidth = width * 0.5f;
        float halfHeight = height * 0.5f;
        float left      = position.X - halfWidth;
        float right     = position.X + halfWidth;
        float top       = position.Y - halfHeight;
        float bottom    = position.Y + halfHeight;

        Shape = new Polygon(
            new Vector2[]{
                new Vector2(left, top),
                new Vector2(right, top),
                new Vector2(right, bottom),
                new Vector2(left, bottom)
            }, 
            position,
            0
        ); 
         
        PhysicsBody = new PhysicsBody(
            position, 
            width * height, 
            density, 
            restitution
        );
    }
}