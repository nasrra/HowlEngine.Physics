using System.Numerics;

namespace HowlEngine.Physics;

public struct PhysicsBodyAABB{
    public RectangleCollider Collider;

    // may need to add previous velocity for previoud frame check in case that velocities decay over time.
    public Vector2 Velocity {get; set;}

    /// <summary>
    /// Gets and Sets the width the of the PhysicsBody.
    /// </summary>
    public float Width {
        get => Collider.Width;
        set => Collider.Width = value;
    }

    /// <summary>
    /// Gets and Sets the height of the PhysicsBody.
    /// </summary>
    public float Height {
        get => Collider.Height;
        set => Collider.Height = value;
    }

    /// <summary>
    /// Gets and Sets the x-position of the PhysicsBody.
    /// </summary>
    public float X {
        get => Collider.X;
        set => Collider.X = value;
    }

    /// <summary>
    /// Gets and Sets the y_posiiton of the PhysicsBody.
    /// </summary>
    public float Y {
        get => Collider.Y;
        set => Collider.Y = value;
    }

    /// <summary>
    /// Gets and Sets the xy-position of the PhysicsBody.
    /// </summary>
    public Vector2 Position{
        get => new Vector2(Collider.X, Collider.Y);
        set {
            Collider.X = (int)value.X;
            Collider.Y = (int)value.Y;
        }
    }

    /// <summary>
    /// Gets the x-coordinate of the left edge of this PhysicsBody.
    /// </summary>
    public float Left => Collider.Left;

    /// <summary>
    /// Gets the x-coordinate of the right edge of this PhysicsBody.
    /// </summary>
    public float Right => Collider.Right;

    /// <summary>
    /// Gets the y-coordinate of the top edge of this PhysicsBody.
    /// </summary>
    public float Top => Collider.Top;

    /// <summary>
    /// Gets the y-coordinate of the bottom edge of this PhysicsBody.
    /// </summary>
    public float Bottom => Collider.Bottom;

    /// <summary>
    /// Gets and sets the level of Elasticity of this RigidBody (how much it bounces).
    /// </summary>
    public float Elasticity { get ; set ; } = 0;

    public PhysicsBodyAABB(RectangleCollider collider){
        Collider = collider;
        Velocity = Vector2.Zero;
    }

    public PhysicsBodyAABB(RectangleCollider collider, Vector2 velocity){
        Collider = collider;
        Velocity = velocity;
    }

    public PhysicsBodyAABB(RectangleCollider collider, Vector2 velocity, float elasticity){
        Collider = collider;
        Velocity = velocity;
        Elasticity = elasticity;
    }
}