using HowlEngine.Math;

namespace HowlEngine.Physics;

public struct PhysicsBody{
    

    /// <summary>
    /// Gets and sets the position of this physics body.
    /// </summary>
    
    public Vector2 Position;
    

    /// <summary>
    /// Gets the mass of this physics body.
    /// </summary>
    
    public readonly float Mass;


    /// <summary>
    /// Gets the density of this physics body.
    /// </summary>

    public readonly float Density;


    /// <summary>
    /// Gets the restitution (bounce) of this physics body.
    /// </summary>

    public readonly float Restitution;


    /// <summary>
    /// Creates a new instance of a physics body.
    /// </summary>
    /// <param name="position">The xy-position to start at.</param>
    /// <param name="mass">The mass of this physics body.</param>
    /// <param name="density">The density of this physics body.</param>
    /// <param name="restitution">The restitution (bounce) of this physics body.</param>

    public PhysicsBody(Vector2 position, float mass, float density, float restitution){
        Position        = position;
        Mass            = mass;
        Density         = density;
        Restitution     = Math.Util.Clamp(restitution,0,1);
    }
}