

using System.Numerics;

namespace HowlEngine.Physics;

internal readonly struct CollisionManifold{


    /// <summary>
    /// Gets the normal/direction/axis the collision occured on.
    /// </summary>
    
    public readonly Vector2 Normal;


    /// <summary>
    /// Gets the first contact point between the two bodies.
    /// </summary>

    public readonly Vector2 Contact1;


    /// <summary>
    /// Gets the second point of contact between the two bodies.
    /// </summary>

    public readonly Vector2 Contact2;


    /// <summary>
    /// Gets the depth of the collision between the two bodies.
    /// </summary>

    public readonly float Depth;


    /// <summary>
    /// Gets the index of body A in  the internal data strcutre storing it. 
    /// </summary>

    public readonly int BodyIndexA;


    /// <summary>
    /// Gets the index of body B in  the internal data strcutre storing it. 
    /// </summary>

    public readonly int BodyIndexB;


    /// <summary>
    /// Creates a new CollisionManifold instance.
    /// </summary>
    /// <param name="normal">The normal/direction/axis the collision occured on.</param>
    /// <param name="contact1">The first contact point between the two bodies.</param>
    /// <param name="contact2">The second point of contact between the two bodies.</param>
    /// <param name="depth">The depth of the collision between the two bodies.</param>
    /// <param name="bodyIndexA">The index of body A in  the internal data strcutre storing it.</param>
    /// <param name="bodyIndexB">The index of body B in  the internal data strcutre storing it.</param>

    public CollisionManifold(
        Vector2 normal, 
        Vector2 contact1, 
        Vector2 contact2, 
        float depth, 
        int bodyIndexA, 
        int bodyIndexB){
        
        Normal      = normal;
        Contact1    = contact1;
        Contact2    = contact2;
        Depth       = depth;
        BodyIndexA  = bodyIndexA;
        BodyIndexB  = bodyIndexB;
    }
}