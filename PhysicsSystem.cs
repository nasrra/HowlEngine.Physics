using System.Numerics;
using HowlEngine.Collections.Shapes;
using HowlEngine.Collections;

namespace HowlEngine.Physics;

public class PhysicsSystem{

    private StructPool<PolygonPhysicsBody> polygonRigidBodies;
    private StructPool<CirclePhysicsBody> circleRigidBodies;
    private StructPool<PolygonPhysicsBody> polygonKinematicBodies;
    private StructPool<CirclePhysicsBody> circleKinematicBodies;


    public PhysicsSystem(
        int polygonRigidBodies, int polygonKinematicBodies, 
        int circleRigidBodies, int circleKinematicBodies){
        this.polygonRigidBodies         = new StructPool<PolygonPhysicsBody>(polygonRigidBodies);
        this.polygonKinematicBodies     = new StructPool<PolygonPhysicsBody>(polygonKinematicBodies);
        this.circleRigidBodies          = new StructPool<CirclePhysicsBody>(circleRigidBodies);
        this.circleKinematicBodies      = new StructPool<CirclePhysicsBody>(circleKinematicBodies);
    }



    public Token AllocateBoxRigidBody(Vector2 position, float width, float height, float density, float restitution){
        float halfWidth     = width * 0.5f;
        float halfHeight    = height * 0.5f;
        float left          = position.X - halfWidth;
        float right         = position.X + halfWidth;
        float top           = position.Y - halfHeight;
        float bottom        = position.Y + halfHeight;
        return AllocatePolygonRigidBody(
            new PolygonPhysicsBody(
                [
                    new Vector2(left, top),
                    new Vector2(right, top),
                    new Vector2(right, bottom),
                    new Vector2(left, bottom)
                ], 
                position,
                width * height,
                density,
                restitution
            )
        );        
    }

    /// <summary>
    /// Allocates a physics body to the internal data structure.
    /// </summary>
    /// <param name="body"></param>
    /// <returns></returns>

    public Token AllocateBoxKinematicBody(Vector2 position, float width, float height, float density, float restitution){
        float halfWidth     = width * 0.5f;
        float halfHeight    = height * 0.5f;
        float left          = position.X - halfWidth;
        float right         = position.X + halfWidth;
        float top           = position.Y - halfHeight;
        float bottom        = position.Y + halfHeight;
        return AllocatePolygonKinematicBody(
            new PolygonPhysicsBody(
                [
                    new Vector2(left, top),
                    new Vector2(right, top),
                    new Vector2(right, bottom),
                    new Vector2(left, bottom)
                ], 
                position,
                width * height,
                density,
                restitution
            )
        );        
    }


    /// <summary>
    /// Allocates a physics body to the internal data structure.
    /// </summary>
    /// <param name="body"></param>
    /// <returns>A Token to reference the newly allocated PhysicsBody.</returns>
    public Token AllocatePolygonRigidBody(PolygonPhysicsBody body){
        // allocate.
        Token token = polygonRigidBodies.Allocate();
        if(token.Valid == false){
            return token;
        }

        polygonRigidBodies.TryGetData(ref token).Data = body;
        return token;
    }


    /// <summary>
    /// Allocates a physics body to the internal data structure.
    /// </summary>
    /// <param name="body"></param>
    /// <returns>A Token to reference the newly allocated PhysicsBody.</returns>
    public Token AllocatePolygonKinematicBody(PolygonPhysicsBody body){
        // allocate.
        Token token = polygonKinematicBodies.Allocate();
        if(token.Valid == false){
            return token;
        }

        polygonKinematicBodies.TryGetData(ref token).Data = body;
        return token;
    }


    /// <summary>
    /// Frees a polygon physics body (box, triangle, hexagon, etc) in the internal data structure at a given index.
    /// </summary>
    /// <param name="token">The specified token used as the index to free in the internal data structure..</param>

    public void FreePolygonRigidBody(ref Token token){
        polygonRigidBodies.Free(token.Id);
    }
    
    /// <summary>
    /// Frees a polygon physics body (box, triangle, hexagon, etc) in the internal data structure at a given index.
    /// </summary>
    /// <param name="index">The specified index to free at.</param>
    
    public void FreePolygonRigidBody(int index){
        polygonRigidBodies.Free(index);
    }




    /// <summary>
    /// Frees a physics body in the internal data structure at a given index.
    /// </summary>
    /// <param name="index">The specified index to free at.</param>
    
    public void FreePolygonKinematicBody(ref Token token){
        polygonKinematicBodies.Free(token.Id);
    }


    /// <summary>
    /// Frees a physics body in the internal data structure at a given index.
    /// </summary>
    /// <param name="index">The specified index to free at.</param>
    
    public void FreePolygonKinematicBody(int index){
        polygonKinematicBodies.Free(index);
    }


    /// <summary>
    /// Allocates a rigid circle body to the internal data structure.
    /// </summary>
    /// <param name="body"></param>
    /// <returns>A Token to reference the newly allocated PhysicsBody.</returns>
    public Token AllocateCircleRigidBody(Vector2 position, float radius, float density, float restitution){
        // allocate.
        Token token = circleRigidBodies.Allocate();
        if(token.Valid == false){
            return token;
        }

        circleRigidBodies.TryGetData(ref token).Data = new CirclePhysicsBody(
            position,
            radius,
            density,
            restitution
        );
        
        return token;
    }


    /// <summary>
    /// Frees a circle rigid body in the internal data structure at a given index.
    /// </summary>
    /// <param name="index">The specified index to free at.</param>

    public void FreeCircleRigidBody(ref Token token){
        circleRigidBodies.Free(token.Id);
    }


    /// <summary>
    /// Frees a rigid circle body in the internal data structure at a given index.
    /// </summary>
    /// <param name="index">The specified index to free at.</param>
    public void FreeCircleRigidBody(int index){
        circleRigidBodies.Free(index);
    }


    /// <summary>
    /// Allocates a physics body to the internal data structure.
    /// </summary>
    /// <param name="body"></param>
    /// <returns></returns>

    public Token AllocateCircleKinematicBody(Vector2 position, float radius, float density, float restitution){
        
        // allcate.
        Token token = circleKinematicBodies.Allocate();

        if(token.Valid == false){
            return token;
        }

        circleKinematicBodies.TryGetData(ref token).Data = new CirclePhysicsBody(
            position,
            radius,
            density,
            restitution
        );
        return token;
    }


    /// <summary>
    /// Frees a physics body in the internal data structure at a given index.
    /// </summary>
    /// <param name="index">The specified index to free at.</param>
    
    public void FreeCircleKinematicBody(ref Token token){
        circleKinematicBodies.Free(token.Id);
    }


    /// <summary>
    /// Frees a physics body in the internal data structure at a given index.
    /// </summary>
    /// <param name="index">The specified index to free at.</param>
    
    public void FreeCircleKinematicBody(int index){
        circleKinematicBodies.Free(index);
    }


    /// <summary>
    /// Gets a copy of all rigid polygon body colliders in this physics system.
    /// </summary>
    /// <returns></returns>

    public List<Vector2[]> CopyPolygonRigidBodyColliders(){

        // create the copy list.

        List<Vector2[]> colliders = new List<Vector2[]>();
        
        // copy all colliders.
        
        for(int i = 0; i < polygonRigidBodies.Capacity; i++){
            if(polygonRigidBodies.IsSlotActive(i) == true){
                ref PolygonPhysicsBody body = ref polygonRigidBodies.GetData(i);
                colliders.Add(body.Shape.Vertices);
            }
        }

        return colliders;
    }


    /// <summary>
    /// Gets a copy of all kinematic polygon body colliders in this physics system.
    /// </summary>
    /// <returns></returns>

    public List<Vector2[]> CopyPolygonKinematicColliders(){

        // create the copy list.

        List<Vector2[]> colliders = new List<Vector2[]>();
        
        // copy all colliders.
        
        for(int i = 0; i < polygonKinematicBodies.Capacity; i++){
            if(polygonKinematicBodies.IsSlotActive(i) == true){
                ref PolygonPhysicsBody body = ref polygonKinematicBodies.GetData(i);
                colliders.Add(body.Shape.Vertices);
            }
        }

        return colliders;
    }


    /// <summary>
    /// Gets a copy of all rigid circle body colliders in this physics system.
    /// </summary>
    /// <returns></returns>

    public List<Circle> CopyCircleRigidBodyColliders(){

        // create the copy list.

        List<Circle> colliders = new List<Circle>();

        // copy all colliders.

        for(int i = 0; i < circleRigidBodies.Capacity; i++){
            if(circleRigidBodies.IsSlotActive(i) == true){
                ref CirclePhysicsBody body = ref circleRigidBodies.GetData(i);
                colliders.Add(body.Shape);
            }
        }

        return colliders;

    }

    
    /// <summary>
    /// Gets a RefView to directly access a physics body within this physics system.
    /// </summary>
    /// <param name="token">The Token used to retrieve the PhysicsBody from the internal data structure.</param>
    /// <returns>A RefView of the retrieved data by the specified Token.</returns>

    public RefView<PolygonPhysicsBody> GetPolygonRigidBody(ref Token token){
        return polygonRigidBodies.TryGetData(ref token);
    }


    /// <summary>
    /// Gets a RefView to directly access a physics body within this physics system.
    /// </summary>
    /// <param name="token">The Token used to retrieve the PhysicsBody from the internal data structure.</param>
    /// <returns>A RefView of the retrieved data by the specified Token.</returns>

    public RefView<CirclePhysicsBody> GetCircleRigidBody(ref Token token){
        return circleRigidBodies.TryGetData(ref token);
    }


    public void FixedUpdate(float deltaTime){
        
        MovementStep(deltaTime);
        
        // handle rigid body collisions.

        CircleRigidCollisions(deltaTime);
        PolygonRigidCollisions(deltaTime);
        CircleToPolygonRigidCollisions(deltaTime);
        
        // handle kinematic body collisions.

        CircleRigidToPolygonKinematicCollisions(deltaTime);
        PolygonRigidToPolygonKinematicCollisions(deltaTime);
    }

    private void MovementStep(float deltaTime){
        
        // move box rigidbodies.
        
        Parallel.For(0, polygonRigidBodies.Capacity, i=>{
            
            // skip if not active.
            
            if(polygonRigidBodies.IsSlotActive(i) == false){
                return;
            }

            // get the body.
            
            ref PolygonPhysicsBody body = ref polygonRigidBodies.GetData(i);
            
            // apply force.
            // force = mass * acceleration.
            // acceleration = force / mass.

            body.PhysicsBody.Acceleration = body.PhysicsBody.Force / body.PhysicsBody.Mass; 
            body.PhysicsBody.LinearVelocity += body.PhysicsBody.Acceleration;
            body.PhysicsBody.Force = Vector2.Zero;


            // movement

            body.Position += body.PhysicsBody.LinearVelocity;
        
        });


        // move circle rigidbodies.
        
        Parallel.For(0, circleRigidBodies.Capacity, i=>{
            
            // skip if not active.
            
            if(circleRigidBodies.IsSlotActive(i) == false){
                return;
            }

            // get the body.

            ref CirclePhysicsBody body = ref circleRigidBodies.GetData(i);

            // apply force.

            body.PhysicsBody.Acceleration = body.PhysicsBody.Force / body.PhysicsBody.Mass; 
            body.PhysicsBody.LinearVelocity += body.PhysicsBody.Acceleration;
            body.PhysicsBody.Force = Vector2.Zero;


            // movement

            body.Position += body.PhysicsBody.LinearVelocity;
        
        });
    }

    private void CircleRigidCollisions(float deltaTime){
        Parallel.For(0, circleRigidBodies.Capacity, i =>{
            // skip if the slot is not active.
            
            if(circleRigidBodies.IsSlotActive(i) == false){
                return;
            }

            // get the current circle.

            ref CirclePhysicsBody a = ref circleRigidBodies.GetData(i);
            
            for(int j = i + 1; j < circleRigidBodies.Capacity; j++){ // <-- ensure the comparison between others only occurs once.
                
                if(circleRigidBodies.IsSlotActive(j) == false){
                    continue;
                }

                // get the other circle.

                ref CirclePhysicsBody b = ref circleRigidBodies.GetData(j);

                // if there is an intersection between them.

                if(Collections.Shapes.Util.Intersect(ref a.Shape, ref b.Shape, out Vector2 normal, out float depth) == true){

                    // push apart by half from eachother.

                    a.Position -= normal * depth * 0.5f;
                    b.Position += normal * depth * 0.5f;

                }
            }
        });
    }

    private void PolygonRigidCollisions(float deltaTime){
        Parallel.For(0, polygonRigidBodies.Capacity, i => {
            // skip if not active.

            if(polygonRigidBodies.IsSlotActive(i) == false){
                return;
                // continue;
            }
            
            // get the current body.

            ref PolygonPhysicsBody a = ref polygonRigidBodies.GetData(i);

            for(int j = i + 1; j < polygonRigidBodies.Capacity; j++){ // <-- ensure the comparison between others only occurs once.
    
                if(polygonRigidBodies.IsSlotActive(j) == false){
                    continue;
                }

                // get the other body.

                ref PolygonPhysicsBody b = ref polygonRigidBodies.GetData(j);

                // if there is an intersection between them.

                if(Collections.Shapes.Util.Intersect(ref a.Shape, ref b.Shape, out Vector2 normal, out float depth)){
                    
                    // push apart from eachother.
                    
                    a.Position -= normal * depth * 0.5f;
                    b.Position += normal * depth * 0.5f;

                    ResolveRigidToRigidCollision(ref a.PhysicsBody, ref b.PhysicsBody, normal, depth);
                }
            
            }
        });
    }

    private void CircleToPolygonRigidCollisions(float deltaTime){
        Parallel.For(0,polygonRigidBodies.Capacity, i => {
            
            // skip if not active.

            if(polygonRigidBodies.IsSlotActive(i) == false){
                return;
            }

            // get the current body.

            ref PolygonPhysicsBody a = ref polygonRigidBodies.GetData(i);

            for(int j = 0; j < circleRigidBodies.Capacity; j++){
                
                // skip if not active.
                
                if(circleRigidBodies.IsSlotActive(j) == false){
                    continue;
                }

                // get the other body.

                ref CirclePhysicsBody b = ref circleRigidBodies.GetData(j);

                // if there is an intersection between them.
                
                if(Collections.Shapes.Util.Intersect(ref a.Shape, ref b.Shape, out Vector2 normal, out float depth)){

                    // push apart from eachother.

                    a.Position -= normal * depth * 0.5f;
                    b.Position += normal * depth * 0.5f;
                
                    ResolveRigidToRigidCollision(ref a.PhysicsBody, ref b.PhysicsBody, normal, depth);                
                }
            }


        });
    }


    /// <summary>
    /// Handles collisions between circle rigid bodies and polygon kinematic bodies.
    /// </summary>
    /// <param name="deltaTime"></param>

    private void CircleRigidToPolygonKinematicCollisions(float deltaTime){
        Parallel.For(0,circleRigidBodies.Capacity, i => {
            
            // skip if not active.
            
            if(circleRigidBodies.IsSlotActive(i) == false){
                return;
            }

            // get the current body.

            ref CirclePhysicsBody rigid = ref circleRigidBodies.GetData(i);

            for(int j = 0; j < polygonKinematicBodies.Capacity; j++){

                // skip if not active.

                if(polygonKinematicBodies.IsSlotActive(j) == false){
                    continue;
                }

                // get the other body.

                ref PolygonPhysicsBody kinematic = ref polygonKinematicBodies.GetData(j);

                // if there is an intersection between them.

                if(Collections.Shapes.Util.Intersect(ref kinematic.Shape, ref rigid.Shape, out Vector2 normal, out float depth)){

                    // push the rigid away from the kinematic.

                    rigid.Position += normal * depth;

                    // collision reponse.

                    ResolveRigidToKinematicCollision(ref rigid.PhysicsBody, ref kinematic.PhysicsBody, normal, depth);                
                }
            }

        });
    }


    /// <summary>
    /// Handles collisions between polygon rigid bodies and polygon kinematic bodies.
    /// </summary>
    /// <param name="deltaTime"></param>

    private void PolygonRigidToPolygonKinematicCollisions(float deltaTime){
        Parallel.For(0, polygonRigidBodies.Capacity, i => {
            // skip if not active.

            if(polygonRigidBodies.IsSlotActive(i) == false){
                return;
                // continue;
            }
            
            // get the current body.

            ref PolygonPhysicsBody rigid = ref polygonRigidBodies.GetData(i);

            for(int j = 0; j < polygonKinematicBodies.Capacity; j++){ // <-- ensure the comparison between others only occurs once.
    
                if(polygonKinematicBodies.IsSlotActive(j) == false){
                    continue;
                }

                // get the other body.

                ref PolygonPhysicsBody kinematic = ref polygonKinematicBodies.GetData(j);

                // if there is an intersection between them.

                if(Collections.Shapes.Util.Intersect(ref rigid.Shape, ref kinematic.Shape, out Vector2 normal, out float depth)){
                    
                    // push the rigid away from the kinematic.
                    
                    rigid.Position -= normal * depth;

                    // collision reponse.

                    ResolveRigidToKinematicCollision(ref rigid.PhysicsBody, ref kinematic.PhysicsBody, normal, depth);
                }
            
            }
        });
    }


    private void ResolveRigidToRigidCollision(ref PhysicsBody a, ref PhysicsBody b, Vector2 normal, float depth){
        
        
        Vector2 relativeVelocity  = b.LinearVelocity - a.LinearVelocity;
        
        // choose the minimum restitution as the softer body would absorb the force of the collision.
        // E.g. pillow (restitution 0.0) would absorb brick (restitution 0.65);

        float restitution = MathF.Min(a.Restitution, b.Restitution); 
    
        // the magnitude of the impulse to apply to both bodies when colliding.

        float j = -(1f + restitution) * Vector2.Dot(relativeVelocity, normal);
        j /= (1/a.Mass) + (1/b.Mass);
        
        // apply the impulse to the colliding bodiesin relation to the axis that they are colliding.

        a.LinearVelocity -= j/a.Mass*normal;
        b.LinearVelocity += j/b.Mass*normal;
    }


    private void ResolveRigidToKinematicCollision(ref PhysicsBody rigid, ref PhysicsBody kinematic, Vector2 normal, float depth){
        
        
        Vector2 relativeVelocity  = kinematic.LinearVelocity - rigid.LinearVelocity;
        
        // choose the minimum restitution as the softer body would absorb the force of the collision.
        // E.g. pillow (restitution 0.0) would absorb brick (restitution 0.65);

        float restitution = MathF.Min(rigid.Restitution, kinematic.Restitution); 
    
        // the magnitude of the impulse to apply to both bodies when colliding.

        float j = -(1f + restitution) * Vector2.Dot(relativeVelocity, normal);
        j /= (1/rigid.Mass) + (1/kinematic.Mass);
        
        // apply the impulse to the colliding rigid body in relation to the axis that the two bodies are colliding.

        rigid.LinearVelocity -= j/rigid.Mass*normal;
    }
}