using System.Numerics;
using HowlEngine.Collections.Shapes;
using HowlEngine.Collections;
using System.Diagnostics;

namespace HowlEngine.Physics;

public class PhysicsSystem{

    private StructPool<PolygonPhysicsBody> polygonRigidBodies;
    private StructPool<CirclePhysicsBody> circleRigidBodies;
    private StructPool<PolygonPhysicsBody> polygonKinematicBodies;
    private StructPool<CirclePhysicsBody> circleKinematicBodies;
    
    
    /// <summary>
    /// Collision manifold between circle rigid bodies.
    /// </summary>
    
    private List<CollisionManifold> _crToCrContacts;


    /// <summary>
    /// Collision manifold between polygon rigid bodies.
    /// </summary>

    private List<CollisionManifold> _prToPrContacts;


    /// <summary>
    /// Collision manifold between polygon to circle rigidbodies.
    /// </summary>

    private List<CollisionManifold> _prToCrContacts;


    /// <summary>
    /// Collision manifold between circle kinematic to circle rigid bodies.
    /// </summary>

    private List<CollisionManifold> _ckToCrContacts;


    /// <summary>
    /// Collision manifold between polygon kinematic to polygon rigid bodies.
    /// </summary>

    private List<CollisionManifold> _pkToPrContacts;


    /// <summary>
    /// Collision manifold between polygon kinematic to circle rigid bodies.
    /// </summary>

    private List<CollisionManifold> _pkToCrContacts;


    /// <summary>
    /// Collision manifold between circle kinematic to polygon rigid bodies.
    /// </summary>

    private List<CollisionManifold> _ckToPrContacts;

    public List<Vector2> ContactPoints;

    /// <summary>
    /// The gravity value to apply to rigidbodies.
    /// </summary>

    private Vector2 gravity = new Vector2(0,0.0981f);

    public Stopwatch StepCallTimer = new Stopwatch();


    /// <summary>
    /// Whether or not gavity is enabled.
    /// </summary>

    private bool gravityEnabled = false;


    public PhysicsSystem(
        int polygonRigidBodies, int polygonKinematicBodies, 
        int circleRigidBodies, int circleKinematicBodies, bool enableGravity){
        
        this.polygonRigidBodies         = new StructPool<PolygonPhysicsBody>(polygonRigidBodies);
        this.polygonKinematicBodies     = new StructPool<PolygonPhysicsBody>(polygonKinematicBodies);
        this.circleRigidBodies          = new StructPool<CirclePhysicsBody>(circleRigidBodies);
        this.circleKinematicBodies      = new StructPool<CirclePhysicsBody>(circleKinematicBodies);
        _crToCrContacts                 = new List<CollisionManifold>();
        _prToPrContacts                 = new List<CollisionManifold>();
        _prToCrContacts                 = new List<CollisionManifold>();
        _ckToCrContacts                 = new List<CollisionManifold>();
        _pkToPrContacts                 = new List<CollisionManifold>();
        _pkToCrContacts                 = new List<CollisionManifold>();
        _ckToPrContacts                 = new List<CollisionManifold>();
        ContactPoints                   = new List<Vector2>();
        gravityEnabled                  = enableGravity;
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


    public void FixedUpdate(float deltaTime, int subSteps){
        StepCallTimer.Reset();
        StepCallTimer.Start();
        int steps = Math.Clamp(subSteps, 1, 64);
        float stepModifier = 1f/(float)steps;
        
        ContactPoints.Clear();
        

        for(int i = 0; i < steps; i++){
            // clear all contacts for the step.        
            
            _crToCrContacts.Clear();
            _prToPrContacts.Clear();
            _prToCrContacts.Clear();
            _ckToCrContacts.Clear();
            _pkToPrContacts.Clear();
            _pkToCrContacts.Clear();
            _ckToPrContacts.Clear();

            MovementStep(stepModifier,i,steps);
            CollisionsStep(deltaTime);
            ResponseStep(deltaTime);
            
        }
        StepCallTimer.Stop();
    }


    /// <summary>
    /// Moves all physics bodies according to their linear velocity.
    /// </summary>
    /// <param name="stepModifier"></param>
    /// <param name="currentStep"></param>
    /// <param name="maxStep"></param>

    private void MovementStep(float stepModifier, int currentStep, int maxStep){
        
        Parallel.For(0, polygonRigidBodies.Capacity, i=>{
        // for(int i = 0; i < polygonRigidBodies.Capacity; i++){
            
            // skip if not active.
            
            if(polygonRigidBodies.IsSlotActive(i) == false){
                return;
                // continue;
            }

            // get the body.
            
            ref PolygonPhysicsBody body = ref polygonRigidBodies.GetData(i);
            
            // apply force.
            // force = mass * acceleration.
            // acceleration = force / mass.

            body.PhysicsBody.Acceleration = (body.PhysicsBody.Force / body.PhysicsBody.Mass); 
            body.PhysicsBody.LinearVelocity += body.PhysicsBody.Acceleration * stepModifier;
            
            if(currentStep == maxStep){
                body.PhysicsBody.Force = Vector2.Zero;
            }

            // apply grvity if enabled.

            if(gravityEnabled == true){
                body.PhysicsBody.LinearVelocity += gravity * stepModifier;
            }

            // movement

            body.Position += body.PhysicsBody.LinearVelocity * stepModifier;
        // }    
        });


        // move circle rigidbodies.
        
        Parallel.For(0, circleRigidBodies.Capacity, i=>{
        // for(int i = 0; i < circleRigidBodies.Capacity; i++){

            // skip if not active.
            
            if(circleRigidBodies.IsSlotActive(i) == false){
                return;
                // continue;
            }

            // get the body.

            ref CirclePhysicsBody body = ref circleRigidBodies.GetData(i);

            // apply force.

            body.PhysicsBody.Acceleration = body.PhysicsBody.Force / body.PhysicsBody.Mass * stepModifier; 
            body.PhysicsBody.LinearVelocity += body.PhysicsBody.Acceleration * stepModifier;
            body.PhysicsBody.Force = Vector2.Zero;

            // apply gravity if enabled.

            if(gravityEnabled == true){
                body.PhysicsBody.LinearVelocity += gravity * stepModifier;
            }

            // movement

            body.Position += body.PhysicsBody.LinearVelocity * stepModifier;
        // }
        });    
    }


    /// <summary>
    /// Detects collisions between all physics body colliders.
    /// </summary>
    /// <param name="deltaTime"></param>

    private void CollisionsStep(float deltaTime){
            
        // detect rigid body collisions.

        CrToCrCollisions(deltaTime);
        PrToPrCollisions(deltaTime);
        PrToCrCollisions(deltaTime);
        
        // detect kinematic body collisions.

        PkToCrCollisions(deltaTime);
        PkToPrCollisions(deltaTime);
    }


    /// <summary>
    /// Performs a collision response to all colliding physics bodies.
    /// </summary>
    /// <param name="deltaTime"></param>

    private void ResponseStep(float deltaTime){
        
        // collision response for circle rigid bodies.

        // Parallel.For(0, _crToCrContacts.Count, i=>{
        for(int i = 0; i < _crToCrContacts.Count; i++){
            CollisionManifold manifold = _crToCrContacts[i];
            ResolveRigidToRigidCollision(
                ref circleRigidBodies.GetData(manifold.BodyIndexA).PhysicsBody,
                ref circleRigidBodies.GetData(manifold.BodyIndexB).PhysicsBody,
                manifold.Normal,
                manifold.Depth
            );
        }
        // });
        
        // collision response for polygon rigid bodies.

        // Parallel.For(0, _prToPrContacts.Count, i=>{
        for(int i = 0; i < _prToPrContacts.Count; i++){
            CollisionManifold manifold = _prToPrContacts[i];
            ResolveRigidToRigidCollision(
                ref polygonRigidBodies.GetData(manifold.BodyIndexA).PhysicsBody,
                ref polygonRigidBodies.GetData(manifold.BodyIndexB).PhysicsBody,
                manifold.Normal,
                manifold.Depth
            );
        }
        // });

        // collision response for polygon to circle rigid bodies.

        // Parallel.For(0, _prToCrContacts.Count, i=>{
        for(int i = 0; i < _prToCrContacts.Count; i++){

            CollisionManifold manifold = _prToCrContacts[i];
            ResolveRigidToRigidCollision(
                ref polygonRigidBodies.GetData(manifold.BodyIndexA).PhysicsBody,
                ref circleRigidBodies.GetData(manifold.BodyIndexB).PhysicsBody,
                manifold.Normal,
                manifold.Depth
            );
        }
        // });

        // collision response for circle kinematic to rigid bodies.

        // Parallel.For(0, _ckToCrContacts.Count, i=>{
        for(int i = 0; i < _ckToCrContacts.Count; i++){

            CollisionManifold manifold = _ckToCrContacts[i];
            ResolveKinematicToRigidCollision(
                ref circleKinematicBodies.GetData(manifold.BodyIndexA).PhysicsBody,
                ref circleRigidBodies.GetData(manifold.BodyIndexB).PhysicsBody,
                manifold.Normal,
                manifold.Depth
            );
        }
        // });

        // collision response for polygon kinematic to rigid bodies.

        // Parallel.For(0, _pkToPrContacts.Count, i=>{
        for(int i = 0; i < _pkToPrContacts.Count; i++){
            CollisionManifold manifold = _pkToPrContacts[i];
            ResolveKinematicToRigidCollision(
                ref polygonKinematicBodies.GetData(manifold.BodyIndexA).PhysicsBody,
                ref polygonRigidBodies.GetData(manifold.BodyIndexB).PhysicsBody,
                manifold.Normal,
                manifold.Depth
            );
        }
        // });

        // collision response for polygon kinematic to circle rigid bodies.

        // Parallel.For(0, _pkToCrContacts.Count, i=>{
        for(int i = 0; i < _pkToCrContacts.Count; i++){

            CollisionManifold manifold = _pkToCrContacts[i];
            ResolveKinematicToRigidCollision(
                ref polygonKinematicBodies.GetData(manifold.BodyIndexA).PhysicsBody,
                ref circleRigidBodies.GetData(manifold.BodyIndexB).PhysicsBody,
                manifold.Normal,
                manifold.Depth
            );
        }
        // });

        // collision response for circle kinematic to polygon rigid bodies.

        // Parallel.For(0, _ckToPrContacts.Count, i=>{
        for(int i = 0; i < _ckToPrContacts.Count; i++){
            CollisionManifold manifold = _ckToPrContacts[i];
            ResolveKinematicToRigidCollision(
                ref circleKinematicBodies.GetData(manifold.BodyIndexA).PhysicsBody,
                ref circleRigidBodies.GetData(manifold.BodyIndexB).PhysicsBody,
                manifold.Normal,
                manifold.Depth
            );
        }
        // });
    }

    /// <summary>
    /// Detect collidions between circle rigidbodies.
    /// </summary>
    /// <param name="deltaTime"></param>

    private void CrToCrCollisions(float deltaTime){
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

                if(Collections.Shapes.Util.Intersect(ref a.Shape, ref b.Shape, out Vector2 normal, out Vector2 contactPoint, out float depth) == true){

                    // push apart by half from eachother.

                    a.Position += normal * depth * 0.5f;
                    b.Position -= normal * depth * 0.5f;

                    // add to collision manifold for cummulative resolutions.

                    _crToCrContacts.Add(new CollisionManifold(
                        normal,
                        Vector2.Zero,
                        Vector2.Zero,
                        depth,
                        i,
                        j
                    ));

                    ContactPoints.Add(contactPoint);

                    // ResolveRigidToRigidCollision(ref a.PhysicsBody, ref b.PhysicsBody, normal, depth);
                }
            }
        });
    }


    /// <summary>
    /// Detect collisions between polygon rigid bodies.
    /// </summary>
    /// <param name="deltaTime"></param>

    private void PrToPrCollisions(float deltaTime){
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

                    // add to collision manifold for cummulative resolutions.

                    _prToPrContacts.Add(new CollisionManifold(
                        normal,
                        Vector2.Zero,
                        Vector2.Zero,
                        depth,
                        i,
                        j
                    ));

                    // ResolveRigidToRigidCollision(ref a.PhysicsBody, ref b.PhysicsBody, normal, depth);
                }
            
            }
        });
    }


    /// <summary>
    /// Detect collisions between polygon and circle rigid bodies.
    /// </summary>
    /// <param name="deltaTime"></param>

    private void PrToCrCollisions(float deltaTime){
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

                    // add to collision manifold for cummulative resolutions.

                    _prToCrContacts.Add(new CollisionManifold(
                        normal,
                        Vector2.Zero,
                        Vector2.Zero,
                        depth,
                        i,
                        j
                    ));

                    // ResolveRigidToRigidCollision(ref a.PhysicsBody, ref b.PhysicsBody, normal, depth);                
                }
            }


        });
    }


    /// <summary>
    /// Detect collisions between polygon kinematic bodies and circle rigid bodies.
    /// </summary>
    /// <param name="deltaTime"></param>

    private void PkToCrCollisions(float deltaTime){
        Parallel.For(0,polygonKinematicBodies.Capacity, i => {
            
            // skip if not active.
            
            if(polygonKinematicBodies.IsSlotActive(i) == false){
                return;
            }

            // get the current body.

            ref PolygonPhysicsBody kinematic = ref polygonKinematicBodies.GetData(i);

            for(int j = 0; j < circleRigidBodies.Capacity; j++){

                // skip if not active.

                if(circleRigidBodies.IsSlotActive(j) == false){
                    continue;
                }

                // get the other body.

                ref CirclePhysicsBody rigid = ref circleRigidBodies.GetData(j);

                // if there is an intersection between them.

                if(Collections.Shapes.Util.Intersect(ref kinematic.Shape, ref rigid.Shape, out Vector2 normal, out float depth)){

                    // push the rigid away from the kinematic.

                    rigid.Position += normal * depth;

                    // add to collision manifold for cummulative resolutions.

                    _pkToCrContacts.Add(new CollisionManifold(
                        normal,
                        Vector2.Zero,
                        Vector2.Zero,
                        depth,
                        i,
                        j
                    ));

                    // ResolveRigidToKinematicCollision(ref rigid.PhysicsBody, ref kinematic.PhysicsBody, normal, depth);                
                }
            }

        });
    }


    /// <summary>
    /// Detect collisions between polygon kinematic and polygon rigid bodies.
    /// </summary>
    /// <param name="deltaTime"></param>

    private void PkToPrCollisions(float deltaTime){
        Parallel.For(0, polygonKinematicBodies.Capacity, i => {
            // skip if not active.

            if(polygonKinematicBodies.IsSlotActive(i) == false){
                return;
                // continue;
            }
            
            // get the current body.

            ref PolygonPhysicsBody kinematic = ref polygonKinematicBodies.GetData(i);

            for(int j = 0; j < polygonRigidBodies.Capacity; j++){ // <-- ensure the comparison between others only occurs once.
    
                if(polygonRigidBodies.IsSlotActive(j) == false){
                    continue;
                }

                // get the other body.

                ref PolygonPhysicsBody rigid = ref polygonRigidBodies.GetData(j);

                // if there is an intersection between them.

                if(Collections.Shapes.Util.Intersect(ref rigid.Shape, ref kinematic.Shape, out Vector2 normal, out float depth)){
                    
                    // push the rigid away from the kinematic.
                    
                    rigid.Position -= normal * depth;

                    // add to collision manifold for cummulative resolutions.

                    _pkToPrContacts.Add(new CollisionManifold(
                        normal,
                        Vector2.Zero,
                        Vector2.Zero,
                        depth,
                        i,
                        j
                    ));

                    // ResolveRigidToKinematicCollision(ref rigid.PhysicsBody, ref kinematic.PhysicsBody, normal, depth);
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


    private void ResolveKinematicToRigidCollision(ref PhysicsBody kinematic, ref PhysicsBody rigid, Vector2 normal, float depth){
        
        
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