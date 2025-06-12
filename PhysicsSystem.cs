using System.Numerics;
using HowlEngine.Collections.Shapes;
using HowlEngine.Collections;
using System.Diagnostics;
using System.Collections.Concurrent;

namespace HowlEngine.Physics;

public class PhysicsSystem{

    public SpatialHash<int> SpatialHash;
    public StructPool<PolygonPhysicsBody>  polygonRigidBodies       {   get; private set;  }
    public StructPool<CirclePhysicsBody>   circleRigidBodies        {   get; private set;  }
    public StructPool<PolygonPhysicsBody>  polygonKinematicBodies   {   get; private set;  }
    public StructPool<CirclePhysicsBody>   circleKinematicBodies    {   get; private set;  }
    private HashSet<int>[] _neighbours;
    
    
    /// <summary>
    /// Collision manifold between circle rigid bodies.
    /// </summary>
    
    private List<CollisionManifold> _crToCrManifolds;


    /// <summary>
    /// Collision manifold between polygon rigid bodies.
    /// </summary>

    private List<CollisionManifold> _prToPrManifolds;


    /// <summary>
    /// Collision manifold between polygon to circle rigidbodies.
    /// </summary>

    private List<CollisionManifold> _crToPrManifolds;


    /// <summary>
    /// Collision manifold between circle kinematic to circle rigid bodies.
    /// </summary>

    private List<CollisionManifold> _crToCkManifolds;


    /// <summary>
    /// Collision manifold between polygon kinematic to polygon rigid bodies.
    /// </summary>

    private List<CollisionManifold> _prToPkManifolds;


    /// <summary>
    /// Collision manifold between polygon kinematic to circle rigid bodies.
    /// </summary>

    private List<CollisionManifold> _crToPkManifolds;


    /// <summary>
    /// Collision manifold between polygon rigid to circle kinematic bodies.
    /// </summary>


    private List<CollisionManifold> _prToCkManifolds;


    /// <summary>
    ///  circle to circle rigid bodies that have passed the broad phase.
    /// </summary>
    
    private List<(int,int)> _crToCrBroadIntersects;


    /// <summary>
    /// polygon to polygon rigid bodies that have passed the broad phase.
    /// </summary>

    private List<(int,int)> _prToPrBroadIntersects;


    /// <summary>
    /// circle to polygon rigid bodies that have passed the broad phase.
    /// </summary>

    private List<(int,int)> _crToPrBroadIntersects;


    /// <summary>
    /// circle rigid to circle kinematic bodies that have passed the broad phase.
    /// </summary>

    private List<(int,int)> _crToCkBroadIntersects;


    /// <summary>
    /// polygon rigid to polygon kinematic bodies that have passed the broad phase.
    /// </summary>

    private List<(int,int)> _prToPkBroadIntersects;


    /// <summary>
    /// circle rigid to polygon kinematic bodies that have passed the broad phase.
    /// </summary>

    private List<(int,int)> _crToPkBroadIntersects;

    /// <summary>
    /// polygon rigid to circle kinematic bodies that have passed the broad phase.
    /// </summary>

    private List<(int,int)> _prToCkBroadIntersects;

    public List<Vector2> ContactPoints;

    /// <summary>
    /// The gravity value to apply to rigidbodies.
    /// </summary>

    private Vector2 gravity         = new Vector2(0,0.0981f);

    public Stopwatch StepCallTimer      = new Stopwatch();
    public Stopwatch MovementStepTimer  = new Stopwatch();
    public Stopwatch BroadTimer         = new Stopwatch();
    public Stopwatch NarrowTimer        = new Stopwatch();
    public Stopwatch ResponseTimer      = new Stopwatch();
    public Stopwatch SpatialUpdateTimer = new Stopwatch();
    public Stopwatch FindNeighboursTimer = new Stopwatch();

    private int _prSpatialIndexUpperBound;
    private int _prSpatialIndexLowerBound;
    private int _crSpatialIndexUpperBound;
    private int _crSpatialIndexLowerBound;
    private int _pkSpatialIndexUpperBound;
    private int _pkSpatialIndexLowerBound;
    private int _ckSpatialIndexUpperBound;
    private int _ckSpatialIndexLowerBound;

    // thread indexes for each list.

    private const int _prToPrLocalThreadIndex = 0;
    private const int _prToPkLocalThreadIndex = 1;
    private const int _prToCkLocalThreadIndex = 2;

    private const int _crToCrLocalThreadIndex = 0;
    private const int _crToPrLocalThreadIndex = 1;
    private const int _crToPkLocalThreadIndex = 2;
    private const int _crToCkLocalThreadIndex = 3;


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
        _neighbours = new HashSet<int>[polygonRigidBodies+circleRigidBodies];
        for(int i = 0; i < _neighbours.Length; i++){
            _neighbours[i] = new HashSet<int>();
        }


        // set index offsets for spatial hash.
        //  Note:
        // Order matters here, do not change these values, ever.

        _prSpatialIndexLowerBound = 0;
        _prSpatialIndexUpperBound = polygonRigidBodies;
        _crSpatialIndexLowerBound = _prSpatialIndexUpperBound;
        _crSpatialIndexUpperBound = _prSpatialIndexUpperBound + circleRigidBodies;
        _pkSpatialIndexLowerBound = _crSpatialIndexUpperBound;
        _pkSpatialIndexUpperBound = _crSpatialIndexUpperBound + polygonKinematicBodies;
        _ckSpatialIndexLowerBound = _pkSpatialIndexUpperBound; 
        _ckSpatialIndexUpperBound = _pkSpatialIndexUpperBound + circleKinematicBodies;

        if(_ckSpatialIndexUpperBound > int.MaxValue){
            throw new InvalidOperationException("[PhysicsSystem]: Maximum physics body amount exceeded, lower a body count for this physics system.");
        }

        // initialise manifold storage.

        _crToCrManifolds                 = new List<CollisionManifold>();
        _prToPrManifolds                 = new List<CollisionManifold>();
        _crToPrManifolds                 = new List<CollisionManifold>();
        _crToCkManifolds                 = new List<CollisionManifold>();
        _prToPkManifolds                 = new List<CollisionManifold>();
        _crToPkManifolds                 = new List<CollisionManifold>();
        _prToCkManifolds                 = new List<CollisionManifold>();
        ContactPoints                    = new List<Vector2>();
        
        // initialise broad phase stoarge.

        _crToCrBroadIntersects                 = new List<(int,int)>();
        _prToPrBroadIntersects                 = new List<(int,int)>();
        _crToPrBroadIntersects                 = new List<(int,int)>();
        _crToCkBroadIntersects                 = new List<(int,int)>();
        _prToPkBroadIntersects                 = new List<(int,int)>();
        _crToPkBroadIntersects                 = new List<(int,int)>();
        _prToCkBroadIntersects                 = new List<(int,int)>();

        // SpatialHash                     = new SpatialHash<int>(new Vector2(-40,-40), new(64,64),12,10);
        SpatialHash                     = new SpatialHash<int>(new Vector2(-40,-40), new(32,32),96,60);
        // SpatialHash                     = new SpatialHash<int>(new Vector2(-40,-40), new(32,32),12,20);

        gravityEnabled                  = enableGravity;
    }



    public Token AllocateBoxRigidBody(Vector2 position, float width, float height, float density, float restitution){
        float halfWidth     = width * 0.5f;
        float halfHeight    = height * 0.5f;
        Token token =  AllocatePolygonRigidBody(
            new PolygonPhysicsBody(
                [
                    new Vector2(-halfWidth, -halfHeight),
                    new Vector2(halfWidth, -halfHeight),
                    new Vector2(halfWidth, halfHeight),
                    new Vector2(-halfWidth, halfHeight)
                ], 
                position,
                width * height,
                density,
                restitution

            )
        );  

        ref PolygonPhysicsBody body = ref polygonRigidBodies.TryGetData(ref token).Data;
        
        // insert the spatial body.

        SpatialHash.Insert(
            token.Id+_prSpatialIndexLowerBound, 
            body.Shape.Min, 
            body.Shape.Max, 
            body.SpatialHashIndices
        );
        

        return token;
    }

    /// <summary>
    /// Allocates a physics body to the internal data structure.
    /// </summary>
    /// <param name="body"></param>
    /// <returns></returns>

    public Token AllocateBoxKinematicBody(Vector2 position, float width, float height, float density, float restitution){
        float halfWidth     = width * 0.5f;
        float halfHeight    = height * 0.5f;
        Token token =  AllocatePolygonKinematicBody(
            new PolygonPhysicsBody(
                [
                    new Vector2(-halfWidth, -halfHeight),
                    new Vector2(halfWidth, -halfHeight),
                    new Vector2(halfWidth, halfHeight),
                    new Vector2(-halfWidth, halfHeight)
                ], 
                position,
                width * height,
                density,
                restitution
            )
        );
        ref PolygonPhysicsBody body = ref polygonKinematicBodies.TryGetData(ref token).Data;
        
        // insert the spatial body.

        SpatialHash.Insert(
            token.Id + _pkSpatialIndexLowerBound, 
            body.Shape.Min, 
            body.Shape.Max, 
            body.SpatialHashIndices
        );
        
        return token;
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

        ref CirclePhysicsBody body = ref circleRigidBodies.TryGetData(ref token).Data;

        body = new CirclePhysicsBody(
            position,
            radius,
            density,
            restitution
        );
        
        // insert the spatial body.

        SpatialHash.Insert(
            token.Id + _crSpatialIndexLowerBound, 
            body.Shape.Min, 
            body.Shape.Max, 
            body.SpatialHashIndices
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
            // clear previous frame temp storage.        
            
            _crToCrManifolds.Clear();
            _prToPrManifolds.Clear();
            _crToPrManifolds.Clear();
            _crToCkManifolds.Clear();
            _prToPkManifolds.Clear();
            _crToPkManifolds.Clear();
            _prToCkManifolds.Clear();

            _crToCrBroadIntersects.Clear();
            _prToPrBroadIntersects.Clear();
            _crToPrBroadIntersects.Clear();
            _crToCkBroadIntersects.Clear();
            _prToPkBroadIntersects.Clear();
            _crToPkBroadIntersects.Clear();
            _prToCkBroadIntersects.Clear();



            MovementStep(stepModifier,i,steps);
            FindNeighbours();
            BroadPhase(deltaTime);
            NarrowPhase(deltaTime);
            ResponseStep(deltaTime);
            SpatialHashStep(deltaTime);
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
        
        MovementStepTimer.Reset();
        MovementStepTimer.Start();

        Parallel.For(0, polygonRigidBodies.ActiveSlots.Count, i=>{                        
            // get the body.
            
            ref PolygonPhysicsBody body = ref polygonRigidBodies.GetData(polygonRigidBodies.ActiveSlots[i]);
            
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
        });

        // move circle rigidbodies.
        
        Parallel.For(0, circleRigidBodies.ActiveSlots.Count, i=>{

            // get the body.

            ref CirclePhysicsBody body = ref circleRigidBodies.GetData(circleRigidBodies.ActiveSlots[i]);

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
        });

        MovementStepTimer.Stop();
    }

    private void FindNeighbours(){
        FindNeighboursTimer.Reset();
        FindNeighboursTimer.Start();
        
        Parallel.For(0, polygonRigidBodies.Capacity, i=>{
            if(polygonRigidBodies.IsSlotActive(i) == false){
                return;
            }

            ref PolygonPhysicsBody a = ref polygonRigidBodies.GetData(i);
            _neighbours[i].Clear();
            SpatialHash.FindNear(a.Shape.Min, a.Shape.Max, 1, _neighbours[i]);
        });
        Parallel.For(0, circleRigidBodies.Capacity, i=>{
            if(circleRigidBodies.IsSlotActive(i) == false){
                return;
            }

            ref CirclePhysicsBody a = ref circleRigidBodies.GetData(i);
            _neighbours[i+_crSpatialIndexLowerBound].Clear();
            SpatialHash.FindNear(a.Shape.Min, a.Shape.Max, 1, _neighbours[i+_crSpatialIndexLowerBound]);
        });
        FindNeighboursTimer.Stop();
    }

    /// <summary>
    /// Detects AABB collisions between all physics body colliders.
    /// </summary>
    /// <param name="deltaTime"></param>

    private void BroadPhase(float deltaTime){
            
        BroadTimer.Reset();
        BroadTimer.Start();



        // =====================================================================================================================
        //  Note:
        //
        // collisions are one way but effect both bodies.
        // only one of the bodies has to go through a loop to ensure collision.
        // specifically with rigid bodies, kinematic and static do not handle collisions but recieve them from dynamic bodies.
        // =====================================================================================================================


        // =====================================
        // PR collisions.
        // including PR, PK, and CK collisions.
        // =====================================

        Parallel.For(0, polygonRigidBodies.ActiveSlots.Count,
            () => new List<(int, int)>[3]  // thread-local initializer creates an array of 7 lists
            {
                new List<(int,int)>(),
                new List<(int,int)>(),
                new List<(int,int)>()
            },
            (x, state, localLists) =>{

                int i = polygonRigidBodies.ActiveSlots[x];
                ref PolygonPhysicsBody a = ref polygonRigidBodies.GetData(i);

                foreach (int n in _neighbours[i + _prSpatialIndexLowerBound]){

                    int j = n;

                    // ==========================
                    // PR to PR collisions
                    // ==========================
                    
                    if(j < _prSpatialIndexUpperBound){
                        
                        // put 'j' into the struct pool index range.

                        j -= _prSpatialIndexLowerBound;
                        
                        if(j <= i){
        
                            // continue to next neighbour.
                        
                            continue;
                        }

                        // get the other body.

                        ref PolygonPhysicsBody b = ref polygonRigidBodies.GetData(j);

                        // Broad phase: AABB collision check.
                        
                        if(Collections.Shapes.Util.Intersect(a.Shape.Min, a.Shape.Max, b.Shape.Min, b.Shape.Max) == false){

                            // continue to next neighbour.
                            
                            continue;
                        }

                        // add to local list.

                        localLists[_prToPrLocalThreadIndex].Add((i, j));

                        // continue to next neighbour.
                        
                        continue;
                    }


                    // =========
                    // CR to PR.
                    // =========

                    if(j < _crSpatialIndexUpperBound){
                    
                        // continue to next neighbour.
                        
                        continue;                
                    }


                    // ====================
                    // PR to PK collisions.
                    // ====================

                    else if(j < _pkSpatialIndexUpperBound){

                        // put 'j' into the struct pool index range.

                        j -= _pkSpatialIndexLowerBound;

                        ref PolygonPhysicsBody b = ref polygonKinematicBodies.GetData(j);

                        // Broad phase: AABB collision check.

                        if(Collections.Shapes.Util.Intersect(a.Shape.Min, a.Shape.Max, b.Shape.Min, b.Shape.Max) == false){
        
                            // continue to next neighbour.
                        
                            continue;
                        }

                        // add to local list.

                        localLists[_prToPkLocalThreadIndex].Add((i,j));

                        // continue to next neighbour.

                        continue;
                    }



                    // ===================
                    // PR to CK collisions
                    // ===================
                

                    else if(j < _ckSpatialIndexUpperBound){
                        throw new NotImplementedException();
                    }
                }

                // return the local threads lists.

                return localLists;

            },
            localLists => {
                // Add all six lists from this thread to your concurrent bag or merge later
                for(int i = 0; i < localLists.Length; i++){
                    switch(i){
                        case _prToPrLocalThreadIndex:
                            lock(_prToPrBroadIntersects){
                                _prToPrBroadIntersects.AddRange(localLists[i]);
                            }
                            break;
                        case _prToPkLocalThreadIndex:
                            lock(_prToPkBroadIntersects){
                                _prToPkBroadIntersects.AddRange(localLists[i]);
                            }
                            break;
                        case _prToCkLocalThreadIndex:
                            lock(_prToCkBroadIntersects){
                                _prToCkBroadIntersects.AddRange(localLists[i]);
                            }
                            break;
                    }
                }
            }
        );


        // ==============
        // CR collisions.
        // including PR, CR, PK, and CK collisions.
        //  Note:
        // CR to PR is done here for performance reasons. 
        // ==============

        Parallel.For(0, circleRigidBodies.ActiveSlots.Count,
            () => new List<(int, int)>[4]  // thread-local initializer creates an array of 7 lists
            {
                new List<(int,int)>(),
                new List<(int,int)>(),
                new List<(int,int)>(),
                new List<(int,int)>()
            },
            (x, state, localLists) =>{

                int i = circleRigidBodies.ActiveSlots[x];
                ref CirclePhysicsBody a = ref circleRigidBodies.GetData(i);

                foreach (int n in _neighbours[i + _crSpatialIndexLowerBound]){

                    int j = n;

                    // =========
                    // CR to PR.
                    // =========

                    if(j < _prSpatialIndexUpperBound){

                        // put 'j' into the struct pool index range.

                        j -= _prSpatialIndexLowerBound;   

                        // get the other circle.

                        ref PolygonPhysicsBody b = ref polygonRigidBodies.GetData(j);

                        // Broad phase: AABB collision check.

                        if(Collections.Shapes.Util.Intersect(a.Shape.Min, a.Shape.Max, b.Shape.Min, b.Shape.Max) == false){

                            // continue to next neighbour.
                        
                            continue;
                        }

                        // add to local thread list.

                        localLists[_crToPrLocalThreadIndex].Add((i,j));

                        // continue to next neighbour.
                        
                        continue;
                    }


                    // ===========
                    // CR to CR.
                    // ===========

                    else if( j < _crSpatialIndexUpperBound){
                        
                        // put 'j' into the struct pool index range.

                        j -= _crSpatialIndexLowerBound;

                        if(j<=i){
                            // continue to next neighbour.
                        
                            continue;
                        }
                    
                        ref CirclePhysicsBody b = ref circleRigidBodies.GetData(j);

                        // Broad phase: AABB collision check.

                        if(Collections.Shapes.Util.Intersect(a.Shape.Min, a.Shape.Max, b.Shape.Min, b.Shape.Max) == false){
        
                            // continue to next neighbour.
                        
                            continue;
                        }

                        // add to local thread list.

                        localLists[_crToCrLocalThreadIndex].Add((i,j));

                        // continue to next neighbour.
                        
                        continue;
                    }


                    // ========= 
                    // CR to PK.
                    // =========

                    else if(j < _pkSpatialIndexUpperBound){

                        // put 'j' into the struct pool index range.

                        j -= _pkSpatialIndexLowerBound;
                    

                        ref PolygonPhysicsBody b = ref polygonKinematicBodies.GetData(j);

                        // Broad phase: AABB collision check.

                        if(Collections.Shapes.Util.Intersect(a.Shape.Min, a.Shape.Max, b.Shape.Min, b.Shape.Max) == false){
        
                            // continue to next neighbour.
                        
                            continue;
                        }

                        // add to local thread list.

                        localLists[_crToPkLocalThreadIndex].Add((i,j));

                        // continue to next neighbour.
                        
                        continue;
                    }

                    // ===================
                    // CR to CK collisions
                    // ===================
                

                    else if(j < _ckSpatialIndexUpperBound){
                        throw new NotImplementedException();
                    }
                
                }

                // return the local threads lists.

                return localLists;

            },
            localLists => {
                
                // Add all six lists from this thread to your concurrent bag or merge later
                
                for(int i = 0; i < localLists.Length; i++){
                    switch(i){
                        case _crToCrLocalThreadIndex:
                            lock(_crToCrBroadIntersects){
                                _crToCrBroadIntersects.AddRange(localLists[i]);
                            }
                            break;
                        case _crToPrLocalThreadIndex:
                            lock(_crToPrBroadIntersects){
                                _crToPrBroadIntersects.AddRange(localLists[i]);
                            }
                            break;
                        case _crToPkLocalThreadIndex:
                            lock(_crToPkBroadIntersects){
                                _crToPkBroadIntersects.AddRange(localLists[i]);
                            }
                            break;
                    }
                }
            }
        );

        BroadTimer.Stop();
    }


    /// <summary>
    /// Performs SAT collision detection on all physics bodies that passed the broad phase.
    /// </summary>
    /// <param name="deltaTime"></param>

    private void NarrowPhase(float deltaTime){
        NarrowTimer.Reset();
        NarrowTimer.Start();
        
        
        Parallel.For(0, _crToCrBroadIntersects.Count,
            () => new List<CollisionManifold>(), // thread-local list
            (i, state, localManifolds) => {

                ref CirclePhysicsBody a = ref circleRigidBodies.GetData(_crToCrBroadIntersects[i].Item1);
                ref CirclePhysicsBody b = ref circleRigidBodies.GetData(_crToCrBroadIntersects[i].Item2);

                if (Collections.Shapes.Util.Intersect(ref a.Shape, ref b.Shape, out Vector2 normal, out Vector2 contactPoint, out float depth)) {
                    a.Position += normal * depth * 0.5f;
                    b.Position -= normal * depth * 0.5f;

                    localManifolds.Add(new CollisionManifold(
                        normal,
                        Vector2.Zero,
                        Vector2.Zero,
                        depth,
                        _crToCrBroadIntersects[i].Item1,
                        _crToCrBroadIntersects[i].Item2
                    ));
                }

                return localManifolds;
            },
            localManifolds => {
                // merge at the end on main thread
                lock (_crToCrManifolds) {
                    _crToCrManifolds.AddRange(localManifolds);
                }
        });

        Parallel.For(0, _prToPrBroadIntersects.Count,
            () => new List<CollisionManifold>(), // Thread-local initializer
            (i, state, localManifolds) =>
            {
                // Get the two colliding bodies.
                ref PolygonPhysicsBody a = ref polygonRigidBodies.GetData(_prToPrBroadIntersects[i].Item1);
                ref PolygonPhysicsBody b = ref polygonRigidBodies.GetData(_prToPrBroadIntersects[i].Item2);

                // Perform SAT check.
                if (Collections.Shapes.Util.Intersect(ref a.Shape, ref b.Shape, out Vector2 normal, out float depth))
                {
                    // Push apart from each other.
                    a.Position -= normal * depth * 0.5f;
                    b.Position += normal * depth * 0.5f;

                    // Add result to thread-local list.
                    localManifolds.Add(new CollisionManifold(
                        normal,
                        Vector2.Zero,
                        Vector2.Zero,
                        depth,
                        _prToPrBroadIntersects[i].Item1,
                        _prToPrBroadIntersects[i].Item2
                    ));
                }

                return localManifolds;
            },
            localManifolds =>
            {
                // Merge local lists with final list once per thread.
                lock (_prToPrManifolds)
                {
                    _prToPrManifolds.AddRange(localManifolds);
                }
        });


        Parallel.For(0, _crToPrBroadIntersects.Count, i=>{
        // for(int i = 0; i < _crToPrBroadIntersects.Count; i++){

            // get the two colliding bodies.

            ref CirclePhysicsBody a = ref circleRigidBodies.GetData(_crToPrBroadIntersects[i].Item1);
            ref PolygonPhysicsBody b = ref polygonRigidBodies.GetData(_crToPrBroadIntersects[i].Item2);            

            // perform SAT  check.

            if(Collections.Shapes.Util.Intersect(ref b.Shape, ref a.Shape, out Vector2 normal, out float depth)){

                // push apart by half from eachother.

                a.Position += normal * depth * 0.5f;
                b.Position -= normal * depth * 0.5f;

                // add to collision manifold for cummulative resolutions.

                _crToPrManifolds.Add(new CollisionManifold(
                    normal,
                    Vector2.Zero,
                    Vector2.Zero,
                    depth,
                    _crToPrBroadIntersects[i].Item1,
                    _crToPrBroadIntersects[i].Item2
                ));
            }
        // }
        });

        Parallel.For(0, _crToCkBroadIntersects.Count, i=>{

        });

        Parallel.For(0, _prToPkBroadIntersects.Count,
            () => new List<CollisionManifold>(), // thread-local init
            (i, state, localManifolds) =>
            {
                // Get the two colliding bodies.
                ref PolygonPhysicsBody a = ref polygonRigidBodies.GetData(_prToPkBroadIntersects[i].Item1);
                ref PolygonPhysicsBody b = ref polygonKinematicBodies.GetData(_prToPkBroadIntersects[i].Item2);

                // Perform SAT check.
                if (Collections.Shapes.Util.Intersect(ref a.Shape, ref b.Shape, out Vector2 normal, out float depth))
                {
                    // Push the rigid away from the kinematic.
                    a.Position -= normal * depth;

                    // Add to local list.
                    localManifolds.Add(new CollisionManifold(
                        normal,
                        Vector2.Zero,
                        Vector2.Zero,
                        depth,
                        _prToPkBroadIntersects[i].Item1,
                        _prToPkBroadIntersects[i].Item2
                    ));
                }

                return localManifolds;
            },
            localManifolds =>
            {
                // Merge results once per thread (minimal locking).
                lock (_prToPkManifolds)
                {
                    _prToPkManifolds.AddRange(localManifolds);
                }
        });


        Parallel.For(0, _crToPkBroadIntersects.Count,
            () => new List<CollisionManifold>(), // thread-local list initializer
            (i, state, localManifolds) => {

                ref CirclePhysicsBody a = ref circleRigidBodies.GetData(_crToPkBroadIntersects[i].Item1);
                ref PolygonPhysicsBody b = ref polygonKinematicBodies.GetData(_crToPkBroadIntersects[i].Item2);

                if (Collections.Shapes.Util.Intersect(ref b.Shape, ref a.Shape, out Vector2 normal, out float depth)) {
                    a.Position += normal * depth * 0.5f;

                    localManifolds.Add(new CollisionManifold(
                        normal,
                        Vector2.Zero,
                        Vector2.Zero,
                        depth,
                        _crToPkBroadIntersects[i].Item1,
                        _crToPkBroadIntersects[i].Item2
                    ));
                }

                return localManifolds;
            },
            localManifolds => {
                lock (_crToPkManifolds) {
                    _crToPkManifolds.AddRange(localManifolds);
                }
        });


        Parallel.For(0, _prToCkBroadIntersects.Count, i=>{

        });
        NarrowTimer.Stop();
    }


    /// <summary>
    /// Performs a collision response to all colliding physics bodies.
    /// </summary>
    /// <param name="deltaTime"></param>

    private void ResponseStep(float deltaTime){
        
        ResponseTimer.Reset();
        ResponseTimer.Start();

        // collision response for circle rigid bodies.

        Parallel.For(0, _crToCrManifolds.Count, i=>{
        // for(int i = 0; i < _crToCrContacts.Count; i++){
            CollisionManifold manifold = _crToCrManifolds[i];
            ResolveRigidToRigidCollision(
                ref circleRigidBodies.GetData(manifold.BodyIndexA).PhysicsBody,
                ref circleRigidBodies.GetData(manifold.BodyIndexB).PhysicsBody,
                manifold.Normal,
                manifold.Depth
            );
        // }
        });
        
        // collision response for polygon rigid bodies.

        Parallel.For(0, _prToPrManifolds.Count, i=>{
        // for(int i = 0; i < _prToPrContacts.Count; i++){
            CollisionManifold manifold = _prToPrManifolds[i];
            ResolveRigidToRigidCollision(
                ref polygonRigidBodies.GetData(manifold.BodyIndexA).PhysicsBody,
                ref polygonRigidBodies.GetData(manifold.BodyIndexB).PhysicsBody,
                manifold.Normal,
                manifold.Depth
            );
        // }
        });

        // collision response for polygon to circle rigid bodies.

        Parallel.For(0, _crToPrManifolds.Count, i=>{
        // for(int i = 0; i < _prToCrContacts.Count; i++){

            CollisionManifold manifold = _crToPrManifolds[i];
            ResolveRigidToRigidCollision(
                ref circleRigidBodies.GetData(manifold.BodyIndexA).PhysicsBody,
                ref polygonRigidBodies.GetData(manifold.BodyIndexB).PhysicsBody,
                manifold.Normal,
                manifold.Depth
            );
        // }
        });


        // collision response for polygon kinematic to rigid bodies.

        Parallel.For(0, _prToPkManifolds.Count, i=>{
        // for(int i = 0; i < _pkToPrContacts.Count; i++){
            CollisionManifold manifold = _prToPkManifolds[i];
            ResolveRigidToKinematicCollision(
                ref polygonRigidBodies.GetData(manifold.BodyIndexA).PhysicsBody,
                ref polygonKinematicBodies.GetData(manifold.BodyIndexB).PhysicsBody,
                manifold.Normal,
                manifold.Depth
            );
        // }
        });

        // // collision response for circle kinematic to rigid bodies.

        // Parallel.For(0, _crToCkContacts.Count, i=>{
        // // for(int i = 0; i < _ckToCrContacts.Count; i++){

        //     CollisionManifold manifold = _crToCkContacts[i];
        //     ResolveKinematicToRigidCollision(
        //         ref circleKinematicBodies.GetData(manifold.BodyIndexB).PhysicsBody,
        //         ref circleRigidBodies.GetData(manifold.BodyIndexA).PhysicsBody,
        //         manifold.Normal,
        //         manifold.Depth
        //     );
        // // }
        // });

        // // collision response for polygon kinematic to circle rigid bodies.

        Parallel.For(0, _crToPkManifolds.Count, i=>{
        // for(int i = 0; i < _pkToCrContacts.Count; i++){

            CollisionManifold manifold = _crToPkManifolds[i];
            ResolveRigidToKinematicCollision(
                ref circleRigidBodies.GetData(manifold.BodyIndexA).PhysicsBody,
                ref polygonKinematicBodies.GetData(manifold.BodyIndexB).PhysicsBody,
                manifold.Normal,
                manifold.Depth
            );
        // }
        });

        // // collision response for circle kinematic to polygon rigid bodies.

        // Parallel.For(0, _prToCkContacts.Count, i=>{
        // // for(int i = 0; i < _ckToPrContacts.Count; i++){
        //     CollisionManifold manifold = _prToCkContacts[i];
        //     ResolveRigidToCollision(
        //         ref circleKinematicBodies.GetData(manifold.BodyIndexB).PhysicsBody,
        //         ref circleRigidBodies.GetData(manifold.BodyIndexA).PhysicsBody,
        //         manifold.Normal,
        //         manifold.Depth
        //     );
        // // }
        // });

        ResponseTimer.Stop();
    }




    private void SpatialHashStep(float deltaTime){
        SpatialUpdateTimer.Reset();
        SpatialUpdateTimer.Start();

    
        Parallel.For(0, circleRigidBodies.ActiveSlots.Count, x => {
            int i = circleRigidBodies.ActiveSlots[x];
            ref CirclePhysicsBody body = ref circleRigidBodies.GetData(i);
            SpatialHash.Update(i + _crSpatialIndexLowerBound, body.SpatialHashIndices, body.Shape.Min, body.Shape.Max);
        });

        Parallel.For(0, polygonRigidBodies.ActiveSlots.Count, x => {
            int i = polygonRigidBodies.ActiveSlots[x];
            ref PolygonPhysicsBody body = ref polygonRigidBodies.GetData(i);
            SpatialHash.Update(i + _prSpatialIndexLowerBound, body.SpatialHashIndices, body.Shape.Min, body.Shape.Max);
        });

        SpatialUpdateTimer.Stop();
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