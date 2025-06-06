using System.Numerics;
using HowlEngine.Collections.Shapes;
using HowlEngine.Collections;

namespace HowlEngine.Physics;

public class PhysicsSystem{

    private StructPool<BoxRigidBody> boxRigidBodies;
    private StructPool<CircleRigidBody> circleRigidBodies;


    public PhysicsSystem(int boxRigidBodyAmount, int circleRigidBodyAmount){
        boxRigidBodies          = new StructPool<BoxRigidBody>(boxRigidBodyAmount);
        circleRigidBodies       = new StructPool<CircleRigidBody>(circleRigidBodyAmount);
    }


    /// <summary>
    /// Allocates a rigid box body to the internal data structure.
    /// </summary>
    /// <param name="body"></param>
    /// <returns>A Token to reference the newly allocated PhysicsBody.</returns>
    public Token AllocateBoxRigidBody(BoxRigidBody body){
        // allocate.
        Token token = boxRigidBodies.Allocate();
        if(token.Valid == false){
            return token;
        }

        boxRigidBodies.TryGetData(ref token).Data = body;
        return token;
    }


    /// <summary>
    /// Frees a rigid box body in the internal data structure at a given index.
    /// </summary>
    /// <param name="token">The specified token used as the index to free in the internal data structure..</param>

    public void FreeBoxRigidBody(ref Token token){
        boxRigidBodies.Free(token.Id);
    }

    /// <summary>
    /// Frees a rigid box body in the internal data structure at a given index.
    /// </summary>
    /// <param name="index">The specified index to free at.</param>
    public void FreeBoxRigidBody(int index){
        boxRigidBodies.Free(index);
    }


    /// <summary>
    /// Allocates a rigid circle body to the internal data structure.
    /// </summary>
    /// <param name="body"></param>
    /// <returns>A Token to reference the newly allocated PhysicsBody.</returns>
    public Token AllocateCircleRigidBody(CircleRigidBody body){
        // allocate.
        Token token = circleRigidBodies.Allocate();
        if(token.Valid == false){
            return token;
        }

        circleRigidBodies.TryGetData(ref token).Data = body;
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
    /// Frees a rigid box body in the internal data structure at a given index.
    /// </summary>
    /// <param name="index">The specified index to free at.</param>
    public void FreeCircleRigidBody(int index){
        circleRigidBodies.Free(index);
    }


    /// <summary>
    /// Gets a copy of all rigid box body colliders in this physics system.
    /// </summary>
    /// <returns></returns>

    public List<Vector2[]> CopyBoxRigidBodyColliders(){

        // create the copy list.

        List<Vector2[]> colliders = new List<Vector2[]>();
        
        // copy all colliders.
        
        for(int i = 0; i < boxRigidBodies.Capacity; i++){
            if(boxRigidBodies.IsSlotActive(i) == true){
                ref BoxRigidBody body = ref boxRigidBodies.GetData(i);
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
                ref CircleRigidBody body = ref circleRigidBodies.GetData(i);
                colliders.Add(body.Shape);
            }
        }

        return colliders;

    }

    
    /// <summary>
    /// Gets a RefView to directly access a BoxRigidBody within this physics system.
    /// </summary>
    /// <param name="token">The Token used to retrieve the PhysicsBody from the internal data structure.</param>
    /// <returns>A RefView of the retrieved data by the specified Token.</returns>

    public RefView<BoxRigidBody> GetBoxRigidBody(ref Token token){
        return boxRigidBodies.TryGetData(ref token);
    }


    /// <summary>
    /// Gets a RefView to directly access a CircleRigidBody within this physics system.
    /// </summary>
    /// <param name="token">The Token used to retrieve the PhysicsBody from the internal data structure.</param>
    /// <returns>A RefView of the retrieved data by the specified Token.</returns>

    public RefView<CircleRigidBody> GetCircleRigidBody(ref Token token){
        return circleRigidBodies.TryGetData(ref token);
    }


    public void FixedUpdate(float deltaTime){
        CircleRigidBodies(deltaTime);
        BoxRigidBodies(deltaTime);
        CircleAgainstBoxRigidBodies(deltaTime);
    }


    private void CircleRigidBodies(float deltaTime){
        Parallel.For(0, circleRigidBodies.Capacity, i =>{
            // skip if the slot is not active.
            
            if(circleRigidBodies.IsSlotActive(i) == false){
                return;
            }

            // get the current circle.

            ref CircleRigidBody a = ref circleRigidBodies.GetData(i);
            
            for(int j = i + 1; j < circleRigidBodies.Capacity; j++){ // <-- ensure the comparison between others only occurs once.
                
                if(circleRigidBodies.IsSlotActive(j) == false){
                    continue;
                }

                // get the other circle.

                ref CircleRigidBody b = ref circleRigidBodies.GetData(j);

                // if there is an intersection between them.

                if(Collections.Shapes.Util.Intersect(ref a.Shape, ref b.Shape, out Vector2 normal, out float depth) == true){

                    // push apart by half from eachother.

                    a.Position -= normal * depth * 0.5f;
                    b.Position += normal * depth * 0.5f;

                }
            }
        });
    }

    private void BoxRigidBodies(float deltaTime){
        Parallel.For(0, boxRigidBodies.Capacity, i => {
            // skip if not active.

            if(boxRigidBodies.IsSlotActive(i) == false){
                return;
                // continue;
            }
            
            // get the current body.

            ref BoxRigidBody a = ref boxRigidBodies.GetData(i);

            for(int j = i + 1; j < boxRigidBodies.Capacity; j++){ // <-- ensure the comparison between others only occurs once.
    
                if(boxRigidBodies.IsSlotActive(j) == false){
                    continue;
                }

                // get the other body.

                ref BoxRigidBody b = ref boxRigidBodies.GetData(j);

                // if there is an intersection between them.

                if(Collections.Shapes.Util.Intersect(ref a.Shape, ref b.Shape, out Vector2 normal, out float depth)){
                    
                    // push apart from eachother.
                    
                    a.Position -= normal * depth * 0.5f;
                    b.Position += normal * depth * 0.5f;
                }
            
            }
        });
    }

    private void CircleAgainstBoxRigidBodies(float deltaTime){
        Parallel.For(0,boxRigidBodies.Capacity, i => {
            
            // skip if not active.

            if(boxRigidBodies.IsSlotActive(i) == false){
                return;
            }

            // get the current body.

            ref BoxRigidBody a = ref boxRigidBodies.GetData(i);

            for(int j = 0; j < circleRigidBodies.Capacity; j++){
                
                // skip if not active.
                
                if(circleRigidBodies.IsSlotActive(j) == false){
                    continue;
                }

                // get the other body.

                ref CircleRigidBody b = ref circleRigidBodies.GetData(j);

                // if there is an intersection between them.
                
                if(Collections.Shapes.Util.Intersect(ref a.Shape, ref b.Shape, out Vector2 normal, out float depth)){
                    Console.WriteLine(1);

                    // push apart from eachother.

                    a.Position -= normal * depth * 0.5f;
                    b.Position += normal * depth * 0.5f;
                }
            }


        });
    }
}