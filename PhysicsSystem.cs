using HowlEngine.Math;
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

    public List<Collections.Shapes.Rectangle> CopyBoxRigidBodyColliders(){

        // create the copy list.

        List<Collections.Shapes.Rectangle> colliders = new List<Collections.Shapes.Rectangle>();
        
        // copy all colliders.
        
        for(int i = 0; i < boxRigidBodies.Capacity; i++){
            if(boxRigidBodies.IsSlotActive(i) == true){
                ref BoxRigidBody body = ref boxRigidBodies.GetData(i);
                colliders.Add(body.collider);
            }
        }

        return colliders;
    }



    /// <summary>
    /// Gets a copy of all rigid circle body colliders in this physics system.
    /// </summary>
    /// <returns></returns>

    public List<Collections.Shapes.Circle> CopyCircleRigidBodyColliders(){

        // create the copy list.

        List<Collections.Shapes.Circle> colliders = new List<Collections.Shapes.Circle>();

        // copy all colliders.

        for(int i = 0; i < circleRigidBodies.Capacity; i++){
            if(circleRigidBodies.IsSlotActive(i) == true){
                ref CircleRigidBody body = ref circleRigidBodies.GetData(i);
                colliders.Add(body.collider);
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
        UpdateCircleRigidBodies(deltaTime);
    }


    private void UpdateCircleRigidBodies(float deltaTime){
        Parallel.For(0, circleRigidBodies.Capacity, i =>{
            // skip if the slot is not active.
            
            if(circleRigidBodies.IsSlotActive(i) == false){
                return;
            }

            // get the current circle.

            ref CircleRigidBody a = ref circleRigidBodies.GetData(i);

            // ensure the comparison between others only occurs once.
            
            for(int j = i + 1; j < circleRigidBodies.Capacity; j++){
                
                // get the other circle.

                ref CircleRigidBody b = ref circleRigidBodies.GetData(j);

                // if we currently intersect.

                if(a.collider.Intersects(b.collider, out Vector2 normal, out float depth) == true){

                    // push apart by half from eachother.

                    a.Position -= normal * depth * 0.5f;
                    b.Position += normal * depth * 0.5f;

                }
            }
        });
    }
}