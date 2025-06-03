using System;
using System.Threading.Tasks;
using HowlEngine.Collections;
using HowlEngine.Math;

namespace HowlEngine.Physics;

//==========================================================================================
// NOTES:
//
// Broad Phase: A quick, simple check to rule out objects that definitely are not colliding.
//  - do a proximity check of double that of the bounding box width as a length.
//
// Narrow Phase: A more precise check only performed on objects that passed the broad phase.
// - AABB or SAT collision checking.
//
//==========================================================================================

public class AABBPhysicSystem{
    private StructPool<PhysicsBodyAABB> physicsBodies;
    private StructPool<RectangleCollider> staticBodies;

    public AABBPhysicSystem(int staticColliderAmount, int physicsBodyAmount){
        staticBodies = new StructPool<RectangleCollider>(staticColliderAmount);
        physicsBodies = new StructPool<PhysicsBodyAABB>(physicsBodyAmount);
    }

    /// <summary>
    /// Allocates a PhysicsBody to the internal data structure.
    /// </summary>
    /// <param name="body"></param>
    /// <returns>A Token to reference the newly allocated PhysicsBody.</returns>
    public Token AllocatePyhsicsBody(PhysicsBodyAABB body){
        // allocate.
        Token token = physicsBodies.Allocate();
        if(token.Valid == false){
            return token;
        }

        physicsBodies.TryGetData(ref token).Data = body;
        return token;
    }

    /// <summary>
    /// Allocates a StaticBody to the internal data structure.
    /// </summary>
    /// <param name="body"></param>
    /// <returns>A Token to reference the newly allocated StaticBody.</returns>
    public Token AllocateStaticBody(RectangleCollider body){
        // allocate.
        Token token = staticBodies.Allocate();
        if(token.Valid == false){
            return token;
        }

        // set data.
        staticBodies.TryGetData(ref token).Data = body;
        return token;
    }

    /// <summary>
    /// Frees a StaticBody in the internal data structure at a given index.
    /// </summary>
    /// <param name="index">The specified index to free at.</param>
    public void FreeStaticBody(int index){
        staticBodies.Free(index);
    }

    /// <summary>
    /// Frees a PhysicsBody in the internal data structure at a given index.
    /// </summary>
    /// <param name="index">The specified index to free at.</param>
    public void FreePhysicsBody(int index){
        physicsBodies.Free(index);
    }

    /// <summary>
    /// Frees a PhysicsBody in the internal data structure at a given index.
    /// </summary>
    /// <param name="index">The specified Token used to index an allocation to free.</param>
    public void FreePhysicsBody(ref Token token){
        physicsBodies.Free(token.Id);
    }

    /// <summary>
    /// Frees the last allocated StaticBody within the internal data structure.
    /// </summary>
    public void FreeLastStaticBody(){
        staticBodies.Free(staticBodies.Count - 1);
    }

    /// <summary>
    /// Frees the last allocated PhysicsBody within the internal data structure.
    /// </summary>
    public void FreeLastPhysicsBody(){
        physicsBodies.Free(physicsBodies.Count - 1);
    }

    /// <summary>
    /// Updates the Physics loop for all stored PhysicsBodies in the internal data structure.
    /// </summary>
    /// <param name="deltaTime"></param>
    public void FixedUpdate(float deltaTime){

        Vector2[] newPositions = new Vector2[physicsBodies.Capacity];
        Vector2[] newVelocities = new Vector2[physicsBodies.Capacity];

        // calculate new positions for each collider.

        Parallel.For(0, physicsBodies.Capacity, i =>{
            
            // pass the slot if its is not in use.
            
            if(physicsBodies.IsSlotActive(i) == false){
                return;
            }

            float?[]? timeOfImpact = null;

            // the primary PhysicsBody to check collisions for.
            
            ref PhysicsBodyAABB pA = ref physicsBodies.GetData(i);
            
            // check collisions against all other physics bodies.
            
            for(int j = 0; j < physicsBodies.Capacity; j++){
                // pass if it is the current collider or the slot ifit is not in use.
            
                if(j==i || physicsBodies.IsSlotActive(j) == false){
                    continue;
                }

                // the secondary PhysicsBody to check a collision against.
            
                ref PhysicsBodyAABB pB = ref physicsBodies.GetData(j);

                // Get the time of impact between the primary and secondary PhysicsBody.
            
                float?[]? newTimeOfImpact = SweptAABB(ref pA, ref pB);
                
                // If there is a new time, calculate if it is less than the current time of impact.
            
                // seperate x, from y.

                if(newTimeOfImpact != null){
                    if(timeOfImpact != null){
                        if(newTimeOfImpact[0] != null){
                            if(timeOfImpact[0] != null){
                                float? x1 = timeOfImpact[0];
                                float? x2 = newTimeOfImpact[0];
                                timeOfImpact[0] = x1<x2? x1 : x2;
                            }
                            else{
                                timeOfImpact[0] = newTimeOfImpact[0];
                            }

                        }
                        if(newTimeOfImpact[1] != null){
                            if(timeOfImpact[1] != null){
                                float? y1 = timeOfImpact[1];
                                float? y2 = newTimeOfImpact[1];
                                timeOfImpact[1] = y1<y2? y1 : y2;
                            }
                            else{
                                timeOfImpact[1] = newTimeOfImpact[1];
                            }
                        }
                    }
                    else{
                        timeOfImpact = newTimeOfImpact;
                    }
                }
            }

            // Apply the time of impact (if there is one) to the movement so we dont go into the other PhysicsBody.
            // Otherwise continue the movement with the same velocity.

            if(timeOfImpact != null){
                newPositions[i] = pA.Position + new Vector2(
                    pA.Velocity.X * (timeOfImpact[0] ?? 1f),
                    pA.Velocity.Y * (timeOfImpact[1] ?? 1f)
                );
                // newVelocities[i] = CalcElasticity(ref pA,timeOfImpact);  
            }
            else{
                newPositions[i] = pA.Position + pA.Velocity;
                newVelocities[i] = pA.Velocity;
            }
        });


        // Apply the new values.
    
        Parallel.For(0, physicsBodies.Capacity, i=>{
            if(physicsBodies.IsSlotActive(i) == false){
                return;
            }
            ref PhysicsBodyAABB body = ref physicsBodies.GetData(i);
            body.Velocity = newVelocities[i];
            body.Position = newPositions[i];
        });
    }

    /// <summary>
    /// Sets the Position of a StaticBody within this physics system.
    /// </summary>
    /// <param name="token">The specified Token to reference a StaticBody within the internal data structure.</param>
    /// <param name="x">The x-position to set the StaticBody to.</param>
    /// <param name="y">The y-position to set the StaticBody to.</param>
    public void SetStaticBodyPosition(ref Token token, int x, int y){
        RefView<RectangleCollider> rf = staticBodies.TryGetData(ref token);
        if(rf.Valid== false){
            return;
        }
        ref RectangleCollider box = ref rf.Data;
        box.X = x;
        box.Y = y;
    }

    /// <summary>
    /// Set the Velocity of a PhysicsBody within this physics system.
    /// </summary>
    /// <param name="token">The specified Token to reference a PhysicsBody within the internal data structure.</param>
    /// <param name="velocity">The velocity to assign to the specified PhysicsBody.</param>
    public void SetPhysicsBodyVelocity(ref Token token, Vector2 velocity){
        RefView<PhysicsBodyAABB> rf = GetPhysicsBody(ref token);
        if(rf.Valid==false){
            return;
        }
        ref PhysicsBodyAABB p = ref rf.Data;
        p.Velocity = velocity;
    }

    // /// <summary>
    // /// Draws all outlines of every collider within this physics system.
    // /// </summary>
    // /// <param name="spriteBatch">The SpriteBatch to batch all draw calls in.</param>
    // /// <param name="color">The colour to draw the outline.</param>
    // /// <param name="thickness">The thickness of the outline.</param>
    // public void DrawAllOutlines(SpriteBatch spriteBatch, Color color, int thickness){
    //     for(int i = 0; i < staticBodies.Capacity; i++){
    //         if(staticBodies.IsSlotActive(i)==false){
    //             continue;
    //         }
    //         DrawOutline(ref staticBodies.GetData(i), spriteBatch, color, thickness);
    //     }
    //     for(int i = 0; i < physicsBodies.Capacity; i++){
    //         if(physicsBodies.IsSlotActive(i)==false){
    //             continue;
    //         }
    //         ref PhysicsBodyAABB p = ref physicsBodies.GetData(i);
    //         ref RectangleCollider collider = ref p.Collider;
    //         DrawOutline(ref collider, spriteBatch, color, thickness);
    //     }
    // }

    // public void DrawOutline(ref Token[] tokens, SpriteBatch spriteBatch, Color color, int thickness){
    //     for(int i = 0; i < tokens.Length; i++){
    //         DrawOutline(ref tokens[i], spriteBatch, color, thickness);
    //     }
    // }

    // public void DrawOutline(ref Token token, SpriteBatch spriteBatch, Color color, int thickness){
    //     RefView<RectangleCollider> rf = staticBodies.TryGetData(ref token);
    //     if(rf.Valid == false){
    //         return;
    //     }
    //     DrawOutline(ref rf.Data,spriteBatch,color, thickness);
    // }

    // private void DrawOutline(ref RectangleCollider box, SpriteBatch spriteBatch, Color color, int thickness){
    //     // Top.
    //     spriteBatch.Draw(HowlApp.Instance.DebugTexture, new Rectangle(box.X, box.Y, box.Width, thickness), color);
    //     // Left.
    //     spriteBatch.Draw(HowlApp.Instance.DebugTexture, new Rectangle(box.X, box.Y, thickness, box.Height), color);
    //     // Right.
    //     spriteBatch.Draw(HowlApp.Instance.DebugTexture, new Rectangle(box.Right-thickness, box.Y, thickness, box.Height), color);
    //     // Bot.
    //     spriteBatch.Draw(HowlApp.Instance.DebugTexture, new Rectangle(box.X, box.Bottom-thickness, box.Width, thickness), color);
    // }


    /// <summary>
    /// Gets a RefView to directly access a PhysicBody within this physics system.
    /// </summary>
    /// <param name="token">The Token used to retrieve the PhysicsBody from the internal data structure.</param>
    /// <returns>A RefView of the retrieved data by the specified Token.</returns>
    public RefView<PhysicsBodyAABB> GetPhysicsBody(ref Token token){
        return physicsBodies.TryGetData(ref token);
    }

    //========================================================================================
    // When using AABB collision detection, there are four conditions that must be true
    // in order to say "a collision has occured". Given Bounding Box [A] and Bounding Box [B]
    // 
    //  1.  The left edge (x-position) of [A] must be less than the right edge
    //      (x-position + width) of [B].
    //
    //  2.  The right edge (x-position+width) of [A] must be less than the left edge
    //      (x-position) of [B].
    //
    //  3.  The top edge (y-position) of [A] must be less than the bottom edge
    //      (y-position + height) of [B].
    //
    //  4.  The bottom edge (y-position+height) of [A] must be less than the top edge
    //      (y-position) of [B].
    //
    //========================================================================================

    /// <summary>
    /// Checks for a collision between two rectangular structures using Axis-Aligned
    /// Bounding Box collision detection.
    /// </summary>
    /// <param name="boxA">The bounding box for the first structure.</param>
    /// <param name="boxB">The bounding box for the second structure.</param>
    /// <returns>true, if the two boxes are colliding; otherwise false.</returns>
    public bool AABB(ref RectangleCollider boxA, ref RectangleCollider boxB){
        return 
            boxA.Left < boxB.Right &&
            boxA.Right > boxB.Left &&
            boxA.Top < boxB.Bottom &&
            boxA.Bottom > boxB.Top;
    }

    //===========================================================================================
    // SweptAABB:
    // First pass:
    //  Check to see if there will be a collision on any of the axis.
    //  Retrieving a time value for when that colllision will occcur.
    //
    // Second Pass:
    //  Verify that the collision is not just on one axes but both.
    //  Performing an AABB collision check for the applied movement.
    //  If there has been a collision, the movement is effected by the calculated time of impact.
    //  If not, then the movement continues as normal.
    //===========================================================================================

    /// <summary>
    /// Calculates the time of impact between two physics bodies on the xy-axes.
    /// </summary>
    /// <param name="pA">The PhysicsBody to calculate the time of impact for.</param>
    /// <param name="pB">The PhysicsBody to check against.</param>
    /// <returns>A nullable float array. Index zero being the x-axis and index one being the y-axis.</returns>
    public static float?[]? SweptAABB(ref PhysicsBodyAABB pA, ref PhysicsBodyAABB pB){
        
        // get the relative velocity between both physics bodies.
        
        Vector2 relVel = pA.Velocity - pB.Velocity;

        // How far the body has to travel on the axes to start touching the other body.
        
        float xEntry, yEntry;

        // How far the body has travel on the axes to completely pass beyond the other body.
        
        float xExit, yExit;

        // if a body is moving the x-direction.
        // calculate the position to be considered a 'hit' and 'pass through'
        
        if (relVel.X > 0){
            // moving right.

            xEntry = pB.Left - pA.Right;
            xExit = pB.Right - pA.Left;
        }
        else{
            // moving left.

            xEntry = pB.Right - pA.Left;
            xExit = pB.Left - pA.Right;
        }

        // if a body is moving the x-direction.
        // calculate the position to be considered a 'hit' and 'pass through'
        
        if (relVel.Y > 0){
            // Moving downward
            
            yEntry = pB.Top - pA.Bottom;
            yExit = pB.Bottom - pA.Top;
        }
        else{
            // Moving upward

            yEntry = pB.Bottom - pA.Top;
            yExit = pB.Top - pA.Bottom;
        }

        // distance / velocity = time.
        // determine timings.

        float xEntryTime = (relVel.X == 0) ? float.NegativeInfinity : xEntry / relVel.X;
        float xExitTime = (relVel.X == 0) ? float.PositiveInfinity : xExit / relVel.X;
        float yEntryTime = (relVel.Y == 0) ? float.NegativeInfinity : yEntry / relVel.Y;
        float yExitTime = (relVel.Y == 0) ? float.PositiveInfinity : yExit / relVel.Y;

        // get the latest possible time of entry to ensure the body can move the full amount.

        float entryTime = System.Math.Max(xEntryTime, yEntryTime);

        // get the earliest possible time of exit to ensure the body can move the full amount.

        float exitTime = System.Math.Min(xExitTime, yExitTime);
        
        if (entryTime > exitTime || entryTime < 0f || entryTime > 1f)
            return null; // no collision within this frame
        
        // calculate the effected movement from the supposed impact.
        
        Vector2 pointOfImpact = pA.Position + pA.Velocity * entryTime;
        RectangleCollider movementA = new RectangleCollider((int)pointOfImpact.X, (int)pointOfImpact.Y, pA.Width, pA.Height);
        
        // Check if there is an AABB overlap at the point of impact.
        // If there is, it is a valid change in movement.
        
        bool validOverlap = false;
        float?[] timeOfImpact = new float?[2];
        if (xEntryTime > yEntryTime){
            if(movementA.Bottom > pB.Top && movementA.Top < pB.Bottom){
                timeOfImpact[0] = entryTime;
                timeOfImpact[1] = null;
                validOverlap = true;
            }
        }
        else{
            if(movementA.Right > pB.Left && movementA.Left < pB.Right){
                timeOfImpact[0] = null;
                timeOfImpact[1] = entryTime;
                validOverlap = true;
            }
        }

        if (!validOverlap)
            return null;

        return timeOfImpact;
    }

    // private Vector2 CalcElasticity(ref PhysicsBodyAABB body, float?[] timeOfImpact){
    //     Vector2 normal = Vector2.Zero;

    //     if(timeOfImpact[0] != null && timeOfImpact[1] != null){
    //         float min = System.Math.Min(timeOfImpact[0]??0, timeOfImpact[1]??0);
    //         if(timeOfImpact[0] == min){

    //             // if we impact on the left.

    //             if(min<0){
                    
    //                 // bounce right.

    //                 normal = Vector2.UnitX;
    //             }
                
    //             // if we impact on the right.

    //             else if(min > 0){
                
    //                 // bounce left.
                
    //                 normal = -Vector2.UnitX;
    //             }
    //         }

    //         else if(timeOfImpact[1] == min){

    //             // if we impact from below.

    //             if(min < 0){

    //                 // bounce down.

    //                 normal = Vector2.UnitY;

    //             }

    //             // if we impact from above.

    //             else if(min > 0){

    //                 // bounce up.

    //                 normal = -Vector2.UnitY;
    //             }

    //         }
    //     }

    //     else if(timeOfImpact[0]!=null){
            
    //         // if we impact on the left.

    //         if(timeOfImpact[0]<0){
                
    //             // bounce right.

    //             normal = Vector2.UnitX;
    //         }
            
    //         // if we impact on the right.

    //         else if(timeOfImpact[0] > 0){
            
    //             // bounce left.
            
    //             normal = -Vector2.UnitX;
    //         }
    //     }

    //     else if(timeOfImpact[1]!=null){
    //         // if we impact from below.

    //         if(timeOfImpact[1] < 0){

    //             // bounce down.

    //             normal = Vector2.UnitY;

    //         }

    //         // if we impact from above.

    //         else if(timeOfImpact[1] > 0){

    //             // bounce up.

    //             normal = -Vector2.UnitY;
    //         }            
    //     }

    //     return Vector2.Reflect(body.Velocity, normal) * body.Elasticity;
    // }

}
