# PhysX Character Controller Movement Analysis

## Overview

The PhysX Character Controller uses a **three-pass sweep system** that decomposes movement into UP, SIDE, and DOWN components. This approach enables automatic step climbing while maintaining proper collision response.

## Core Algorithm Flow

### 1. Movement Decomposition
The controller decomposes the input movement vector into three components:
- **UP Vector**: Positive vertical motion (jumping) + artificial step offset
- **SIDE Vector**: Horizontal/tangential motion (walking)
- **DOWN Vector**: Negative vertical motion (gravity/falling)

```cpp
// Decompose direction into normal (up/down) and tangent (side) components
Ps::decomposeVector(normal_compo, tangent_compo, direction, upDirection);

if(dir_dot_up <= 0.0f)
    DownVector = normal_compo;  // Gravity/falling
else
    UpVector = normal_compo;    // Jumping

SideVector = tangent_compo;     // Horizontal movement
```

### 2. Step Offset Logic
The controller adds an artificial upward displacement to enable auto-stepping:
```cpp
if(!sideVectorIsZero)
    UpVector += upDirection * stepOffset;
```

**Key Rules:**
- Step offset is only added when there's horizontal movement
- Step offset is disabled when moving upward (jumping)
- Step offset is later "undone" in the down pass

## Three-Pass Movement System

### Pass 1: UP PASS
**Purpose**: Handle upward movement and create space for auto-stepping

**Behavior:**
- Sweeps the character upward by `UpVector`
- If collision occurs, collision flags are set to `COLLISION_UP`
- The step offset is clamped to prevent undoing more than was added
- **Position**: Updated to new swept position
- **Impact on following passes**: Creates vertical clearance for side movement

**Special Cases:**
- Skipped entirely in "walk experiment" mode (slope handling)
- Uses fewer iterations when no side movement is present

### Pass 2: SIDE PASS  
**Purpose**: Handle horizontal movement and detect slope/wall collisions

**Behavior:**
- Sweeps horizontally using `SideVector`
- Detects wall/slope collisions for climbing validation
- Sets `STF_VALIDATE_TRIANGLE_SIDE` flag when hitting static geometry
- **Position**: Updated to new swept position after collision response
- **Impact on following passes**: Affects down pass slope validation

**Collision Response:**
```cpp
// Actual function signature and behavior:
collisionResponse(targetOrientation, currentPosition, currentDirection, 
                 WorldNormal, Bump=0.0f, Friction=1.0f, normalize);

// What it does:
// 1. Computes reflection vector from currentDirection and hitNormal
// 2. Decomposes reflection into normal and tangent components
// 3. Sets targetOrientation = currentPosition (resets target)
// 4. Adds bump component: targetOrientation += normalCompo * bump * amplitude
// 5. Adds friction component: targetOrientation += tangentCompo * friction * amplitude
```

**Slope Detection:**
- Records `mContactNormalSidePass` for slope validation
- May prevent vertical motion if hitting ceiling while moving up

### Pass 3: DOWN PASS
**Purpose**: Handle downward movement, grounding, and slope validation

**Behavior:**
- **Undoes step offset**: `DownVector -= upDirection * stepOffset`
- Sweeps downward to find ground contact
- **Position**: Updated to final resting position
- Records detailed grounding information

**Grounding State Management:**
The down pass is responsible for determining and maintaining grounding state:

```cpp
// Grounding validation flags
mFlags |= STF_VALIDATE_TRIANGLE_DOWN;  // Mark that we have ground contact
mContactNormalDownPass = C.mWorldNormal;  // Store ground normal

// Store touched geometry for persistent tracking
touchedShapeOut = touchedShape;
touchedActorOut = touchedActor;
mTouchedPosShape_World = worldPos;
mTouchedPosShape_Local = shapeTransform.transformInv(worldPos);
```

**Slope Validation:**
After the down pass, the controller performs critical slope checks:

```cpp
if(mUserParams.mHandleSlope && (mFlags & STF_VALIDATE_TRIANGLE_DOWN)) {
    PxVec3 Normal = mContactNormalDownPass;
    if(testSlope(Normal, upDirection, mUserParams.mSlopeLimit)) {
        mFlags |= STF_HIT_NON_WALKABLE;
        // Triggers "walk experiment" retry if enabled
    }
}
```

## Position vs Velocity Handling

### Position Management
- **Direct Position Updates**: The controller directly modifies the character's position (`volume.mCenter`)
- **No Velocity Integration**: Unlike typical physics bodies, CCT doesn't use velocity/acceleration
- **Immediate Response**: Each pass immediately updates position based on sweep results

### Collision Response
When a collision occurs:
```cpp
// Move to collision point minus contact offset
if(C.mDistance > contactOffset)
    currentPosition += currentDirection * (C.mDistance - contactOffset);

// Calculate reflection for continued movement
collisionResponse(targetOrientation, currentPosition, currentDirection, 
                 WorldNormal, Bump, Friction);
```

## Inter-Pass Dependencies

### UP Pass → SIDE Pass
- UP pass creates vertical clearance
- Enables side movement over obstacles
- Failed UP pass may affect step offset clamping

### SIDE Pass → DOWN Pass  
- SIDE pass collision detection affects slope validation:
```cpp
if((mFlags & STF_VALIDATE_TRIANGLE_SIDE) && 
   testSlope(mContactNormalSidePass, upDirection, mUserParams.mSlopeLimit)) {
    // May prevent climbing based on side collision
    if(constrainedClimbingMode && contactHeight > originalBottom + stepOffset) {
        mFlags |= STF_HIT_NON_WALKABLE;
        return CollisionFlags; // Early exit
    }
}
```

### DOWN Pass Results
The down pass results can trigger a complete retry:
```cpp
if(mFlags & STF_HIT_NON_WALKABLE) {
    // Reset to original position and retry with modified movement
    mFlags |= STF_WALK_EXPERIMENT;
    volume.mCenter = Backup;
    
    // Retry with only horizontal component or projected movement
    if(mUserParams.mNonWalkableMode == ePREVENT_CLIMBING_AND_FORCE_SLIDING) {
        // Use only horizontal component of original displacement
        Ps::decomposeVector(xpDisp, tangent_compo, disp, upDirection);
    }
}
```

## Grounding State Details

The grounding system is a sophisticated multi-frame tracking mechanism that maintains persistent contact with supporting geometry.

### Ground Contact Detection

#### Initial Detection (findTouchedObject)
When no ground contact exists, the controller performs a raycast downward:

```cpp
void findTouchedObject() {
    const PxF32 probeLength = getHalfHeightInternal();  // Distance from center to feet
    const PxVec3 rayOrigin = toVec3(mPosition);         // Character center position
    
    // Raycast downward from character center
    if(mScene->raycast(rayOrigin, -upDirection, probeLength, hit, ...)) {
        // Store the geometry we're standing on
        mTouchedShape = hit.block.shape;
        mTouchedActor = hit.block.actor;
        
        // Calculate contact point positions
        const PxTransform shapeTransform = getShapeGlobalPose(*hit.block.shape, *hit.block.actor);
        
        // World space: relative to character center, offset by hit distance
        mTouchedPosShape_World = PxVec3(0) - upDirection * (probeLength - hit.block.distance);
        
        // Local space: contact point in the shape's coordinate system
        mTouchedPosShape_Local = shapeTransform.transformInv(PxVec3(0));
    }
}
```

**Key Insight**: The system stores contact points in BOTH coordinate systems:
- **World coordinates** (`mTouchedPosShape_World`): For detecting movement between frames
- **Local coordinates** (`mTouchedPosShape_Local`): Stays fixed relative to the moving object

#### Down Pass Contact Establishment
During movement, the DOWN pass updates ground contact:

```cpp
// When collision occurs in DOWN pass
if(sweepPass == SWEEP_PASS_DOWN) {
    // Update current ground contact
    touchedShapeOut = touchedShape;
    touchedActorOut = touchedActor;
    
    const PxTransform shapeTransform = getShapeGlobalPose(*touchedShape, *touchedActor);
    const PxVec3 worldPos = toVec3(C.mWorldPos);  // Actual collision point
    
    // Store contact point in both coordinate systems
    mTouchedPosShape_World = worldPos;
    mTouchedPosShape_Local = shapeTransform.transformInv(worldPos);
    
    // For static geometry, perform additional slope validation
    if(touchedActor->getConcreteType() == PxConcreteType::eRIGID_STATIC) {
        mFlags |= STF_VALIDATE_TRIANGLE_DOWN;
        
        // Calculate triangle height range for step validation
        const PxTriangle& tri = mWorldTriangles.getTriangle(C.mInternalIndex);
        // ... store triangle bounds and normal for slope testing
        mTouchedTriMin = minHeight;
        mTouchedTriMax = maxHeight;
        tri.normal(mContactNormalDownPass);
    }
}
```

### Ground Validation

The system validates ground contact through a **4-stage validation process** each frame:

#### Stage 1: Geometry Existence Check
```cpp
// Verify the shape still exists in the actor
PxU32 nbShapes = mTouchedActor->getNbShapes();
bool found = false;
for(PxU32 i = 0; i < nbShapes; i++) {
    PxShape* shape = NULL;
    mTouchedActor->getShapes(&shape, 1, i);
    if(mTouchedShape == shape) {
        found = true;
        break;
    }
}
if(!found) {
    // Shape was removed - invalidate ground contact
    mTouchedActor = NULL;
    mTouchedShape = NULL;
}
```

#### Stage 2: Scene Membership Check
```cpp
// Verify actor is still in the same scene
if(mTouchedActor->getScene() != mScene) {
    mTouchedShape = NULL;
    mTouchedActor = NULL;
}
```

#### Stage 3: Query Flag Validation
```cpp
// Verify shape is still collidable
if(!(mTouchedShape->getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE)) {
    mTouchedShape = NULL;
    mTouchedActor = NULL;
}
```

#### Stage 4: User Filter Validation
```cpp
// Apply user-defined filtering
if(!filterTouchedShape(filters)) {
    mTouchedShape = NULL;
    mTouchedActor = NULL;
}
```

#### Stage 5: Fallback Detection
```cpp
// If validation failed, attempt to find new ground contact
if(!mTouchedShape && (mTouchedObstacleHandle == INVALID_OBSTACLE_HANDLE)) {
    findTouchedObject(filters, obstacleContext, upDirection);
}
```

### Moving Platform Support

The dual coordinate system enables sophisticated moving platform tracking:

#### Platform Movement Detection
```cpp
bool rideOnTouchedObject() {
    if(mTouchedShape) {
        const PxRigidActor& rigidActor = *mTouchedActor.get();
        
        // Skip static geometry (no movement expected)
        if(rigidActor.getConcreteType() != PxConcreteType::eRIGID_STATIC) {
            
            // Only update when physics simulation has stepped
            const PxU32 timestamp = mScene->getTimestamp();
            bool canDoUpdate = (timestamp != mPreviousSceneTimestamp);
            
            if(canDoUpdate) {
                mPreviousSceneTimestamp = timestamp;
                
                // Get current shape transform
                const PxTransform shapeTransform = getShapeGlobalPose(*mTouchedShape.get(), rigidActor);
                
                // Previous frame: where contact point was in world space
                const PxVec3 posPreviousFrame = mTouchedPosShape_World;
                
                // Current frame: transform local contact point to current world position
                const PxVec3 posCurrentFrame = shapeTransform.transform(mTouchedPosShape_Local);
                
                // Calculate platform movement delta
                PxVec3 delta = posCurrentFrame - posPreviousFrame;
                
                // Apply platform movement to character
                if(!Ps::isAlmostZero(delta)) {
                    PxVec3 deltaUpDisp, deltaSideDisp;
                    Ps::decomposeVector(deltaUpDisp, deltaSideDisp, delta, upDirection);
                    
                    const bool deltaMovingUp = delta.dot(upDirection) > 0.0f;
                    
                    if(deltaMovingUp) {
                        // Platform moving up: immediately update character position
                        volume.mCenter += PxExtended(deltaUpDisp);
                    } else {
                        // Platform moving down: add to movement displacement
                        disp += deltaUpDisp;
                    }
                    
                    // Add horizontal platform motion
                    if(behaviorFlags & PxControllerBehaviorFlag::eCCT_CAN_RIDE_ON_OBJECT)
                        disp += deltaSideDisp;
                }
            }
        }
    }
}
```

#### Key Coordinate System Benefits

1. **Local Coordinates Persistence**: `mTouchedPosShape_Local` remains constant relative to the moving platform
2. **World Coordinates Detection**: `mTouchedPosShape_World` changes each frame, enabling movement detection
3. **Transform-Based Tracking**: Uses the platform's transform to convert between coordinate systems
4. **Timestamp Validation**: Only updates when physics simulation has actually stepped

#### Platform Movement Integration

The system handles platform movement differently based on direction:
- **Upward Movement**: Applied immediately to character position (prevents falling through rising platforms)
- **Downward Movement**: Added to movement displacement (allows natural falling)
- **Horizontal Movement**: Added to displacement if riding behavior is enabled

This sophisticated tracking system ensures characters smoothly follow moving platforms while maintaining proper physics behavior and avoiding common issues like:
- Falling through rapidly moving platforms
- Jittering on oscillating platforms  
- Incorrect movement accumulation over multiple frames
- Loss of contact due to transform precision issues

## "Walk Experiment" and Retry Mechanism

The "walk experiment" is a sophisticated retry system that activates when the character attempts to climb slopes that exceed the slope limit. Here's how it works:

### Trigger Conditions
The walk experiment activates when ALL of these conditions are met:
1. **Slope handling is enabled**: `mUserParams.mHandleSlope == true`
2. **Not touching CCT/obstacle**: `!(mFlags & (STF_TOUCH_OTHER_CCT|STF_TOUCH_OBSTACLE))`
3. **Valid ground contact**: `mFlags & STF_VALIDATE_TRIANGLE_DOWN`
4. **Moving down/horizontally**: `dir_dot_up <= 0.0f` (not jumping)
5. **Slope too steep**: `testSlope(Normal, upDirection, mUserParams.mSlopeLimit) == true`
6. **Triangle height exceeds step**: `touchedTriHeight > mUserParams.mStepOffset`

### Two-Phase Retry Process

#### Phase 1: Detection and Early Exit
```cpp
if(touchedTriHeight > mUserParams.mStepOffset && testSlope(Normal, upDirection, mUserParams.mSlopeLimit)) {
    mFlags |= STF_HIT_NON_WALKABLE;
    
    // First time detection - exit early to trigger retry
    if(!(mFlags & STF_WALK_EXPERIMENT))
        return CollisionFlags;
    
    // If we're already in retry mode, continue with recovery...
}
```

#### Phase 2: Full Retry with Modified Movement
```cpp
if(mCctModule.mFlags & STF_HIT_NON_WALKABLE) {
    // Reset to original position before any movement
    mCctModule.mFlags |= STF_WALK_EXPERIMENT;
    volume.mCenter = Backup;  // Complete position reset
    
    // Modify the displacement based on non-walkable mode
    PxVec3 xpDisp;
    if(mUserParams.mNonWalkableMode == ePREVENT_CLIMBING_AND_FORCE_SLIDING) {
        // Use only the horizontal component - removes all vertical climbing
        PxVec3 tangent_compo;
        Ps::decomposeVector(xpDisp, tangent_compo, disp, upDirection);
        // xpDisp gets the horizontal component, vertical is discarded
    } else {
        // Use original displacement (default behavior)
        xpDisp = disp;
    }
    
    // Retry the entire movement with modified displacement
    collisionFlags = mCctModule.moveCharacter(..., xpDisp, ...);
    
    mCctModule.mFlags &= ~STF_WALK_EXPERIMENT;
}
```

### Behavioral Changes During Walk Experiment

When `STF_WALK_EXPERIMENT` flag is set, the movement behavior changes:

1. **UP Pass is Skipped**:
```cpp
// In moveCharacter, UP pass section:
if(!(mFlags & STF_WALK_EXPERIMENT)) {
    // Normal UP pass logic
    if(doSweepTest(..., UpVector, ...)) {
        // Handle upward movement
    }
}
// When STF_WALK_EXPERIMENT is set, UP pass is completely bypassed
```

2. **Enhanced Collision Response**:
```cpp
// During walk experiment, use normalized response
mFlags |= STF_NORMALIZE_RESPONSE;

// This affects collisionResponse behavior:
if(preventVerticalMotion || 
   ((mFlags & STF_WALK_EXPERIMENT) && 
    (mUserParams.mNonWalkableMode != ePREVENT_CLIMBING_AND_FORCE_SLIDING))) {
    
    // Cancel out normal component - keep only tangential movement
    PxVec3 normalCompo, tangentCompo;
    Ps::decomposeVector(normalCompo, tangentCompo, WorldNormal, mUserParams.mUpDirection);
    WorldNormal = tangentCompo;
    WorldNormal.normalize();
}
```

3. **Recovery Movement**:
```cpp
// If still hitting non-walkable after retry, perform recovery
const PxExtended tmp = volume.mCenter.dot(upDirection);
float Delta = tmp > originalHeight ? float(tmp - originalHeight) : 0.0f;
Delta += fabsf(direction.dot(upDirection));
float Recover = Delta;

PxVec3 RecoverPoint = -upDirection * Recover;
// Sweep downward to recover from invalid position
doSweepTest(..., RecoverPoint, ...);
```

### Key Insights

1. **Complete State Reset**: The retry completely resets the character position to before any movement occurred
2. **Movement Modification**: The retry can use a modified displacement vector that removes vertical components
3. **Behavioral Override**: During retry, normal UP pass logic is disabled to prevent auto-stepping over non-walkable surfaces
4. **Fallback Recovery**: If the retry still fails, a recovery sweep moves the character away from the problematic geometry

The system essentially provides two strategies:
- **ePREVENT_CLIMBING_AND_FORCE_SLIDING**: Projects movement onto the horizontal plane, allowing sliding along slopes
- **Default mode**: Retries with original movement but modified collision response to prevent climbing

This sophisticated retry mechanism ensures characters can't exploit auto-stepping to climb surfaces that exceed the slope limit, while still providing smooth movement along valid surfaces.
