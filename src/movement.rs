use std::mem;

use avian3d::prelude::*;
use bevy::prelude::*;

use crate::{
    Character, KinematicVelocity,
    debug::DebugMode,
    ground::{Grounding, GroundingConfig},
};

pub(crate) fn clear_movement_input(mut query: Query<&mut MoveInput>) {
    for mut move_input in &mut query {
        move_input.update();
    }
}

pub(crate) fn character_gravity(
    default_gravity: Res<Gravity>,
    mut query: Query<(
        &mut KinematicVelocity,
        Option<&CharacterGravity>,
        Option<&Grounding>,
        Option<&GravityScale>,
    )>,
    time: Res<Time>,
) {
    for (mut velocity, character_gravity, grounding, gravity_scale) in &mut query {
        // if grounding.map_or(false, |g| g.is_grounded()) {
        //     continue;
        // }

        let mut gravity = character_gravity.map_or(default_gravity.0, |g| g.0);

        if let Some(gravity_scale) = gravity_scale {
            gravity *= gravity_scale.0;
        }

        velocity.0 += gravity * time.delta_secs(); // TODO: dont 10x gravity
    }
}
pub(crate) fn character_friction(
    mut characters: Query<(&mut KinematicVelocity, &Grounding, &CharacterFriction)>,
    frictions: Query<&FrictionScale>,
    colliders: Query<&ColliderOf>,
    time: Res<Time>,
) {
    // You can tune this constant to get the desired air resistance effect.
    // A smaller value like 0.1-0.3 often works well for light drag.
    const DEFAULT_AIR_FRICTION: f32 = 0.8;

    for (mut velocity, grounding, character_friction) in &mut characters {
        // Determine the friction value based on whether the character is grounded or airborne.
        let friction = if let Some(ground) = grounding.ground() {
            // --- Grounded State ---
            // Start with the character's base ground friction.
            let mut ground_friction = character_friction.0;

            // Check the ground entity for a `FrictionScale` and apply it.
            if let Ok(scale) = frictions.get(ground.entity) {
                ground_friction *= scale.0;
            } else if let Ok(collider_of) = colliders.get(ground.entity) {
                // If not on the collider, check the parent body it's attached to.
                if let Ok(scale) = frictions.get(collider_of.body) {
                    ground_friction *= scale.0;
                }
            }
            ground_friction
        } else {
            // --- Airborne State ---
            // Not on the ground, so apply a constant air friction.
            DEFAULT_AIR_FRICTION
        };

        // Apply the calculated friction to the character's velocity.
        // This assumes you have a `friction_factor` function defined.
        let factor = friction_factor(velocity.0, friction, time.delta_secs());
        velocity.0 *= factor;
    }
}

pub(crate) fn character_drag(
    mut query: Query<(&mut KinematicVelocity, &CharacterDrag)>,
    time: Res<Time>,
) {
    for (mut velocity, drag) in &mut query {
        velocity.0 *= drag_factor(drag.0, time.delta_secs());
    }
}

pub(crate) fn character_acceleration(
    mut query: Query<(
        &Character,
        &MoveInput,
        &mut KinematicVelocity,
        Option<(&Grounding, &GroundingConfig)>,
        &CharacterMovement,
        Has<DebugMode>,
    )>,
    time: Res<Time>,
) {
    for (character, move_input, mut character_velocity, grounding, movement, debug_mode) in
        &mut query
    {
        let Ok((direction, throttle)) = Dir3::new_and_length(move_input.value) else {
            continue;
        };

        if debug_mode {
            character_velocity.0 = direction * movement.target_speed * throttle;
            continue;
        }

        let old_velocity = character_velocity.0;
        
        // Get the desired world-space movement direction
        let desired_world_movement = *direction * movement.target_speed * throttle;
        
        // Check if we're grounded and have a ground normal
        let ground_normal = grounding
            .and_then(|(grounding, _)| grounding.ground())
            .map(|ground| *ground.normal);
        
        let new_horizontal_velocity = if let Some(ground_normal) = ground_normal {
            // We're grounded - project movement onto the ground plane
            project_movement_onto_ground_plane(desired_world_movement, ground_normal, *character.up)
        } else {
            // We're airborne - use world-space horizontal movement
            desired_world_movement.reject_from(*character.up)
        };

        // Preserve vertical velocity and combine with new horizontal
        let vertical_velocity = old_velocity.project_onto(*character.up);
        character_velocity.0 = new_horizontal_velocity + vertical_velocity;
    }
}

/// Project desired movement onto the ground plane while preserving movement speed,
/// but clamp out any upward velocity to prevent artificial jumping
fn project_movement_onto_ground_plane(
    desired_movement: Vec3, 
    ground_normal: Vec3, 
    up_direction: Vec3
) -> Vec3 {
    // Remove any vertical component from the desired movement
    let horizontal_movement = desired_movement.reject_from(up_direction);
    
    if horizontal_movement.length_squared() < 1e-6 {
        return Vec3::ZERO;
    }
    
    // Project the horizontal movement onto the ground plane
    let projected_movement = horizontal_movement.reject_from(ground_normal);
    
    // CRITICAL: Clamp out any upward component to prevent artificial jumping
    let upward_component = projected_movement.dot(up_direction);
    let clamped_movement = if upward_component > 0.0 {
        // Remove the upward part, keep only horizontal and downward
        projected_movement - up_direction * upward_component
    } else {
        // Downward or purely horizontal - keep as is
        projected_movement
    };
    
    // Preserve the original horizontal speed by scaling the clamped vector
    let original_speed = horizontal_movement.length();
    let clamped_speed = clamped_movement.length();
    
    if clamped_speed > 1e-6 {
        clamped_movement * (original_speed / clamped_speed)
    } else {
        // If clamping resulted in near-zero vector, return zero movement
        Vec3::ZERO
    }
}

/// Used for moving a character based on it's [`MoveInput`].
#[derive(Component, Reflect, Debug)]
#[reflect(Component)]
#[require(MoveInput, KinematicVelocity)]
pub struct CharacterMovement {
    pub target_speed: f32,
    pub acceleration: f32,
}

impl CharacterMovement {
    pub const DEFAULT_GROUND: Self = Self {
        target_speed: 8.0,
        acceleration: 100.0,
    };

    pub const DEFAULT_AIR: Self = Self {
        target_speed: 8.0,
        acceleration: 20.0,
    };
}

/// The gravity force affecting a character while it's not grounded.
/// If no gravity is defined, then the [`Gravity`] resource will be used instead.
#[derive(Component, Reflect, Debug, Clone, Deref, DerefMut)]
#[reflect(Component, Default)]
#[require(KinematicVelocity)]
pub struct CharacterGravity(pub Vec3);

impl Default for CharacterGravity {
    fn default() -> Self {
        Self(Vec3::Y * -9.81)
    }
}

impl CharacterGravity {
    pub const ZERO: Self = Self(Vec3::ZERO);
    pub const EARTH: Self = Self(Vec3::new(0.0, -9.81, 0.0));
}

/// The friction scale when a character walks on an entity.
#[derive(Component, Reflect, Default, Debug, Deref, DerefMut)]
#[reflect(Component)]
pub struct FrictionScale(pub f32);

/// The friction applied to [`KinematicVelocity`] when a character is grounded.
/// Multiplied by the [`FrictionScale`] of the ground entity.
#[derive(Component, Reflect, Debug, Clone, Deref, DerefMut)]
#[reflect(Component)]
#[require(KinematicVelocity)]
pub struct CharacterFriction(pub f32);

impl Default for CharacterFriction {
    fn default() -> Self {
        Self(60.0)
    }
}

impl CharacterFriction {
    pub const ZERO: Self = Self(0.0);
}

/// The drag force applied to the [`KinematicVelocity`] of a character.
#[derive(Component, Reflect, Debug, Clone, Deref, DerefMut)]
#[reflect(Component)]
#[require(KinematicVelocity)]
pub struct CharacterDrag(pub f32);

impl Default for CharacterDrag {
    fn default() -> Self {
        Self(0.01)
    }
}

impl CharacterDrag {
    pub const ZERO: Self = Self(0.0);
}

/// The desired movement direction of a character.
/// The length of the value will be used to scale the acceleration and target speed when [`CharacterMovement`] is used.
#[derive(Component, Reflect, Default, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct MoveInput {
    pub value: Vec3,
    previous: Vec3,
}

impl MoveInput {
    pub fn update(&mut self) -> Vec3 {
        self.previous = mem::take(&mut self.value);
        self.previous
    }

    pub fn set(&mut self, value: Vec3) {
        self.value = value;
    }

    pub fn previous(&self) -> Vec3 {
        self.previous
    }
}

pub fn jump(impulse: f32, velocity: &mut KinematicVelocity, grounding: &mut Grounding, up: Dir3) {
    if grounding.inner_ground().is_none() {
        // can't air jump
        return;
    }
    // Remove vertical velocity
    velocity.0 = velocity.0.reject_from(*up);

    // Push the character away from ramps
    if let Some(ground) = grounding.detach() {
        let impulse = ground.normal * velocity.0.dot(*ground.normal).min(0.0);

        let vertical = impulse.project_onto(*up);
        let mut horizontal = impulse - vertical;

        // Reorient horizontal impulse to original velocity direction
        if let Ok(direction) = Dir3::new(velocity.0) {
            horizontal = horizontal.project_onto(*direction);
        }

        velocity.0 -= horizontal + vertical;
    }

    velocity.0 += up * impulse;
}

pub fn feet_position(shape: &Collider, rotation: Quat, up: Dir3, skin_width: f32) -> Vec3 {
    let aabb = shape.aabb(Vec3::ZERO, rotation);
    let down = aabb.min.dot(*up) - skin_width;
    up * down
}

#[must_use]
pub(crate) fn drag_factor(drag: f32, delta: f32) -> f32 {
    f32::exp(-drag * delta)
}

/// Constant acceleration in the opposite direction of velocity.
#[must_use]
pub(crate) fn friction_factor(velocity: Vec3, friction: f32, delta: f32) -> f32 {
    let speed_sq = velocity.length_squared();

    if speed_sq < 1e-4 {
        return 0.0;
    }

    f32::exp(-friction / speed_sq.sqrt() * delta)
}
