use std::mem;

use avian3d::prelude::*;
use bevy::prelude::*;

use crate::{
    debug::DebugMode, ground::{Grounding, GroundingConfig}, Character, KinematicVelocity
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

        velocity.0 += gravity * time.delta_secs();
    }
}
pub(crate) fn character_friction(
    mut characters: Query<(&mut KinematicVelocity, &Grounding, &CharacterFriction)>,
    frictions: Query<&FrictionScale>,
    colliders: Query<&ColliderOf>,
    time: Res<Time>,
) {
    for (mut velocity, grounding, character_friction) in &mut characters {
        let Some(ground) = grounding.ground() else {
            continue;
        };

        let mut friction = character_friction.0;

        // Multiply friction by the friction scale
        if let Ok(s) = frictions.get(ground.entity) {
            friction *= s.0;
        } else if let Ok(collider_of) = colliders.get(ground.entity) {
            if let Ok(s) = frictions.get(collider_of.body) {
                friction *= s.0;
            }
        }

        let f = friction_factor(velocity.0, friction, time.delta_secs());

        velocity.0 *= f;
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
    for (
        character,
        move_input,
        mut character_velocity,
        grounding,
        movement,
        debug_mode,
    ) in &mut query
    {
        let Ok((direction, throttle)) = Dir3::new_and_length(move_input.value) else {
            continue;
        };

        if debug_mode {
            character_velocity.0 = direction * movement.target_speed * throttle;
            continue;
        }

        let mut desired_direction = *direction;
        let mut velocity = character_velocity.0;


        let move_accel = acceleration_with_horizontal_limit(
            velocity,
            desired_direction,
            movement.acceleration * throttle,
            movement.target_speed * throttle,
            time.delta_secs(),
            character.up,
        );

        character_velocity.0 += move_accel;
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
fn acceleration_with_horizontal_limit(
    velocity: Vec3,
    direction: Vec3,
    max_acceleration: f32,
    target_speed: f32,
    delta: f32,
    up: Dir3,
) -> Vec3 {
    // Split velocity into horizontal and vertical components
    let horizontal_velocity = velocity.reject_from(*up);
    let horizontal_speed = horizontal_velocity.length();

    // Project direction to horizontal plane
    let horizontal_direction = (direction - direction.project_onto(*up)).normalize_or_zero();

    // If we're under the speed limit, use normal acceleration
    if horizontal_speed < target_speed {
        let remaining_speed = target_speed - horizontal_speed;
        let max_accel_this_frame = max_acceleration * delta;
        let accel_magnitude = max_accel_this_frame.min(remaining_speed);
        return horizontal_direction * accel_magnitude;
    }

    // We're at max speed - redirect the horizontal velocity toward the input direction
    // while preserving the magnitude
    if horizontal_speed < 1e-6 {
        return Vec3::ZERO;
    }

    let current_horizontal_direction = horizontal_velocity / horizontal_speed;

    // Calculate how fast we can rotate toward the desired direction
    let rotation_rate = max_acceleration * delta / horizontal_speed; // radians per frame
    let max_rotation_this_frame = rotation_rate.min(1.0); // Cap at 1 radian per frame

    // Slerp (spherical linear interpolation) between current and desired direction
    let dot = current_horizontal_direction
        .dot(horizontal_direction)
        .clamp(-1.0, 1.0);
    let angle = dot.acos();

    let new_direction = if angle < 1e-6 {
        // Already pointing in the right direction
        current_horizontal_direction
    } else {
        let t = (max_rotation_this_frame / angle).min(1.0);
        current_horizontal_direction
            .lerp(horizontal_direction, t)
            .normalize()
    };

    // Return the acceleration needed to change to the new velocity
    let new_horizontal_velocity = new_direction * horizontal_speed;
    new_horizontal_velocity - horizontal_velocity
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
