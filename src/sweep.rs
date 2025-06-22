use std::mem;

use avian3d::prelude::*;
use bevy::prelude::*;

use crate::{
    debug_log, ground::Ground, projection::Surface, CollideAndSlideFilter, CollisionState, MovementDebugConfig
};

#[derive(Reflect, Debug, Clone, Copy)]
pub struct SweepHitData {
    pub distance: f32,
    pub point: Vec3,
    pub normal: Vec3,
    pub entity: Entity,
}

/// Returns the safe hit distance and the hit data from the spatial query.
#[must_use]
pub(crate) fn sweep(
    shape: &Collider,
    origin: Vec3,
    rotation: Quat,
    direction: Dir3,
    max_distance: f32,
    skin_width: f32,
    spatial_query: &SpatialQuery,
    filter: &SpatialQueryFilter,
    ignore_origin_penetration: bool,
) -> Option<SweepHitData> {
    let hit = spatial_query.cast_shape(
        shape,
        origin,
        rotation,
        direction,
        &ShapeCastConfig {
            max_distance: max_distance + skin_width, // extend the trace slightly
            target_distance: skin_width, // I'm not sure what this does, but I think this is correct ;)
            ignore_origin_penetration,
            ..Default::default()
        },
        filter,
    )?;

    // How far is safe to translate by
    // let distance = hit.distance - skin_width;
    let distance = (hit.distance - skin_width).max(0.0);

    Some(SweepHitData {
        distance,
        point: hit.point1,
        normal: hit.normal1,
        entity: hit.entity,
    })
}

#[derive(Debug, Clone)]
pub(crate) struct MovementState {
    pub velocity: Vec3,
    pub offset: Vec3,
    pub remaining_time: f32,
    pub ground: Option<Ground>,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct MovementImpact {
    pub start: Vec3,
    pub end: Vec3,
    pub direction: Dir3,
    pub remaining_motion: f32,
    pub hit: SweepHitData,
}

#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component, Default)]
#[require(CollideAndSlideFilter)]
pub struct CollideAndSlideConfig {
    pub max_iterations: u8,
    pub skin_width: f32,
}

impl Default for CollideAndSlideConfig {
    fn default() -> Self {
        Self {
            max_iterations: 4,
            skin_width: 0.1,
        }
    }
}

pub(crate) fn collide_and_slide(
    shape: &Collider,
    origin: Vec3,
    rotation: Quat,
    velocity: Vec3,
    current_ground_normal: Option<Dir3>,
    config: &CollideAndSlideConfig,
    filter: &SpatialQueryFilter,
    spatial_query: &SpatialQuery,
    delta: f32,
    is_character_grounded: bool, // NEW PARAMETER
    up_direction: Dir3, // NEW PARAMETER
    mut project_velocity: impl FnMut(Vec3, Surface) -> Vec3,
    mut on_hit: impl FnMut(&mut MovementState, MovementImpact) -> Option<Surface>,
    debug_config: &MovementDebugConfig,
) -> MovementState {
    let mut state = MovementState {
        velocity,
        offset: Vec3::ZERO,
        remaining_time: delta,
        ground: None,
    };

    let mut previous_velocity = state.velocity;
    let mut collision_state = CollisionState::default();

    debug_log!(debug_config, "    Collide_and_slide START: velocity={:?}, delta={:.3}, max_iter={}, character_grounded={}", 
          velocity, delta, config.max_iterations, is_character_grounded);

    for iteration in 0..config.max_iterations {
        let Ok((direction, max_distance)) =
            Dir3::new_and_length(state.velocity * state.remaining_time)
        else {
            debug_log!(debug_config, "      Iteration {}: No movement needed (zero velocity)", iteration);
            break;
        };

        let start = origin + state.offset;

        debug_log!(debug_config, "      Iteration {}: direction={:?}, max_distance={:.3}, start={:?}", 
              iteration, direction, max_distance, start);

        let Some(hit) = sweep(
            shape,
            start,
            rotation,
            direction,
            max_distance,
            config.skin_width,
            spatial_query,
            filter,
            true,
        ) else {
            debug_log!(debug_config, "      Iteration {}: No collision, moving full distance", iteration);
            state.offset += direction * max_distance;
            break;
        };

        let time_consumed = hit.distance / max_distance;
        state.remaining_time *= 1.0 - time_consumed;
        state.offset += direction * hit.distance;

        debug_log!(debug_config, "      Iteration {}: HIT at distance={:.3}, normal={:?}, entity={:?}", 
              iteration, hit.distance, hit.normal, hit.entity);
        debug_log!(debug_config, "      Time consumed: {:.3}, remaining: {:.3}", time_consumed, state.remaining_time);

        let impact = MovementImpact {
            start,
            end: origin + state.offset,
            direction,
            remaining_motion: max_distance - hit.distance,
            hit,
        };

        let Some(surface) = on_hit(&mut state, impact) else {
            debug_log!(debug_config, "      Iteration {}: on_hit returned None, continuing", iteration);
            continue;
        };

        if surface.is_walkable {
            debug_log!(debug_config, "      Iteration {}: Setting walkable ground", iteration);
            state.ground = Some(Ground::new(hit.entity, hit.normal));
        }

        // *** KEY CHANGE: Handle stepping through position displacement, not velocity ***
        let blocked_velocity = state.velocity.dot(hit.normal);
        let is_moving_into_wall = blocked_velocity < 0.0;
        
        debug_log!(debug_config, "      Iteration {}: blocked_velocity={:.3}, is_character_grounded={}, is_moving_into_wall={}", 
                  iteration, blocked_velocity, is_character_grounded, is_moving_into_wall);
        
        if !surface.is_walkable && is_character_grounded && is_moving_into_wall {
            // STEPPING: Add upward position displacement instead of changing velocity
            let blocked_speed = blocked_velocity.abs();
            let step_height = blocked_speed * delta * 0.5; // Convert blocked velocity to step height
            
            debug_log!(debug_config, "      Iteration {}: STEPPING - blocked_speed={:.3}, step_height={:.3}", 
                      iteration, blocked_speed, step_height);
            
            // Add upward displacement for stepping
            state.offset += *up_direction * step_height;
        }

        let velocity_before_projection = state.velocity;
        
        state.velocity = collision_state.update(
            surface,
            state.velocity,
            mem::replace(&mut previous_velocity, state.velocity),
            current_ground_normal.is_some(),
            |vel| {
                let projected = project_velocity(vel, surface);
                debug_log!(debug_config, "        Velocity projection: {:?} -> {:?}", vel, projected);
                projected
            },
        );

        debug_log!(debug_config, "      Iteration {}: Velocity after collision_state.update: {:?} -> {:?}", 
              iteration, velocity_before_projection, state.velocity);
        
        if state.velocity.length_squared() < 1e-6 {
            debug_log!(debug_config, "      Iteration {}: Velocity too small, stopping", iteration);
            break;
        }
    }

    debug_log!(debug_config, "    Collide_and_slide END: final_offset={:?}, final_velocity={:?}, remaining_time={:.3}", 
          state.offset, state.velocity, state.remaining_time);

    state
}
