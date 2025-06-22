use avian3d::prelude::*;
use bevy::{
    color::palettes::css::*,
    ecs::{intern::Interned, schedule::ScheduleLabel},
    input::InputSystem,
    prelude::*,
};

use debug::{CharacterGizmos, DebugHit, DebugMode, DebugMotion, DebugPoint};
use ground::{Ground, Grounding, GroundingConfig, ground_check, is_walkable};
use interactions::physics_interactions;
use movement::{
    CharacterDrag, CharacterFriction, CharacterGravity, CharacterMovement, MoveInput,
    character_acceleration, character_drag, character_friction, character_gravity,
    clear_movement_input, feet_position,
};
use platform::{
    InheritedVelocity, PhysicsMover, inherit_platform_velocity, move_with_platform,
    update_physics_mover, update_platform_velocity,
};
use projection::{CollisionState, Surface, align_with_surface, project_velocity};
use stepping::{MotionBudget, PassType, SteppingBehaviour, SteppingConfig};
use sweep::{CollideAndSlideConfig, MovementImpact, SweepHitData, collide_and_slide, sweep};

// ... (all the mod declarations and imports stay the same) ...

pub mod debug;
pub mod ground;
pub(crate) mod interactions;
pub mod movement;
pub mod platform;
pub(crate) mod projection;
pub mod stepping;
pub mod sweep;

pub mod prelude {
    pub use crate::{
        Character, CharacterPlugin, KinematicVelocity, OnGroundEnter, OnGroundLeave, OnStep,
        ground::{Grounding, GroundingConfig},
        movement::{
            CharacterDrag, CharacterFriction, CharacterGravity, CharacterMovement, MoveInput,
        },
        platform::PhysicsMover,
        stepping::{SteppingBehaviour, SteppingConfig},
        sweep::CollideAndSlideConfig,
    };
}

#[derive(SystemSet, Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub enum CharacterSystems {
    Prepare,
    Forces,
    PhysicsInteractions,
    ApplyMovement,
}

pub struct CharacterPlugin {
    schedule: Interned<dyn ScheduleLabel>,
}

impl Default for CharacterPlugin {
    fn default() -> Self {
        Self::new(FixedPostUpdate)
    }
}

impl CharacterPlugin {
    pub fn new<S: ScheduleLabel>(schedule: S) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Plugin for CharacterPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((debug::plugin,));
        app.init_resource::<MovementDebugConfig>();
        app.add_systems(Update, toggle_movement_debug);
        app.register_type::<(
            Character,
            PhysicsMover,
            CharacterMovement,
            CharacterFriction,
            CharacterGravity,
            CharacterDrag,
            SteppingConfig,
            SteppingBehaviour,
            Grounding,
            GroundingConfig,
        )>();

        app.add_systems(PreUpdate, clear_movement_input.before(InputSystem));

        app.configure_sets(
            self.schedule,
            (
                CharacterSystems::Prepare.before(CharacterSystems::PhysicsInteractions),
                CharacterSystems::PhysicsInteractions.before(PhysicsSet::Prepare),
                CharacterSystems::ApplyMovement.after(PhysicsSet::Sync),
            ),
        );

        app.add_systems(
            self.schedule,
            update_character_filter.in_set(CharacterSystems::Prepare),
        );

        app.add_systems(
            self.schedule,
            (
                character_drag,
                character_friction,
                character_gravity,
                character_acceleration,
            )
                // TODO: this should probably have it's own set
                .in_set(CharacterSystems::Prepare)
                .chain(),
        );

        app.add_systems(
            self.schedule,
            (
                update_platform_velocity,
                move_with_platform,
                move_character_three_pass,
            )
                .in_set(CharacterSystems::ApplyMovement)
                .chain(),
        );

        app.add_systems(
            self.schedule,
            physics_interactions.in_set(CharacterSystems::PhysicsInteractions),
        );

        app.add_systems(
            self.schedule,
            update_physics_mover.in_set(PhysicsSet::Prepare),
        );

        app.add_systems(
            PhysicsSchedule,
            depenetrate_character.in_set(NarrowPhaseSet::Last),
        );

        app.add_observer(inherit_platform_velocity);
    }
}

pub(crate) fn update_character_filter(
    mut query: Query<(Entity, &mut CollideAndSlideFilter, &CollisionLayers)>,
    sensors: Query<Entity, With<Sensor>>,
) {
    for (entity, mut filter, collidion_layers) in &mut query {
        // Filter out any entity not in the character's collision filter
        filter.0.mask = collidion_layers.filters;

        // Filter out all sensor entities along with the character entity
        filter.0.excluded_entities.clear();
        filter
            .0
            .excluded_entities
            .extend(sensors.iter().chain([entity]));
    }
}

pub(crate) fn depenetrate_character(
    mut commands: Commands,
    mut overlaps: Local<Vec<(Dir3, f32)>>,
    mut gizmos: Gizmos<CharacterGizmos>,
    collisions: Collisions,
    mut query: Query<(
        Entity,
        &Character,
        &mut KinematicVelocity,
        Option<(&mut Grounding, &GroundingConfig)>,
        &mut Transform,
    )>,
    colliders: Query<&ColliderOf, Without<Sensor>>,
) {
    for contacts in collisions.iter() {
        overlaps.clear();

        // Get the rigid body entities of the colliders (colliders could be children)
        let Ok([&ColliderOf { body: rb1 }, &ColliderOf { body: rb2 }]) =
            colliders.get_many([contacts.collider1, contacts.collider2])
        else {
            continue;
        };

        let other: Entity;

        let (entity, character, mut velocity, mut grounding, mut transform) =
            if let Ok(character) = query.get_mut(rb1) {
                other = rb2;
                character
            } else if let Ok(character) = query.get_mut(rb2) {
                other = rb1;
                character
            } else {
                continue;
            };

        // TODO: crease / corner handling (?)

        for manifold in &contacts.manifolds {
            let hit_normal = match entity == rb1 {
                true => -manifold.normal,
                false => manifold.normal,
            };

            let (surface, obstruction_normal, ground_normal) = match grounding.as_ref() {
                Some((grounding, grounding_settings)) => {
                    let surface =
                        Surface::new(hit_normal, grounding_settings.max_angle, character.up);
                    let ground_normal = grounding.normal();
                    let obstruction_normal = surface
                        .obstruction_normal(ground_normal, character.up)
                        .unwrap();
                    (surface, obstruction_normal, ground_normal)
                }
                None => {
                    let surface = Surface {
                        normal: Dir3::new(hit_normal).unwrap(),
                        is_walkable: false,
                    };
                    (surface, surface.normal, None)
                }
            };

            for contact in &manifold.points {
                let depth = contact.penetration * hit_normal.dot(*obstruction_normal);

                match overlaps.binary_search_by(|(_, d)| depth.total_cmp(d)) {
                    Ok(index) => {
                        if overlaps[index].0.dot(*obstruction_normal) > 1.0 - 1e-4 {
                            overlaps.push((obstruction_normal, depth));
                            overlaps.swap_remove(index);
                        } else {
                            overlaps.insert(index, (obstruction_normal, depth));
                        }
                    }
                    Err(index) => {
                        overlaps.insert(index, (obstruction_normal, depth));
                    }
                }
            }

            if velocity.0.dot(hit_normal) > 0.0 {
                continue;
            }

            match grounding {
                Some(_) => {
                    velocity.0 = project_velocity(
                        velocity.0,
                        *obstruction_normal,
                        surface.is_walkable,
                        ground_normal,
                        character.up,
                    );
                }
                None => {
                    velocity.0 = velocity.0.reject_from(*obstruction_normal);
                }
            }

            if surface.is_walkable {
                if let Some((grounding, _)) = grounding.as_mut() {
                    let ground = Ground::new(other, hit_normal);

                    if !grounding.is_grounded() {
                        commands.entity(entity).trigger(OnGroundEnter(ground));
                    }

                    **grounding = ground.into();
                }
            }
        }

        for i in 0..overlaps.len() {
            let (direction, depth) = overlaps[i];

            if depth <= 0.0 {
                continue;
            }

            gizmos.line_gradient(
                transform.translation,
                transform.translation - direction * depth,
                CRIMSON,
                CRIMSON.with_alpha(0.0),
            );

            transform.translation += direction * depth;

            for j in i..overlaps.len() {
                let (next_direction, ref mut next_depth) = overlaps[j];

                let fixed = f32::max(0.0, direction.dot(*next_direction) * depth);
                *next_depth -= fixed;
            }
        }
    }
}

/// Triggered when the character becomes grounded during a movement update.
///
/// This is only triggered for the last ground the character touched during the update and will not be triggered
/// if the character was already grounded prior to the start of the update.
#[derive(Event, Deref)]
pub struct OnGroundEnter(pub Ground);

/// Triggered when the character becomes ungrounded during a movement update.
///
/// This is only triggered if the character is ungrounded at the end of the update.
#[derive(Event, Deref)]
pub struct OnGroundLeave(pub Ground);

/// Triggered when a character stepped over an obstacle.
#[derive(Event)]
pub struct OnStep {
    /// The translation of the character before stepping.
    pub position_before_step: Vec3,
    /// The movement of the character during the step.
    pub step_offset: Vec3,
    pub hit: SweepHitData,
}

#[derive(Debug, Clone)]
pub(crate) struct MovementState {
    pub position: Vec3,
    pub velocity: Vec3,
    pub ground: Option<Ground>,
    pub remaining_time: f32, // PhysX-style time budget
    pub collision_flags: u32,
}

impl MovementState {
    fn new(position: Vec3, velocity: Vec3, delta_time: f32) -> Self {
        Self {
            position,
            velocity,
            ground: None,
            remaining_time: delta_time, // Start with full time budget
            collision_flags: 0,
        }
    }
}

fn decompose_velocity(velocity: Vec3, up_direction: Dir3) -> (Vec3, Vec3) {
    let vertical = velocity.project_onto(*up_direction);
    let horizontal = velocity - vertical;
    (vertical, horizontal)
}

fn compute_pass_vectors(
    velocity: Vec3,
    up_direction: Dir3,
    stepping_config: Option<&SteppingConfig>,
    is_grounded: bool,
) -> (Option<Vec3>, Option<Vec3>, Option<Vec3>) {
    let (vertical_component, horizontal_component) = decompose_velocity(velocity, up_direction);

    let dir_dot_up = velocity.dot(*up_direction);
    let has_horizontal_motion = horizontal_component.length_squared() > 1e-6;
    let is_moving_up = dir_dot_up > 0.0;

    // UP PASS: Artificial step preparation OR actual upward motion
    let up_vector = if is_moving_up {
        // Always prioritize actual upward motion (jumping)
        Some(vertical_component)
    } else if has_horizontal_motion && stepping_config.is_some() && is_grounded {
        // Only add artificial step offset when not jumping
        let step_offset = stepping_config.unwrap().max_step_up;
        Some(*up_direction * step_offset)
    } else {
        None
    };

    // SIDE PASS: Always do horizontal motion if present
    let side_vector = if has_horizontal_motion {
        Some(horizontal_component)
    } else {
        None
    };

    // DOWN PASS: Gravity + potentially undo artificial up motion
    // KEY FIX: Don't assume we stepped the full amount - this will be calculated dynamically
    let down_vector = if !is_moving_up {
        Some(vertical_component) // Just gravity, no step offset subtraction here
    } else {
        None // No down pass when jumping
    };

    (up_vector, side_vector, down_vector)
}

/// Resource to control movement debug logging
#[derive(Resource, Default)]
pub struct MovementDebugConfig {
    pub enabled: bool,
}

/// System to toggle movement debug logging with a key press
pub fn toggle_movement_debug(
    mut debug_config: ResMut<MovementDebugConfig>,
    input: Res<ButtonInput<KeyCode>>,
) {
    if input.just_pressed(KeyCode::F9) {
        debug_config.enabled = !debug_config.enabled;
        if debug_config.enabled {
            warn!("🔍 Movement debug logging ENABLED - Press F9 to disable");
        } else {
            warn!("🔇 Movement debug logging DISABLED - Press F9 to enable");
        }
    }
}

// Macro to conditionally log based on debug config
#[macro_export]
macro_rules! debug_log {
    ($debug_config:expr, $($arg:tt)*) => {
        if $debug_config.enabled {
            info!($($arg)*);
        }
    };
}

// Key changes based on actual PhysX behavior:
// 1. Ground state PERSISTS between frames (like mTouchedShape in PhysX)
// 2. Ground is validated/cleared only when movement happens
// 3. No additional ground checks after movement
// 4. Simple ground state management

pub(crate) fn move_character_three_pass(
    mut commands: Commands,
    spatial_query: SpatialQuery,
    debug_config: Res<MovementDebugConfig>,
    mut query: Query<(
        Entity,
        &Character,
        &CollideAndSlideConfig,
        &mut KinematicVelocity,
        Option<(&mut Grounding, &GroundingConfig)>,
        Option<(&SteppingConfig, &SteppingBehaviour)>,
        &mut Transform,
        &Collider,
        &CollideAndSlideFilter,
        Has<Sensor>,
        Option<&mut DebugMotion>,
        Has<DebugMode>,
    )>,
    time: Res<Time>,
) {
    for (
        entity,
        character,
        collide_and_slide_config,
        mut velocity,
        mut grounding,
        stepping_config,
        mut transform,
        collider,
        filter,
        is_sensor,
        mut debug_motion,
        debug_mode,
    ) in &mut query
    {
        if is_sensor {
            transform.translation += velocity.0 * time.delta_secs();
            continue;
        }

        let initial_velocity = velocity.0;
        let current_ground_normal = grounding.as_ref().and_then(|(g, _)| g.normal());
        let is_grounded = grounding.as_ref().map_or(false, |(g, _)| g.is_grounded());
        let original_height = transform.translation.dot(*character.up);

        debug_log!(debug_config, "=== MOVEMENT FRAME START ===");
        debug_log!(
            debug_config,
            "Initial velocity: {:?} (magnitude: {:.3})",
            initial_velocity,
            initial_velocity.length()
        );
        debug_log!(
            debug_config,
            "Is grounded: {}, Ground normal: {:?}",
            is_grounded,
            current_ground_normal
        );
        debug_log!(debug_config, "Position: {:?}", transform.translation);

        let duration = time.delta_secs();

        // PhysX decomposition
        let (vertical_component, horizontal_component) = decompose_velocity(velocity.0, character.up);
        let dir_dot_up = velocity.0.dot(*character.up);
        let has_horizontal_motion = horizontal_component.length_squared() > 1e-6;
        let is_moving_up = dir_dot_up > 0.0;

        // PhysX step offset logic - key insight: this is the INTENDED step offset, may be clamped later
        let mut step_offset = stepping_config.map(|(config, _)| config.max_step_up).unwrap_or(0.0);

        // Disable step offset when moving upward (PhysX line ~1296)
        if is_moving_up {
            step_offset = 0.0;
            debug_log!(debug_config, "Disabled step offset - character is moving upward");
        }

        // PhysX pass vector computation - EXACTLY like PhysX
        let up_vector = if is_moving_up {
            // Actual upward motion (jumping) - this is velocity * time
            Some(vertical_component * duration)
        } else if has_horizontal_motion && is_grounded && step_offset > 0.0 {
            // Artificial step offset (PhysX line ~1308) - this is pure displacement
            Some(*character.up * step_offset)
        } else {
            None
        };

        let side_vector = if has_horizontal_motion {
            // Horizontal motion - this is velocity * time  
            Some(horizontal_component * duration)
        } else {
            None
        };

        let down_vector = if !is_moving_up {
            // Gravity motion - this is velocity * time
            Some(vertical_component * duration)
        } else {
            None
        };

        debug_log!(
            debug_config,
            "Pass vectors - Up: {:?}, Side: {:?}, Down: {:?}",
            up_vector.map(|v| (v, v.length())),
            side_vector.map(|v| (v, v.length())),
            down_vector.map(|v| (v, v.length()))
        );
        debug_log!(debug_config, "Initial step_offset: {:.6}", step_offset);

        let mut movement_state = MovementState::new(transform.translation, velocity.0, duration);
        movement_state.ground = grounding.as_ref().and_then(|(g, _)| g.inner_ground());

        // PhysX state tracking booleans
        let mut validate_triangle_down = false;
        let mut validate_triangle_side = false;
        let mut hit_non_walkable = false;
        let mut collision_up = false;
        let mut collision_sides = false;
        let mut collision_down = false;

        debug_log!(
            debug_config,
            "Movement state initialized - Velocity: {:?}, Ground: {:?}, Remaining time: {:.3}",
            movement_state.velocity,
            movement_state.ground,
            movement_state.remaining_time
        );

        // PASS 1: UP
        if let Some(up_motion) = up_vector {
            debug_log!(debug_config, "--- UP PASS START ---");
            debug_log!(debug_config, "UP motion: {:?} (artificial: {})", up_motion, !is_moving_up);

            let initial_position = movement_state.position;
            
            let up_result = execute_pass(
                &mut movement_state,
                up_motion,
                PassType::Up,
                character,
                collider,
                transform.rotation,
                collide_and_slide_config,
                &filter.0,
                &spatial_query,
                stepping_config.map(|(config, _)| config),
                grounding.as_ref().map(|(_, config)| config).map(|v| &**v),
                &debug_config,
            );

            if up_result.had_collision {
                collision_up = true;
            }

            // CRITICAL PhysX step offset clamping (line ~1330)
            if has_horizontal_motion && !is_moving_up && step_offset > 0.0 {
                let actual_up_movement = (movement_state.position - initial_position).dot(*character.up);
                
                debug_log!(debug_config, "UP pass movement: intended={:.6}, actual={:.6}", 
                          step_offset, actual_up_movement);
                
                // Clamp step offset to prevent undoing more than we did
                if actual_up_movement < step_offset {
                    step_offset = actual_up_movement.max(0.0);
                    debug_log!(debug_config, "Clamped step offset: {:.6} -> {:.6}", 
                              stepping_config.map(|(config, _)| config.max_step_up).unwrap_or(0.0), 
                              step_offset);
                }
            }

            debug_log!(
                debug_config,
                "Post-up: ground={:?}, offset={:?}",
                movement_state.ground,
                movement_state.position - transform.translation
            );
        }

        // PASS 2: SIDE
        if let Some(side_motion) = side_vector {
            debug_log!(debug_config, "--- SIDE PASS START ---");

            let side_result = execute_pass(
                &mut movement_state,
                side_motion,
                PassType::Side,
                character,
                collider,
                transform.rotation,
                collide_and_slide_config,
                &filter.0,
                &spatial_query,
                stepping_config.map(|(config, _)| config),
                grounding.as_ref().map(|(_, config)| config).map(|v| &**v),
                &debug_config,
            );

            if side_result.had_collision {
                collision_sides = true;
                if side_result.hit_static_geometry {
                    validate_triangle_side = true;
                    debug_log!(debug_config, "Side pass hit static geometry - enabling side validation");
                }
            }

            debug_log!(
                debug_config,
                "Post-side: ground={:?}, offset={:?}",
                movement_state.ground,
                movement_state.position - transform.translation
            );
        }

        // Clear ground before DOWN pass (PhysX pattern)
        movement_state.ground = None;
        debug_log!(debug_config, "  Cleared ground state for DOWN pass");

        // PASS 3: DOWN with PhysX step offset subtraction
        if let Some(down_motion) = down_vector {
            let corrected_down_vector = if has_horizontal_motion && is_grounded && step_offset > 0.0 {
                // PhysX line ~1359: Undo our artificial up motion
                let corrected = down_motion - *character.up * step_offset;
                debug_log!(
                    debug_config,
                    "DOWN pass: original={:?}, corrected={:?} (subtracted step_offset {:.6})",
                    down_motion,
                    corrected,
                    step_offset
                );
                corrected
            } else {
                debug_log!(
                    debug_config,
                    "DOWN pass: no step offset subtraction (has_horizontal={}, was_grounded={}, step_offset={:.6})",
                    has_horizontal_motion,
                    is_grounded,
                    step_offset
                );
                down_motion
            };

            debug_log!(debug_config, "--- DOWN PASS START ---");

            let down_result = execute_pass(
                &mut movement_state,
                corrected_down_vector,
                PassType::Down,
                character,
                collider,
                transform.rotation,
                collide_and_slide_config,
                &filter.0,
                &spatial_query,
                stepping_config.map(|(config, _)| config),
                grounding.as_ref().map(|(_, config)| config).map(|v| &**v),
                &debug_config,
            );

            if down_result.had_collision {
                if dir_dot_up <= 0.0 {
                    collision_down = true;
                }
                if down_result.hit_static_geometry {
                    validate_triangle_down = true;
                    debug_log!(debug_config, "Down pass hit static geometry - enabling down validation");
                }
            }

            debug_log!(
                debug_config,
                "Post-down: ground={:?}, offset={:?}",
                movement_state.ground,
                movement_state.position - transform.translation
            );
        }

        // PhysX post-DOWN-pass slope validation
        if validate_triangle_down && has_horizontal_motion && !hit_non_walkable {
            if let Some(ground) = movement_state.ground {
                let grounding_config = grounding.as_ref().map(|(_, config)| config).map(|v| &**v);
                let max_slope = grounding_config.map_or(std::f32::consts::FRAC_PI_4, |g| g.max_angle);
                
                let slope_too_steep = ground.normal.dot(*character.up).acos() > max_slope;
                if slope_too_steep {
                    hit_non_walkable = true;
                    debug_log!(debug_config, "Post-down slope validation failed - surface too steep");
                }
            }
        }

        // Update grounding state and trigger events
        if let Some((grounding, _grounding_config)) = grounding.as_mut() {
            let old_ground = grounding.inner_ground;
            let new_ground = movement_state.ground;

            // ENHANCED PhysX-style ground validation
            let final_ground = if let Some(ground) = new_ground {
                let net_vertical_displacement = (movement_state.position - transform.translation).dot(*character.up);
                
                // More permissive validation - allow small upward displacement due to step offset
                let max_allowed_displacement = if has_horizontal_motion && is_grounded {
                    // Allow step offset + small tolerance
                    stepping_config.map(|(config, _)| config.max_step_up).unwrap_or(0.0) + collide_and_slide_config.skin_width
                } else {
                    collide_and_slide_config.skin_width * 2.0
                };
                
                if net_vertical_displacement > max_allowed_displacement {
                    debug_log!(debug_config, "Rejecting ground contact - too far from surface (displacement: {:.6}, max_allowed: {:.6})", 
                              net_vertical_displacement, max_allowed_displacement);
                    None
                } else {
                    Some(ground)
                }
            } else {
                None
            };

            match (old_ground, final_ground) {
                (Some(old_ground), None) => {
                    debug_log!(debug_config, "Lost ground: {:?}", old_ground);
                    commands.entity(entity).trigger(OnGroundLeave(old_ground));
                }
                (None, Some(new_ground)) => {
                    debug_log!(debug_config, "Gained ground: {:?}", new_ground);
                    commands.entity(entity).trigger(OnGroundEnter(new_ground));
                }
                (Some(old_ground), Some(new_ground)) if old_ground.entity != new_ground.entity => {
                    debug_log!(
                        debug_config,
                        "Changed ground: {:?} -> {:?}",
                        old_ground,
                        new_ground
                    );
                }
                _ => {
                    // Same ground or no change
                }
            }

            **grounding = Grounding::new(final_ground);
        }

        // Apply final results
        let final_displacement = movement_state.position - transform.translation;
        transform.translation = movement_state.position;

        debug_log!(
            debug_config,
            "Final displacement: {:?} (magnitude: {:.3})",
            final_displacement,
            final_displacement.length()
        );
        debug_log!(debug_config, "Net vertical displacement: {:.6}", 
                  final_displacement.dot(*character.up));

        // PhysX-style velocity handling: only modify when landing on ground AND moving downward
        let (vertical_velocity, horizontal_velocity) = decompose_velocity(velocity.0, character.up);

        if let Some(_ground) = movement_state.ground {
            let is_moving_downward = vertical_velocity.dot(*character.up) <= 0.0;
            
            if is_moving_downward {
                velocity.0 = horizontal_velocity;
                debug_log!(
                    debug_config,
                    "Velocity (landed): removed downward velocity, keeping horizontal={:?}",
                    horizontal_velocity
                );
            } else {
                debug_log!(
                    debug_config,
                    "Velocity (grounded, jumping): keeping all velocity={:?}",
                    velocity.0
                );
            }
        } else {
            debug_log!(
                debug_config,
                "Velocity (airborne): keeping original {:?}",
                velocity.0
            );
        }

        debug_log!(
            debug_config,
            "Velocity change: {:?} -> {:?} (magnitude: {:.3} -> {:.3})",
            initial_velocity,
            velocity.0,
            initial_velocity.length(),
            velocity.0.length()
        );

        debug_log!(
            debug_config,
            "Collision flags: UP={}, SIDE={}, DOWN={}, validate_tri_down={}, validate_tri_side={}, hit_non_walkable={}",
            collision_up,
            collision_sides, 
            collision_down,
            validate_triangle_down,
            validate_triangle_side,
            hit_non_walkable
        );

        // Debug motion tracking
        if let Some(debug_motion) = debug_motion.as_mut() {
            if debug_mode
                || debug_motion.points.back().map_or(true, |(_, point)| {
                    point.translation.distance_squared(transform.translation) > 0.01
                })
            {
                let mut point = transform.translation;
                point += feet_position(
                    collider,
                    transform.rotation,
                    character.up,
                    collide_and_slide_config.skin_width,
                ) / 1.5;

                debug_motion.push(
                    duration,
                    DebugPoint {
                        translation: transform.translation,
                        velocity: velocity.0,
                        hit: movement_state.ground.map(|ground| DebugHit {
                            point,
                            normal: *ground.normal,
                            is_walkable: true,
                        }),
                    },
                );
            }
        }

        debug_log!(debug_config, "=== MOVEMENT FRAME END ===\n");
    }
}

// Enhanced result struct to track PhysX-style state
#[derive(Debug, Clone)]
struct PassResult {
    had_collision: bool,
    hit_static_geometry: bool,
    contact_normal: Option<Vec3>,
}

fn execute_pass(
    movement_state: &mut MovementState,
    motion_vector: Vec3,
    pass_type: PassType,
    character: &Character,
    collider: &Collider,
    transform_rotation: Quat,
    collide_and_slide_config: &CollideAndSlideConfig,
    filter: &SpatialQueryFilter,
    spatial_query: &SpatialQuery,
    stepping_config: Option<&SteppingConfig>,
    grounding_config: Option<&GroundingConfig>,
    debug_config: &MovementDebugConfig,
) -> PassResult {
    // Use live ground state from movement_state (updated during passes)
    let current_ground_normal = movement_state.ground.map(|g| g.normal);
    let is_currently_grounded = movement_state.ground.is_some();

    debug_log!(
        debug_config,
        "  Execute pass {:?}: motion_vector={:?}, current_ground={:?}",
        pass_type,
        motion_vector,
        current_ground_normal
    );

    let config = match pass_type {
        PassType::Up => CollideAndSlideConfig {
            max_iterations: 1,
            ..*collide_and_slide_config
        },
        PassType::Side => CollideAndSlideConfig {
            max_iterations: 4,
            ..*collide_and_slide_config
        },
        PassType::Down => CollideAndSlideConfig {
            max_iterations: 1,
            ..*collide_and_slide_config
        },
    };

    // motion_vector is already the displacement we want to move
    let motion_displacement = motion_vector;
    
    debug_log!(debug_config, "  Motion displacement: {:?} (magnitude: {:.6})", 
              motion_displacement, motion_displacement.length());

    let result = collide_and_slide(
        collider,
        movement_state.position,
        transform_rotation,
        motion_displacement, // This is the displacement we want to move
        current_ground_normal,
        &config,
        filter,
        spatial_query,
        1.0, // delta=1.0 since motion_displacement is already in world units
        is_currently_grounded,
        character.up.into(),
        |velocity, surface| match pass_type {
            PassType::Up => velocity.reject_from(*surface.normal),
            PassType::Side | PassType::Down => {
                surface.project_velocity(velocity, current_ground_normal, character.up)
            }
        },
        |state, impact| {
            let surface = Surface::new(
                impact.hit.normal,
                grounding_config.map_or(std::f32::consts::FRAC_PI_4, |g| g.max_angle),
                character.up,
            );
            Some(surface)
        },
        debug_config,
    );

    // Update movement state with results
    movement_state.position += result.offset;
    // Don't update remaining_time since we're not using time-based integration

    // Track collision results
    let mut pass_result = PassResult {
        had_collision: false,
        hit_static_geometry: false,
        contact_normal: None,
    };

    // Ground comes from sweep results
    if let Some(ground) = result.ground {
        debug_log!(
            debug_config,
            "  Found ground in {:?} pass: {:?}",
            pass_type,
            ground
        );
        movement_state.ground = Some(ground);
        pass_result.had_collision = true;
        pass_result.hit_static_geometry = true;
        pass_result.contact_normal = Some(*ground.normal);
    }

    pass_result
}


// Rest of the structs and functions remain the same...
#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component, Default)]
#[require(
    RigidBody = RigidBody::Kinematic,
    Collider = Capsule3d::new(0.4, 1.0),
    CollideAndSlideConfig,
    CollideAndSlideFilter,
    KinematicVelocity,
    InheritedVelocity,
    Grounding,
    GroundingConfig,
    CharacterFriction,
    MoveInput,
)]
pub struct Character {
    pub up: Dir3,
}

impl Default for Character {
    fn default() -> Self {
        Self { up: Dir3::Y }
    }
}

#[derive(Component, Reflect, Debug, Default, Clone, Copy, Deref, DerefMut)]
#[reflect(Component)]
pub struct KinematicVelocity(pub Vec3);

#[derive(Component, Reflect, Default, Debug)]
#[reflect(Component)]
pub struct CollideAndSlideFilter(pub(crate) SpatialQueryFilter);
