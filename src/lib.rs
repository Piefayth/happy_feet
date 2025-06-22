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
                move_character_physx_style,
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

pub(crate) fn move_character_physx_style(
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
        let duration = time.delta_secs();

        debug_log!(debug_config, "=== PHYSX MOVEMENT FRAME START ===");
        debug_log!(debug_config, "Initial velocity: {:?} (magnitude: {:.3})", 
                  initial_velocity, initial_velocity.length());
        debug_log!(debug_config, "Is grounded: {}, Ground normal: {:?}", 
                  is_grounded, current_ground_normal);
        debug_log!(debug_config, "Position: {:?}", transform.translation);

        // Convert velocity to displacement for this frame
        let total_displacement = initial_velocity * duration;
        
        // PhysX decomposition into up/side/down vectors
        let (vertical_component, horizontal_component) = decompose_displacement(total_displacement, character.up);
        let dir_dot_up = total_displacement.dot(*character.up);
        let has_horizontal_motion = horizontal_component.length_squared() > 1e-6;
        let is_moving_up = dir_dot_up > 0.0;

        // PhysX step offset logic
        let mut step_offset = if is_moving_up {
            0.0 // Disable step offset when moving upward
        } else {
            stepping_config.map(|(config, _)| config.max_step_up).unwrap_or(0.0)
        };

        // PhysX pass vector computation
        let up_vector = if is_moving_up {
            Some(vertical_component) // Actual upward motion (jumping)
        } else if has_horizontal_motion && is_grounded && step_offset > 0.0 {
            Some(*character.up * step_offset) // Artificial step offset
        } else {
            None
        };

        let side_vector = if has_horizontal_motion {
            Some(horizontal_component) // Horizontal motion
        } else {
            None
        };

        let down_vector = if !is_moving_up {
            Some(vertical_component) // Gravity motion
        } else {
            None
        };

        debug_log!(debug_config, "Pass vectors - Up: {:?}, Side: {:?}, Down: {:?}",
                  up_vector.map(|v| (v, v.length())),
                  side_vector.map(|v| (v, v.length())),
                  down_vector.map(|v| (v, v.length())));

        // Initialize PhysX movement state
        let mut movement_state = PhysXMovementState::new(transform.translation, total_displacement);
        movement_state.ground = grounding.as_ref().and_then(|(g, _)| g.inner_ground());

        let min_distance = collide_and_slide_config.skin_width * 0.1; // PhysX uses small minimum
        let grounding_config_ref = grounding.as_ref().map(|(_, config)| &**config);

        // PhysX collision flags
        let mut collision_up = false;
        let mut collision_sides = false;
        let mut collision_down = false;

        // PASS 1: UP
        if let Some(up_motion) = up_vector {
            let backup_position = movement_state.current_position;
            
            // Temporarily set target for UP pass only
            movement_state.target_orientation = movement_state.current_position + up_motion;
            
            let had_collision = execute_physx_movement_pass(
                &mut movement_state,
                1, // PhysX uses maxIterUp (usually 1)
                character,
                collider,
                transform.rotation,
                collide_and_slide_config.skin_width,
                &filter.0,
                &spatial_query,
                grounding_config_ref,
                min_distance,
                "UP",
                &debug_config,
            );

            if had_collision {
                collision_up = true;
            }

            // PhysX step offset clamping
            if has_horizontal_motion && !is_moving_up && step_offset > 0.0 {
                let actual_up_movement = (movement_state.current_position - backup_position).dot(*character.up);
                if actual_up_movement < step_offset {
                    step_offset = actual_up_movement.max(0.0);
                    debug_log!(debug_config, "Clamped step offset to: {:.6}", step_offset);
                }
            }

            // Reset target for remaining passes
            movement_state.target_orientation = movement_state.current_position;
        }

        // PASS 2: SIDE
        if let Some(side_motion) = side_vector {
            // Add side motion to current target
            movement_state.target_orientation += side_motion;
            
            let had_collision = execute_physx_movement_pass(
                &mut movement_state,
                4, // PhysX uses maxIterSides
                character,
                collider,
                transform.rotation,
                collide_and_slide_config.skin_width,
                &filter.0,
                &spatial_query,
                grounding_config_ref,
                min_distance,
                "SIDE",
                &debug_config,
            );

            if had_collision {
                collision_sides = true;
            }
        }

        // Clear ground before DOWN pass (TODO: Does PhysX do this? is this right? it WORKS...)
        let executed_other_passes = up_vector.is_some() || side_vector.is_some();
        if executed_other_passes {
            movement_state.ground = None;
            debug_log!(debug_config, "Cleared ground state for DOWN pass after other passes");
        } else {
            debug_log!(debug_config, "Preserving ground state - DOWN pass only");
        }

        // PASS 3: DOWN with step offset correction
        if let Some(down_motion) = down_vector {
            let corrected_down_motion = if has_horizontal_motion && is_grounded && step_offset > 0.0 {
                // PhysX: Undo artificial up motion
                down_motion - *character.up * step_offset
            } else {
                down_motion
            };

            debug_log!(debug_config, "DOWN motion: original={:?}, corrected={:?}", 
                      down_motion, corrected_down_motion);

            // Add down motion to current target
            movement_state.target_orientation += corrected_down_motion;
            
            let had_collision = execute_physx_movement_pass(
                &mut movement_state,
                1, // PhysX uses maxIterDown (usually 1)
                character,
                collider,
                transform.rotation,
                collide_and_slide_config.skin_width,
                &filter.0,
                &spatial_query,
                grounding_config_ref,
                min_distance,
                "DOWN",
                &debug_config,
            );

            if had_collision && dir_dot_up <= 0.0 {
                collision_down = true;
            }
        }

        // PhysX slope validation
        if movement_state.validate_triangle_down && has_horizontal_motion {
            if let Some(ground) = movement_state.ground {
                let max_slope = grounding_config_ref.map_or(std::f32::consts::FRAC_PI_4, |g| g.max_angle);
                let slope_too_steep = ground.normal.dot(*character.up).acos() > max_slope;
                if slope_too_steep {
                    movement_state.hit_non_walkable = true;
                    debug_log!(debug_config, "Slope validation failed - surface too steep");
                }
            }
        }

        // Update grounding state and trigger events
        if let Some((grounding, _)) = grounding.as_mut() {
            let old_ground = grounding.inner_ground;
            let new_ground = movement_state.ground;

            // PhysX-style ground validation with displacement check
            let final_ground = if let Some(ground) = new_ground {
                let net_vertical_displacement = (movement_state.current_position - transform.translation).dot(*character.up);
                let max_allowed_displacement = if has_horizontal_motion && is_grounded {
                    stepping_config.map(|(config, _)| config.max_step_up).unwrap_or(0.0) + collide_and_slide_config.skin_width
                } else {
                    collide_and_slide_config.skin_width * 2.0
                };
                
                if net_vertical_displacement > max_allowed_displacement {
                    debug_log!(debug_config, "Rejecting ground - displacement {:.6} > allowed {:.6}", 
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
                    commands.entity(entity).trigger(OnGroundLeave(old_ground));
                }
                (None, Some(new_ground)) => {
                    commands.entity(entity).trigger(OnGroundEnter(new_ground));
                }
                _ => {}
            }

            **grounding = Grounding::new(final_ground);
        }

        // Apply final position
        let final_displacement = movement_state.current_position - transform.translation;
        transform.translation = movement_state.current_position;

        // PhysX velocity handling: remove downward velocity when landing
        let (vertical_velocity, horizontal_velocity) = decompose_velocity(velocity.0, character.up);
        if movement_state.ground.is_some() && vertical_velocity.dot(*character.up) <= 0.0 {
            velocity.0 = horizontal_velocity;
            debug_log!(debug_config, "Landed: removed downward velocity, keeping horizontal={:?}", 
                      horizontal_velocity);
        }

        debug_log!(debug_config, "Final displacement: {:?} (magnitude: {:.3})", 
                  final_displacement, final_displacement.length());
        debug_log!(debug_config, "Collision flags: UP={}, SIDE={}, DOWN={}", 
                  collision_up, collision_sides, collision_down);
        debug_log!(debug_config, "Velocity: {:?} -> {:?}", initial_velocity, velocity.0);

        // Debug motion tracking
        if let Some(debug_motion) = debug_motion.as_mut() {
            if debug_mode || debug_motion.points.back().map_or(true, |(_, point)| {
                point.translation.distance_squared(transform.translation) > 0.01
            }) {
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

        debug_log!(debug_config, "=== PHYSX MOVEMENT FRAME END ===\n");
    }
}

fn decompose_displacement(displacement: Vec3, up_direction: Dir3) -> (Vec3, Vec3) {
    let vertical = displacement.project_onto(*up_direction);
    let horizontal = displacement - vertical;
    (vertical, horizontal)
}

fn decompose_velocity(velocity: Vec3, up_direction: Dir3) -> (Vec3, Vec3) {
    let vertical = velocity.project_onto(*up_direction);
    let horizontal = velocity - vertical;
    (vertical, horizontal)
}

// Enhanced result struct to track PhysX-style state
#[derive(Debug, Clone)]
struct PassResult {
    had_collision: bool,
    hit_static_geometry: bool,
    contact_normal: Option<Vec3>,
}

#[derive(Debug, Clone)]
pub(crate) struct PhysXMovementState {
    pub current_position: Vec3,
    pub target_orientation: Vec3,
    pub ground: Option<Ground>,
    pub collision_flags: u32,
    // PhysX state flags
    pub validate_triangle_down: bool,
    pub validate_triangle_side: bool,
    pub hit_non_walkable: bool,
}

impl PhysXMovementState {
    fn new(position: Vec3, total_displacement: Vec3) -> Self {
        Self {
            current_position: position,
            target_orientation: position + total_displacement,
            ground: None,
            collision_flags: 0,
            validate_triangle_down: false,
            validate_triangle_side: false,
            hit_non_walkable: false,
        }
    }
    
    fn current_direction(&self) -> Option<(Dir3, f32)> {
        let displacement = self.target_orientation - self.current_position;
        let length = displacement.length();
        if length < 1e-6 {
            None
        } else {
            Some((Dir3::new_unchecked(displacement / length), length))
        }
    }
    
    fn remaining_distance(&self) -> f32 {
        (self.target_orientation - self.current_position).length()
    }
}

/// PhysX collision response - modifies target orientation, not velocity
fn physx_collision_response(
    state: &mut PhysXMovementState,
    current_direction: Vec3,
    hit_normal: Vec3,
) {
    // Calculate original amplitude for reflection scaling
    let amplitude = (state.target_orientation - state.current_position).length();
    
    if amplitude < 1e-6 {
        return;
    }
    
    // Compute reflection vector (PhysX computeReflexionVector)
    let reflect_dir = current_direction - hit_normal * 2.0 * current_direction.dot(hit_normal);
    let reflect_dir = reflect_dir.normalize_or_zero();
    
    // Decompose reflection into normal and tangent components
    let normal_component = reflect_dir.project_onto(hit_normal);
    let tangent_component = reflect_dir - normal_component;
    
    // PhysX collision response parameters
    let bump = 0.0;     // PhysX uses 0.0 for bump
    let friction = 1.0; // PhysX uses 1.0 for friction
    
    // CRITICAL: Reset target to current position first (PhysX behavior)
    state.target_orientation = state.current_position;
    
    // Apply reflection components
    if bump != 0.0 {
        let normal_normalized = normal_component.normalize_or_zero();
        state.target_orientation += normal_normalized * bump * amplitude;
    }
    if friction != 0.0 {
        let tangent_normalized = tangent_component.normalize_or_zero();
        state.target_orientation += tangent_normalized * friction * amplitude;
    }
}

/// Execute a single movement pass with PhysX-style iteration and termination
fn execute_physx_movement_pass(
    state: &mut PhysXMovementState,
    max_iterations: u32,
    character: &Character,
    collider: &Collider,
    transform_rotation: Quat,
    skin_width: f32,
    filter: &SpatialQueryFilter,
    spatial_query: &SpatialQuery,
    grounding_config: Option<&GroundingConfig>,
    min_distance: f32,
    pass_name: &str,
    debug_config: &MovementDebugConfig,
) -> bool {
    debug_log!(debug_config, "  {} PASS START", pass_name);
    
    let mut had_collision = false;
    let original_direction = if let Some((dir, _)) = state.current_direction() {
        *dir
    } else {
        debug_log!(debug_config, "  {} PASS: No movement needed", pass_name);
        return false;
    };
    
    // PhysX iteration loop with termination conditions
    for iteration in 0..max_iterations {
        // Check if movement is complete
        let Some((current_direction, max_distance)) = state.current_direction() else {
            debug_log!(debug_config, "    {} iter {}: No more movement needed", pass_name, iteration);
            break;
        };
        
        if max_distance <= min_distance {
            debug_log!(debug_config, "    {} iter {}: Distance {} below minimum {}", 
                      pass_name, iteration, max_distance, min_distance);
            break;
        }
        
        // PhysX "Quake2 hack" - prevent oscillation by checking direction reversal
        if current_direction.dot(original_direction) <= 0.0 {
            debug_log!(debug_config, "    {} iter {}: Direction reversed, stopping (Quake2 hack)", 
                      pass_name, iteration);
            break;
        }
        
        debug_log!(debug_config, "    {} iter {}: direction={:?}, distance={:.6}", 
                  pass_name, iteration, current_direction, max_distance);
        
        // Perform sweep
        let Some(hit) = sweep(
            collider,
            state.current_position,
            transform_rotation,
            current_direction,
            max_distance,
            skin_width,
            spatial_query,
            filter,
            true,
        ) else {
            // No collision - move to target
            debug_log!(debug_config, "    {} iter {}: No collision, moving full distance", 
                      pass_name, iteration);
            state.current_position = state.target_orientation;
            break;
        };
        
        had_collision = true;
        
        // Move to collision point minus skin width
        let safe_distance = hit.distance.max(0.0);
        state.current_position += *current_direction * safe_distance;
        
        debug_log!(debug_config, "    {} iter {}: Hit at distance {:.6}, normal={:?}, entity={:?}", 
                  pass_name, iteration, safe_distance, hit.normal, hit.entity);
        
        // Check for walkable surface and update ground state
        if let Some(grounding_config) = grounding_config {
            let surface = Surface::new(hit.normal, grounding_config.max_angle, character.up);
            
            if surface.is_walkable && pass_name == "DOWN" {
                let ground = Ground::new(hit.entity, hit.normal);
                state.ground = Some(ground);
                debug_log!(debug_config, "    {} iter {}: Found walkable ground", pass_name, iteration);
            }
            
            // Set validation flags for slope checking
            if pass_name == "DOWN" && surface.is_walkable {
                state.validate_triangle_down = true;
            }
            if pass_name == "SIDE" && !surface.is_walkable {
                state.validate_triangle_side = true;
            }
        }
        
        // Apply PhysX collision response
        physx_collision_response(state, *current_direction, hit.normal);
        
        debug_log!(debug_config, "    {} iter {}: After collision response, new target={:?}", 
                  pass_name, iteration, state.target_orientation);
    }
    
    debug_log!(debug_config, "  {} PASS END: had_collision={}, final_pos={:?}", 
              pass_name, had_collision, state.current_position);
    
    had_collision
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
