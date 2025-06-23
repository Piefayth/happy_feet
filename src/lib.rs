use avian3d::prelude::*;
use bevy::{
    color::palettes::css::*,
    ecs::{intern::Interned, schedule::ScheduleLabel},
    input::InputSystem,
    prelude::*,
};

use debug::{CharacterGizmos, DebugHit, DebugMode, DebugMotion, DebugPoint};
use ground::{Ground, Grounding, GroundingConfig, is_walkable};
use movement::{
    CharacterDrag, CharacterFriction, CharacterGravity, CharacterMovement, MoveInput,
    character_acceleration, character_drag, character_friction, character_gravity,
    clear_movement_input, feet_position,
};
use projection::{Surface, align_with_surface};
use stepping::{SteppingBehaviour, SteppingConfig};
use sweep::{CollideAndSlideConfig, SweepHitData, sweep};

// ... (all the mod declarations and imports stay the same) ...

pub mod debug;
pub mod ground;
pub mod movement;
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
                move_character_physx_style,
            )
                .in_set(CharacterSystems::ApplyMovement)
                .chain(),
        );
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

        // Convert velocity to displacement for this frame
        let total_displacement = initial_velocity * duration;

        // PhysX decomposition into up/side/down vectors
        let (vertical_component, horizontal_component) =
            decompose_displacement(total_displacement, character.up);
        let dir_dot_up = total_displacement.dot(*character.up);
        let has_horizontal_motion = horizontal_component.length_squared() > 1e-6;
        let is_moving_up = dir_dot_up > 0.0;

        // PhysX step offset logic
        let mut step_offset = if is_moving_up {
            0.0 // PhysX: disable step offset when jumping (unless on moving platform)
        } else {
            stepping_config
                .map(|(config, _)| config.max_step_up)
                .unwrap_or(0.0)
        };

        // Store original position for walk experiment retry
        let original_position = transform.translation;
        let original_displacement = total_displacement;
        let current_ground = grounding.as_ref().and_then(|(g, _)| g.inner_ground());

        // =================================================================
        // MAIN MOVEMENT ATTEMPT
        // =================================================================
        let mut final_movement_state = execute_movement_attempt(
            original_position,
            original_displacement,
            step_offset,
            has_horizontal_motion,
            is_moving_up,
            character,
            collider,
            transform.rotation,
            collide_and_slide_config.skin_width,
            &filter.0,
            &spatial_query,
            grounding.as_ref().map(|(_, config)| &**config),
            &debug_config,
            false, // not walk experiment
            current_ground,
        );

        // =================================================================
        // WALK EXPERIMENT & RETRY LOGIC (PhysX style)
        // =================================================================
        if final_movement_state.hit_non_walkable {
            debug_log!(
                debug_config,
                "WALK EXPERIMENT: Hit unwalkable surface, retrying movement..."
            );

            // PhysX modifies the displacement based on non-walkable mode
            // CRITICAL: Only remove UPWARD motion, preserve downward motion (gravity)
            let (vertical_component, horizontal_component) =
                decompose_displacement(original_displacement, character.up);
            let vertical_is_upward = vertical_component.dot(*character.up) > 0.0;

            let modified_displacement = if vertical_is_upward {
                // Remove upward motion, keep only horizontal
                horizontal_component
            } else {
                // Keep both horizontal and downward motion (gravity should still work)
                original_displacement
            };

            debug_log!(
                debug_config,
                "WALK EXPERIMENT: Original displacement: {:?}",
                original_displacement
            );
            debug_log!(
                debug_config,
                "WALK EXPERIMENT: Modified displacement: {:?}",
                modified_displacement
            );
            debug_log!(
                debug_config,
                "WALK EXPERIMENT: Vertical was upward: {}",
                vertical_is_upward
            );

            // Retry with modified displacement
            let retry_result = execute_movement_attempt(
                original_position,
                modified_displacement,
                step_offset,
                has_horizontal_motion,
                !vertical_is_upward, // is_moving_up = false if we removed upward motion
                character,
                collider,
                transform.rotation,
                collide_and_slide_config.skin_width,
                &filter.0,
                &spatial_query,
                grounding.as_ref().map(|(_, config)| &**config),
                &debug_config,
                true, // this is walk experiment
                current_ground,
            );

            // Use retry result as final result
            final_movement_state = retry_result;
            debug_log!(debug_config, "WALK EXPERIMENT: Retry completed");
        }

        // PhysX collision flags from final result
        let collision_up = final_movement_state.collision_flags & 0x1 != 0;
        let collision_sides = final_movement_state.collision_flags & 0x2 != 0;
        let collision_down = final_movement_state.collision_flags & 0x4 != 0;

        // Update grounding state and trigger events
        if let Some((grounding, _)) = grounding.as_mut() {
            let old_ground = grounding.inner_ground;
            let new_ground = final_movement_state.ground;

            // PhysX-style ground validation with displacement check
            let final_ground = if let Some(ground) = new_ground {
                let net_vertical_displacement = (final_movement_state.current_position
                    - transform.translation)
                    .dot(*character.up);
                let max_allowed_displacement = if has_horizontal_motion && is_grounded {
                    stepping_config
                        .map(|(config, _)| config.max_step_up)
                        .unwrap_or(0.0)
                        + collide_and_slide_config.skin_width
                } else {
                    collide_and_slide_config.skin_width * 2.0
                };

                if net_vertical_displacement > max_allowed_displacement {
                    debug_log!(
                        debug_config,
                        "Rejecting ground - displacement {:.6} > allowed {:.6}",
                        net_vertical_displacement,
                        max_allowed_displacement
                    );
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
        let final_displacement = final_movement_state.current_position - transform.translation;
        transform.translation = final_movement_state.current_position;

        // PhysX velocity handling: remove downward velocity when landing
        let (vertical_velocity, horizontal_velocity) = decompose_velocity(velocity.0, character.up);
        if final_movement_state.ground.is_some() && vertical_velocity.dot(*character.up) <= 0.0 {
            velocity.0 = horizontal_velocity;
            debug_log!(
                debug_config,
                "Landed: removed downward velocity, keeping horizontal={:?}",
                horizontal_velocity
            );
        }

        debug_log!(
            debug_config,
            "Final displacement: {:?} (magnitude: {:.3})",
            final_displacement,
            final_displacement.length()
        );
        debug_log!(
            debug_config,
            "Collision flags: UP={}, SIDE={}, DOWN={}",
            collision_up,
            collision_sides,
            collision_down
        );
        debug_log!(
            debug_config,
            "Velocity: {:?} -> {:?}",
            initial_velocity,
            velocity.0
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
                        hit: final_movement_state.ground.map(|ground| DebugHit {
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

/// Execute a single movement attempt (either normal or walk experiment retry)
fn execute_movement_attempt(
    start_position: Vec3,
    displacement: Vec3,
    step_offset: f32,
    has_horizontal_motion: bool,
    is_moving_up: bool,
    character: &Character,
    collider: &Collider,
    transform_rotation: Quat,
    skin_width: f32,
    filter: &SpatialQueryFilter,
    spatial_query: &SpatialQuery,
    grounding_config: Option<&GroundingConfig>,
    debug_config: &MovementDebugConfig,
    is_walk_experiment: bool,
    current_ground: Option<Ground>, // Only need this one
) -> PhysXMovementState {
    let mut movement_state = PhysXMovementState::new(start_position);
    movement_state.ground = current_ground; // Initialize with current ground
    movement_state.walk_experiment = is_walk_experiment;

    let is_grounded = current_ground.is_some(); // Derive from current_ground

    let min_distance = skin_width * 0.1; // PhysX uses small minimum

    // PhysX decomposition into up/side/down vectors
    let (vertical_component, horizontal_component) =
        decompose_displacement(displacement, character.up);

    let step_offset_was_applied =
        has_horizontal_motion && step_offset > 0.0 && is_grounded && !is_walk_experiment;
    let up_vector = if is_moving_up && !is_walk_experiment {
        let mut up_motion = vertical_component;
        if step_offset_was_applied {
            up_motion += *character.up * step_offset;
        }
        Some(up_motion)
    } else if step_offset_was_applied {
        Some(*character.up * step_offset)
    } else {
        None
    };

    let side_vector = if has_horizontal_motion {
        Some(horizontal_component)
    } else {
        None
    };

    let down_vector = if !is_moving_up {
        Some(vertical_component)
    } else {
        None
    };

    // =================================================================
    // TARGETED FIX: Clear ground state when there's intentional upward movement
    // =================================================================
    // If there is any upward movement planned (from jumping or stepping up),
    // we must invalidate the old ground state. The character is now responsible
    // for finding a *new* ground with its subsequent DOWN pass.
    if up_vector.is_some() {
        debug_log!(
            debug_config, 
            "  {}: Upward motion detected, clearing initial ground state",
            if is_walk_experiment { "RETRY" } else { "MAIN" }
        );
        movement_state.ground = None;
    }
    // =================================================================

    debug_log!(
        debug_config,
        "  {} - Pass vectors - Up: {:?}, Side: {:?}, Down: {:?}",
        if is_walk_experiment { "RETRY" } else { "MAIN" },
        up_vector.map(|v| (v, v.length())),
        side_vector.map(|v| (v, v.length())),
        down_vector.map(|v| (v, v.length()))
    );

    let mut collision_up = false;
    let mut collision_sides = false;
    let mut collision_down = false;

    // PASS 1: UP (skipped in walk experiment)
    if let Some(up_motion) = up_vector {
        if !is_walk_experiment {
            movement_state.target_orientation = movement_state.current_position + up_motion;

            let had_collision = execute_physx_movement_pass(
                &mut movement_state,
                1,
                character,
                collider,
                transform_rotation,
                skin_width,
                filter,
                spatial_query,
                grounding_config,
                min_distance,
                "UP",
                debug_config,
            );

            if had_collision {
                collision_up = true;
            }

            movement_state.target_orientation = movement_state.current_position;
        } else {
            debug_log!(debug_config, "  RETRY: Skipping UP PASS (walk experiment)");
        }
    }

    // PASS 2: SIDE
    if let Some(side_motion) = side_vector {
        movement_state.target_orientation = movement_state.current_position + side_motion;

        let had_collision = execute_physx_movement_pass(
            &mut movement_state,
            4,
            character,
            collider,
            transform_rotation,
            skin_width,
            filter,
            spatial_query,
            grounding_config,
            min_distance,
            "SIDE",
            debug_config,
        );

        if had_collision {
            collision_sides = true;
        }
    }

    // PhysX Constrained Climbing Check (only in main attempt, not retry)
    if !is_walk_experiment && movement_state.validate_triangle_side && !is_moving_up {
        let max_slope_angle = grounding_config.map_or(std::f32::consts::FRAC_PI_4, |g| g.max_angle);
        let slope_is_unwalkable = !is_walkable(
            movement_state.contact_normal_side_pass,
            max_slope_angle,
            *character.up,
        );

        if slope_is_unwalkable {
            let half_height = collider.aabb(Vec3::ZERO, Quat::IDENTITY).size().y / 2.;
            let original_bottom_point = start_position.dot(*character.up) - half_height;

            let height_gained_too_much =
                movement_state.contact_point_height > original_bottom_point + step_offset;
            let is_trying_to_move_up = displacement.dot(*character.up) > 0.0;

            let constrained_by_ceiling = collision_up;

            if height_gained_too_much && is_trying_to_move_up {
                movement_state.hit_non_walkable = true;
                debug_log!(
                    debug_config,
                    "Constrained Climbing: Hit unwalkable side surface and gained too much height while trying to move up."
                );
            } else if constrained_by_ceiling && is_trying_to_move_up {
                movement_state.hit_non_walkable = true;
                debug_log!(
                    debug_config,
                    "Constrained Climbing: Hit unwalkable side surface while constrained by ceiling and trying to move up."
                );
            } else {
                debug_log!(
                    debug_config,
                    "Constrained Climbing: Hit unwalkable side surface but no climbing detected."
                );
                debug_log!(
                    debug_config,
                    "  Height gained: {}, trying to move up: {}, ceiling collision: {}",
                    height_gained_too_much,
                    is_trying_to_move_up,
                    constrained_by_ceiling
                );
            }
        }
    }

    // REMOVED: The aggressive ground clearing that was causing the oscillation
    // The DOWN pass will only clear ground if it actually moves and finds nothing

    // PASS 3: DOWN
    let down_motion = if !is_moving_up {
        vertical_component
    } else {
        Vec3::ZERO
    };

    let corrected_down_motion = if step_offset_was_applied {
        down_motion - *character.up * step_offset
    } else {
        down_motion
    };

    movement_state.target_orientation = movement_state.current_position + corrected_down_motion;

    let had_collision = execute_physx_movement_pass(
        &mut movement_state,
        1,
        character,
        collider,
        transform_rotation,
        skin_width,
        filter,
        spatial_query,
        grounding_config,
        min_distance,
        "DOWN",
        debug_config,
    );

    if had_collision && displacement.dot(*character.up) <= 0.0 {
        collision_down = true;
    }

    // PhysX slope validation (only in main attempt)
    if !is_walk_experiment && movement_state.validate_triangle_down && has_horizontal_motion {
        if let Some(ground) = movement_state.ground {
            let max_slope = grounding_config.map_or(std::f32::consts::FRAC_PI_4, |g| g.max_angle);
            let slope_too_steep = ground.normal.dot(*character.up).acos() > max_slope;
            if slope_too_steep {
                movement_state.hit_non_walkable = true;
                debug_log!(debug_config, "Slope validation failed - surface too steep");
            }
        }
    }

    // Store collision flags
    movement_state.collision_flags = 0;
    if collision_up {
        movement_state.collision_flags |= 0x1;
    }
    if collision_sides {
        movement_state.collision_flags |= 0x2;
    }
    if collision_down {
        movement_state.collision_flags |= 0x4;
    }

    movement_state
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
    // Fields to store side pass collision data
    pub contact_normal_side_pass: Vec3,
    pub contact_point_height: f32,
    // Walk experiment flag
    pub walk_experiment: bool,
}

impl PhysXMovementState {
    fn new(position: Vec3) -> Self {
        Self {
            current_position: position,
            target_orientation: position,
            ground: None,
            collision_flags: 0,
            validate_triangle_down: false,
            validate_triangle_side: false,
            hit_non_walkable: false,
            contact_normal_side_pass: Vec3::ZERO,
            contact_point_height: 0.0,
            walk_experiment: false,
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

fn physx_collision_response(
    state: &mut PhysXMovementState,
    current_direction: Vec3,
    hit_normal: Vec3,
) {
    // Calculate original amplitude for scaling
    let amplitude = (state.target_orientation - state.current_position).length();
    
    if amplitude < 1e-6 {
        return;
    }

    // PhysX computeReflexionVector: reflect = incoming - normal * 2 * dot(incoming, normal)
    let reflect_dir = current_direction - hit_normal * 2.0 * current_direction.dot(hit_normal);
    let reflect_dir = reflect_dir.normalize_or_zero();

    // PhysX decomposeVector: split reflection into normal and tangent components relative to hit normal
    let normal_component = reflect_dir.project_onto(hit_normal);
    let tangent_component = reflect_dir - normal_component;

    // PhysX parameters (from C++ code)
    let bump = 0.0;     // PhysX uses 0.0 - no bouncing away from surface
    let friction = 1.0; // PhysX uses 1.0 - full sliding along surface
    let normalize = false; // PhysX usually uses false

    // PhysX behavior: reset target to current position first
    state.target_orientation = state.current_position;

    // Add bump component (usually zero, so usually no effect)
    if bump != 0.0 {
        let normal_to_add = if normalize {
            normal_component.normalize_or_zero()
        } else {
            normal_component
        };
        state.target_orientation += normal_to_add * bump * amplitude;
    }

    // Add friction component (this is the sliding motion along the surface)
    if friction != 0.0 {
        let tangent_to_add = if normalize {
            tangent_component.normalize_or_zero()
        } else {
            tangent_component
        };
        state.target_orientation += tangent_to_add * friction * amplitude;
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
    let pass_label = if state.walk_experiment {
        format!("RETRY {}", pass_name)
    } else {
        pass_name.to_string()
    };

    debug_log!(debug_config, "  {} PASS START", pass_label);

    let mut had_collision = false;
    let original_direction = if let Some((dir, _)) = state.current_direction() {
        *dir
    } else {
        debug_log!(debug_config, "  {} PASS: No movement needed", pass_label);
        return false;
    };
    
    // PhysX iteration loop with ONLY actual PhysX termination conditions
    for iteration in 0..max_iterations {
        // Check if movement is complete
        let Some((current_direction, max_distance)) = state.current_direction() else {
            debug_log!(
                debug_config,
                "    {} iter {}: No more movement needed",
                pass_label,
                iteration
            );
            break;
        };

        // PhysX minimum distance check: if(Length<=min_dist) break;
        if max_distance <= min_distance {
            debug_log!(
                debug_config,
                "    {} iter {}: Distance {:.6} below minimum {:.6}",
                pass_label,
                iteration,
                max_distance,
                min_distance
            );
            break;
        }

        // PhysX "Quake2 hack": if((currentDirection.dot(direction)) <= 0.0f) break;
        if current_direction.dot(original_direction) <= 0.0 {
            debug_log!(
                debug_config,
                "    {} iter {}: Direction reversed, stopping (Quake2 hack)",
                pass_label,
                iteration
            );
            break;
        }

        debug_log!(
            debug_config,
            "    {} iter {}: direction={:?}, distance={:.6}",
            pass_label,
            iteration,
            current_direction,
            max_distance
        );

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
            debug_log!(
                debug_config,
                "    {} iter {}: No collision, moving full distance",
                pass_label,
                iteration
            );
            if max_distance > min_distance {
                state.current_position = state.target_orientation;
            }
            
            // CRITICAL FIX: Only clear ground state if this was a DOWN pass
            // that moved the full intended distance without finding ground
            if pass_name == "DOWN" && max_distance > min_distance {
                debug_log!(
                    debug_config,
                    "    {} iter {}: DOWN pass found no ground after full movement, becoming airborne",
                    pass_label,
                    iteration
                );
                state.ground = None;
            }
            
            break;
        };

        had_collision = true;

        // PhysX: Move to collision point minus skin width
        let safe_distance = hit.distance.max(0.0);
        state.current_position += *current_direction * safe_distance;

        debug_log!(
            debug_config,
            "    {} iter {}: Hit at distance {:.6}, normal={:?}, entity={:?}",
            pass_label,
            iteration,
            safe_distance,
            hit.normal,
            hit.entity
        );

        // Check for walkable surface and update ground state
        if let Some(grounding_config) = grounding_config {
            let surface = Surface::new(hit.normal, grounding_config.max_angle, character.up);

            if surface.is_walkable && pass_name == "DOWN" {
                let ground = Ground::new(hit.entity, hit.normal);
                state.ground = Some(ground);
                debug_log!(
                    debug_config,
                    "    {} iter {}: Found walkable ground",
                    pass_label,
                    iteration
                );
            }

            if pass_name == "DOWN" && surface.is_walkable {
                state.validate_triangle_down = true;
            }

            if pass_name == "SIDE" {
                state.validate_triangle_side = true;
                state.contact_normal_side_pass = hit.normal;
                state.contact_point_height = hit.point.dot(*character.up);

                debug_log!(
                    debug_config,
                    "    {} iter {}: Stored side collision data - normal={:?}, height={:.6}",
                    pass_label,
                    iteration,
                    hit.normal,
                    state.contact_point_height
                );
            }
        }

        let response_normal = hit.normal;

        // PhysX collision response (no modifications)
        physx_collision_response(state, *current_direction, response_normal);

        debug_log!(
            debug_config,
            "    {} iter {}: After collision response, new target={:?}",
            pass_label,
            iteration,
            state.target_orientation
        );
    }

    debug_log!(
        debug_config,
        "  {} PASS END: had_collision={}, final_pos={:?}",
        pass_label,
        had_collision,
        state.current_position
    );

    had_collision
}

#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component, Default)]
#[require(
    RigidBody = RigidBody::Kinematic,
    Collider = Capsule3d::new(0.4, 1.0),
    CollideAndSlideConfig,
    CollideAndSlideFilter,
    KinematicVelocity,
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
