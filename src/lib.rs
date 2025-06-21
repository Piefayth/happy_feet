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
use stepping::{StepOutput, SteppingBehaviour, SteppingConfig, step_up, PassType, MotionBudget};
use sweep::{CollideAndSlideConfig, MovementImpact, SweepHitData, collide_and_slide, sweep};

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
            (update_platform_velocity, move_with_platform, move_character_three_pass)
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
    pub remaining_time: f32,  // PhysX-style time budget
    pub collision_flags: u32,
}

impl MovementState {
    fn new(position: Vec3, velocity: Vec3, delta_time: f32) -> Self {
        Self {
            position,
            velocity,
            ground: None,
            remaining_time: delta_time,  // Start with full time budget
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
    
    // DOWN PASS: Gravity + undo artificial up motion
    let down_vector = if !is_moving_up || has_horizontal_motion {
        let mut down = if is_moving_up { Vec3::ZERO } else { vertical_component };
        
        // Only subtract artificial up motion if we added it (and we're not jumping)
        if has_horizontal_motion && stepping_config.is_some() && is_grounded && !is_moving_up {
            let step_offset = stepping_config.unwrap().max_step_up;
            down -= *up_direction * step_offset;
        }
        
        Some(down)
    } else {
        None
    };
    
    (up_vector, side_vector, down_vector)
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
) -> MovementResult {
    let current_ground_normal = movement_state.ground.map(|g| g.normal);
    
    let config = match pass_type {
        PassType::Up => CollideAndSlideConfig {
            max_iterations: 1,
            ..collide_and_slide_config.clone()
        },
        PassType::Side => CollideAndSlideConfig {
            max_iterations: 4,
            ..collide_and_slide_config.clone()
        },
        PassType::Down => CollideAndSlideConfig {
            max_iterations: 1,
            ..collide_and_slide_config.clone()
        },
    };
    
    let mut step_occurred = false;
    let mut step_info = None;
    
    let result = collide_and_slide(
        collider,
        movement_state.position,
        transform_rotation,
        motion_vector,
        current_ground_normal,
        &config,
        filter,
        spatial_query,
        movement_state.remaining_time, // Use actual remaining time
        |velocity, surface| {
            match pass_type {
                PassType::Up => velocity.reject_from(*surface.normal),
                PassType::Side => surface.project_velocity(velocity, current_ground_normal, character.up),
                PassType::Down => surface.project_velocity(velocity, current_ground_normal, character.up),
            }
        },
        |state, impact| {
            let surface = Surface::new(
                impact.hit.normal,
                grounding_config.map_or(std::f32::consts::FRAC_PI_4, |g| g.max_angle),
                character.up,
            );
            
            // Try stepping only during SIDE pass
            if pass_type == PassType::Side && !surface.is_walkable {
                if let Some(stepping_config) = stepping_config {
                    // Calculate remaining horizontal motion based on time budget
                    let remaining_horizontal_velocity = motion_vector * state.remaining_time;
                    
                    if let Ok((direction, motion)) = Dir3::new_and_length(remaining_horizontal_velocity) {
                        info!("🦶 STEP ATTEMPT: direction={:?}, motion={:.3}, time_remaining={:.3}", 
                              direction, motion, state.remaining_time);
                        
                        if let Some(step_result) = step_up(
                            collider,
                            movement_state.position + state.offset,
                            transform_rotation,
                            direction,
                            motion,
                            character.up,
                            collide_and_slide_config.skin_width,
                            stepping_config,
                            filter,
                            spatial_query,
                            |hit| is_walkable(hit.normal, grounding_config.map_or(std::f32::consts::FRAC_PI_4, |g| g.max_angle), *character.up),
                        ) {
                            info!("✅ STEP SUCCESS: forward={:.3}, up={:.3}", step_result.step_forward, step_result.step_up);
                            
                            let offset = direction * step_result.step_forward + character.up * step_result.step_up;
                            
                            // PhysX-style: consume time based on horizontal distance traveled
                            let time_consumed = step_result.step_forward / motion.max(0.001);
                            let new_remaining_time = (state.remaining_time - time_consumed).max(0.0);
                            
                            info!("⏱️  TIME BUDGET: consumed {:.3} of {:.3}, remaining: {:.3}", 
                                  time_consumed, state.remaining_time, new_remaining_time);
                            
                            state.velocity = align_with_surface(state.velocity, step_result.hit.normal, *character.up);
                            state.ground = Some(Ground::new(step_result.hit.entity, step_result.hit.normal));
                            state.offset += offset;
                            state.remaining_time = new_remaining_time;
                            
                            step_occurred = true;
                            step_info = Some(step_result);
                            
                            return None; // Skip normal collision response
                        } else {
                            info!("❌ STEP FAILED");
                        }
                    }
                }
            }
            
            Some(surface)
        },
    );
    
    // Update movement state with results from collide_and_slide
    movement_state.position += result.offset;
    movement_state.remaining_time = result.remaining_time;
    
    // For side pass, preserve the original motion vector velocity, not the collision result
    if pass_type == PassType::Side {
        movement_state.velocity = motion_vector;
    } else {
        movement_state.velocity = result.velocity;
    }
    
    if let Some(ground) = result.ground {
        movement_state.ground = Some(ground);
    }
    
    MovementResult {
        step_occurred,
        step_info,
        collision_flags: if result.offset.length() < motion_vector.length() * movement_state.remaining_time * 0.99 { 1 } else { 0 },
    }
}

#[derive(Debug)]
struct MovementResult {
    step_occurred: bool,
    step_info: Option<StepOutput>,
    collision_flags: u32,
}

pub(crate) fn move_character_three_pass(
    mut commands: Commands,
    spatial_query: SpatialQuery,
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
    mut rigidbodies: Query<(&RigidBody, &CollisionLayers)>,
    mut collision_started_events: EventWriter<CollisionStarted>,
    mut collision_ended_events: EventWriter<CollisionEnded>,
    time: Res<Time>,
) -> Result {
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

        let current_ground_normal = grounding.as_ref().and_then(|(g, _)| g.normal());
        let is_grounded = grounding.as_ref().map_or(false, |(g, _)| g.is_grounded());

        let duration = time.delta_secs();

        // Compute the three pass vectors
        let (up_vector, side_vector, down_vector) = compute_pass_vectors(
            velocity.0,
            character.up,
            stepping_config.map(|(config, _)| config),
            is_grounded,
        );

        let mut movement_state = MovementState::new(transform.translation, velocity.0, duration);
        let mut total_collision_flags = 0u32;
        let mut did_step = false;

        // PASS 1: UP
        if let Some(up_motion) = up_vector {
            let result = execute_pass(
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
            );
            
            total_collision_flags |= result.collision_flags << 2;
        }

        // PASS 2: SIDE
        if let Some(side_motion) = side_vector {
            let result = execute_pass(
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
            );
            
            total_collision_flags |= result.collision_flags << 1;
            
            if result.step_occurred {
                did_step = true;
                if let Some(step_info) = result.step_info {
                    commands.entity(entity).trigger(OnStep {
                        position_before_step: transform.translation,
                        step_offset: Vec3::new(step_info.step_forward, step_info.step_up, 0.0),
                        hit: step_info.hit,
                    });
                }
            }
        }

        // PASS 3: DOWN
        if let Some(down_motion) = down_vector {
            let result = execute_pass(
                &mut movement_state,
                down_motion,
                PassType::Down,
                character,
                collider,
                transform.rotation,
                collide_and_slide_config,
                &filter.0,
                &spatial_query,
                stepping_config.map(|(config, _)| config),
                grounding.as_ref().map(|(_, config)| config).map(|v| &**v),
            );
            
            total_collision_flags |= result.collision_flags;
        }

        // Ground detection after all passes
        if let Some((grounding, grounding_settings)) = grounding.as_ref() {
            let walkable_angle = if grounding.is_grounded() {
                grounding_settings.max_angle + 0.01 // Add epsilon for grounded characters
            } else {
                grounding_settings.max_angle
            };

            if let Some((ground, hit)) = ground_check(
                collider,
                movement_state.position,
                transform.rotation,
                character.up,
                grounding_settings.max_distance,
                collide_and_slide_config.skin_width,
                walkable_angle,
                &spatial_query,
                &filter.0,
            ) {
                movement_state.ground = Some(ground);

                if grounding_settings.snap_to_surface && hit.distance < 0.0 {
                    let hit_roof = sweep(
                        collider,
                        transform.translation,
                        transform.rotation,
                        character.up,
                        -hit.distance,
                        collide_and_slide_config.skin_width,
                        &spatial_query,
                        &filter.0,
                        true,
                    ).is_some();

                    if !hit_roof {
                        movement_state.position -= character.up * hit.distance;
                    }
                }
            } else {
                movement_state.ground = None;
            }
        }

        // Update grounding state and trigger events
        if let Some((grounding, _grounding_config)) = grounding.as_mut() {
            match (grounding.inner_ground, movement_state.ground) {
                (Some(old_ground), None) => {
                    commands.entity(entity).trigger(OnGroundLeave(old_ground));
                }
                (None, Some(new_ground)) => {
                    commands.entity(entity).trigger(OnGroundEnter(new_ground));
                }
                _ => {}
            }

            if let Some(ground) = movement_state.ground {
                // Align velocity with ground surface after stepping
                if did_step {
                    movement_state.velocity = align_with_surface(
                        movement_state.velocity, 
                        *ground.normal, 
                        *character.up
                    );
                }
            } else if grounding.is_grounded() && movement_state.velocity.dot(*character.up) < 0.0 {
                movement_state.velocity = align_with_surface(
                    movement_state.velocity, 
                    *character.up, 
                    *character.up
                );
            }

            **grounding = Grounding::new(movement_state.ground);
        }

        // Apply final results
        let final_displacement = movement_state.position - transform.translation;
        transform.translation = movement_state.position;
        
        // Velocity handling: Don't let the movement system destroy our velocity
        // The issue is that movement_state.velocity gets overwritten by collision response
        if let Some(ground) = movement_state.ground {
            // Just landed - preserve horizontal velocity, zero out downward velocity
            let horizontal_vel = velocity.0.reject_from(*character.up);
            let vertical_component = velocity.0.project_onto(*character.up);
            let upward_vel = if vertical_component.dot(*character.up) > 0.0 { 
                vertical_component 
            } else { 
                Vec3::ZERO 
            };
            velocity.0 = horizontal_vel + upward_vel;
        } else {
            // In air - don't modify velocity at all, the physics/input systems handle this
            // The movement state velocity gets corrupted by collision response, so ignore it
        }         // Debug motion tracking for visualization
        if debug_mode {
            if let Some(debug_motion) = debug_motion.as_mut() {
                let mut point = transform.translation;
                point += feet_position(
                    collider,
                    transform.rotation,
                    character.up,
                    collide_and_slide_config.skin_width,
                );

                debug_motion.push(
                    duration,
                    DebugPoint {
                        translation: transform.translation,
                        velocity: velocity.0,
                        hit: current_ground_normal.map(|normal| DebugHit {
                            point,
                            normal: *normal,
                            is_walkable: true,
                        }),
                    },
                );
            }
        } else if let Some(debug_motion) = debug_motion.as_mut() {
            // Regular debug motion tracking
            let should_add_point = debug_motion.points.back().map_or(true, |(_, point)| {
                point.translation.distance_squared(transform.translation) > 0.01
            });

            if should_add_point {
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
    }

    Ok(())
}

/// Triggered when the character becomes grounded during a movement update.
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
    // Not sure if this should be here or in GroundingConfig
    pub up: Dir3,
}

impl Default for Character {
    fn default() -> Self {
        Self { up: Dir3::Y }
    }
}

/// The velocity of a character.
#[derive(Component, Reflect, Debug, Default, Clone, Copy, Deref, DerefMut)]
#[reflect(Component)]
pub struct KinematicVelocity(pub Vec3);

/// Cache the [`SpatialQueryFilter`] of the character to avoid re-allocating the excluded entities map every time it's used.
#[derive(Component, Reflect, Default, Debug)]
#[reflect(Component)]
pub struct CollideAndSlideFilter(pub(crate) SpatialQueryFilter);
