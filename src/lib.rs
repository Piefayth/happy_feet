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
use stepping::{StepOutput, SteppingBehaviour, SteppingConfig, step_up};
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
            (update_platform_velocity, move_with_platform, move_character)
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

#[derive(Component, Default, Debug)]
pub struct BlockingNormals {
    pub normals: Vec<Vec3>,
}

impl BlockingNormals {
    pub fn clear(&mut self) {
        self.normals.clear();
    }
    
    pub fn add_blocking_normal(&mut self, normal: Vec3) {
        if !self.normals.iter().any(|n| n.dot(normal) > 0.95) {
            self.normals.push(normal);
        }
    }
    
    /// Calculate what fraction of movement in a given direction would be preserved after collision
    pub fn effective_movement_ratio(&self, direction: Vec3) -> f32 {
        if direction.length_squared() < 1e-6 {
            return 1.0;
        }
        
        let mut projected_direction = direction;
        
        for &normal in &self.normals {
            let into_obstruction = projected_direction.dot(normal);
            if into_obstruction < 0.0 {
                let removal = normal * into_obstruction;
                projected_direction -= removal;
            }
        }
        
        let original_length = direction.length();
        let preserved_length = projected_direction.length();
        
        if original_length < 1e-6 {
            1.0
        } else {
            preserved_length / original_length
        }
    }
}


pub(crate) fn move_character(
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
        &mut BlockingNormals,
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
        mut blocking_normals
    ) in &mut query
    {

        blocking_normals.clear();

        if is_sensor {
            transform.translation += velocity.0 * time.delta_secs();
            continue;
        }

        // let current_ground_normal = character.grounding.normal();
        let current_ground_normal = grounding.as_ref().and_then(|(g, _)| g.normal());

        if debug_mode {
            if let Some(lines) = debug_motion.as_mut() {
                let mut point = transform.translation;

                point += feet_position(
                    collider,
                    transform.rotation,
                    character.up,
                    collide_and_slide_config.skin_width,
                );

                lines.push(
                    0.0,
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
        }

        let duration = match debug_mode {
            true => 1.0,
            false => time.delta_secs(),
        };

        // When already grounded, add a small epsilon to make sure we don't randomly
        // lose grip of the surface when the ground angle matches the max angle perfectly
        let walkable_angle = |base_angle, is_grounded| match is_grounded {
            true => base_angle + 0.01,
            false => base_angle,
        };

        let mut did_step = false;

        let mut movement = collide_and_slide(
            collider,
            transform.translation,
            transform.rotation,
            velocity.0,
            current_ground_normal,
            collide_and_slide_config,
            &filter.0,
            &spatial_query,
            duration,
            |velocity, surface| match grounding {
                Some(_) => {
                    let projected_velocity = surface.project_velocity(velocity, current_ground_normal, character.up);
                    projected_velocity

                }
                None => velocity.reject_from(*surface.normal),
            },
            |state,
             MovementImpact {
                 remaining_motion,
                 hit,
                 ..
             }| {
                let surface = match grounding.as_ref() {
                    Some((grounding, grounding_settings)) => Surface::new(
                        hit.normal,
                        walkable_angle(grounding_settings.max_angle, grounding.is_grounded()),
                        character.up,
                    ),
                    None => Surface {
                        normal: Dir3::new(hit.normal).unwrap(),
                        is_walkable: false,
                    },
                };

                if !surface.is_walkable {
                    blocking_normals.add_blocking_normal(*surface.normal);
                }

                // Try to step over obstacles
                if let Some((
                    (stepping_config, stepping_behaviour),
                    (grounding, grounding_settings),
                )) = stepping_config.zip(grounding.as_ref())
                {
                    let step_condition = match stepping_behaviour {
                        SteppingBehaviour::Never => false,
                        SteppingBehaviour::Grounded => grounding.is_grounded(),
                        SteppingBehaviour::Always => true,
                    };

                    let is_dynamic = match rigidbodies.get_mut(hit.entity) {
                        Ok((rb, _)) => rb.is_dynamic(),
                        Err(_) => false,
                    };

                    let try_step = !surface.is_walkable && !is_dynamic && step_condition;

                    if try_step {
                        let remaining_horizontal_velocity =
                            (velocity.0 * remaining_motion).reject_from(*character.up);

                        if let Ok((direction, motion)) =
                            Dir3::new_and_length(remaining_horizontal_velocity)
                        {
                            if let Some(StepOutput {
                                step_forward,
                                step_up,
                                hit,
                            }) = step_up(
                                collider,
                                transform.translation + state.offset,
                                transform.rotation,
                                direction,
                                motion,
                                character.up,
                                collide_and_slide_config.skin_width,
                                stepping_config,
                                &filter.0,
                                &spatial_query,
                                |hit| {
                                    is_walkable(
                                        hit.normal,
                                        grounding_settings.max_angle - 0.01,
                                        *character.up,
                                    )
                                },
                            ) {
                                let offset = direction * step_forward + character.up * step_up;

                                commands.entity(entity).trigger(OnStep {
                                    position_before_step: transform.translation + state.offset,
                                    step_offset: offset,
                                    hit,
                                });

                                state.velocity =
                                    align_with_surface(state.velocity, hit.normal, *character.up);
                                state.ground = Some(Ground::new(hit.entity, hit.normal));
                                state.offset += offset;
                                state.remaining_time =
                                    (state.remaining_time - step_forward * duration).max(0.0);

                                did_step = true;

                                return None;
                            }
                        }
                    }
                }

                // Trigger collision events
                collision_started_events.write(CollisionStarted(entity, hit.entity));

                // For now, assume the collision is ended immediately, which is probably the case with move and slide anyway
                collision_ended_events.write(CollisionEnded(entity, hit.entity));

                if debug_mode {
                    if let Some(lines) = debug_motion.as_mut() {
                        let duration = duration - state.remaining_time;
                        lines.push(
                            duration,
                            DebugPoint {
                                translation: transform.translation + state.offset,
                                velocity: state.velocity,
                                hit: Some(DebugHit {
                                    point: hit.point,
                                    normal: hit.normal,
                                    is_walkable: surface.is_walkable,
                                }),
                            },
                        );
                    }
                }

                Some(surface)
            },
        );

        let mut new_translation = transform.translation + movement.offset;

        if let Some((grounding, grounding_settings)) = grounding.as_ref() {
            if grounding.is_grounded() || movement.ground.is_some() {
                if let Some((ground, hit)) = ground_check(
                    collider,
                    new_translation,
                    transform.rotation,
                    character.up,
                    grounding_settings.max_distance,
                    collide_and_slide_config.skin_width,
                    walkable_angle(grounding_settings.max_angle, grounding.is_grounded()),
                    &spatial_query,
                    &filter.0,
                ) {
                    movement.ground = Some(ground);

                    let mut hit_roof = !grounding_settings.snap_to_surface;

                    if grounding_settings.snap_to_surface && hit.distance < 0.0 {
                        hit_roof = sweep(
                            collider,
                            transform.translation,
                            transform.rotation,
                            character.up,
                            -hit.distance,
                            collide_and_slide_config.skin_width,
                            &spatial_query,
                            &filter.0,
                            true,
                        )
                        .is_some();
                    }

                    if !hit_roof {
                        new_translation -= character.up * hit.distance;
                    }
                } else {
                    movement.ground = None;
                }
            }
        }

        let draw_line = debug_mode
            || debug_motion.as_ref().is_some_and(|lines| {
                lines.points.back().is_none_or(|(_, point)| {
                    point.translation.distance_squared(new_translation) > 0.5
                })
            });

        if draw_line {
            let lines = debug_motion.as_mut().unwrap();

            let mut point = new_translation;

            point += feet_position(
                collider,
                transform.rotation,
                character.up,
                collide_and_slide_config.skin_width,
            );

            lines.push(
                movement.remaining_time,
                DebugPoint {
                    translation: new_translation,
                    velocity: movement.velocity,
                    hit: movement.ground.map(|ground| DebugHit {
                        point,
                        normal: *ground.normal,
                        is_walkable: true,
                    }),
                },
            );
        }

        if !debug_mode {
            if let Some((grounding, _grounding_config)) = grounding.as_mut() {
                match (grounding.inner_ground, movement.ground) {
                    (Some(ground), None) => {
                        commands.entity(entity).trigger(OnGroundLeave(ground));
                    }
                    (None, Some(ground)) => {
                        commands.entity(entity).trigger(OnGroundEnter(ground));
                    }
                    _ => {}
                }

                if let Some(ground) = movement.ground {
                    // Make sure the character is not launched up after stepping.
                    // FIXME: doesn't really work
                    if did_step {
                        movement.velocity =
                            align_with_surface(movement.velocity, *ground.normal, *character.up);
                    }
                } else if grounding.is_grounded() && movement.velocity.dot(*character.up) < 0.0 {
                    movement.velocity =
                        align_with_surface(movement.velocity, *character.up, *character.up);
                }

                **grounding = Grounding::new(movement.ground);
            }

            transform.translation = new_translation;
            velocity.0 = movement.velocity;
        }
    }

    Ok(())
}

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
    BlockingNormals
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
