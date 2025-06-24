use std::f32::consts::PI;

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
use stepping::{SteppingBehaviour, SteppingConfig};
use sweep::{CollideAndSlideConfig, SweepHitData};

use crate::debug::MovementDebugConfig;
use crate::controller::CharacterController;

pub mod controller;
pub mod collision;
pub mod debug;
pub mod ground;
pub mod movement;
pub mod stepping;
pub mod sweep;

pub mod prelude {
    pub use crate::{
        Character,
        CharacterPlugin,
        KinematicVelocity,
        OnGroundEnter,
        OnGroundLeave,
        OnStep,
        // Re-export PhysX types from collision module
        collision::{CharacterCollisionFlags, CharacterMoveResult},
        // Re-export the controller
        controller::CharacterController,
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
                .in_set(CharacterSystems::Prepare)
                .chain(),
        );

        app.add_systems(
            self.schedule,
            character_movement_system.in_set(CharacterSystems::ApplyMovement),
        );
    }
}

pub(crate) fn update_character_filter(
    mut query: Query<(Entity, &mut CollideAndSlideFilter, &CollisionLayers)>,
    sensors: Query<Entity, With<Sensor>>,
) {
    for (entity, mut filter, collision_layers) in &mut query {
        filter.0.mask = collision_layers.filters;
        filter.0.excluded_entities.clear();
        filter
            .0
            .excluded_entities
            .extend(sensors.iter().chain([entity]));
    }
}

#[derive(Event, Deref)]
pub struct OnGroundEnter(pub Ground);

#[derive(Event, Deref)]
pub struct OnGroundLeave(pub Ground);

#[derive(Event)]
pub struct OnStep {
    pub position_before_step: Vec3,
    pub step_offset: Vec3,
    pub hit: SweepHitData,
}

/// The high-level character movement system that uses the unified controller API.
pub(crate) fn character_movement_system(
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
        let duration = time.delta_secs();

        debug_log!(debug_config, "=== GAME LOGIC FRAME START ===");
        debug_log!(
            debug_config,
            "Initial velocity: {:?} (magnitude: {:.3})",
            initial_velocity,
            initial_velocity.length()
        );

        // Create the unified controller
        let mut controller = CharacterController::new(
            transform.translation,
            transform.rotation,
            *character,
            collider.clone(),
            *collide_and_slide_config,
            grounding.as_ref().map(|(_, config)| **config),
            stepping_config.map(|(config, behaviour)| (*config, *behaviour)),
            grounding.as_ref().and_then(|(g, _)| g.inner_ground()),
            filter.0.clone(),
            &spatial_query,
            *debug_config,
        );

        // Calculate desired displacement for this frame
        let desired_displacement = initial_velocity * duration;

        // Call the unified movement function
        let move_result = controller.move_character(desired_displacement);

        // Update position based on movement result
        transform.translation = move_result.final_position;

        // Handle grounding state changes and events
        if let Some((grounding, _)) = grounding.as_mut() {
            let old_ground = grounding.inner_ground;
            let new_ground = move_result.ground;

            match (old_ground, new_ground) {
                (Some(old_ground), None) => {
                    commands.entity(entity).trigger(OnGroundLeave(old_ground));
                }
                (None, Some(new_ground)) => {
                    commands.entity(entity).trigger(OnGroundEnter(new_ground));
                }
                _ => {}
            }

            **grounding = Grounding::new(new_ground);
        }

        // Update velocity based on movement results
        let new_velocity =
            controller.calculate_post_movement_velocity(initial_velocity, &move_result);
        velocity.0 = new_velocity;

        debug_log!(
            debug_config,
            "Velocity: {:?} -> {:?}",
            initial_velocity,
            velocity.0
        );

        // Debug visualization
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
                        hit: move_result.ground.map(|ground| DebugHit {
                            point,
                            normal: *ground.normal,
                            is_walkable: true,
                        }),
                    },
                );
            }
        }

        debug_log!(debug_config, "=== GAME LOGIC FRAME END ===\n");
    }
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
