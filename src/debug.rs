use std::collections::VecDeque;

use avian3d::prelude::*;
use bevy::{color::palettes::css::*, prelude::*};

use crate::{
    feet_position, ground::Grounding, move_character_physx_style, movement::{CharacterMovement, MoveInput}, sweep::CollideAndSlideConfig, Character, KinematicVelocity
};

pub(crate) fn plugin(app: &mut App) {
    app.insert_gizmo_config(
        CharacterGizmos,
        GizmoConfig {
            line: GizmoLineConfig {
                width: 8.0,
                joints: GizmoLineJoint::Bevel,
                ..Default::default()
            },
            depth_bias: -0.1,
            ..Default::default()
        },
    );

    app.add_systems(
        FixedPostUpdate,
        (draw_input_arrow, draw_motion).after(move_character_physx_style),
    );
}

#[derive(GizmoConfigGroup, Reflect, Default)]
pub(crate) struct CharacterGizmos;

#[derive(Component, Reflect, Debug)]
#[reflect(Component)]
pub struct DebugInput;

fn draw_input_arrow(
    mut gizmos: Gizmos<CharacterGizmos>,
    query: Query<
        (
            &Transform,
            &Character,
            &KinematicVelocity,
            &CharacterMovement,
            &Collider,
            &MoveInput,
        ),
        With<DebugInput>,
    >,
) {
    for (transform, character, velocity, movement, collider, move_input) in &query {
        let half_height = collider
            .aabb(Vec3::ZERO, transform.rotation)
            .size()
            .dot(character.up * 0.5);

        if let Ok(direction) = Dir3::new(move_input.previous()) {
            let speed_len = velocity.0.dot(*direction) / movement.target_speed;
            let accel_len = 1.0 - speed_len.clamp(0.0, 1.0);

            let origin = transform.translation - character.up * half_height;
            let right = character.up.cross(*direction);

            let color = BLACK
                .mix(&VIOLET, accel_len)
                // .mix(&YELLOW, accel_len.powi(2))
                .mix(&RED, accel_len.powi(4));

            let arrow_head = 0.1;

            gizmos.linestrip(
                [
                    origin + *direction - right * arrow_head,
                    origin + direction * (1.0 + arrow_head),
                    origin + *direction + right * arrow_head,
                ],
                color,
            );
        }
    }
}

#[derive(Component, Reflect, Debug)]
#[reflect(Component)]
pub struct DebugMode;

#[derive(Reflect, Debug, Clone, Copy)]
pub(crate) struct DebugHit {
    pub point: Vec3,
    pub normal: Vec3,
    pub is_walkable: bool,
}

#[derive(Reflect, Debug, Clone, Copy)]
pub(crate) struct DebugPoint {
    pub translation: Vec3,
    pub velocity: Vec3,
    pub hit: Option<DebugHit>,
}

#[derive(Component, Reflect, Debug)]
#[reflect(Component)]
pub struct DebugMotion {
    pub(crate) duration: f32,
    pub(crate) points: VecDeque<(f32, DebugPoint)>,
}

impl Default for DebugMotion {
    fn default() -> Self {
        Self::new(32)
    }
}

impl DebugMotion {
    pub fn new(capacity: usize) -> Self {
        Self {
            duration: 0.0,
            points: VecDeque::with_capacity(capacity),
        }
    }

    pub(crate) fn push(&mut self, duration: f32, point: DebugPoint) {
        self.duration += duration;
        self.points.push_back((duration, point));

        if self.points.capacity() < self.points.len() + 1 {
            self.duration = self.points.iter().map(|(d, _)| d).sum();
            self.points.pop_front();
        }
    }

    pub(crate) fn duration_at(&self, index: usize) -> f32 {
        self.points.range(..=index).map(|(d, _)| d).sum()
    }

    pub(crate) fn clear(&mut self) {
        self.duration = 0.0;
        self.points.clear()
    }
}

fn draw_motion(
    mut gizmos: Gizmos<CharacterGizmos>,
    mut query: Query<(
        &CollideAndSlideConfig,
        &KinematicVelocity,
        &Grounding,
        &Character,
        &CharacterMovement,
        &Collider,
        &Transform,
        &mut DebugMotion,
        Has<DebugMode>,
    )>,
) {
    for (
        config,
        velocity,
        grounding,
        character,
        movement,
        collider,
        transform,
        mut debug_motion,
        debug_mode,
    ) in &mut query
    {
        let line_color = |t: f32, velocity: Vec3| {
            let target_speed_sq = movement.target_speed * movement.target_speed;

            let mut color = Hsla::from(MIDNIGHT_BLUE).mix(
                &Hsla::from(CRIMSON),
                (velocity.length_squared() / target_speed_sq).min(1.0),
            );

            let extra = (velocity.length_squared() - target_speed_sq).max(0.0);

            let fac = (extra / target_speed_sq / 10.0).min(1.0);
            color = color.mix(&Hsla::from(WHEAT), fac);

            match debug_mode {
                true => color,
                false => color.with_alpha(t.powf(2.0)),
            }
        };

        // lines
        let mut points = Vec::with_capacity(debug_motion.points.len());

        for i in 0..debug_motion.points.len() {
            let (_, a) = debug_motion.points[i];

            points.push((
                a.translation
                    + feet_position(
                        collider,
                        transform.rotation,
                        character.up,
                        config.skin_width,
                    ) / 1.5,
                line_color(
                    debug_motion.duration_at(i) / debug_motion.duration,
                    a.velocity,
                ),
            ));
        }

        // Draw the last line to character if debug mode is disabled
        if !debug_mode {
            points.push((
                transform.translation
                    + feet_position(
                        collider,
                        transform.rotation,
                        character.up,
                        config.skin_width,
                    ) / 1.5,
                line_color(1.0, velocity.0),
            ));
        }

        gizmos.linestrip_gradient(points);

        // hits
        let mut dbg_point = |debug: DebugPoint| {
            let radius = 0.2;

            let color = [LIGHT_GREEN, CRIMSON];

            let (point, normal, color) = match debug.hit {
                Some(hit) => {
                    let point = hit.point + hit.normal * config.skin_width;
                    let normal = hit.normal;

                    let color = match hit.is_walkable {
                        true => color[0],
                        false => color[1],
                    };

                    gizmos.line_gradient(point, point + normal * 0.1, WHITE, WHITE.with_alpha(0.0));

                    if let Ok((direction, speed)) = Dir3::new_and_length(debug.velocity) {
                        gizmos.line_gradient(
                            point + direction * radius,
                            point + direction * (radius + speed / movement.target_speed / 2.0),
                            color,
                            color.with_alpha(0.0),
                        );
                    }

                    (point, normal, color)
                }
                None => {
                    let point = debug.translation
                        + feet_position(
                            collider,
                            transform.rotation,
                            character.up,
                            config.skin_width,
                        );

                    let color = color[1];

                    gizmos.line_gradient(
                        point,
                        point - character.up * 0.5,
                        color,
                        color.with_alpha(0.0),
                    );

                    (point, *character.up, color)
                }
            };

            let forward = Dir3::new(normal.cross(*character.up)).unwrap_or(Dir3::NEG_Z);

            let transform = Transform::from_translation(point).looking_to(normal, forward);

            gizmos.circle(
                Isometry3d::new(transform.translation, transform.rotation),
                radius,
                color,
            );
        };

        if debug_mode {
            // Draw hit points
            for i in 0..debug_motion.points.len() {
                let (_, debug) = debug_motion.points[i];
                dbg_point(debug);
            }

            debug_motion.clear();
        } else {
            // Only draw the last point at character if debug mode is disabled
            let ground_normal = grounding.normal();

            dbg_point(DebugPoint {
                translation: transform.translation,
                velocity: velocity.0,
                hit: ground_normal.map(|normal| DebugHit {
                    point: transform.translation
                        + feet_position(
                            collider,
                            transform.rotation,
                            character.up,
                            config.skin_width,
                        ),
                    normal: *normal,
                    is_walkable: true,
                }),
            });
        }
    }
}
