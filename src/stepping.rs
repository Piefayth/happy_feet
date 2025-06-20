use crate::{
    ground::GroundingConfig,
    sweep::{SweepHitData, sweep},
};
use avian3d::prelude::*;
use bevy::prelude::*;

pub(crate) struct StepOutput {
    pub step_forward: f32,
    pub step_up: f32,
    pub hit: SweepHitData,
}

pub(crate) fn step_up(
    shape: &Collider,
    origin: Vec3,
    rotation: Quat,
    direction: Dir3,
    forward_motion: f32,
    up: Dir3,
    skin_width: f32,
    config: &SteppingConfig,
    filter: &SpatialQueryFilter,
    spatial_query: &SpatialQuery,
    mut can_step: impl FnMut(SweepHitData) -> bool,
) -> Option<StepOutput> {
    const EPSILON: f32 = 1e-4;

    // Validate input
    if !config.is_valid() || forward_motion < EPSILON {
        return None;
    }

    // Step up
    let mut step_up = config.max_step_up;

    if let Some(hit) = sweep(
        shape,
        origin,
        rotation,
        up,
        config.max_step_up,
        skin_width,
        spatial_query,
        filter,
        false,
    ) {
        step_up = hit.distance;
    }

    // Head is already touching a roof or wall
    if step_up < EPSILON {
        return None;
    }

    let step_up_position = origin + up * step_up;
    let step_size = forward_motion.max(config.min_step_forward) / config.max_iterations as f32;
    
    for i in 0..config.max_iterations + 1 {
        // Step forward
        let mut step_forward = step_size * i as f32;
        let mut hit_wall = false;

        if let Some(hit) = sweep(
            shape,
            step_up_position,
            rotation,
            direction,
            step_forward,
            skin_width,
            spatial_query,
            filter,
            false,
        ) {
            // We're stuck :(
            if hit.distance < EPSILON {
                break;
            }

            hit_wall = true;
            step_forward = hit.distance;
        }


        let step_forward_position = step_up_position + direction * step_forward;

        // Step down
        if let Some(hit) = sweep(
            shape,
            step_forward_position,
            rotation,
            -up,
            config.max_step_up,
            skin_width,
            spatial_query,
            filter,
            true,
        ) {
            // We can step here!
            if step_up - hit.distance > EPSILON && can_step(hit) {
                step_up -= hit.distance;
                return Some(StepOutput {
                    step_forward,
                    step_up,
                    hit,
                });
            }
        }

        if hit_wall {
            break;
        }
    }
    
    None
}

/// Determines when the character should attempt to step up.
#[derive(Component, Reflect, Default, Debug, PartialEq, Eq, Clone, Copy)]
#[reflect(Component, Default)]
pub enum SteppingBehaviour {
    Never,
    #[default]
    Grounded,
    Always,
}

/// Configure stepping for a character.
#[derive(Component, Reflect, Debug, PartialEq, Clone, Copy)]
#[reflect(Component, Default)]
#[require(GroundingConfig, SteppingBehaviour)]
pub struct SteppingConfig {
    pub max_step_up: f32,
    pub min_step_forward: f32,
    pub max_iterations: usize,
}

impl SteppingConfig {
    pub fn is_valid(&self) -> bool {
        self.max_step_up > 0.0 && self.min_step_forward > 0.0
    }
}

impl Default for SteppingConfig {
    fn default() -> Self {
        Self {
            max_step_up: 0.25,
            min_step_forward: 0.4,
            max_iterations: 8,
        }
    }
}
