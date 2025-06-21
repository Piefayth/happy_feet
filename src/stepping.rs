use crate::{
    ground::GroundingConfig,
    sweep::{SweepHitData, sweep},
};
use avian3d::prelude::*;
use bevy::prelude::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PassType {
    Up,
    Side,
    Down,
}

#[derive(Debug, Clone)]
pub struct MotionBudget {
    pub horizontal_remaining: f32,
    pub vertical_remaining: f32,
    pub original_horizontal: f32,
    pub original_vertical: f32,
}

impl MotionBudget {
    pub fn new(velocity: Vec3, delta_time: f32) -> Self {
        // Motion budget should be the actual distance we plan to move this frame
        let frame_displacement = velocity * delta_time;
        let horizontal_magnitude = frame_displacement.reject_from(Vec3::Y).length();
        let vertical_magnitude = frame_displacement.project_onto(Vec3::Y).length();
        
        Self {
            horizontal_remaining: horizontal_magnitude,
            vertical_remaining: vertical_magnitude,
            original_horizontal: horizontal_magnitude,
            original_vertical: vertical_magnitude,
        }
    }
    
    pub fn consume_horizontal(&mut self, distance: f32) {
        self.horizontal_remaining = (self.horizontal_remaining - distance).max(0.0);
    }
    
    pub fn consume_vertical(&mut self, distance: f32) {
        self.vertical_remaining = (self.vertical_remaining - distance).max(0.0);
    }
    
    pub fn horizontal_fraction_remaining(&self) -> f32 {
        if self.original_horizontal == 0.0 {
            1.0
        } else {
            self.horizontal_remaining / self.original_horizontal
        }
    }
    
    pub fn can_step(&self, step_distance: f32) -> bool {
        self.horizontal_remaining >= step_distance
    }
}

#[derive(Debug)]
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

    info!("🦶 STEP UP ATTEMPT");
    info!("  Origin: {:?}", origin);
    info!("  Direction: {:?}", direction);
    info!("  Forward motion: {}", forward_motion);
    info!("  Max step up: {}", config.max_step_up);
    info!("  Min step forward: {}", config.min_step_forward);

    // Validate input
    if !config.is_valid() || forward_motion < EPSILON {
        info!("  ❌ Invalid config or insufficient motion");
        return None;
    }

    // Step 1: Try to move up to clear the obstacle
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
        info!("  🚧 Hit ceiling at {} units up", hit.distance);
    } else {
        info!("  ✅ Clear path upward for {} units", step_up);
    }

    // Head is already touching a roof or we can't go up at all
    if step_up < EPSILON {
        info!("  ❌ No vertical clearance");
        return None;
    }

    let step_up_position = origin + up * step_up;
    info!("  ⬆️  Stepped up to position: {:?}", step_up_position);

    // Step 2: Try to move forward at the elevated position
    // We use an iterative approach to find the furthest we can step forward
    let step_size = forward_motion.max(config.min_step_forward) / config.max_iterations as f32;
    
    info!("  ➡️  Trying forward motion in {} steps of {} units each", 
          config.max_iterations + 1, step_size);
    
    for i in 0..=config.max_iterations {
        let mut step_forward = step_size * i as f32;
        let mut hit_wall = false;

        info!("    Step iteration {}: trying {} units forward", i, step_forward);

        // Try to move forward at the elevated position
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
            if hit.distance < EPSILON {
                info!("    🚧 Stuck against wall immediately");
                break;
            }

            hit_wall = true;
            step_forward = hit.distance;
            info!("    🚧 Hit wall at {} units forward", step_forward);
        } else {
            info!("    ✅ Clear path for {} units forward", step_forward);
        }

        let step_forward_position = step_up_position + direction * step_forward;

        // Step 3: Try to step down from the forward position
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
            let final_step_up = step_up - hit.distance;
            
            info!("    ⬇️  Found ground {} units down (net step up: {})", 
                  hit.distance, final_step_up);
            
            // Check if this is a valid step (net upward movement and walkable surface)
            if final_step_up > EPSILON && can_step(hit) {
                info!("    ✅ VALID STEP FOUND!");
                info!("      Forward distance: {}", step_forward);
                info!("      Net step up: {}", final_step_up);
                info!("      Ground entity: {:?}", hit.entity);
                info!("      Ground normal: {:?}", hit.normal);
                
                return Some(StepOutput {
                    step_forward,
                    step_up: final_step_up,
                    hit,
                });
            } else {
                info!("    ❌ Invalid step: net_up={}, walkable={}", 
                      final_step_up, can_step(hit));
            }
        } else {
            info!("    ❌ No ground found when stepping down");
        }

        // If we hit a wall, we can't go any further
        if hit_wall {
            info!("    🛑 Hit wall, stopping search");
            break;
        }
    }
    
    info!("  ❌ Step up failed - no valid step found");
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
        let valid = self.max_step_up > 0.0 && self.min_step_forward > 0.0 && self.max_iterations > 0;
        if !valid {
            info!("❌ Invalid stepping config: max_step_up={}, min_step_forward={}, max_iterations={}", 
                  self.max_step_up, self.min_step_forward, self.max_iterations);
        }
        valid
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

