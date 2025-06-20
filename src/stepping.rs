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
    const NORMAL_SIMILARITY_THRESHOLD: f32 = 0.99;
    const REQUIRED_CONSISTENT_HITS: usize = 3;

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

    if step_up < EPSILON {
        return None;
    }

    let step_up_position = origin + up * step_up;
    let step_size = forward_motion.max(config.min_step_forward) / config.max_iterations as f32;
    
    // Track recent normals for consistency checking
    let mut recent_normals: Vec<Vec3> = Vec::new();
    let mut best_step: Option<StepOutput> = None;
    
    println!("=== STEPPING WITH SINGLE PASS CONSISTENCY ===");
    println!("Looking for {} consistent normals with {:.3} similarity", 
        REQUIRED_CONSISTENT_HITS, NORMAL_SIMILARITY_THRESHOLD);
    
    for i in 0..config.max_iterations + 1 {
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
            if hit.distance < EPSILON {
                break;
            }
            hit_wall = true;
            step_forward = hit.distance;
        }

        let step_forward_position = step_up_position + direction * step_forward;

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
            
            if final_step_up > EPSILON && can_step(hit) {
                println!("  Iteration {}: Found walkable surface, normal: {:?} (angle: {:.1}°)", 
                    i, hit.normal, hit.normal.angle_between(*up).to_degrees());
                
                // Check consistency with recent normals
                let consistent_with_recent = recent_normals.iter()
                    .all(|prev_normal| hit.normal.dot(*prev_normal) >= NORMAL_SIMILARITY_THRESHOLD);
                
                if consistent_with_recent {
                    // Add this normal to our sequence
                    recent_normals.push(hit.normal);
                    
                    // Update best step to this furthest consistent position
                    best_step = Some(StepOutput {
                        step_forward,
                        step_up: final_step_up,
                        hit,
                    });
                    
                    println!("    Consistent with recent normals - sequence length: {}", recent_normals.len());
                    
                    // If we have enough consistent hits, we could return now, but let's keep going
                    // to find the furthest position in this consistent sequence
                    if recent_normals.len() >= REQUIRED_CONSISTENT_HITS {
                        println!("    ✓ Found {} consistent normals, continuing to find furthest...", recent_normals.len());
                    }
                    
                    // Limit the sequence length to avoid growing indefinitely
                    if recent_normals.len() > REQUIRED_CONSISTENT_HITS {
                        recent_normals.remove(0);
                    }
                } else {
                    println!("    Not consistent with recent normals - starting new sequence");
                    // Start a new sequence
                    recent_normals.clear();
                    recent_normals.push(hit.normal);
                    
                    // Reset best step since this breaks consistency
                    best_step = None;
                }
            } else {
                println!("  Iteration {}: Surface not walkable or step too small", i);
                // Reset sequence on non-walkable surface
                recent_normals.clear();
                best_step = None;
            }
        } else {
            println!("  Iteration {}: No ground found", i);
            // Reset sequence when we lose ground
            recent_normals.clear();
            best_step = None;
        }

        // Continue even if we hit a wall - we can still step at the wall position
    }
    
    // Return the furthest consistent position we found
    if let Some(step) = best_step {
        if recent_normals.len() >= REQUIRED_CONSISTENT_HITS {
            println!("=== STEP SUCCESS - Using furthest consistent position with {} normals ===", recent_normals.len());
            return Some(step);
        }
    }
    
    println!("=== STEP FAILED - No consistent surface found ===");
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
