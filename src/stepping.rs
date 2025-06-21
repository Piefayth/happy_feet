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

    println!("=== STEP UP ATTEMPT ===");
    println!("Origin: {:?}", origin);
    println!("Direction: {:?}", direction);
    println!("Forward motion: {}", forward_motion);
    println!("Max step up: {}", config.max_step_up);

    // Validate input
    if !config.is_valid() {
        println!("STEP FAILED: Invalid config");
        return None;
    }
    
    // We need enough forward motion to make stepping worthwhile
    // Since we're climbing walls directly, any forward motion is enough
    if forward_motion < EPSILON {
        println!("STEP FAILED: Forward motion too small ({} < {})", forward_motion, EPSILON);
        return None;
    }

    // Step 1: Cast forward to get distance to wall (the front of the next stair)
    println!("\n--- Step 1: Forward cast to find wall ---");
    let wall_hit = match sweep(
        shape,
        origin,
        rotation,
        direction,
        forward_motion,
        skin_width,
        spatial_query,
        filter,
        false,
    ) {
        Some(hit) => {
            println!("Wall found at distance: {}", hit.distance);
            println!("Wall normal: {:?}", hit.normal);
            println!("Wall entity: {:?}", hit.entity);
            hit
        }
        None => {
            println!("STEP FAILED: No wall found in forward direction");
            return None;
        }
    };

    // Step 2: Check if we can climb at max_step_up height
    println!("\n--- Step 2: Check ceiling clearance ---");
    if let Some(ceiling_hit) = sweep(
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
        println!("STEP FAILED: Ceiling blocking at height {} (entity: {:?})", ceiling_hit.distance, ceiling_hit.entity);
        return None;
    }
    println!("Ceiling clear for full step height");

    // Step 3: Find what's on top of the wall to step onto
    println!("\n--- Step 3: Find step surface on top of wall ---");
    let wall_position = origin + direction * wall_hit.distance;
    
    // Since we hit the wall at distance 0, we need to search forward to find the step
    // Search from a small distance forward from the wall to find the step surface
    let search_distance = if wall_hit.distance < EPSILON { 0.1 } else { wall_hit.distance };
    let step_search_position = origin + direction * search_distance;
    let wall_top_search_start = step_search_position + up * config.max_step_up;
    
    println!("Wall position: {:?}", wall_position);
    println!("Step search position: {:?}", step_search_position);
    println!("Searching for step surface from: {:?}", wall_top_search_start);
    
    let climb_height = if let Some(step_hit) = sweep(
        shape,
        wall_top_search_start,
        rotation,
        -up,
        config.max_step_up,
        skin_width,
        spatial_query,
        filter,
        true,
    ) {
        let step_surface_y = wall_top_search_start.y - step_hit.distance;
        println!("Step surface found at Y: {}", step_surface_y);
        println!("Step surface normal: {:?}", step_hit.normal);
        println!("Step surface entity: {:?}", step_hit.entity);
        println!("Character origin Y: {}", origin.y);
        
        // Check if this step is actually above us
        let height = step_surface_y - origin.y;
        println!("Calculated step height: {}", height);
        
        if height <= EPSILON {
            println!("STEP FAILED: Step surface not above character (height: {})", height);
            return None;
        }
        
        if height > config.max_step_up {
            println!("STEP FAILED: Step too high ({} > {})", height, config.max_step_up);
            return None;
        }
        
        height
    } else {
        // No step surface found, just climb the full height (maybe it's a tall wall)
        println!("No step surface found on wall, climbing full height");
        config.max_step_up
    };

    let climb_end_position = wall_position + up * climb_height;
    let remaining_motion = forward_motion - wall_hit.distance;
    println!("Climb end position: {:?}", climb_end_position);
    println!("Remaining motion after wall: {}", remaining_motion);

    // Step 4: If there's remaining motion, find a valid landing surface
    let mut best_landing: Option<SweepHitData> = None;
    
    if remaining_motion > EPSILON {
        println!("\n--- Step 4: Find landing surface ---");
        // Step forward from our climb position and look for a landing
        let landing_samples = config.landing_samples.max(1);
        let sample_distance = remaining_motion / landing_samples as f32;
        
        println!("Landing samples: {}", landing_samples);
        println!("Sample distance: {}", sample_distance);
        
        let mut consecutive_similar = 0;
        let mut last_normal: Option<Vec3> = None;
        
        for i in 1..=landing_samples {
            let sample_forward = sample_distance * i as f32;
            let sample_position = climb_end_position + direction * sample_forward;
            
            println!("  Sample {}: forward={}, position={:?}", i, sample_forward, sample_position);
            
            // Cast down to find landing surface
            if let Some(landing_hit) = sweep(
                shape,
                sample_position,
                rotation,
                -up,
                config.max_step_up,
                skin_width,
                spatial_query,
                filter,
                true,
            ) {
                println!("    Landing found: distance={}, normal={:?}", landing_hit.distance, landing_hit.normal);
                
                // Check if this is a valid landing surface
                let is_valid_landing = can_step(landing_hit);
                println!("    Is valid landing: {}", is_valid_landing);
                
                if is_valid_landing {
                    // Check if this normal is similar to the last one
                    let is_similar = if let Some(last) = last_normal {
                        let similarity = landing_hit.normal.dot(last);
                        println!("    Normal similarity: {} (threshold: {})", similarity, config.normal_similarity_threshold);
                        similarity > config.normal_similarity_threshold
                    } else {
                        println!("    First valid landing, automatically similar");
                        true
                    };
                    
                    if is_similar {
                        consecutive_similar += 1;
                        best_landing = Some(landing_hit);
                        println!("    Consecutive similar count: {} (required: {})", consecutive_similar, config.required_consecutive_similar);
                        
                        // If we have enough consecutive similar normals, we're confident
                        if consecutive_similar >= config.required_consecutive_similar {
                            let final_step_forward = wall_hit.distance + sample_forward;
                            let final_step_up = climb_height - landing_hit.distance;
                            println!("STEP SUCCESS: Confident landing found!");
                            println!("Final step forward: {}", final_step_forward);
                            println!("Final step up: {}", final_step_up);
                            return Some(StepOutput {
                                step_forward: final_step_forward,
                                step_up: final_step_up,
                                hit: landing_hit,
                            });
                        }
                    } else {
                        println!("    Normal not similar, resetting consecutive count");
                        consecutive_similar = 1;
                        best_landing = Some(landing_hit);
                    }
                    
                    last_normal = Some(landing_hit.normal);
                } else {
                    println!("    Invalid landing surface, resetting");
                    // Reset if we hit an invalid surface
                    consecutive_similar = 0;
                    last_normal = None;
                }
            } else {
                println!("    No landing surface found");
                // Reset if we don't hit anything
                consecutive_similar = 0;
                last_normal = None;
            }
        }
        
        // If we found any valid landing, use the best one
        if let Some(landing) = best_landing {
            let final_step_forward = wall_hit.distance + remaining_motion;
            let final_step_up = climb_height - landing.distance;
            println!("STEP SUCCESS: Using best landing found");
            println!("Final step forward: {}", final_step_forward);
            println!("Final step up: {}", final_step_up);
            return Some(StepOutput {
                step_forward: final_step_forward,
                step_up: final_step_up,
                hit: landing,
            });
        } else {
            println!("No valid landing surfaces found in remaining motion");
        }
    } else {
        println!("\n--- Step 4: Skipped (no remaining motion) ---");
    }
    
    // Step 5: Return the step result
    println!("\n--- Step 5: Return step result ---");
    let final_step_forward = wall_hit.distance;
    let final_step_up = climb_height;
    
    // Use the original wall hit as our result surface if we didn't find a better landing
    let result_hit = best_landing.unwrap_or(wall_hit);
    
    println!("STEP SUCCESS: Climbing wall");
    println!("Final step forward: {}", final_step_forward);
    println!("Final step up: {}", final_step_up);
    println!("Result surface normal: {:?}", result_hit.normal);
    println!("=== END STEP UP ===\n");
    
    Some(StepOutput {
        step_forward: final_step_forward,
        step_up: final_step_up,
        hit: result_hit,
    })
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
    /// Number of forward samples to take when looking for landing surface
    pub landing_samples: usize,
    /// How similar normals need to be to be considered "consecutive similar"
    pub normal_similarity_threshold: f32,
    /// How many consecutive similar normals we need to be confident in landing
    pub required_consecutive_similar: usize,
}

impl SteppingConfig {
    pub fn is_valid(&self) -> bool {
        self.max_step_up > 0.0 
            && self.landing_samples > 0
            && self.normal_similarity_threshold > 0.0
            && self.required_consecutive_similar > 0
    }
}

impl Default for SteppingConfig {
    fn default() -> Self {
        Self {
            max_step_up: 0.25,
            landing_samples: 4,
            normal_similarity_threshold: 0.95,
            required_consecutive_similar: 3,
        }
    }
}
