use crate::{
    ground::GroundingConfig,
    sweep::{SweepHitData, sweep},
};
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

