use crate::{
    ground::GroundingConfig,
};
use bevy::prelude::*;

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

