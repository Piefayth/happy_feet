use avian3d::prelude::*;
use bevy::prelude::*;

use crate::{
    Character,
    collision::{
        CharacterCollisionFlags, CharacterMoveResult, MovementState, SweepPass, execute_sweep_pass,
    },
    debug::MovementDebugConfig,
    debug_log,
    ground::{Ground, GroundingConfig, NonWalkableMode},
    stepping::{SteppingBehaviour, SteppingConfig},
    sweep::{CollideAndSlideConfig, SweepHitData},
};

/// A simple character controller data structure for movement calculations.
/// Just fill in the fields and call the methods - no lifetime hassles.
pub struct CharacterController<'a> {
    pub position: Vec3,
    pub rotation: Quat,
    pub character: Character,
    pub collider: Collider,
    pub config: CollideAndSlideConfig,
    pub grounding_config: Option<GroundingConfig>,
    pub stepping_config: Option<(SteppingConfig, SteppingBehaviour)>,
    pub current_ground: Option<Ground>,
    pub filter: SpatialQueryFilter,
    pub spatial_query: &'a SpatialQuery<'a, 'a>,
    pub debug_config: MovementDebugConfig,
}

impl<'a> CharacterController<'a> {
    pub fn new(
        position: Vec3,
        rotation: Quat,
        character: Character,
        collider: Collider,
        config: CollideAndSlideConfig,
        grounding_config: Option<GroundingConfig>,
        stepping_config: Option<(SteppingConfig, SteppingBehaviour)>,
        current_ground: Option<Ground>,
        filter: SpatialQueryFilter,
        spatial_query: &'a SpatialQuery<'a, 'a>,
        debug_config: MovementDebugConfig,
    ) -> Self {
        Self {
            position,
            rotation,
            character,
            collider,
            config,
            grounding_config,
            stepping_config,
            current_ground,
            filter,
            spatial_query,
            debug_config,
        }
    }

    /// The core PhysX-style character movement function.
    /// This is the "black box" equivalent to PhysX's Controller::move().
    pub fn move_character(&self, desired_displacement: Vec3) -> CharacterMoveResult {
        debug_log!(self.debug_config, "=== CHARACTER MOVE START ===");
        debug_log!(
            self.debug_config,
            "Desired displacement: {:?} (magnitude: {:.3})",
            desired_displacement,
            desired_displacement.length()
        );
        debug_log!(self.debug_config, "Start position: {:?}", self.position);

        let (vertical_component, horizontal_component) =
            decompose_displacement(desired_displacement, self.character.up);
        let dir_dot_up = desired_displacement.dot(*self.character.up);
        let has_horizontal_motion = horizontal_component.length_squared() > 1e-6;
        let is_moving_up = dir_dot_up > 0.0;

        let original_height = self.position.dot(*self.character.up);
        let half_height = self.collider.aabb(Vec3::ZERO, Quat::IDENTITY).size().y / 2.0;
        let original_bottom_point = original_height - half_height;

        // Determine if we should auto-step based on grounding and movement
        let is_grounded = self.current_ground.is_some();

        // PhysX logic: Don't auto-step when moving up (jumping) or when not moving horizontally
        let basic_auto_step_allowed = match &self.stepping_config {
            Some((_, SteppingBehaviour::Never)) => false,
            Some((_, SteppingBehaviour::Always)) => true,
            Some((_, SteppingBehaviour::Grounded)) => is_grounded,
            None => false,
        };

        // PhysX constraint: Cancel auto-step when moving upward (unless on moving platform)
        let should_auto_step = basic_auto_step_allowed && has_horizontal_motion && !is_moving_up; // This is the key PhysX logic!

        let auto_step_offset = if should_auto_step {
            self.stepping_config
                .as_ref()
                .map(|(config, _)| config.max_step_up)
                .unwrap_or(0.0)
        } else {
            0.0
        };

        let slope_validation_step_offset = self
            .stepping_config
            .as_ref()
            .map(|(config, _)| config.max_step_up)
            .unwrap_or(0.0);

        debug_log!(
            self.debug_config,
            "Movement analysis: horizontal={:?}, vertical={:?}, is_moving_up={}, should_auto_step={}, auto_step_offset={}",
            horizontal_component,
            vertical_component,
            is_moving_up,
            should_auto_step,
            auto_step_offset
        );

        // Continue with the rest of the movement logic...
        let mut final_movement_state = self.execute_movement_attempt(
            desired_displacement,
            auto_step_offset,
            slope_validation_step_offset,
            has_horizontal_motion,
            is_moving_up,
            false, // is_walk_experiment
            original_bottom_point,
        );

        // Walk experiment retry if needed
        if final_movement_state.hit_non_walkable {
            debug_log!(
                self.debug_config,
                "WALK EXPERIMENT: Hit unwalkable surface, retrying movement..."
            );

            // PhysX-style displacement modification based on non-walkable mode
            let modified_displacement = match self
                .grounding_config
                .as_ref()
                .map(|g| g.non_walkable_mode)
                .unwrap_or(NonWalkableMode::PreventClimbing)
            {
                NonWalkableMode::PreventClimbingAndForceSliding => {
                    // Use only horizontal component - removes all vertical climbing
                    debug_log!(
                        self.debug_config,
                        "WALK EXPERIMENT: Using horizontal-only displacement (force sliding mode)"
                    );
                    horizontal_component
                }
                NonWalkableMode::PreventClimbing => {
                    // Use original displacement with modified collision response
                    debug_log!(
                        self.debug_config,
                        "WALK EXPERIMENT: Using original displacement (prevent climbing mode)"
                    );
                    desired_displacement
                }
            };

            debug_log!(
                self.debug_config,
                "WALK EXPERIMENT: Modified displacement: {:?} (was: {:?})",
                modified_displacement,
                desired_displacement
            );

            let retry_result = self.execute_movement_attempt(
                modified_displacement,
                auto_step_offset,
                slope_validation_step_offset,
                has_horizontal_motion,
                modified_displacement.dot(*self.character.up) > 0.0, // Check if retry is moving up
                true,                                                // is_walk_experiment
                original_bottom_point,
            );

            final_movement_state = retry_result;
        }

        let collision_flags = CharacterCollisionFlags {
            up: final_movement_state.collision_flags & 0x1 != 0,
            sides: final_movement_state.collision_flags & 0x2 != 0,
            down: final_movement_state.collision_flags & 0x4 != 0,
        };

        let actual_displacement = final_movement_state.current_position - self.position;

        debug_log!(
            self.debug_config,
            "Movement result: final_pos={:?}, displacement={:?}, collisions={:?}",
            final_movement_state.current_position,
            actual_displacement,
            collision_flags
        );
        debug_log!(self.debug_config, "=== CHARACTER MOVE END ===\n");

        CharacterMoveResult {
            final_position: final_movement_state.current_position,
            collision_flags,
            ground: final_movement_state.ground,
            hit_non_walkable: final_movement_state.hit_non_walkable,
            actual_displacement,
        }
    }

    /// Calculate what the velocity should be after movement, based on collision results.
    /// This implements the game logic for how to interpret PhysX movement results.
    pub fn calculate_post_movement_velocity(
        &self,
        initial_velocity: Vec3,
        move_result: &CharacterMoveResult,
    ) -> Vec3 {
        let (mut vertical_velocity, horizontal_velocity) =
            decompose_velocity(initial_velocity, self.character.up);

        // Handle landing: zero downward velocity when we land normally
        // Key insight: Don't zero velocity if hit_non_walkable is true (sliding case)
        if move_result.ground.is_some()
            && vertical_velocity.dot(*self.character.up) <= 0.0
            && !move_result.hit_non_walkable
        // This is the crucial condition!
        {
            vertical_velocity = Vec3::ZERO;
            debug_log!(
                self.debug_config,
                "Landed normally: removed downward velocity, keeping horizontal={:?}",
                horizontal_velocity
            );
        }

        // Handle ceiling collision: zero upward velocity
        if move_result.collision_flags.up && vertical_velocity.dot(*self.character.up) > 0.0 {
            vertical_velocity = Vec3::ZERO;
            debug_log!(self.debug_config, "Ceiling hit: removed upward velocity");
        }

        horizontal_velocity + vertical_velocity
    }

    // Private implementation methods
    fn execute_movement_attempt(
        &self,
        displacement: Vec3,
        auto_step_offset: f32,
        slope_validation_step_offset: f32,
        has_horizontal_motion: bool,
        is_moving_up: bool,
        is_walk_experiment: bool,
        original_bottom_point: f32,
    ) -> MovementState {
        let mut movement_state = MovementState::new(self.position);
        movement_state.ground = self.current_ground;
        movement_state.walk_experiment = is_walk_experiment;

        let is_grounded = self.current_ground.is_some();
        let min_distance = self.config.skin_width * 0.001;

        let (vertical_component, horizontal_component) =
            decompose_displacement(displacement, self.character.up);

        let step_offset_was_applied =
            has_horizontal_motion && auto_step_offset > 0.0 && is_grounded && !is_walk_experiment;
        let up_vector = if is_moving_up && !is_walk_experiment {
            let mut up_motion = vertical_component;
            if step_offset_was_applied {
                up_motion += *self.character.up * auto_step_offset;
            }
            Some(up_motion)
        } else if step_offset_was_applied {
            Some(*self.character.up * auto_step_offset)
        } else {
            None
        };

        let side_vector = if has_horizontal_motion {
            Some(horizontal_component)
        } else {
            None
        };

        if up_vector.is_some() {
            debug_log!(
                self.debug_config,
                "  {}: Upward motion detected, clearing initial ground state",
                if is_walk_experiment { "RETRY" } else { "MAIN" }
            );
            movement_state.ground = None;
        }

        debug_log!(
            self.debug_config,
            "  {} - Pass vectors - Up: {:?}, Side: {:?}, Down: {:?}",
            if is_walk_experiment { "RETRY" } else { "MAIN" },
            up_vector.map(|v| (v, v.length())),
            side_vector.map(|v| (v, v.length())),
            if !is_moving_up {
                Some((vertical_component, vertical_component.length()))
            } else {
                None
            }
        );

        let mut collision_up = false;
        let mut collision_sides = false;
        let mut collision_down = false;

        // PASS 1: UP (skipped in walk experiment)
        if let Some(up_motion) = up_vector {
            if !is_walk_experiment {
                movement_state.target_orientation = movement_state.current_position + up_motion;

                let had_collision = execute_sweep_pass(
                    &mut movement_state,
                    SweepPass::Up,
                    1,
                    min_distance,
                    original_bottom_point,
                    &self.collider,
                    self.rotation,
                    self.grounding_config.as_ref(),
                    &self.spatial_query,
                    &self.filter,
                    self.character.up,
                    self.config.skin_width,
                    self.debug_config,
                );

                if had_collision {
                    collision_up = true;
                }

                movement_state.target_orientation = movement_state.current_position;
            } else {
                debug_log!(
                    self.debug_config,
                    "  RETRY: Skipping UP PASS (walk experiment)"
                );
            }
        }

        // PASS 2: SIDE
        if let Some(side_motion) = side_vector {
            movement_state.target_orientation = movement_state.current_position + side_motion;
            movement_state.prevent_vertical_motion = is_walk_experiment;

            let had_collision = execute_sweep_pass(
                &mut movement_state,
                SweepPass::Side,
                4,
                min_distance,
                original_bottom_point,
                &self.collider,
                self.rotation,
                self.grounding_config.as_ref(),
                &self.spatial_query,
                &self.filter,
                self.character.up,
                self.config.skin_width,
                self.debug_config,
            );

            if had_collision {
                collision_sides = true;
            }
        }

        // Side collision slope validation
    if !is_walk_experiment && !is_moving_up && movement_state.validate_triangle_side {
        let max_slope_angle = self
            .grounding_config
            .as_ref()
            .map_or(std::f32::consts::FRAC_PI_4, |g| g.max_angle);

        let slope_is_unwalkable = test_slope(
            movement_state.contact_normal_side_pass,
            *self.character.up,
            max_slope_angle,
        );

        if slope_is_unwalkable {
            let start_height = self.position.dot(*self.character.up);
            let current_height = movement_state.current_position.dot(*self.character.up);
            let height_gained = current_height - start_height;
            
            if height_gained > 1e-6 {
                movement_state.hit_non_walkable = true;
                debug_log!(
                    self.debug_config,
                    "HIT NON-WALKABLE: Gained height ({:.6}) while sliding on an unwalkable slope. Triggering walk experiment.",
                    height_gained
                );
            }
        }
    }

        // PASS 3: DOWN
        let down_motion = if !is_moving_up {
            vertical_component
        } else {
            Vec3::ZERO
        };

        let corrected_down_motion = if step_offset_was_applied {
            down_motion - *self.character.up * auto_step_offset
        } else {
            down_motion
        };

        movement_state.target_orientation = movement_state.current_position + corrected_down_motion;

        let had_collision = execute_sweep_pass(
            &mut movement_state,
            SweepPass::Down,
            1,
            min_distance,
            original_bottom_point,
            &self.collider,
            self.rotation,
            self.grounding_config.as_ref(),
            &self.spatial_query,
            &self.filter,
            self.character.up,
            self.config.skin_width,
            self.debug_config,
        );

        if had_collision && displacement.dot(*self.character.up) <= 0.0 {
            collision_down = true;
        }

        // Triangle height validation
        if !is_walk_experiment && movement_state.validate_triangle_down && has_horizontal_motion {
            let max_slope = self
                .grounding_config
                .as_ref()
                .map_or(std::f32::consts::FRAC_PI_4, |g| g.max_angle);
            let touched_tri_height = movement_state.touched_tri_max - original_bottom_point;

            if touched_tri_height > slope_validation_step_offset
                && test_slope(
                    movement_state.contact_normal_down_pass,
                    *self.character.up,
                    max_slope,
                )
            {
                movement_state.hit_non_walkable = true;
                debug_log!(
                    self.debug_config,
                    "Triangle slope validation failed - surface too steep. Tri height: {:.6}, step offset: {:.6}",
                    touched_tri_height,
                    slope_validation_step_offset
                );
            }
        }

        // Store collision flags
        movement_state.collision_flags = 0;
        if collision_up {
            movement_state.collision_flags |= 0x1;
        }
        if collision_sides {
            movement_state.collision_flags |= 0x2;
        }
        if collision_down {
            movement_state.collision_flags |= 0x4;
        }

        movement_state
    }
}

fn test_slope(normal: Vec3, up_direction: Vec3, slope_limit: f32) -> bool {
    normal.angle_between(up_direction) > slope_limit
}

fn decompose_displacement(displacement: Vec3, up_direction: Dir3) -> (Vec3, Vec3) {
    let vertical = displacement.project_onto(*up_direction);
    let horizontal = displacement - vertical;
    (vertical, horizontal)
}

fn decompose_velocity(velocity: Vec3, up_direction: Dir3) -> (Vec3, Vec3) {
    let vertical = velocity.project_onto(*up_direction);
    let horizontal = velocity - vertical;
    (vertical, horizontal)
}
