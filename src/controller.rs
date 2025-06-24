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
    pub fn move_character(&mut self, desired_displacement: Vec3) -> CharacterMoveResult {
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

        let original_position = self.position;

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

        let did_hit_non_walkable = final_movement_state.hit_non_walkable;
        // Walk experiment retry if needed
        if final_movement_state.hit_non_walkable {
            debug_log!(
                self.debug_config,
                "WALK EXPERIMENT: Hit unwalkable surface, retrying movement..."
            );

            // PhysX-style displacement modification based on non-walkable mode
            let modified_displacement = match self.config.slide_mode {
                NonWalkableMode::PreventClimbingAndForceSliding => {
                    // CRITICAL FIX: PhysX decomposes into (xpDisp, tangent_compo)
                    // where xpDisp gets the VERTICAL component, not horizontal!
                    // This is counter-intuitive but matches the PhysX code exactly
                    let (vertical_component, _horizontal_component) =
                        decompose_displacement(desired_displacement, self.character.up);

                    debug_log!(
                        self.debug_config,
                        "WALK EXPERIMENT: Using vertical-only displacement (force sliding mode): {:?}",
                        vertical_component
                    );
                    vertical_component // This removes horizontal climbing forces
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

            self.position = original_position;

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
            hit_non_walkable: did_hit_non_walkable, // use the ORIGINAL did_hit_non_walkable state
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
        let (mut vertical_velocity, mut horizontal_velocity) =
            decompose_velocity(initial_velocity, self.character.up);

        // Handle landing: zero downward velocity when we land normally
        if move_result.ground.is_some() && vertical_velocity.dot(*self.character.up) <= 0.0 {
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

        // CRITICAL PhysX behavior: Handle non-walkable surface collisions
        if move_result.hit_non_walkable {
            debug_log!(
                self.debug_config,
                "Hit non-walkable surface - checking if this was gravity-induced motion"
            );

            let horizontal_speed = horizontal_velocity.length();
            let horizontal_displacement = {
                let horizontal_actual = move_result
                    .actual_displacement
                    .reject_from(*self.character.up);
                horizontal_actual.length()
            };

            if horizontal_speed > 1e-6 && horizontal_displacement < horizontal_speed * 0.2 {
                // This indicates gravity-induced motion that was blocked
                // Reduce horizontal velocity significantly to prevent sliding
                horizontal_velocity *= 0.1;
                debug_log!(
                    self.debug_config,
                    "Detected blocked motion on non-walkable surface - reduced horizontal velocity to {:?}",
                    horizontal_velocity
                );
            }
        }

        // Handle side collisions: reduce velocity in collision direction
        if move_result.collision_flags.sides {
            // If we hit a wall but it's not a steep slope (i.e., normal ground-level wall),
            // we should still allow some sliding behavior for normal wall-running

            // But if we have very little actual displacement despite having velocity,
            // it means we're stuck against something and should reduce velocity
            let velocity_magnitude = initial_velocity.length();
            let displacement_magnitude = move_result.actual_displacement.length();

            if velocity_magnitude > 0.1 && displacement_magnitude < velocity_magnitude * 0.1 {
                // We're not moving much despite having velocity - we're stuck
                // Reduce horizontal velocity significantly

                // TODO: Temporarily disabled, this feels unnatural
                // Is there not a more normal way to prevent excess sliding against unwalkable surfaces?
                //horizontal_velocity *= 0.2;
                debug_log!(
                    self.debug_config,
                    "Detected stuck against wall - reduced horizontal velocity to {:?}",
                    horizontal_velocity
                );
            }
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

        let max_iter = self.config.max_iterations;
        let max_iter_side = max_iter;
        let max_iter_up = if side_vector.is_some_and(|it| it.length().abs() < 0.001) {
            max_iter
        } else {
            1
        };
        let max_iter_down = if is_walk_experiment
            && matches!(
                self.config.slide_mode,
                NonWalkableMode::PreventClimbingAndForceSliding
            ) {
            10
        } else {
            1
        };

        // PASS 1: UP (skipped in walk experiment)
        if let Some(up_motion) = up_vector {
            if !is_walk_experiment {
                movement_state.target_orientation = movement_state.current_position + up_motion;

                let had_collision = execute_sweep_pass(
                    &mut movement_state,
                    SweepPass::Up,
                    max_iter_up,
                    min_distance,
                    original_bottom_point,
                    &self.collider,
                    self.rotation,
                    self.grounding_config.as_ref(),
                    &self.config,
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
                max_iter_side,
                min_distance,
                original_bottom_point,
                &self.collider,
                self.rotation,
                self.grounding_config.as_ref(),
                &self.config,
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
        if !is_walk_experiment && movement_state.validate_triangle_side {
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
                movement_state.hit_non_walkable = true;
                debug_log!(
                    self.debug_config,
                    "HIT NON-WALKABLE: Side pass hit an unwalkable slope at height {:.3}, which is > character bottom {:.3} + step offset {:.3}",
                    movement_state.contact_point_height,
                    original_bottom_point,
                    slope_validation_step_offset
                );
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
            max_iter_down,
            min_distance,
            original_bottom_point,
            &self.collider,
            self.rotation,
            self.grounding_config.as_ref(),
            &self.config,
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
        if movement_state.validate_triangle_down {
            let max_slope = self
                .grounding_config
                .as_ref()
                .map_or(std::f32::consts::FRAC_PI_4, |g| g.max_angle);

            // First, check if the slope itself is too steep. This is our simple check from before.
            let is_steep_slope = test_slope(
                movement_state.contact_normal_down_pass,
                *self.character.up,
                max_slope,
            );

            if is_steep_slope {
                // PhysX can get the actual triangle hit and derive a height that way
                // We can't do that, so to get an accurate height, we are going to raycast from above the point of contact
                // To get an object height.

                // 1. Get the initial contact point from the main down sweep.
                let contact_point = movement_state.current_position
                    + (movement_state.contact_normal_down_pass * self.config.skin_width); // A reasonable approximation of the hit point

                // 2. Define the starting position for our verification ray, lifted up by the step offset.
                let vertical_offset = *self.character.up * (slope_validation_step_offset + 0.05); // Add a small buffer
                let ray_start = contact_point + vertical_offset;

                // 3. Cast a short ray straight down from the elevated position.
                let ray_dir = Dir3::new(-*self.character.up).unwrap_or(Dir3::NEG_Y);
                let max_toi = slope_validation_step_offset + 0.1; // Ray just needs to be slightly longer than the offset

                if let Some(hit) = self.spatial_query.cast_ray(
                    ray_start,
                    ray_dir,
                    max_toi, // max_distance
                    true,    // solid
                    &self.filter,
                ) {
                    let hit_point = ray_start + (*ray_dir * hit.distance);
                    // 4. Calculate the height of the obstacle relative to the character's starting position.
                    let obstacle_height = hit_point.dot(*self.character.up);
                    let obstacle_height_above_feet = obstacle_height - original_bottom_point;

                    // 5. If this accurate height is greater than the step offset, it's a non-walkable wall.
                    if obstacle_height_above_feet > slope_validation_step_offset {
                        movement_state.hit_non_walkable = true;
                        movement_state.touched_obstacle_height = obstacle_height;

                        debug_log!(
                            self.debug_config,
                            "Hit non-walkable from VERIFICATION CAST: Accurate obstacle height {:.3} > step offset {:.3}",
                            obstacle_height_above_feet,
                            slope_validation_step_offset
                        );
                    }
                }
            }
        }

        // If we're in walk experiment mode, perform the PhysX recovery sweep
        if movement_state.hit_non_walkable && movement_state.walk_experiment {
            debug_log!(
                self.debug_config,
                "RECOVERY SWEEP: Performing PhysX-style recovery sweep for steep slope sliding"
            );

            // Set PhysX normalize response flag for recovery sweep
            movement_state.normalize_response = true;
            movement_state.prevent_vertical_motion = true;

            // Calculate recovery distance (PhysX logic)
            let current_height = movement_state.current_position.dot(*self.character.up);
            let original_height = self.position.dot(*self.character.up);
            let mut delta = if current_height > original_height {
                current_height - original_height
            } else {
                0.0
            };
            delta += displacement.dot(*self.character.up).abs();
            let recover_distance = delta;

            // Create downward recovery vector
            let recovery_vector = -*self.character.up * recover_distance;

            debug_log!(
                self.debug_config,
                "RECOVERY SWEEP: delta={:.6}, recover_distance={:.6}, recovery_vector={:?}",
                delta,
                recover_distance,
                recovery_vector
            );

            // Set target for recovery sweep
            movement_state.target_orientation = movement_state.current_position + recovery_vector;

            // Execute the recovery sweep with multiple iterations (this is the key!)
            let recovery_min_dist = if recover_distance < min_distance {
                recover_distance / max_iter as f32
            } else {
                min_distance
            };

            execute_sweep_pass(
                &mut movement_state,
                SweepPass::Down, // PhysX uses SWEEP_PASS_UP for compatibility, but it's technically a down pass
                max_iter,
                recovery_min_dist,
                original_bottom_point,
                &self.collider,
                self.rotation,
                self.grounding_config.as_ref(),
                &self.config,
                &self.spatial_query,
                &self.filter,
                self.character.up,
                self.config.skin_width,
                self.debug_config,
            );

            // Clear the normalize response flag
            movement_state.normalize_response = false;
            movement_state.prevent_vertical_motion = false;

            debug_log!(
                self.debug_config,
                "RECOVERY SWEEP: Final position after recovery: {:?}",
                movement_state.current_position
            );
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
