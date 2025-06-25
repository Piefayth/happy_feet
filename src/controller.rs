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
    movement::feet_position,
    stepping::{SteppingBehaviour, SteppingConfig},
    sweep::{CollideAndSlideConfig, SweepHitData},
};

// Just use CharacterCollisionFlags directly everywhere!

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
        let half_height = self.collider.aabb(Vec3::ZERO, self.rotation).size().y / 2.0;
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
        let should_auto_step = basic_auto_step_allowed && has_horizontal_motion && !is_moving_up;
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

        // Primary movement attempt. Happy path is colliding and sliding into a safe place to stand.
        let final_movement_state = self.execute_movement_attempt(
            desired_displacement,
            auto_step_offset,
            slope_validation_step_offset,
            has_horizontal_motion,
            is_moving_up,
            false, // is_walk_experiment
            original_bottom_point,
        );

        let did_hit_non_walkable = final_movement_state.hit_non_walkable;
        let final_movement_state = if final_movement_state.hit_non_walkable {
            // Sad path, we're trying to end our step somewhere invalid
            // Try again WITHOUT stepping up

            debug_log!(
                self.debug_config,
                "WALK EXPERIMENT: Hit unwalkable surface, retrying movement..."
            );

            let modified_displacement = match self.config.slide_mode {
                NonWalkableMode::PreventClimbingAndForceSliding => {
                    let (vertical_component, _horizontal_component) =
                        decompose_displacement(desired_displacement, self.character.up);

                    debug_log!(
                        self.debug_config,
                        "WALK EXPERIMENT: Using vertical-only displacement (force sliding mode): {:?}",
                        vertical_component
                    );

                    // You cannot "walk along" a steep slope if ForceSliding is on
                    // So horizontal movement is removed
                    vertical_component
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

            self.execute_movement_attempt(
                modified_displacement,
                auto_step_offset,
                slope_validation_step_offset,
                has_horizontal_motion,
                modified_displacement.dot(*self.character.up) > 0.0, // Check if retry is moving up
                true,                                                // is_walk_experiment
                original_bottom_point,
            )
        } else {
            final_movement_state
        };

        let actual_displacement = final_movement_state.current_position - self.position;

        debug_log!(
            self.debug_config,
            "Movement result: final_pos={:?}, displacement={:?}, collisions={:?}",
            final_movement_state.current_position,
            actual_displacement,
            final_movement_state.collision_flags
        );
        debug_log!(self.debug_config, "=== CHARACTER MOVE END ===\n");

        CharacterMoveResult {
            final_position: final_movement_state.current_position,
            collision_flags: final_movement_state.collision_flags,
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

        let started_attempt_on_walkable_slope = !is_walk_experiment && self.current_ground
            .map(|ground| {
                let max_slope_angle = self
                    .grounding_config
                    .as_ref()
                    .map_or(std::f32::consts::FRAC_PI_4, |g| g.max_angle);
                !test_slope(*ground.normal, *self.character.up, max_slope_angle)
            })
            .unwrap_or(false);

        let mut collisions = CharacterCollisionFlags::new();

        let is_grounded = self.current_ground.is_some();
        let min_distance = self.config.skin_width * 0.001;
        let max_slope_angle = self
            .grounding_config
            .as_ref()
            .map_or(std::f32::consts::FRAC_PI_4, |g| g.max_angle);

        let (vertical_component, horizontal_component) =
            decompose_displacement(displacement, self.character.up);

        let step_offset_was_applied =
            has_horizontal_motion && auto_step_offset > 0.0 && is_grounded;
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
                debug_log!(
                    self.debug_config,
                    "  MAIN: Starting UP PASS with motion: {:?} (magnitude: {:.3})",
                    up_motion,
                    up_motion.length()
                );

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

                collisions.up = had_collision;
                movement_state.collision_flags = collisions;
                movement_state.target_orientation = movement_state.current_position;

                debug_log!(
                    self.debug_config,
                    "  MAIN: Completed UP PASS - collision: {}, final_pos: {:?}",
                    had_collision,
                    movement_state.current_position
                );
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

            collisions.sides = had_collision;
            movement_state.collision_flags = collisions;
        }

        if collisions.sides {
            // 1817	if(1 && constrainedClimbingMode && volume.getType()==SweptVolumeType::eCAPSULE && !(mFlags & STF_VALIDATE_TRIANGLE_SIDE))
            // There is some kind of sensor pass going on here, I think we can skip for now?
            // LLM: ignore this for now thank you
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

        // 1859-1861 if(NbCollisions && dir_dot_up<=0.0f)
        if had_collision && displacement.dot(*self.character.up) <= 0.0 {
            collisions.down = true;
            movement_state.collision_flags = collisions;

            if collisions.sides {
                let is_steep_slope = test_slope(
                    movement_state.contact_normal_side_pass,
                    *self.character.up,
                    max_slope_angle,
                );
                let contact_height = movement_state.touched_obstacle_height;
                let feet_height = original_bottom_point;
                let step_threshold = slope_validation_step_offset;

                let step_is_too_high = contact_height > feet_height + step_threshold;
                if step_is_too_high && is_steep_slope { 
                    movement_state.hit_non_walkable = true;
                    return movement_state;
                }
            }
        }

        debug_log!(
            self.debug_config,
            "  DOWN PASS RESULTS: validate_triangle_down={}, hit_non_walkable={}, ground={:?}",
            movement_state.validate_triangle_down,
            movement_state.hit_non_walkable,
            movement_state.ground
        );

        // We hit something below us, is it walkable?
        // 1897 (mFlags & STF_VALIDATE_TRIANGLE_DOWN) && dir_dot_up<=0.0f)
        // if movement_state.validate_triangle_down && displacement.dot(*self.character.up) <= 0.0 {
        //     let is_steep_slope = test_slope(
        //         movement_state.contact_normal_down_pass,
        //         *self.character.up,
        //         max_slope_angle,
        //     );

        //     let contact_height = movement_state.touched_obstacle_height;
        //     let feet_height = original_bottom_point;
        //     let step_threshold = slope_validation_step_offset;

        //     debug_log!(
        //         self.debug_config,
        //         "  SLOPE VALIDATION: normal={:?}, angle={:.3}°, max={:.3}°, is_steep={}, contact_height={:.6}, feet_height={:.6}, step_threshold={:.6}",
        //         movement_state.contact_normal_down_pass,
        //         movement_state
        //             .contact_normal_down_pass
        //             .angle_between(*self.character.up)
        //             .to_degrees(),
        //         max_slope_angle.to_degrees(),
        //         is_steep_slope,
        //         contact_height,
        //         feet_height,
        //         step_threshold
        //     );

        //     let step_is_too_high = contact_height > feet_height + step_threshold;

        //     // 1919: if(touchedTriHeight>mUserParams.mStepOffset && testSlope(Normal, upDirection, mUserParams.mSlopeLimit))
        //     // PhysX actually tests against the highest point of the collided triangle here, we don't have the ability to do that
        //     // Subbing this "start on walkable slope" logic for now...
        //     // The idea is you can STAND on a too steep slope if you set the option, but can't move to a steep slope from a steep slope.
        //     let prevented_from_climbing =  matches!(self.config.slide_mode, NonWalkableMode::PreventClimbing) && 
        //         !started_attempt_on_walkable_slope &&
        //         is_steep_slope;

        //     if prevented_from_climbing {
        //         movement_state.hit_non_walkable = true;
                
        //         if !movement_state.walk_experiment {
        //             // Continuing is pointless if we aren't in the retry since we know we're going to run again
        //             return movement_state;
        //         }

        //         // Begin recovery sweep
        //         // I believe we should only hit this if we hit an unwalkable surface in the main pass
        //         // Then, on the retry, we STILL hit an unwalkable surface.

        //         movement_state.normalize_response = true; // 1932

        //         let current_height = movement_state.current_position.dot(*self.character.up);
        //         let original_height = self.position.dot(*self.character.up);
        //         let mut delta = if current_height > original_height {
        //             current_height - original_height
        //         } else {
        //             0.0
        //         };
        //         delta += displacement.dot(*self.character.up).abs();
        //         let recover_distance = delta;

        //         movement_state.collision_flags = CharacterCollisionFlags::default(); // 1939

        //         let recovery_min_dist = if recover_distance < min_distance {
        //             recover_distance / max_iter as f32
        //         } else {
        //             min_distance
        //         };

        //         let recovery_vector = -*self.character.up * recover_distance;
        //         movement_state.target_orientation =
        //             movement_state.current_position + recovery_vector;

        //         debug_log!(
        //             self.debug_config,
        //             "BEGIN RECOVERY SWEEP: delta={:.6}, recover_distance={:.6}, recovery_vector={:?}",
        //             delta,
        //             recover_distance,
        //             recovery_vector
        //         );

        //         execute_sweep_pass(
        //             &mut movement_state,
        //             SweepPass::Down, // or SweepPass::Up for PhysX compatibility
        //             max_iter,
        //             recovery_min_dist,
        //             original_bottom_point,
        //             &self.collider,
        //             self.rotation,
        //             self.grounding_config.as_ref(),
        //             &self.config,
        //             &self.spatial_query,
        //             &self.filter,
        //             self.character.up,
        //             self.config.skin_width,
        //             self.debug_config,
        //         );

        //         movement_state.normalize_response = false; // 1954 after the recovoery sweep

        //         debug_log!(
        //             self.debug_config,
        //             "END RECOVERY SWEEP: Final position after recovery: {:?}",
        //             movement_state.current_position
        //         );
        //     }
        // }

        movement_state.collision_flags = collisions;
        movement_state
    }
}

pub fn test_slope(normal: Vec3, up_direction: Vec3, slope_limit: f32) -> bool {
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
