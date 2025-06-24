use avian3d::prelude::*;
use bevy::prelude::*;

use crate::{
    debug::MovementDebugConfig,
    debug_log,
    ground::{Ground, GroundingConfig, is_walkable},
    sweep::{CollideAndSlideConfig, SweepHitData, sweep},
};

/// Collision flags returned by character movement, matching PhysX API
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CharacterCollisionFlags {
    pub up: bool,
    pub sides: bool,
    pub down: bool,
}

impl CharacterCollisionFlags {
    pub fn new() -> Self {
        Self {
            up: false,
            sides: false,
            down: false,
        }
    }

    pub fn any(&self) -> bool {
        self.up || self.sides || self.down
    }
}

/// Result of a character movement operation, matching PhysX Controller::move API
#[derive(Debug, Clone)]
pub struct CharacterMoveResult {
    /// The final position after movement
    pub final_position: Vec3,
    /// Collision flags indicating which surfaces were hit
    pub collision_flags: CharacterCollisionFlags,
    /// The ground surface the character is standing on (if any)
    pub ground: Option<Ground>,
    /// Whether the character hit a non-walkable surface during movement
    pub hit_non_walkable: bool,
    /// The actual displacement that occurred (may be less than requested due to collisions)
    pub actual_displacement: Vec3,
}

/// PhysX-style movement pass types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SweepPass {
    Up,
    Side,
    Down,
}

impl SweepPass {
    pub fn name(&self) -> &'static str {
        match self {
            SweepPass::Up => "UP",
            SweepPass::Side => "SIDE",
            SweepPass::Down => "DOWN",
        }
    }
}

/// Internal movement state for collision detection, closely mirrors PhysX's PhysXMovementState
#[derive(Debug, Clone)]
pub struct MovementState {
    pub current_position: Vec3,
    pub target_orientation: Vec3,
    pub up: Dir3,
    pub ground: Option<Ground>,
    pub collision_flags: u32,
    pub validate_triangle_down: bool,
    pub validate_triangle_side: bool,
    pub hit_non_walkable: bool,
    pub contact_normal_side_pass: Vec3,
    pub contact_normal_down_pass: Vec3,
    pub contact_point_height: f32,
    pub walk_experiment: bool,
    pub touched_obstacle_height: f32,
    pub prevent_vertical_motion: bool,
    pub normalize_response: bool,
}

impl MovementState {
    pub fn new(position: Vec3) -> Self {
        Self {
            current_position: position,
            target_orientation: position,
            up: Dir3::Y,
            ground: None,
            collision_flags: 0,
            validate_triangle_down: false,
            validate_triangle_side: false,
            hit_non_walkable: false,
            contact_normal_side_pass: Vec3::ZERO,
            contact_normal_down_pass: Vec3::ZERO,
            contact_point_height: 0.0,
            walk_experiment: false,
            touched_obstacle_height: 0.0,
            prevent_vertical_motion: false,
            normalize_response: true,
        }
    }

    pub fn current_direction(&self) -> Option<(Dir3, f32)> {
        let displacement = self.target_orientation - self.current_position;
        let length = displacement.length();
        if length < 1e-6 {
            None
        } else {
            Some((Dir3::new_unchecked(displacement / length), length))
        }
    }
}

/// Execute a single PhysX-style sweep pass
pub fn execute_sweep_pass(
    state: &mut MovementState,
    pass_type: SweepPass,
    max_iterations: u8,
    min_distance: f32,
    original_bottom_point: f32,
    collider: &Collider,
    rotation: Quat,
    grounding_config: Option<&GroundingConfig>,
    config: &CollideAndSlideConfig,
    spatial_query: &SpatialQuery,
    filter: &SpatialQueryFilter,
    up_direction: Dir3,
    skin_width: f32,
    debug_config: MovementDebugConfig,
) -> bool {
    let pass_label = if state.walk_experiment {
        format!("RETRY {}", pass_type.name())
    } else {
        pass_type.name().to_string()
    };

    debug_log!(debug_config, "  {} PASS START", pass_label);

    let mut had_collision = false;
    let original_direction = if let Some((dir, _)) = state.current_direction() {
        *dir
    } else {
        debug_log!(debug_config, "  {} PASS: No movement needed", pass_label);
        return false;
    };

    for iteration in 0..max_iterations {
        let Some((current_direction, max_distance)) = state.current_direction() else {
            debug_log!(
                debug_config,
                "    {} iter {}: No more movement needed",
                pass_label,
                iteration
            );
            break;
        };

        if max_distance <= min_distance {
            debug_log!(
                debug_config,
                "    {} iter {}: Distance {:.6} below minimum {:.6}",
                pass_label,
                iteration,
                max_distance,
                min_distance
            );
            break;
        }

        // PhysX "Quake2 hack" - prevent tiny oscillations in sloping corners
        if current_direction.dot(original_direction) <= 0.0 {
            debug_log!(
                debug_config,
                "    {} iter {}: Direction reversed, stopping (Quake2 hack)",
                pass_label,
                iteration
            );
            break;
        }

        let Some(hit) = sweep(
            collider,
            state.current_position,
            rotation,
            current_direction,
            max_distance,
            skin_width,
            spatial_query,
            filter,
            true, // ignore_origin_penetration - PhysX uses true for CCT
        ) else {
            debug_log!(
                debug_config,
                "    {} iter {}: No collision, moving full distance",
                pass_label,
                iteration
            );
            if max_distance > min_distance {
                state.current_position = state.target_orientation;
            }

            // Clear ground when moving down with no collision (falling)
            if pass_type == SweepPass::Down && max_distance > min_distance {
                state.ground = None;
            }

            break;
        };

        had_collision = true;
        let safe_distance = hit.distance.max(0.0);
        state.current_position += *current_direction * safe_distance;

        let mut effective_normal = hit.normal;
        if state.walk_experiment && pass_type == SweepPass::Side {
            // This is a walk experiment retry on a steep slope.
            // Flatten the normal to prevent climbing from the side pass.
            let normal_component = hit.normal.project_onto(state.up.into());
            let tangent_component = hit.normal - normal_component;
            if tangent_component.length_squared() > 1e-6 {
                effective_normal = tangent_component.normalize();
            }
        }

        // Handle collision based on pass type and grounding configuration
        if let Some(grounding_config) = grounding_config {
            match pass_type {
                SweepPass::Down => {
                    state.validate_triangle_down = true;
                    state.contact_normal_down_pass = hit.normal;

                    if is_walkable(hit.normal, grounding_config.max_angle, *up_direction) {
                        let ground = Ground::new(hit.entity, hit.normal);
                        state.ground = Some(ground);
                    }

                    // Equivalent of PhysX triangle height tracking for slope validation
                    // We get the actual height for free from our hit!
                    state.touched_obstacle_height = hit.point.dot(*up_direction);
                }
                SweepPass::Side => {
                    state.validate_triangle_side = true;
                    state.contact_normal_side_pass = hit.normal;
                    state.contact_point_height = hit.point.dot(*up_direction);

                    debug_log!(
                        debug_config,
                        "    {} SIDE HIT: normal={:?}, hit_point={:?}, contact_height={:.6}, character_pos={:?}",
                        pass_label,
                        hit.normal,
                        hit.point,
                        hit.point.dot(*up_direction),
                        state.current_position
                    );
                }
                SweepPass::Up => {
                    // Up pass doesn't need special grounding handling
                }
            }
        }

        // PhysX collision response - modify target_orientation for next iteration
        physx_collision_response(state, *current_direction, effective_normal);
    }

    debug_log!(
        debug_config,
        "  {} PASS END: had_collision={}, final_pos={:?}",
        pass_label,
        had_collision,
        state.current_position
    );

    had_collision
}

/// PhysX-style collision response that modifies the target orientation
/// This is the core collision response algorithm from PhysX CCT
// fn physx_collision_response(
//     state: &mut MovementState,
//     current_direction: Vec3,
//     // This is the "effective normal" passed in by the caller.
//     hit_normal: Vec3,
// ) {
//     let amplitude = (state.target_orientation - state.current_position).length();

//     if amplitude < 1e-6 {
//         return;
//     }

//     // The logic to flatten the normal is now handled by the caller.
//     // This function just performs the response with the given normal.
//     let reflect_dir = current_direction - hit_normal * 2.0 * current_direction.dot(hit_normal);
//     let reflect_dir = reflect_dir.normalize_or_zero();

//     let normal_component = reflect_dir.project_onto(hit_normal);
//     let tangent_component = reflect_dir - normal_component;

//     let friction = 1.0;

//     state.target_orientation = state.current_position;

//     if friction != 0.0 {
//         state.target_orientation += tangent_component * friction * amplitude;
//     }
// }

fn physx_collision_response(state: &mut MovementState, current_direction: Vec3, hit_normal: Vec3) {
    let amplitude = (state.target_orientation - state.current_position).length();

    if amplitude < 1e-6 {
        return;
    }

    // Compute reflect direction (PhysX: computeReflexionVector)
    let reflect_dir = current_direction - hit_normal * 2.0 * current_direction.dot(hit_normal);
    let reflect_dir = reflect_dir.normalize_or_zero();

    // Decompose reflection into normal and tangent components (PhysX: Ps::decomposeVector)
    let normal_component = reflect_dir.project_onto(hit_normal);
    let tangent_component = reflect_dir - normal_component;

    // PhysX constants
    let bump = 0.0; // PhysX always uses 0.0 for bump
    let friction = 1.0; // PhysX always uses 1.0 for friction

    // Reset target to current position (PhysX behavior)
    state.target_orientation = state.current_position;

    // Apply bump component (usually zero in PhysX)
    if bump != 0.0 {
        let mut bump_component = normal_component;
        if state.normalize_response {
            bump_component = bump_component.normalize_or_zero();
        }
        state.target_orientation += bump_component * bump * amplitude;
    }

    // Apply friction component (tangential movement)
    if friction != 0.0 {
        let mut friction_component = tangent_component;
        if state.normalize_response {
            friction_component = friction_component.normalize_or_zero();
        }
        state.target_orientation += friction_component * friction * amplitude;
    }
}
