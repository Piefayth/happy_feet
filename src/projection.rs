use std::fmt::Debug;

use bevy::{math::InvalidDirectionError, prelude::*};

use crate::is_walkable;

#[derive(Debug, Clone, Copy)]
pub(crate) struct Surface {
    pub normal: Dir3,
    pub is_walkable: bool,
}

impl Surface {
    pub(crate) fn new<D>(normal: D, walkable_angle: f32, up_direction: Dir3) -> Self
    where
        D: TryInto<Dir3>,
        <D as TryInto<Dir3>>::Error: Debug,
    {
        let normal = normal.try_into().unwrap();
        Self {
            normal,
            is_walkable: is_walkable(*normal, walkable_angle, *up_direction),
        }
    }

    pub fn obstruction_normal(
        &self,
        current_ground_normal: Option<Dir3>,
        up_direction: Dir3,
    ) -> Result<Dir3, InvalidDirectionError> {
        if !self.is_walkable {
            if let Some(ground_normal) = current_ground_normal {
                // Calculate obstruction normal perpendicular to both ground normal and the up direction
                let tangent = Dir3::new(ground_normal.cross(*self.normal))?;
                return Dir3::new(tangent.cross(*up_direction));
            }
        }

        Ok(self.normal)
    }


    #[must_use]
    pub fn project_velocity(
        &self,
        velocity: Vec3,
        current_ground_normal: Option<Dir3>,
        up_direction: Dir3,
    ) -> Vec3 {
        let obstruction_normal = *self
            .obstruction_normal(current_ground_normal, up_direction)
            .unwrap();

        project_velocity(
            velocity,
            obstruction_normal,
            self.is_walkable,
            current_ground_normal,
            up_direction,
        )
    }
}


pub(crate) fn project_velocity(
    velocity: Vec3,
    obstruction_normal: Vec3,
    is_walkable: bool,
    current_ground_normal: Option<Dir3>,
    up_direction: Dir3,
) -> Vec3 {
    match (current_ground_normal, is_walkable) {
        // Character on ground, moving to walkable surface
        (Some(_), true) => {
            // Align the velocity to the surface while maintaining the horizontal direction
            align_with_surface(velocity, obstruction_normal, *up_direction)
        }
        // Character on ground, moving to non-walkable surface - SLIDE ONLY
        (Some(_ground_normal), false) => {
            // Just slide along the wall - stepping happens through position displacement
            velocity.reject_from(obstruction_normal)
        }
        // Character in air, hitting walkable surface
        (None, true) => {
            // Remove the vertical component and align with the surface
            let velocity = velocity.reject_from(*up_direction);
            align_with_surface(velocity, obstruction_normal, *up_direction)
        }
        // Character in air, hitting non-walkable surface
        (None, false) => {
            // Simply slide along the surface
            velocity.reject_from(obstruction_normal)
        }
    }
}

/// Detects if two colliding surfaces form a crease that requires special handling.
pub(crate) fn detect_crease(
    current_surface: Surface,
    previous_surface: Surface,
    current_velocity: Vec3,
    previous_velocity: Vec3,
    is_grounded: bool,
) -> Option<Dir3> {
    // Skip if both surfaces are walkable and character is currently grounded
    if is_grounded && current_surface.is_walkable && previous_surface.is_walkable {
        return None;
    }

    // Skip if normals are nearly parallel
    if current_surface.normal.dot(*previous_surface.normal) > 1.0 - 1e-3 {
        return None;
    }

    // Calculate the direction of the crease
    let mut crease_direction =
        Dir3::new(current_surface.normal.cross(*previous_surface.normal)).unwrap();

    // Project normals onto the plane perpendicular to the crease
    let current_normal_on_crease_plane =
        *Dir3::new(current_surface.normal.reject_from(*crease_direction)).unwrap();
    let previous_normal_on_crease_plane =
        *Dir3::new(previous_surface.normal.reject_from(*crease_direction)).unwrap();

    // Project previous velocity onto the crease plane
    let entering_velocity_on_crease_plane = previous_velocity.reject_from(*crease_direction);

    // Check if the angle between planes indicates a concave corner and if the velocity is going into it
    let dot_planes_on_crease_planes = current_normal_on_crease_plane.dot(*previous_surface.normal);

    if dot_planes_on_crease_planes
        > entering_velocity_on_crease_plane.dot(-current_normal_on_crease_plane) + 1e-3
        || dot_planes_on_crease_planes
            > entering_velocity_on_crease_plane.dot(-previous_normal_on_crease_plane) + 1e-3
    {
        return None;
    }

    // Flip the crease direction to match the direction of the current velocity
    if crease_direction.dot(current_velocity) < 0.0 {
        crease_direction = -crease_direction;
    }

    Some(crease_direction)
}

/// Align the vector with the `normal` plane along the `up` axis.
///
/// The returned vector maintains the same total magnitude as the input vector.
pub(crate) fn align_with_surface(vector: Vec3, normal: Vec3, up: Vec3) -> Vec3 {
    let right = vector.cross(up);
    let forward = normal.cross(right);
    forward.normalize_or_zero() * vector.length()
}
