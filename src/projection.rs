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
}

/// Align the vector with the `normal` plane along the `up` axis.
///
/// The returned vector maintains the same total magnitude as the input vector.
pub(crate) fn align_with_surface(vector: Vec3, normal: Vec3, up: Vec3) -> Vec3 {
    let right = vector.cross(up);
    let forward = normal.cross(right);
    forward.normalize_or_zero() * vector.length()
}
