use std::mem;

use avian3d::prelude::*;
use bevy::prelude::*;

use crate::{
    debug_log, ground::Ground, projection::Surface, CollideAndSlideFilter,
};

#[derive(Reflect, Debug, Clone, Copy)]
pub struct SweepHitData {
    pub distance: f32,
    pub point: Vec3,
    pub normal: Vec3,
    pub entity: Entity,
}

/// Returns the safe hit distance and the hit data from the spatial query.
#[must_use]
pub(crate) fn sweep(
    shape: &Collider,
    origin: Vec3,
    rotation: Quat,
    direction: Dir3,
    max_distance: f32,
    skin_width: f32,
    spatial_query: &SpatialQuery,
    filter: &SpatialQueryFilter,
    ignore_origin_penetration: bool,
) -> Option<SweepHitData> {
    let hit = spatial_query.cast_shape(
        shape,
        origin,
        rotation,
        direction,
        &ShapeCastConfig {
            max_distance: max_distance + skin_width, // extend the trace slightly
            target_distance: skin_width, // I'm not sure what this does, but I think this is correct ;)
            ignore_origin_penetration,
            ..Default::default()
        },
        filter,
    )?;

    // How far is safe to translate by
    // let distance = hit.distance - skin_width;
    let distance = (hit.distance - skin_width).max(0.0);

    Some(SweepHitData {
        distance,
        point: hit.point1,
        normal: hit.normal1,
        entity: hit.entity,
    })
}

#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component, Default)]
#[require(CollideAndSlideFilter)]
pub struct CollideAndSlideConfig {
    pub max_iterations: u8,
    pub skin_width: f32,
}

impl Default for CollideAndSlideConfig {
    fn default() -> Self {
        Self {
            max_iterations: 4,
            skin_width: 0.1,
        }
    }
}
