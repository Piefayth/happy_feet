use std::{f32::consts::PI, fmt::Debug};

use bevy::prelude::*;

#[derive(Reflect, Debug, Clone, Copy, PartialEq, Eq)]
pub enum NonWalkableMode {
    /// Prevent climbing, use normal collision response (PhysX default)
    PreventClimbing,
    /// Prevent climbing and force sliding along the slope base
    PreventClimbingAndForceSliding,
}

impl Default for NonWalkableMode {
    fn default() -> Self {
        Self::PreventClimbing
    }
}

#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component, Default)]
#[require(Grounding)]
pub struct GroundingConfig {
    /// Max walkable angle
    pub max_angle: f32,
    /// Max distance from the ground
    pub max_distance: f32,
    pub snap_to_surface: bool,
}

impl Default for GroundingConfig {
    fn default() -> Self {
        Self {
            max_angle: PI / 4.0,
            max_distance: 0.2,
            snap_to_surface: true,
        }
    }
}
/// The ground state of a character.
#[derive(Component, Reflect, Default, Debug, PartialEq, Clone, Copy)]
#[reflect(Component, Default)]
pub struct Grounding {
    pub(crate) inner_ground: Option<Ground>,
    /// If the character should be forced to detach from the ground, e.g., after jumping.
    should_detach: bool,
}

impl Grounding {
    pub fn new(surface: Option<Ground>) -> Self {
        Self {
            inner_ground: surface,
            ..Default::default()
        }
    }

    pub fn is_grounded(&self) -> bool {
        !self.should_detach && self.inner_ground.is_some()
    }

    pub fn ground(&self) -> Option<Ground> {
        if self.should_detach {
            return None;
        }
        self.inner_ground
    }

    /// Detach from the ground without clearing the [`inner_ground`](Self::inner_ground).
    pub fn detach(&mut self) -> Option<Ground> {
        if !self.is_grounded() {
            return None;
        }

        self.should_detach = true;
        self.inner_ground
    }

    pub fn normal(&self) -> Option<Dir3> {
        self.ground().map(|ground| ground.normal)
    }

    pub fn entity(&self) -> Option<Entity> {
        self.ground().map(|ground| ground.entity)
    }

    pub fn inner_ground(&self) -> Option<Ground> {
        self.inner_ground
    }
}

impl From<Option<Ground>> for Grounding {
    fn from(value: Option<Ground>) -> Self {
        Self {
            inner_ground: value,
            ..Default::default()
        }
    }
}

impl From<Ground> for Grounding {
    fn from(value: Ground) -> Self {
        Self::from(Some(value))
    }
}

/// Represents a surface that a character can stand on.
#[derive(Reflect, Debug, PartialEq, Clone, Copy)]
pub struct Ground {
    /// The surface normal vector, pointing outward from the surface.
    pub normal: Dir3,
    pub entity: Entity,
}

impl Ground {
    pub fn new<D>(entity: Entity, normal: D) -> Self
    where
        D: TryInto<Dir3>,
        <D as TryInto<Dir3>>::Error: Debug,
    {
        Self {
            entity,
            normal: normal.try_into().unwrap(),
        }
    }

    /// Construct a new [`Ground`] if the `normal` is walkable with the given `walkable_angle` and `up` direction.
    pub fn new_if_walkable(
        entity: Entity,
        normal: impl TryInto<Dir3>,
        up: Dir3,
        walkable_angle: f32,
    ) -> Option<Self> {
        let normal = normal.try_into().ok()?;

        if !is_walkable(*normal, walkable_angle, *up) {
            return None;
        }

        Some(Self { entity, normal })
    }
}


pub(crate) fn is_walkable(normal: Vec3, walkable_angle: f32, up: Vec3) -> bool {
    normal.angle_between(up) <= walkable_angle
}
