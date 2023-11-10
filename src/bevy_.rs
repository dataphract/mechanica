use bevy::prelude::*;

use super::Ray;

impl From<bevy::math::Ray> for Ray {
    fn from(value: bevy::math::Ray) -> Self {
        Ray {
            origin: value.origin,
            dir: value.direction,
        }
    }
}

impl Ray {
    pub fn screenspace(
        camera: &Camera,
        camera_transform: &GlobalTransform,
        viewport_position: Vec2,
    ) -> Option<Ray> {
        camera
            .viewport_to_world(camera_transform, viewport_position)
            .map(Ray::from)
    }
}
