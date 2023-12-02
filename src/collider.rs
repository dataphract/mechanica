use crate::{hull::Hull, Aabb, Capsule, Sphere};

#[derive(Clone)]
pub enum ColliderShape {
    Sphere(Sphere),
    Capsule(Capsule),
    Hull(Hull),
}

impl ColliderShape {
    pub fn compute_aabb(&self) -> Aabb {
        match self {
            ColliderShape::Sphere(s) => s.compute_aabb(),
            ColliderShape::Capsule(c) => c.compute_aabb(),
            ColliderShape::Hull(h) => h.aabb(),
        }
    }
}
