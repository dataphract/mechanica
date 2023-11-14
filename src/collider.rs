use crate::{hull::Hull, Capsule, Sphere};

#[derive(Clone)]
pub enum ColliderShape {
    Sphere(Sphere),
    Capsule(Capsule),
    Hull(Hull),
}
