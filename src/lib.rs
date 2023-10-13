use std::{cmp::Ordering, ptr::NonNull};

use glam::{Mat3, Vec3};

pub mod list;
pub mod mesh_edit;
pub mod nmesh;
pub mod polyhedron;

pub struct Ray {
    origin: Vec3,
    dir: Vec3,
}

impl Ray {
    /// Constructs a ray given its origin and direction.
    ///
    /// Returns `None` if `dir` cannot be normalized.
    pub fn new(origin: Vec3, dir: Vec3) -> Option<Ray> {
        Some(Ray {
            origin,
            dir: dir.try_normalize()?,
        })
    }

    /// Constructs a ray, assuming that the direction vector is normalized.
    pub(crate) fn new_unchecked(origin: Vec3, dir_normalized: Vec3) -> Ray {
        Ray {
            origin,
            dir: dir_normalized,
        }
    }

    /// Returns the point at distance `t` along the ray.
    ///
    /// If `t` is negative, returns the ray origin.
    #[inline]
    pub fn at(&self, t: f32) -> Vec3 {
        self.origin + t.max(0.0) * self.dir
    }

    /// Returns the distance along the ray of the projection of `point`.
    ///
    /// The result may be negative.
    #[inline]
    pub fn project_point(&self, point: Vec3) -> f32 {
        self.dir.dot(point - self.origin)
    }
}

pub struct Line {
    // Ray based at the projected origin.
    ray: Ray,
}

impl Line {
    pub fn from_point_dir(point: Vec3, dir: Vec3) -> Option<Line> {
        let dir = dir.try_normalize()?;
        let projected_origin = point + dir.dot(-point) * dir;

        Some(Line {
            ray: Ray::new_unchecked(projected_origin, dir),
        })
    }

    pub fn from_points(a: Vec3, b: Vec3) -> Option<Line> {
        Line::from_point_dir(a, b - a)
    }

    #[inline]
    pub fn project_point(&self, point: Vec3) -> f32 {
        self.ray.project_point(point)
    }
}

pub struct Plane {
    // Unit vector normal to the plane.
    normal: Vec3,
    // Distance from the origin to its projection on the plane.
    dist: f32,
}

impl Plane {
    /// Constructs a plane given a plane normal and the magnitude of the projected origin.
    pub fn new(normal: Vec3, dist: f32) -> Option<Plane> {
        if !dist.is_finite() {
            return None;
        }

        Some(Plane {
            normal: normal.try_normalize()?,
            dist,
        })
    }

    /// Signed distance between the plane and the given point.
    #[inline]
    pub fn distance_to_point(&self, point: Vec3) -> f32 {
        point.dot(self.normal) - self.dist
    }

    // TODO: tolerance
    pub fn side(&self, point: Vec3) -> PlaneSide {
        match point.dot(self.normal).partial_cmp(&self.dist).unwrap() {
            Ordering::Less => PlaneSide::Back,
            Ordering::Equal => PlaneSide::Planar,
            Ordering::Greater => PlaneSide::Front,
        }
    }

    pub fn projected_origin(&self) -> Vec3 {
        self.dist * self.normal
    }

    /// Constructs an orthonormal basis whose XY plane is the plane and whose Z axis is the plane
    /// normal.
    pub fn orthonormal_basis(&self) -> Mat3 {
        let normal = self.normal;

        // Construct an orthogonal vector from a zero component and the two largest components of
        // `normal`.
        let u = if normal.x.abs() >= normal.y.abs() {
            Vec3::new(normal.z, 0.0, -normal.x)
        } else {
            Vec3::new(0.0, normal.z, -normal.y)
        };

        let v = normal.cross(u);

        Mat3 {
            x_axis: u,
            y_axis: v,
            z_axis: normal,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PlaneSide {
    Back,
    Planar,
    Front,
}

#[cfg(test)]
mod tests {
    use approx::assert_ulps_eq;

    use super::*;

    #[test]
    fn ray() {
        assert!(Ray::new(Vec3::ZERO, Vec3::ZERO).is_none());

        assert_ulps_eq!(
            Ray::new(Vec3::ZERO, Vec3::ONE).unwrap().at(5.0),
            Vec3::splat(5.0 / 3.0_f32.sqrt()),
            max_ulps = 1,
        );
    }

    #[test]
    fn plane() {
        assert!(Plane::new(Vec3::ZERO, 1.0).is_none());
        assert!(Plane::new(Vec3::ONE, f32::INFINITY).is_none());
        assert!(Plane::new(Vec3::ONE, f32::NAN).is_none());

        let pos_x = Plane::new(Vec3::X, 0.0).unwrap();
        assert_eq!(pos_x.side(Vec3::X), PlaneSide::Front);
        assert_eq!(pos_x.side(-Vec3::X), PlaneSide::Back);
        assert_eq!(pos_x.side(Vec3::ZERO), PlaneSide::Planar);
    }

    #[test]
    fn line() {}
}
