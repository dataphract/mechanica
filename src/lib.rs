//! A high-performance 3D physics simulation library.

#![cfg_attr(feature = "portable_simd", feature(portable_simd))]

use std::{cmp::Ordering, ops::Mul};

use bevy::prelude::Transform;
use glam::{Mat3, Mat4, Quat, Vec3, Vec3A};

use crate::glam_ext::Mat3Ext;

mod aabb;
pub mod bevy_;
pub mod bvh;
pub mod closest;
pub mod collider;
pub mod constraint;
pub mod contact;
pub mod gjk;
mod glam_ext;
pub mod hull;
pub mod nmesh;
pub mod rigid;
pub mod testbench;
pub mod zorder;

#[doc(inline)]
pub use aabb::Aabb;

const FRAC_1_3: f32 = 1.0 / 3.0;
const FRAC_1_10: f32 = 1.0 / 10.0;
const FRAC_1_12: f32 = 1.0 / 12.0;
const FRAC_2_3: f32 = 2.0 / 3.0;
const FRAC_2_5: f32 = 2.0 / 5.0;
const FRAC_3_8: f32 = 3.0 / 8.0;
const TWO_PI: f32 = 2.0 * std::f32::consts::PI;

/// An isometry, or rigid transformation.
#[derive(Copy, Clone, Debug, Default)]
pub struct Isometry {
    pub rotation: Quat,
    pub translation: Vec3,
}

impl Isometry {
    pub const IDENTITY: Self = Isometry {
        rotation: Quat::IDENTITY,
        translation: Vec3::ZERO,
    };

    /// Constructs an isometry from an arbitrary `Transform` by dropping the scale component.
    pub fn from_transform(transform: Transform) -> Isometry {
        Isometry {
            rotation: transform.rotation,
            translation: transform.translation,
        }
    }

    pub fn from_translation(v: Vec3) -> Isometry {
        Isometry {
            translation: v,
            ..Default::default()
        }
    }

    pub fn from_rotation(rotation: Quat) -> Isometry {
        Isometry {
            rotation,
            ..Default::default()
        }
    }

    #[inline]
    pub fn inverse(self) -> Isometry {
        Isometry {
            rotation: self.rotation.inverse(),
            translation: -self.translation,
        }
    }

    /// Computes the transformation matrix of the isometry.
    pub fn compute_matrix(&self) -> Mat4 {
        Mat4::from_rotation_translation(self.rotation, self.translation)
    }
}

impl From<Isometry> for Transform {
    #[inline]
    fn from(iso: Isometry) -> Transform {
        Transform {
            translation: iso.translation,
            rotation: iso.rotation,
            scale: Vec3::ONE,
        }
    }
}

impl Mul<Isometry> for Isometry {
    type Output = Isometry;

    fn mul(self, rhs: Isometry) -> Self::Output {
        let rotation = self.rotation * rhs.rotation;
        let translation = rhs.rotation * self.translation + rhs.translation;

        Isometry {
            rotation,
            translation,
        }
    }
}

impl Mul<Vec3> for Isometry {
    type Output = Vec3;

    fn mul(self, rhs: Vec3) -> Self::Output {
        (self * Vec3A::from(rhs)).into()
    }
}

impl Mul<Vec3A> for Isometry {
    type Output = Vec3A;

    fn mul(self, rhs: Vec3A) -> Self::Output {
        self.rotation * rhs + Vec3A::from(self.translation)
    }
}

impl Mul<Plane> for Isometry {
    type Output = Plane;

    fn mul(self, rhs: Plane) -> Self::Output {
        // TODO: this can probably be optimized
        Plane::from_point_normal(
            rhs.project_origin() + self.translation,
            self.rotation * rhs.normal,
        )
        .unwrap()
    }
}

impl Mul<Segment> for Isometry {
    type Output = Segment;

    #[inline]
    fn mul(self, rhs: Segment) -> Self::Output {
        Segment {
            a: self * rhs.a,
            b: self * rhs.b,
        }
    }
}

/// A ray.
pub struct Ray {
    pub origin: Vec3,
    pub dir: Vec3,
}

impl Ray {
    /// Constructs a ray given its origin and direction.
    ///
    /// Returns `None` if `origin` is not finite or `dir` cannot be normalized.
    pub fn new(origin: Vec3, dir: Vec3) -> Option<Ray> {
        if !origin.is_finite() {
            return None;
        }

        Some(Ray {
            origin,
            dir: dir.try_normalize()?,
        })
    }

    /// Constructs a ray, assuming that the direction vector is normalized.
    #[inline]
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

    /// Returns `true` _iff_ `self` intersects `sphere`.
    pub fn intersects_sphere(&self, sphere: &Sphere) -> bool {
        let t = self.project_point(sphere.center).max(0.0);

        let proj = self.origin + t * self.dir;

        proj.distance_squared(sphere.center) < sphere.radius.powi(2)
    }
}

/// An infinite line.
pub struct Line {
    // Ray based at the projected origin.
    pub ray: Ray,
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

    pub fn at(&self, t: f32) -> Vec3 {
        self.ray.origin + t * self.ray.dir
    }

    /// Returns the distance along the ray of the projection of `point`.
    ///
    /// The result may be negative.
    #[inline]
    pub fn project_point(&self, point: Vec3) -> f32 {
        self.ray.project_point(point)
    }

    pub fn intersect_plane(&self, plane: Plane) -> Option<Vec3A> {
        println!("ray dir: {}", self.ray.dir);
        println!("plane normal: {}", plane.normal);
        let denom = self.ray.dir.dot(plane.normal);
        println!("denom: {denom}");

        if denom.abs() < f32::EPSILON {
            // TODO: handle planar case
            return None;
        }

        let t = (plane.dist - plane.normal.dot(self.ray.origin)) / denom;
        Some(self.at(t).into())
    }
}

#[derive(Clone, Debug)]
pub struct Segment {
    pub a: Vec3A,
    pub b: Vec3A,
}

/// The closest point on a given segment to some other primitive.
pub struct SegmentClosest {
    /// The closest point on the segment to the other primitive.
    pub point: Vec3A,

    /// The parameter value obtained by projecting the closest point on the other primitive onto
    /// the segment.
    ///
    /// The closest point on the segment is obtained by clamping this parameter value to the range
    /// [0, 1].
    pub t: f32,
}

/// For a pair of segments, the closest point on each segment to the other and the squared distance
/// between them.
pub struct SegmentClosestPair {
    pub first: SegmentClosest,
    pub second: SegmentClosest,
    pub distance_squared: f32,
}

impl Segment {
    pub fn new(a: Vec3A, b: Vec3A) -> Segment {
        Segment { a, b }
    }

    fn barycentric_origin(&self) -> [f32; 2] {
        let ab = self.b - self.a;
        let denom = ab.length_squared();

        if denom < f32::EPSILON.powi(2) {
            // Points too close together, just pick the closest.
            if self.a.length_squared() < self.b.length_squared() {
                return [1.0, 0.0];
            } else {
                return [0.0, 1.0];
            }
        }

        let v = -self.a.dot(ab) / denom;
        [1.0 - v, v]
    }

    /// Finds the point on the segment closest to `c`.
    pub fn closest_point_to_point(&self, c: Vec3A) -> SegmentClosest {
        let ab = self.b - self.a;
        let t = (c - self.a).dot(ab) / ab.length_squared();
        let t_clamp = t.clamp(0.0, 1.0);

        SegmentClosest {
            point: self.a + t_clamp * ab,
            t,
        }
    }

    pub fn closest_point_to_segment(&self, other: &Segment) -> SegmentClosestPair {
        // Given segments S1 and S2 where
        //   S1(t1) = A1 + t1d1, d1 = B1 - A1, 0 <= t1 <= 1
        //   S2(t2) = A2 + t2d2, d2 = B2 - A2, 0 <= t2 <= 2
        // ...find the closest points on the two segments.

        let a1 = self.a;
        let b1 = self.b;
        let a2 = other.a;
        let b2 = other.b;
        let d1 = b1 - a1;
        let d2 = b2 - a2;

        let r = self.a - other.a;

        let a = d1.length_squared();
        let e = d2.length_squared();

        let degen_1 = a <= f32::EPSILON;
        let degen_2 = e <= f32::EPSILON;

        // If both segments degenerate to points, return those points.
        if degen_1 && degen_2 {
            return SegmentClosestPair {
                first: SegmentClosest { point: a1, t: 0.0 },
                second: SegmentClosest { point: a2, t: 0.0 },
                distance_squared: a1.distance(a2),
            };
        }

        // t1 is the parameter value corresponding to the projection of `d2 * t2` onto d1.
        //
        //      (a2 + t2*d2 - a1) . d1   (a2 - a1) . d1 + (t2 * d2) . d1
        // t1 = ---------------------- = -------------------------------
        //              d1 . d1                      d1 . d1
        //
        // t2 is the parameter value corresponding to the projection of `d1 * t1` onto d2.
        //
        //      (a1 + t1*d1 - a2) . d2   (a1 - a2) . d2 + (t1 * d1) . d2
        // t2 = ---------------------- = -------------------------------
        //              d2 . d2                      d2 . d2
        //
        // Let
        //     r = a1 - a2
        //     a = d1 . d1
        //     b = d1 . d2
        //     c = d1 . r
        //     e = d2 . d2
        //     f = d2 . r
        // Then,
        //
        //      b * t2 - c
        // t1 = ----------
        //          a
        //
        //      b * t1 + f
        // t2 = ----------
        //          e

        let c = d1.dot(r);
        let f = d2.dot(r);

        // If one segment degenerates to a point, just find the closest point on the other segment.
        let (t1, t2);
        if degen_1 {
            t1 = 0.0;
            t2 = (f / e).clamp(0.0, 1.0);
        } else if degen_2 {
            t2 = 0.0;
            t1 = (-c / a).clamp(0.0, 1.0);
        } else {
            // TODO: this is black magic. explain it.

            let b = d1.dot(d2);
            let denom = a * e - b * b;

            // Calculate the closest point on L1 to L2. If the lines are parallel, set t1 = 0.
            let s = if denom != 0.0 {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };

            // Calculate the closest point on L2 to S1(t1).
            let mut t = (b * s + f) / e;

            // If t2 is outside [0, 1], recalculate t1 and clamp.
            let s = if t < 0.0 {
                t = 0.0;
                (-c / a).clamp(0.0, 1.0)
            } else if t > 1.0 {
                t = 1.0;
                ((b - c) / a).clamp(0.0, 1.0)
            } else {
                s
            };

            t1 = s;
            t2 = t;
        }

        let c1 = self.a + d1 * t1;
        let c2 = other.a + d2 * t2;

        SegmentClosestPair {
            first: SegmentClosest { point: c1, t: t1 },
            second: SegmentClosest { point: c2, t: t2 },
            distance_squared: c1.distance_squared(c2),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Plane {
    // Unit vector normal to the plane.
    normal: Vec3,
    // Distance from the origin to its projection on the plane.
    dist: f32,
}

impl Plane {
    /// Constructs a plane given a plane normal and the magnitude of the projected origin.
    pub fn new(normal: Vec3, dist: f32) -> Option<Plane> {
        if !dist.is_finite() || !normal.is_normalized() {
            return None;
        }

        Some(Plane {
            normal: normal.try_normalize()?,
            dist,
        })
    }

    /// Constructs a plane given a point on the plane and the plane normal.
    pub fn from_point_normal(point: Vec3, normal: Vec3) -> Option<Plane> {
        if !point.is_finite() || !normal.is_normalized() {
            return None;
        }

        Plane::new(normal, point.dot(normal))
    }

    /// Signed distance between the plane and the given point.
    #[inline]
    pub fn distance_to_point(&self, point: Vec3A) -> f32 {
        point.dot(self.normal.into()) - self.dist
    }

    // TODO: tolerance
    pub fn side(&self, point: Vec3) -> PlaneSide {
        match point.dot(self.normal).partial_cmp(&self.dist).unwrap() {
            Ordering::Less => PlaneSide::Back,
            Ordering::Equal => PlaneSide::Planar,
            Ordering::Greater => PlaneSide::Front,
        }
    }

    #[inline]
    pub fn project_point(&self, point: Vec3A) -> Vec3A {
        point - self.distance_to_point(point) * Vec3A::from(self.normal)
    }

    /// Clamps `point` to the half-space in front of the plane.
    ///
    /// If the signed distance from the plane to the point is negative, this returns the projection
    /// of `point` on the plane. Otherwise, this returns `point`.
    #[inline]
    pub fn clamp_point(&self, point: Vec3A) -> Vec3A {
        point - self.distance_to_point(point).min(0.0) * Vec3A::from(self.normal)
    }

    #[inline]
    pub fn project_origin(&self) -> Vec3 {
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

/// A sphere.
#[derive(Clone, Debug, PartialEq)]
pub struct Sphere {
    /// The center of the sphere.
    pub center: Vec3,
    /// The radius of the sphere.
    pub radius: f32,
}

impl Sphere {
    #[inline]
    pub fn compute_aabb(&self) -> Aabb {
        Aabb::new(self.center, Vec3::splat(self.radius))
    }

    /// Returns `true` if `point` is within the sphere.
    #[inline]
    pub fn contains_point(&self, point: Vec3) -> bool {
        // Distance from center to point is less than or equal to radius.
        (self.center - point).length() <= self.radius
    }

    /// Returns `true` if `self` intersects the sphere `other`.
    #[inline]
    pub fn intersects_sphere(&self, other: &Sphere) -> bool {
        // Distance between centers is less than or equal to sum of radii.
        let dist_squared = self.center.distance_squared(other.center);
        dist_squared <= (self.radius + other.radius).powi(2)
    }

    /// Returns `true` if `self` intersects the AABB `aabb`.
    #[inline]
    pub fn intersects_aabb(&self, aabb: &Aabb) -> bool {
        // Closest point to sphere center which is contained by AABB is also contained by sphere.
        self.contains_point(aabb.closest_point_to(self.center))
    }
}

/// A capsule.
#[derive(Clone, Debug)]
pub struct Capsule {
    pub segment: Segment,
    pub radius: f32,
}

impl Capsule {
    pub fn compute_aabb(&self) -> Aabb {
        let origin = 0.5 * (self.segment.b + self.segment.a);
        let half_extents = 0.5 * (self.segment.b - self.segment.a);

        Aabb::new(origin.into(), half_extents.into())
    }

    pub fn intersects_sphere(&self, sphere: &Sphere) -> bool {
        let center = sphere.center.into();
        let closest = self.segment.closest_point_to_point(center);
        (center - closest.point).length_squared() <= self.radius.powi(2)
    }

    pub fn intersects_capsule(&self, other: &Capsule) -> bool {
        let closest = self.segment.closest_point_to_segment(&other.segment);
        let radius = self.radius + other.radius;
        closest.distance_squared < radius * radius
    }
}

// Avoiding use of Vec3A/Mat3A keeps OBBs small enough to fit in one cache line on most platforms.
#[derive(Clone, Debug)]
pub struct Obb {
    /// The center of the OBB in world space.
    pub world_center: Vec3,
    /// Local orientation matrix.
    ///
    /// This must be an orthonormal basis.
    pub local_to_world: Mat3,
    /// The half-extents of the OBB along each local axis.
    pub half_extents: Vec3,
}

impl Obb {
    #[inline]
    fn center_a(&self) -> Vec3A {
        self.world_center.into()
    }
    #[inline]
    fn half_extents_a(&self) -> Vec3A {
        self.half_extents.into()
    }

    #[inline]
    fn mins_a(&self) -> Vec3A {
        self.center_a() - self.half_extents_a()
    }

    #[inline]
    pub fn mins(&self) -> Vec3A {
        self.mins_a()
    }

    #[inline]
    fn maxs_a(&self) -> Vec3A {
        self.center_a() + self.half_extents_a()
    }

    #[inline]
    pub fn maxs(&self) -> Vec3 {
        self.maxs_a().into()
    }

    pub fn transform(&self, transform: Isometry) -> Obb {
        Obb {
            world_center: transform.translation + self.world_center,
            local_to_world: Mat3::from_quat(transform.rotation) * self.local_to_world,
            half_extents: self.half_extents,
        }
    }

    pub fn closest_point_to_point(&self, other: Vec3) -> Vec3 {
        let delta: Vec3A = (other - self.world_center).into();

        let mut point: Vec3A = self.world_center.into();

        for i in 0..3 {
            let axis: Vec3A = self.local_to_world.col(i).into();
            let bound = self.half_extents[i];
            let dist = delta.dot(axis).clamp(-bound, bound);

            point += dist * axis;
        }

        point.into()
    }

    pub fn intersects_sphere(&self, other: &Sphere) -> bool {
        other
            .center
            .distance_squared(self.closest_point_to_point(other.center))
            < other.radius.powi(2)
    }

    pub fn intersects_obb(&self, other: &Obb) -> bool {
        // See [Gottschalk00] for in-depth explanation of optimizations.

        // Because all matrices involved are orthonormal bases, their inverses can be calculated as
        // their transposes.

        // Convert `other` to the local coordinate space of A.
        let world_to_local = self.local_to_world.transpose();
        let other_to_local = world_to_local * other.local_to_world;

        // Translation vector from self to other, in local space of self.
        let local_translate: Vec3A =
            (world_to_local * (other.world_center - self.world_center)).into();

        // 1. Take the component-wise absolute value of the orientation matrix. Since the OBB is
        //    a box, reflecting it over its axes produces an equivalent OBB.
        // 2. Add a small epsilon value to each element, which will result in a negligible effect on
        //    the rotation of the box but will
        //    a) prevent the cross product of parallel axes from being zero, and
        //    b) outweigh any accumulated error during the separating axis tests.
        let abs_other_to_local = Mat3 {
            x_axis: other_to_local.x_axis.abs() + f32::EPSILON,
            y_axis: other_to_local.y_axis.abs() + f32::EPSILON,
            z_axis: other_to_local.z_axis.abs() + f32::EPSILON,
        };

        // Perform 15 separating axis tests.

        // The first two sets of tests project both OBBs onto each axis of one box, then the other.
        // The projection is performed in the local space of the OBB whose axes are being tested,
        // which allows the axis to be represented as a scalar.

        // Axes of `self`.
        let ra: Vec3A = self.half_extents.into();
        let rb: Vec3A = (abs_other_to_local * other.half_extents).into();
        if local_translate.abs().cmpgt(ra + rb).any() {
            return false;
        }

        // Translation vector from self to other, in local space of other.
        let other_translate = other_to_local.transpose_mul_vec3a(local_translate);

        // Axes of `other`.
        let ra: Vec3A = abs_other_to_local
            .transpose_mul_vec3a(self.half_extents.into())
            .into();
        let rb: Vec3A = other.half_extents.into();
        if other_translate.abs().cmpgt(ra + rb).any() {
            return false;
        }

        // The remaining tests use as their separating axes the cross products of axes from each
        // OBB. However, the actual cross product computation can be avoided by making use of the
        // scalar triple product identity
        //
        //   u ⋅ (v ⨯ w) = w ⋅ (u ⨯ v) = v ⋅ (w ⨯ u)
        //
        // to rotate the cross products to use two axes from the same OBB, and then substitute the
        // third axis.
        //
        // Thus, given:
        //
        //   - OBBs A and B, with rotation matrices Ra and Rb and half-lengths a and b,
        //   - some permutation [i, j, k] of [0, 1, 2], and
        //   - some permutation [m, n, o] of [0, 1, 2], which may be identical to [i, j, k],
        //
        // to calculate the half-length of the projection of A onto A[i] ⨯ B[m]:
        //
        //   =   a[i] * |Ra[i] ⋅ (Ra[i] ⨯ Rb[m])|
        //     + a[j] * |Ra[j] ⋅ (Ra[i] ⨯ Rb[m])|
        //     + a[k] * |Ra[k] ⋅ (Ra[i] ⨯ Rb[m])|
        //
        //   =   a[i] * |Rb[m] ⋅ (Ra[i] ⨯ Ra[i])|    By the above scalar triple product identity.
        //     + a[k] * |Rb[m] ⋅ (Ra[j] ⨯ Ra[i])|
        //     + a[k] * |Rb[m] ⋅ (Ra[k] ⨯ Ra[i])|
        //
        //   =   a[j] * |Rb[m] ⋅ (Ra[j] ⨯ Ra[i])|    By elimination of the first term, since
        //     + a[k] * |Rb[m] ⋅ (Ra[k] ⨯ Ra[i])|    Ra[i] ⨯ Ra[i] is the zero vector.
        //
        //   =   a[j] * |Rb[m] ⋅ Ra[k]|              By substitution of cross products of first two
        //     + a[k] * |Rb[m] ⋅ Ra[j]|              axes with the third axis.
        //
        // The resulting value is the same regardless of the coordinate space in which the
        // calculation is performed. By ensuring that Ra is expressed in the local space of B (and
        // thus making Rb the identity matrix), the calculation simply becomes
        //
        //   =   a[j] * |Ra[k][m]|
        //     + a[k] * |Ra[j][m]|
        //
        // Similarly, to calculate the half-length of the projection of B onto A[i] ⨯ B[m], where Rb
        // is expressed in A's local space,
        //
        //   =   b[n] * |Rb[n][i]|
        //     + b[o] * |Rb[o][i]|

        // Given axis `i`, use axes `PAIRS[i]` in calculating projection.
        const PAIRS: [[usize; 2]; 3] = [[1, 2], [0, 2], [0, 1]];

        let test = |i: usize, m: usize| -> bool {
            let [j, k] = PAIRS[i];
            let [n, o] = PAIRS[m];

            let ra = self.half_extents[j] * abs_other_to_local.col(k)[m]
                + self.half_extents[k] * abs_other_to_local.col(j)[m];

            // Note use of abs_other_to_local in transposed (i.e., inverse) form.
            let rb = other.half_extents[n] * abs_other_to_local.col(i)[o]
                + other.half_extents[o] * abs_other_to_local.col(i)[n];

            (local_translate[j] * abs_other_to_local.col(k)[m]
                + local_translate[k] * abs_other_to_local.col(j)[m])
                .abs()
                > ra + rb
        };

        test(0, 0)
            && test(0, 1)
            && test(0, 2)
            && test(1, 0)
            && test(1, 1)
            && test(1, 2)
            && test(2, 0)
            && test(2, 1)
            && test(2, 2)
    }

    /// Computes the supporting point of the `Obb` in the direction given by `dir`.
    pub fn compute_support(&self, dir: Vec3) -> Vec3 {
        let local_dir: Vec3A = self.local_to_world.transpose_mul_vec3a(dir.into());
        let half: Vec3A = self.half_extents.into();
        let local_support: Vec3 = Vec3A::select(local_dir.cmpge(Vec3A::ZERO), half, -half).into();
        self.local_to_world * local_support
    }
}

/// Stores the duration of a single physics substep.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Substep {
    seconds: f32,
    inv_2: f32,
}

impl Substep {
    pub fn new(seconds: f32) -> Substep {
        assert!(seconds.is_finite() && seconds > 0.0);

        let inv_2 = seconds.recip().powi(2);

        Substep { seconds, inv_2 }
    }

    #[inline]
    pub fn seconds(&self) -> f32 {
        self.seconds
    }

    #[inline]
    pub fn inverse_squared(&self) -> f32 {
        self.inv_2
    }
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
    fn line_intersect_plane() {
        let line = Line::from_point_dir(Vec3::ZERO, Vec3::X).unwrap();
        let plane = Plane::from_point_normal(Vec3::ZERO, Vec3::X).unwrap();
        assert_eq!(line.intersect_plane(plane), Some(Vec3A::ZERO));

        let line = Line::from_point_dir(Vec3::ZERO, Vec3::ONE.normalize()).unwrap();
        let plane = Plane::from_point_normal(Vec3::ONE, Vec3::ONE.normalize()).unwrap();
        assert_ulps_eq!(line.intersect_plane(plane).unwrap(), Vec3A::ONE);

        let line = Line::from_point_dir(Vec3::ZERO, Vec3::ONE.normalize()).unwrap();
        let plane =
            Plane::from_point_normal(Vec3::ONE, Vec3::new(-1.0, -1.0, 0.0).normalize()).unwrap();
        assert_ulps_eq!(line.intersect_plane(plane).unwrap(), Vec3A::ONE);

        let line = Line::from_point_dir(Vec3::ZERO, Vec3::ONE.normalize()).unwrap();
        let plane = Plane::from_point_normal(Vec3::ONE, Vec3::Y).unwrap();
        assert_ulps_eq!(line.intersect_plane(plane).unwrap(), Vec3A::ONE);

        let line =
            Line::from_points([-0.88, 1.5, 0.71].into(), [-1.56, 1.5, -0.03].into()).unwrap();
        println!("line dir = {}", line.ray.dir);
        let plane = Plane::from_point_normal(Vec3::ZERO, -Vec3::Z).unwrap();
        println!("plane = {plane:?}");
        let pt = line.intersect_plane(plane).unwrap();
        println!("intersection = {pt}");

        assert_ulps_eq!(plane.distance_to_point(pt), 0.0);
    }
}
