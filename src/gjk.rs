use bevy::prelude::{Color, Gizmos};
/// The Gilbert-Johnson-Keerthi least distance algorithm.
// Implementation based on "A Fast and Robust GJK Implementation for Collision Detection of Convex
// Objects" by Gino van den Bergen (https://doi.org/10.1080/10867651.1999.10487502).
use glam::{Quat, Vec3, Vec3A};

use crate::{hull::Hull, Isometry, Segment, Sphere};

/// A trait for types which can compute supporting points in a given direction.
pub trait Support {
    /// Computes the supporting point of this convex set in the direction given by `dir`.
    fn local_support(&self, dir: Vec3A) -> Vec3A;

    /// Computes the supporting point of this convex set, transformed by `transform`, in the direction
    /// given by `dir`.
    ///
    /// The default implementation rotates `dir` by the inverse of `transform.rotation`, then calls
    /// `local_support`, and finally applies `transform` to the local support point.
    #[inline]
    fn support(&self, transform: Isometry, dir: Vec3A) -> Vec3A {
        let Isometry {
            rotation,
            translation,
        } = transform;
        Vec3A::from(translation) + rotation * self.local_support(rotation.inverse() * dir)
    }
}

impl Support for Vec3A {
    #[inline]
    fn local_support(&self, _dir: Vec3A) -> Vec3A {
        *self
    }

    #[inline]
    fn support(&self, transform: Isometry, _dir: Vec3A) -> Vec3A {
        *self + Vec3A::from(transform.translation)
    }
}

impl Support for Vec3 {
    #[inline]
    fn local_support(&self, dir: Vec3A) -> Vec3A {
        Vec3A::from(*self).local_support(dir)
    }

    #[inline]
    fn support(&self, transform: Isometry, dir: Vec3A) -> Vec3A {
        Vec3A::from(*self).support(transform, dir)
    }
}

impl Support for Sphere {
    #[inline]
    fn local_support(&self, dir: Vec3A) -> Vec3A {
        Vec3A::from(self.center) + self.radius * dir.normalize()
    }

    #[inline]
    fn support(&self, transform: Isometry, dir: Vec3A) -> Vec3A {
        // Invariant to rotation.
        let center = Vec3A::from(self.center);
        let trans = Vec3A::from(transform.translation);
        trans + center + self.radius * dir.normalize()
    }
}

impl Support for Segment {
    fn local_support(&self, dir: Vec3A) -> Vec3A {
        let a = Vec3A::from(self.a);
        let b = Vec3A::from(self.b);
        let ab = b - a;

        if ab.dot(dir) < 0.0 {
            a
        } else {
            b
        }
    }

    fn support(&self, transform: Isometry, dir: Vec3A) -> Vec3A {
        let a = transform * Vec3A::from(self.a);
        let b = transform * Vec3A::from(self.b);
        let ab = b - a;

        if ab.dot(dir) < 0.0 {
            a
        } else {
            b
        }
    }
}

impl Support for Hull {
    fn local_support(&self, dir: Vec3A) -> Vec3A {
        self.compute_supporting_point(dir)
    }
}

#[derive(Copy, Clone, Debug)]
struct LowerBound(f32);

impl LowerBound {
    #[inline(always)]
    fn new() -> LowerBound {
        LowerBound(0.0)
    }

    #[inline(always)]
    fn get(&self) -> f32 {
        self.0
    }

    #[inline(always)]
    fn update(&mut self, delta: f32) {
        self.0 = delta.max(self.0)
    }
}

pub fn closest<T, U>(
    obj_a: &T,
    iso_a: Isometry,
    obj_b: &U,
    iso_b: Isometry,
    mut gizmos: Option<&mut Gizmos>,
) -> (Vec3A, Vec3A)
where
    T: Support,
    U: Support,
{
    const REL_ERROR: f32 = 1.0e-6;

    // Assuming units are in meters, this terminates the loop if objects are within one micrometer
    // of each other.
    const ABS_ERROR: f32 = 1.0e-10;

    let mut points_a = [Vec3A::ZERO; 4];
    let mut points_b = [Vec3A::ZERO; 4];

    println!("==============================================================");

    // Initialize v with an arbitrary support point on the Minkowski difference.
    let init_a = obj_a.support(iso_a, Vec3A::X);
    println!("init_a = {init_a}");
    let init_b = obj_b.support(iso_b, -Vec3A::X);
    println!("init_b = {init_b}");
    let mut v = init_a - init_b;
    let mut dist = v.length();
    let mut dist_lower_bound = LowerBound::new();

    let mut simplex = Simplex::new();

    let mut hue = 0.0;
    while simplex.bits < 0b1111 && dist > ABS_ERROR {
        hue += 16.0;

        // Compute the support point on the Minkowski difference.
        let point_a = obj_a.support(iso_a, -v);
        println!("point_a = {point_a:?}");
        let point_b = obj_b.support(iso_b, v);
        println!("point_b = {point_b:?}");
        let new_point = point_a - point_b;
        println!("v = {v:?}");
        println!("new_point = {new_point:?}");
        if let Some(g) = gizmos.as_mut() {
            g.sphere(
                point_a.into(),
                Quat::IDENTITY,
                0.05,
                Color::Hsla {
                    hue,
                    saturation: 1.0,
                    lightness: 0.5,
                    alpha: 1.0,
                },
            );
            g.sphere(
                point_b.into(),
                Quat::IDENTITY,
                0.05,
                Color::Hsla {
                    hue,
                    saturation: 1.0,
                    lightness: 0.5,
                    alpha: 1.0,
                },
            );
            // g.line(point_a.into(), point_b.into(), Color::RED);
        }

        // Update the lower bound.
        dist_lower_bound.update(v.dot(new_point) / v.length());

        // If the previous distance is less than the lower bound, terminate.
        if dist - dist_lower_bound.get() <= dist * REL_ERROR {
            println!(
                "terminating on lower bound: {} <= {}",
                dist - dist_lower_bound.get(),
                dist * REL_ERROR
            );
            break;
        }

        if simplex.insert(new_point).is_err() {
            break;
        }

        points_a[simplex.new_idx as usize] = point_a;
        points_b[simplex.new_idx as usize] = point_b;

        let Some(closest) = simplex.find_closest() else {
            break;
        };
        println!("simplex dim: {}", simplex.bits.count_ones());

        v = closest;
        dist = v.length();
    }

    let mut bary_sum = 0.0;
    let mut point_a = Vec3A::ZERO;
    let mut point_b = Vec3A::ZERO;
    for i in 0..4 {
        let bit = 1 << i;
        if simplex.bits & bit == 0 {
            continue;
        }

        let d = simplex.det[simplex.bits as usize][i];
        println!("b{i} = {d}");
        bary_sum += d;
        point_a += d * points_a[i];
        point_b += d * points_b[i];
    }
    assert!(point_a.is_finite());
    assert!(point_b.is_finite());

    println!("point_a = {point_a}");
    println!("point_b = {point_b}");

    let bary_denom = bary_sum.recip();
    println!("bary_denom = {bary_denom}");
    assert!(bary_denom.is_normal());
    point_a *= bary_denom;
    point_b *= bary_denom;

    assert!(point_a.is_finite());
    assert!(point_b.is_finite());

    (point_a, point_b)
}

// Memoization table for determinant calculations.
struct Simplex {
    prev_bits: u8,
    bits: u8,
    new_bit: u8,
    new_idx: u8,

    points: [Vec3A; 4],

    // self.dot[i][j] stores the value of points[i].dot(points[j]).
    dot: [[f32; 4]; 4],

    // TODO: this indexing scheme results in a lot of unused space:
    //
    // - det[0] is not used at all (4 wasted f32s)
    // - det[i] is not used where popcnt(i) == 1 (16 wasted f32s)
    // - det[i][j] is not used where `i` does not contain `1 << j`
    //   - 2 unused per value of `i` where popcnt(i) == 2 (12 wasted f32s)
    //   - 1 unused per value of `i` where popcnt(i) == 3 (4 wasted f32s)
    //
    // This wastes 36 * 4 = 144 bytes of space, more than 2 cache lines' worth. See if an index
    // lookup table into a more compact representation could save space. At the very least the first
    // 5 indices can be omitted for free, for a more modest 80 bytes' savings.
    det: [[f32; 4]; 16],
}

#[derive(Copy, Clone, Debug)]
struct DuplicatePoint;

impl Simplex {
    fn new() -> Simplex {
        Simplex {
            prev_bits: 0,
            bits: 0b0000,
            new_bit: 0b0000,
            new_idx: 0,
            points: [Vec3A::ZERO; 4],
            dot: Default::default(),
            det: Default::default(),
        }
    }

    fn insert(&mut self, point: Vec3A) -> Result<(), DuplicatePoint> {
        let new_idx = self.bits.trailing_ones() as usize;
        assert!(new_idx < 4);

        for (i, p) in self.points.iter().enumerate() {
            if self.bits & (1 << i) == 0 {
                continue;
            }

            if point == *p {
                return Err(DuplicatePoint);
            }
        }

        self.points[new_idx] = point;
        self.prev_bits = self.bits;
        self.new_idx = new_idx as u8;
        self.new_bit = 1 << new_idx;
        self.bits |= self.new_bit;

        self.update_det();

        Ok(())
    }

    #[inline]
    fn det3(&mut self, p1: usize, p2: usize, p3: usize) {
        let in_bits = (1 << p2) | (1 << p3);
        let out_bits = (1 << p1) | in_bits;

        let d1 = self.det[in_bits][p2] * (self.dot[p2][p2] - self.dot[p2][p1]);
        let d2 = self.det[in_bits][p3] * (self.dot[p3][p2] - self.dot[p3][p1]);

        self.det[out_bits][p1] = d1 + d2;
    }

    #[inline]
    fn det4(&mut self, p1: usize, p2: usize, p3: usize, p4: usize) {
        let in_bits = (1 << p2) | (1 << p3) | (1 << p4);

        let d1 = self.det[in_bits][p2] * (self.dot[p2][p2] - self.dot[p2][p1]);
        let d2 = self.det[in_bits][p3] * (self.dot[p3][p2] - self.dot[p3][p1]);
        let d3 = self.det[in_bits][p4] * (self.dot[p4][p2] - self.dot[p4][p1]);

        self.det[0b1111][p1] = d1 + d2 + d3;
    }

    fn update_det(&mut self) {
        let points = &self.points;
        let old_bits = self.prev_bits;
        let new_bits = self.bits;
        let new_bit = self.new_bit;
        let new_idx = self.new_idx as usize;

        let bit_was_set = |&i: &usize| old_bits & (1 << i) != 0;

        // Recompute dot products which change with the new vertex.
        for i in (0..4).filter(bit_was_set) {
            if old_bits & (1 << i) == 0 {
                continue;
            }

            let d = points[i].dot(points[new_idx]);
            println!("dot[{new_idx}][{i}] = {d}");
            self.dot[new_idx][i] = d;
            self.dot[i][new_idx] = d;
        }

        println!(
            "dot[{new_idx}][{new_idx}] = {}",
            points[new_idx].dot(points[new_idx])
        );
        self.dot[new_idx][new_idx] = points[new_idx].dot(points[new_idx]);

        // Recompute the determinant for `new_bits`. This requires recomputing the determinant for
        // all subsets of `new_bits` that contain `new_bit`.

        self.det[new_bit as usize][new_idx] = 1.0;

        for q in (0..4).filter(bit_was_set) {
            let b2 = (1 << q) | new_bit as usize;

            // Recompute determinants for updated 1-simplices.
            println!(
                "self.det[{b2:04b}][{q}] = {}",
                self.dot[new_idx][new_idx] - self.dot[new_idx][q]
            );
            self.det[b2][q] = self.dot[new_idx][new_idx] - self.dot[new_idx][q];
            println!(
                "self.det[{b2:04b}][{new_idx}] = {}",
                self.dot[q][q] - self.dot[q][new_idx]
            );
            self.det[b2][new_idx] = self.dot[q][q] - self.dot[q][new_idx];

            for p in (0..q).filter(bit_was_set) {
                // Recompute determinants for updated 2-simplices.
                self.det3(p, q, new_idx);
                self.det3(q, p, new_idx);
                self.det3(new_idx, p, q);
            }
        }

        if new_bits == 0b1111 {
            self.det4(0, 1, 2, 3);
            self.det4(1, 0, 2, 3);
            self.det4(2, 0, 1, 3);
            self.det4(3, 0, 1, 2);
        }
    }

    fn find_closest(&mut self) -> Option<Vec3A> {
        let old_bits = self.prev_bits;

        for sub_bits in (1..=old_bits).rev() {
            if old_bits & sub_bits != sub_bits {
                // Not a subset.
                continue;
            }

            println!("test sub_bits = {sub_bits:b}");
            if self.is_valid_solution(sub_bits | self.new_bit) {
                self.bits = sub_bits | self.new_bit;

                let mut v = Vec3A::ZERO;
                let mut bary_sum = 0.0;

                for i in 0..4 {
                    if self.bits & (1 << i) == 0 {
                        continue;
                    }

                    let det = self.det[self.bits as usize][i];
                    bary_sum += det;
                    v += self.points[i] * det;
                }

                v *= bary_sum.recip();

                return Some(v);
            }
        }

        if self.is_valid_solution(self.new_bit) {
            self.bits = self.new_bit;
            return Some(self.points[self.new_idx as usize]);
        }

        None
    }

    // Determines whether the subset `sub_bits` of `all_bits` represents a valid solution.
    //
    // A subset is a valid solution iff its associated determinants are positive and the remaining
    // determinants are non-positive.
    fn is_valid_solution(&self, sub_bits: u8) -> bool {
        let all_bits = self.bits;
        debug_assert_eq!(all_bits & sub_bits, sub_bits);

        println!("sub_bits = 0b{sub_bits:04b}");
        for i in 0..4 {
            let bit = 1 << i;

            if all_bits & bit == 0 {
                continue;
            }

            let bit_included = sub_bits & bit != 0;
            let det_positive = self.det[(sub_bits | bit) as usize][i] > 0.0;

            println!(
                "----\n1 << {i}: det = {}",
                self.det[(sub_bits | bit) as usize][i]
            );
            println!("1 << {i}: bit_included = {bit_included}");
            println!("1 << {i}: det_positive = {det_positive}");

            if bit_included != det_positive {
                return false;
            }
        }

        true
    }
}

// van den Bergen notes
//
// - A: first convex object
// - B: second convex object
// - δ_k: signed distance from origin to supporting plane, (v_k · w_k) / mag(v_k)
// - Δ(X): determinant of X, used in calculation of λ via Cramer's rule
// - ɛ: absolute error tolerance
// - I_X: set of indices which gives X ⊆ Y s.t. X = {Y[i] : i ∈ I_X}.
// - λ_k: barycentric coordinate vector at kth iteration
// - μ_k: corrected lower bound at kth iteration, max(0, δ_0, ... , δ_k)
// - v(A - B): point closest to origin in A - B
// - v_k: point in W_k closest to origin
// - W_k: simplex at kth iteration
// - X: subset of W whose convex hull contains v

#[cfg(test)]
mod tests {
    use approx::assert_ulps_eq;
    use glam::Quat;

    use super::*;

    #[test]
    fn sphere_support() {
        let origin = Vec3::new(2.0, -1.0, 10.0);

        let at_origin = Sphere {
            center: Vec3::ZERO,
            radius: 1.0,
        };

        for w in -10..=10 {
            let x = (w as f32).cos();
            let y = (w as f32).sin();
            let z = w as f32;

            let v = Vec3A::new(x, y, z).normalize();

            assert_ulps_eq!(at_origin.local_support(v), v);
            assert_ulps_eq!(
                at_origin.support(
                    Isometry {
                        rotation: Quat::IDENTITY,
                        translation: origin
                    },
                    v
                ),
                Vec3A::from(origin) + v
            );
        }
    }

    #[test]
    fn simplex_1d() {
        let mut simplex = Simplex::new();

        let v = Vec3A::new(1.0, 2.0, 3.0);

        simplex.insert(v).unwrap();

        let closest = simplex.find_closest().unwrap();

        assert_eq!(v, closest);
    }

    #[test]
    fn simplex_2d() {
        // Closest point to origin between two points
        let mut simplex = Simplex::new();

        let v1 = Vec3A::new(1.0, 1.0, 1.0);
        let v2 = Vec3A::new(-1.0, -1.0, 1.0);

        simplex.insert(v1).unwrap();
        simplex.insert(v2).unwrap();

        let closest = simplex.find_closest().unwrap();

        assert_eq!(Vec3A::new(0.0, 0.0, 1.0), closest);

        // Closest point to origin is one of the points
        let mut simplex = Simplex::new();

        let v1 = Vec3A::new(3.0, 3.0, 1.0);
        let v2 = Vec3A::new(1.0, 1.0, 1.0);

        simplex.insert(v1).unwrap();
        simplex.insert(v2).unwrap();

        let closest = simplex.find_closest().unwrap();

        assert_eq!(v2, closest);
    }

    #[test]
    fn simplex_3d() {
        let mut simplex = Simplex::new();

        let v1 = Vec3A::X;
        let v2 = Vec3A::Y;
        let v3 = Vec3A::Z;

        simplex.insert(v1).unwrap();
        simplex.insert(v2).unwrap();
        simplex.insert(v3).unwrap();

        assert_ulps_eq!(simplex.find_closest().unwrap(), Vec3A::splat(0.3333333));

        let mut simplex = Simplex::new();

        let v1 = Vec3A::new(1.0, 1.0, 0.0);
        let v2 = Vec3A::X;
        let v3 = Vec3A::Y;

        simplex.insert(v1).unwrap();
        simplex.insert(v2).unwrap();
        simplex.insert(v3).unwrap();

        assert_eq!(simplex.find_closest().unwrap(), Vec3A::new(0.5, 0.5, 0.0));
    }
}
