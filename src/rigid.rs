//! Rigid-body dynamics simulation.
//!
//! This simulation is based on ["Detailed Rigid Body Simulation with Extended Position Based
//! Dynamics"][rigid] by Matthias Müller et al.
//!
//! The core simulation loop is provided by:
//!
//! - [`update_bvh`]
//! - [`gather_collision_candidates`]
//! - For each substep:
//!   - [`integrate_velocity`]
//!   - [`resolve_positional_constraints`]
//!   - [`solve_velocity`]
//!
//! [rigid]: https://matthias-research.github.io/pages/publications/PBDBodies.pdf

use bevy::prelude::*;
use glam::{Mat3A, Quat, Vec3, Vec3A};

use crate::{
    bvh::{AabbAabbQuery, Bvh, BvhKey, BvhQuery},
    collider::ColliderShape,
    Isometry,
};

pub fn record_transform<I, T>(recorders: I)
where
    I: Iterator<Item = T>,
    T: TransformRecorder,
{
    for mut recorder in recorders.into_iter() {
        recorder.set_prev_position(recorder.position());
        recorder.set_prev_orientation(recorder.orientation());
    }
}

pub trait BvhUpdate {
    type Key;

    fn key(&self) -> Self::Key;
    fn transform(&self) -> Isometry;
    fn velocity(&self) -> PhysicsVelocity;
    fn collider(&self) -> &ColliderShape;
    fn bvh_key(&self) -> BvhKey;
    fn set_bvh_key(&mut self, key: BvhKey);
}

/// Recomputes the bounding volumes of all simulation elements and updates the BVH accordingly.
// TODO: tune this to only recalculate if necessary, probably by dilating by more than the safety factor.
pub fn update_bvh<I, B>(step: f32, bvh: &mut Bvh<B::Key>, updates: I)
where
    I: IntoIterator<Item = B>,
    B: BvhUpdate,
{
    /// Safety multiplier to account for acceleration during physics step.
    const SAFETY: f32 = 2.0;

    let factor = SAFETY * step;

    for mut update in updates.into_iter() {
        let dilate = Vec3::splat(factor * update.velocity().velocity.length());

        let aabb = update
            .collider()
            .compute_aabb()
            .transform(update.transform())
            .dilate(dilate);

        bvh.remove(update.bvh_key());
        update.set_bvh_key(bvh.insert(aabb, update.key()));
    }
}

pub trait BroadPhaseCollider {
    type Key: Copy + Ord;

    fn key(&self) -> Self::Key;
    fn bvh_key(&self) -> BvhKey;
}

/// Represents a broad-phase collision between two simulation elements.
#[derive(Debug)]
pub struct CollisionCandidate<K> {
    pub a: K,
    pub b: K,
}

struct BroadPhaseColliderQuery<'bvh, C>
where
    C: BroadPhaseCollider,
{
    key: C::Key,
    query: BvhQuery<'bvh, C::Key, AabbAabbQuery>,
}

pub struct CollisionCandidatesIter<'bvh, I, C>
where
    I: IntoIterator<Item = C>,
    C: BroadPhaseCollider,
{
    bvh: &'bvh Bvh<C::Key>,
    iter: I,
    current: Option<BroadPhaseColliderQuery<'bvh, C>>,
}

impl<'bvh, I, C> Iterator for CollisionCandidatesIter<'bvh, I, C>
where
    I: Iterator<Item = C>,
    C: BroadPhaseCollider,
{
    type Item = CollisionCandidate<C::Key>;

    fn next(&mut self) -> Option<Self::Item> {
        // Resume the current query if there is one.
        if let Some(cur) = self.current.as_mut() {
            if let Some(&other) = cur.query.by_ref().find(|&&other| cur.key < other) {
                return Some(CollisionCandidate {
                    a: cur.key,
                    b: other,
                });
            }
        }

        // Either there's no current query or the query is exhausted. Find a query that returns an
        // intersection.
        for c in self.iter.by_ref() {
            let &aabb = self
                .bvh
                .get(c.bvh_key())
                .expect("no AABB associated with BVH key");
            let mut query = self.bvh.aabb_query(aabb);

            if let Some(&other) = query.by_ref().find(|&&other| c.key() < other) {
                self.current = Some(BroadPhaseColliderQuery {
                    key: c.key(),
                    query,
                });

                return Some(CollisionCandidate {
                    a: c.key(),
                    b: other,
                });
            }
        }

        // No broad-phase colliders left.
        None
    }
}

pub fn gather_collision_candidates<I, C>(
    bvh: &Bvh<C::Key>,
    colliders: I,
) -> CollisionCandidatesIter<'_, I::IntoIter, C>
where
    I: IntoIterator<Item = C>,
    C: BroadPhaseCollider,
{
    CollisionCandidatesIter {
        bvh,
        iter: colliders.into_iter(),
        current: None,
    }
}

pub fn integrate_velocity<I, V>(substep: f32, integrators: I)
where
    I: IntoIterator<Item = V>,
    V: VelocityIntegrator,
{
    for mut item in integrators.into_iter() {
        let delta = substep * item.velocity();
        item.set_position(item.position() + delta);

        let orientation = item.orientation();
        let ang_vel = item.ang_vel();
        let ang_delta =
            orientation + Quat::from_vec4((0.5 * ang_vel.get()).extend(0.0)) * orientation;
        item.set_orientation((orientation + ang_delta).normalize());
    }
}

fn positional_gen_inv_mass(inv_mass: f32, point: Vec3A, dir: Vec3A, inv_inertia: Mat3A) -> f32 {
    let axis = point.cross(dir);
    inv_mass + axis.dot(inv_inertia * axis)
}

/// Resolves positional (i.e. distance) constraints between simulation elements.
pub fn resolve_positional_constraints<'a, I, M>(substep: f32, constraints: I, mut element_map: M)
where
    I: IntoIterator<Item = &'a PositionalConstraint<M::Key>> + 'a,
    M: ConstraintElementMap,
    M::Key: 'a,
{
    let inv_substep_2 = substep.powi(2).recip();

    for pc in constraints.into_iter() {
        let [mut elem_a, mut elem_b] = element_map.get([pc.elem_a, pc.elem_b]);

        let anchor_a = elem_a.position() + elem_a.orientation() * pc.anchor_a;
        let anchor_b = elem_b.position() + elem_b.orientation() * pc.anchor_b;

        let delta: Vec3A = anchor_b - anchor_a;
        let dist = delta.length();
        let dir = delta / dist;

        let inv_mass_a = elem_a.mass().inv_mass;
        let inv_inertia_a = elem_a.inertia().inv_inertia;
        let inv_mass_b = elem_b.mass().inv_mass;
        let inv_inertia_b = elem_b.inertia().inv_inertia;

        let gen_inv_mass_a = positional_gen_inv_mass(inv_mass_a, pc.anchor_a, dir, inv_inertia_a);
        let gen_inv_mass_b = positional_gen_inv_mass(inv_mass_b, pc.anchor_b, dir, inv_inertia_b);

        let error = (dist - pc.lower_bound).min(0.0) + (dist - pc.upper_bound).max(0.0);
        let lagrange_denom = (gen_inv_mass_a + gen_inv_mass_b) + inv_substep_2 * pc.compliance;
        let lagrange = -error / lagrange_denom;
        let impulse = lagrange * dir;

        elem_a.set_position(elem_a.position() + impulse * inv_mass_a);
        elem_b.set_position(elem_b.position() + impulse * inv_mass_b);

        let axis_a = 0.5 * inv_inertia_a * pc.anchor_a.cross(impulse);
        let axis_b = 0.5 * inv_inertia_b * pc.anchor_b.cross(impulse);
        let rotate_a = Quat::from_vec4(axis_a.extend(0.0)) * elem_a.orientation();
        let rotate_b = Quat::from_vec4(axis_b.extend(0.0)) * elem_b.orientation();

        // Müller et al.:
        //   "To update the quaternions based on the angular velocity and to derive the angular
        //   velocity from the change of the quaternions we use linearized formulas. They are
        //   fast and robust and well suited for the small time steps used in substepping."
        elem_a.set_orientation((elem_a.orientation() + rotate_a).normalize());
        elem_b.set_orientation((elem_b.orientation() - rotate_b).normalize());
    }
}

/// Solves for the final velocities of simulation elements at the end of a substep.
pub fn solve_velocity<I, V>(substep: f32, solvers: I)
where
    I: IntoIterator<Item = V>,
    V: VelocitySolver,
{
    let inv_substep = substep.recip();

    for mut solver in solvers.into_iter() {
        let mvmt = solver.position() - solver.prev_position();
        let vel = PhysicsVelocity::new(inv_substep * mvmt);
        solver.set_velocity(vel);

        let rot = solver.prev_orientation() * solver.orientation().inverse();
        solver.set_ang_vel(PhysicsAngVel::new(rot.to_scaled_axis().into()));
    }
}

/// A positional constraint between a pair of simulation elements.
#[derive(Component)]
pub struct PositionalConstraint<K> {
    pub elem_a: K,
    pub anchor_a: Vec3A,
    pub elem_b: K,
    pub anchor_b: Vec3A,

    // -inf for no lower bound.
    pub lower_bound: f32,
    // +inf for no upper bound.
    pub upper_bound: f32,

    pub compliance: f32,
}

#[derive(Component)]
pub struct PhysicsCollider {
    pub shape: ColliderShape,
}

impl PhysicsCollider {
    pub fn shape(&self) -> &ColliderShape {
        &self.shape
    }
}

/// Represents the mass of a simulation element in kilograms.
#[derive(Copy, Clone, Component)]
pub struct RigidBodyMass {
    inv_mass: f32,
}

impl RigidBodyMass {
    pub fn new(mass: f32) -> RigidBodyMass {
        RigidBodyMass {
            inv_mass: mass.recip(),
        }
    }
}

/// The moment of inertia of a rigid body.
#[derive(Copy, Clone, Component)]
pub struct RigidBodyInertia {
    inertia: Mat3A,
    inv_inertia: Mat3A,
}

impl RigidBodyInertia {
    #[inline]
    fn diagonal(xx: f32, yy: f32, zz: f32) -> RigidBodyInertia {
        RigidBodyInertia {
            inertia: Mat3A::from_cols(
                Vec3A::new(xx, 0.0, 0.0),
                Vec3A::new(0.0, yy, 0.0),
                Vec3A::new(0.0, 0.0, zz),
            ),

            inv_inertia: Mat3A::from_cols(
                Vec3A::new(xx.recip(), 0.0, 0.0),
                Vec3A::new(0.0, yy.recip(), 0.0),
                Vec3A::new(0.0, 0.0, zz.recip()),
            ),
        }
    }

    /// Returns the moment of inertia of a solid cube with the specified `side_length` and `mass`.
    ///
    /// If either of `side_length` or `mass` is non-finite or non-positive, returns `None`.
    pub fn solid_cube(side_length: f32, mass: f32) -> Option<RigidBodyInertia> {
        const FRAC_1_6: f32 = 1.0 / 6.0;

        if !side_length.is_finite() || side_length <= 0.0 || mass <= 0.0 {
            return None;
        }

        let term = FRAC_1_6 * mass * side_length.powi(2);

        Some(RigidBodyInertia::diagonal(term, term, term))
    }

    /// Returns the moment of inertia of a solid rectangular cuboid with the specified `dimensions`
    /// and `mass`.
    pub fn solid_cuboid(dimensions: [f32; 3], mass: f32) -> Option<RigidBodyInertia> {
        const FRAC_1_12: f32 = 1.0 / 12.0;

        if dimensions.iter().any(|&d| !d.is_finite() || d <= 0.0) || mass <= 0.0 {
            return None;
        }

        let [x, y, z] = dimensions;

        let x2 = x.powi(2);
        let y2 = y.powi(2);
        let z2 = z.powi(2);

        let factor = FRAC_1_12 * mass;

        let xx = factor * (y2 + z2);
        let yy = factor * (x2 + z2);
        let zz = factor * (x2 + y2);

        Some(RigidBodyInertia::diagonal(xx, yy, zz))
    }

    /// Returns the moment of inertia of a solid sphere with the specified `radius` and `mass`.
    pub fn solid_sphere(radius: f32, mass: f32) -> Option<RigidBodyInertia> {
        if !radius.is_finite() || radius <= 0.0 || mass <= 0.0 {
            return None;
        }

        let r2 = radius.powi(2);
        let term = 0.4 * mass * r2;

        Some(RigidBodyInertia::diagonal(term, term, term))
    }
}

#[derive(Copy, Clone, Debug, Default, Component)]
pub struct PhysicsVelocity {
    velocity: Vec3,
}

impl PhysicsVelocity {
    #[inline]
    pub fn new(velocity: Vec3A) -> PhysicsVelocity {
        PhysicsVelocity {
            velocity: velocity.into(),
        }
    }

    #[inline]
    pub fn get(&self) -> Vec3A {
        self.velocity.into()
    }

    #[inline]
    pub fn set(&mut self, velocity: Vec3A) {
        self.velocity = velocity.into();
    }
}

#[derive(Copy, Clone, Debug, Default, Component)]
pub struct PhysicsAngVel {
    ang_vel: Vec3,
    axis: Vec3,
    magnitude: f32,
}

impl PhysicsAngVel {
    pub fn new(ang_vel: Vec3A) -> PhysicsAngVel {
        let magnitude = ang_vel.length();
        let axis = (ang_vel / magnitude).into();

        PhysicsAngVel {
            ang_vel: ang_vel.into(),
            axis,
            magnitude,
        }
    }

    pub fn get(&self) -> Vec3A {
        self.ang_vel.into()
    }

    pub fn set(&mut self, ang_vel: Vec3A) {
        self.ang_vel = ang_vel.into();
        self.magnitude = ang_vel.length();
        self.axis = (ang_vel / self.magnitude).into();
    }

    pub fn axis(&self) -> Vec3A {
        self.axis.into()
    }

    pub fn magnitude(&self) -> f32 {
        self.magnitude
    }
}

pub trait TransformRecorder {
    fn position(&self) -> Vec3A;

    fn set_prev_position(&mut self, position: Vec3A);

    fn orientation(&self) -> Quat;

    fn set_prev_orientation(&mut self, orientation: Quat);
}

/// The state of a physics body necessary to integrate its linear and angular velocity.
pub trait VelocityIntegrator {
    /// Returns the position of the body.
    fn position(&self) -> Vec3A;

    /// Sets the position of the body.
    fn set_position(&mut self, position: Vec3A);

    /// Returns the orientation of the body.
    fn orientation(&self) -> Quat;

    /// Sets the orientation of the body.
    fn set_orientation(&mut self, orientation: Quat);

    /// Returns the linear velocity of the body.
    fn velocity(&self) -> Vec3A;

    /// Returns the angular velocity of the body.
    fn ang_vel(&self) -> PhysicsAngVel;
}

/// Provides random-access lookup for simulation elements affected by a constraint.
pub trait ConstraintElementMap {
    /// The key type used to look up each simulation element.
    type Key: Copy;

    /// The element type.
    type Element<'elem>: ConstraintElement<'elem>
    where
        Self: 'elem;

    type Iter<'elem>: Iterator<Item = Self::Element<'elem>>
    where
        Self: 'elem;

    /// Retrieves the elements associated with `keys`.
    fn get(&mut self, keys: [Self::Key; 2]) -> [Self::Element<'_>; 2];

    /// Returns an iterator over all constrained simulation elements.
    fn iter(&mut self) -> Self::Iter<'_>;
}

pub trait ConstraintElement<'elem> {
    fn mass(&self) -> RigidBodyMass;

    fn inertia(&self) -> RigidBodyInertia;

    fn position(&self) -> Vec3A;

    fn set_position(&mut self, position: Vec3A);

    fn orientation(&self) -> Quat;

    fn set_orientation(&mut self, orientation: Quat);
}

/// Solves for the linear and angular velocity of a simulation element after all constraints have
/// been solved.
pub trait VelocitySolver {
    fn position(&self) -> Vec3A;

    fn prev_position(&self) -> Vec3A;

    fn orientation(&self) -> Quat;

    fn prev_orientation(&self) -> Quat;

    fn set_velocity(&mut self, velocity: PhysicsVelocity);

    fn set_ang_vel(&mut self, ang_vel: PhysicsAngVel);
}
