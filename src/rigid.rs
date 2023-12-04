//! Rigid-body dynamics simulation.
//!
//! This simulation is based on ["Detailed Rigid Body Simulation with Extended Position Based
//! Dynamics"][rigid] by Matthias Müller et al.
//!
//! The core simulation loop is provided by:
//!
//! - [`update_bvh`]
//! - [`generate_collision_candidates`]
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
    constraint::{Constraint, ConstraintSolver, ContactConstraint, PositionalConstraint},
    contact::{contact_collider_collider, Contact},
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
    /// The key type used to uniquely identify simulation elements.
    ///
    /// The `Ord` bound is used to deduplicate collision candidates: a pair of elements `(a, b)` is
    /// only considered a candidate if `a.key() < b.key()`.
    type Key: Copy + Ord;

    fn key(&self) -> Self::Key;
    fn bvh_key(&self) -> BvhKey;
}

/// Represents a broad-phase collision between two simulation elements.
#[derive(Copy, Clone, Debug)]
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

/// Generates collision candidates by testing each element of `colliders` against `bvh`.
pub fn generate_collision_candidates<I, C>(
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
        let ang_delta = Quat::from_vec4((0.5 * ang_vel.get()).extend(0.0)) * orientation;
        let new_orientation = (orientation + ang_delta).normalize();
        item.set_orientation(new_orientation);
    }
}

pub trait ColliderMap {
    type Key: Copy;
    type Element<'elem>: CandidateCollider
    where
        Self: 'elem;

    fn get(&self, key: Self::Key) -> Self::Element<'_>;
}

pub trait CandidateCollider {
    fn collider(&self) -> &ColliderShape;
    fn transform(&self) -> Isometry;
}

pub fn generate_collision_constraints<I, C>(
    candidates: I,
    map: C,
) -> impl Iterator<Item = ContactConstraint<C::Key>>
where
    I: IntoIterator<Item = CollisionCandidate<C::Key>>,
    C: ColliderMap,
{
    candidates
        .into_iter()
        .filter_map(move |candidate| {
            let a = map.get(candidate.a);
            let b = map.get(candidate.b);

            let contact =
                contact_collider_collider(a.collider(), a.transform(), b.collider(), b.transform());

            let p = match contact {
                Contact::Disjoint(_) => return None,
                Contact::Penetrating(p) => p,
            };

            println!("contact: {p:?}");

            let a_transform = a.transform();
            let b_transform = b.transform();

            // TODO: return the contact manifold in local space of the colliders so this
            // inverse-transform-multiply isn't necessary.
            Some(p.points.into_iter().map(move |point| ContactConstraint {
                keys: [candidate.a, candidate.b],
                contact_points: [a_transform.inverse() * point, b_transform.inverse() * point],
                normal: p.axis,
                penetration_depth: p.depth,
            }))
        })
        .flatten()
}

fn positional_gen_inv_mass(inv_mass: f32, point: Vec3A, dir: Vec3A, inv_inertia: Mat3A) -> f32 {
    let axis = point.cross(dir);
    inv_mass + axis.dot(inv_inertia * axis)
}

/// Resolves positional (i.e. distance) constraints between simulation elements.
pub fn solve_constraints<'a, I, C, S, K, const N: usize>(
    substep: f32,
    constraints: I,
    mut solver: S,
) where
    I: IntoIterator<Item = &'a C> + 'a,
    C: Constraint<K, N> + 'a,
    S: ConstraintSolver<K>,
    K: Copy,
{
    let inv_substep_2 = substep.powi(2).recip();

    for constraint in constraints.into_iter() {
        solver.solve(inv_substep_2, constraint);
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
        assert!(vel.velocity.length() < 10.0);
        solver.set_velocity(vel);

        let rot = solver.prev_orientation() * solver.orientation().inverse();
        solver.set_ang_vel(PhysicsAngVel::new(rot.to_scaled_axis().into()));
    }
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
pub struct Mass {
    inv_mass: f32,
}

impl Mass {
    pub fn new(mass: f32) -> Mass {
        Mass {
            inv_mass: mass.recip(),
        }
    }

    pub fn inverse(&self) -> f32 {
        self.inv_mass
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

    pub fn get(&self) -> Mat3A {
        self.inertia
    }

    pub fn inverse(&self) -> Mat3A {
        self.inv_inertia
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
        let axis = if magnitude == 0.0 {
            Vec3::X
        } else {
            (ang_vel / magnitude).into()
        };

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
