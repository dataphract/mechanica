//! Bevy integration for Mechanica.
//!
//! # Plugins
//!
//! Add the [`PhysicsPlugin`] to an [`App`] to enable physics simulation.
//!
//! ```ignore
//! use bevy::prelude::*;
//! use mechanica::bevy_::PhysicsPlugin;
//!
//! let mut app = App::new();
//! app.add_plugins(PhysicsPlugin);
//! ```
//!
//! # Resources
//!
//! The [`PhysicsPlugin`] provides the following public resources:
//!
//! - [`PhysicsIgnoredPairs`] allows pairs of physics objects to be excluded from collision detection.
//! - [`PhysicsSubstep`] provides the current physics substep duration. This value is not set
//!   directly but is instead computed based on the fixed timestep and the value of
//!   [`PhysicsSubstepCount`] (see below).
//! - [`PhysicsSubstepCount`] controls the number of simulation substeps per fixed timestep.
//!   Increasing the value results in shorter substeps, and thus more realistic simulation, at the
//!   cost of CPU time.

use bevy::{
    ecs::{query::WorldQuery, schedule::ScheduleLabel},
    prelude::*,
};
use glam::Vec3A;
use hashbrown::{hash_map::Entry, HashMap, HashSet};
use smallvec::{smallvec, SmallVec};

use crate::{
    bvh::{Bvh, BvhKey},
    collider::ColliderShape,
    constraint::{
        Constraint, ConstraintElement, ConstraintSolver, ContactConstraint, PositionalConstraint,
        RevoluteConstraint,
    },
    hull::Hull,
    rigid::{
        BroadPhaseCollider, BvhUpdate, CandidateCollider, ColliderMap, CollisionCandidate, Inertia,
        Mass, PhysicsAngVel, PhysicsCollider, PhysicsVelocity, TransformRecorder,
        VelocityIntegrator, VelocitySolver,
    },
    Aabb, Capsule, Isometry, Ray, Segment, Sphere,
};

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

/// Core physics plugin.
pub struct PhysicsPlugin {
    /// Configures the initial number of substeps per fixed timestep.
    pub num_substeps: u32,
    /// Configures the initial capacity of the physics BVH.
    pub bvh_capacity: usize,
}

impl Default for PhysicsPlugin {
    fn default() -> Self {
        Self {
            num_substeps: 20,
            bvh_capacity: 1024,
        }
    }
}

impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        let mut physics_substep_schedule = Schedule::new();

        physics_substep_schedule.add_systems(
            (
                record_transform,
                apply_global_acceleration,
                integrate_velocity,
                generate_collision_constraints,
                solve_explicit_constraints::<PositionalConstraint<Entity>, 2>,
                solve_explicit_constraints::<RevoluteConstraint<Entity>, 2>,
                solve_contact_constraints,
                solve_velocity,
            )
                .chain(),
        );

        app.add_schedule(PhysicsSubstepSchedule, physics_substep_schedule)
            .add_systems(
                FixedUpdate,
                (
                    init_bvh_key,
                    update_bvh,
                    gather_collision_candidates,
                    run_physics_substep_loop,
                ),
            );

        app.insert_resource(CollisionCandidates::with_capacity(1024));
        app.insert_resource(ContactConstraints::with_capacity(1024));
        app.insert_resource(PhysicsBvh(Bvh::with_capacity(self.bvh_capacity)));
        app.insert_resource(PhysicsGlobalAccelLayers {
            layers: [Vec3::ZERO; 32],
        });
        app.insert_resource(PhysicsIgnoredPairs::with_capacity(256));
        app.insert_resource(PhysicsSubstepCount::new(self.num_substeps));
    }
}

/// The schedule which runs physics simulation substeps.
#[derive(Clone, Debug, PartialEq, Eq, Hash, ScheduleLabel)]
struct PhysicsSubstepSchedule;

/// The schedule which runs explicit constraint solvers.
#[derive(Clone, Debug, PartialEq, Eq, Hash, ScheduleLabel)]
struct SolveExplicitConstraintsSchedule;

// Based on bevy_time::fixed::run_fixed_update_schedule
pub fn run_physics_substep_loop(world: &mut World) {
    let timestep = world.resource::<FixedTime>().period.as_secs_f32();
    let substep_count = world.resource::<PhysicsSubstepCount>().get();
    let substep = timestep / substep_count as f32;

    world.insert_resource(PhysicsSubstep::new(substep));

    let _ = world.try_schedule_scope(PhysicsSubstepSchedule, |world, schedule| {
        for _ in 0..substep_count {
            schedule.run(world);
        }
    });
}

/// Stores the number of physics substeps per fixed-tick timestep.
#[derive(Resource)]
pub struct PhysicsSubstepCount {
    count: u32,
}

impl PhysicsSubstepCount {
    pub fn new(count: u32) -> PhysicsSubstepCount {
        PhysicsSubstepCount { count }
    }

    pub fn get(&self) -> u32 {
        self.count
    }

    pub fn set(&mut self, count: u32) {
        self.count = count;
    }
}

/// Stores the duration of a single physics substep.
#[derive(Resource)]
pub struct PhysicsSubstep {
    seconds: f32,

    // The square of the inverse of the substep duration.
    inv_2: f32,
}

impl PhysicsSubstep {
    /// Constructs a new `PhysicsSubstep` with the specified duration in seconds.
    pub fn new(seconds: f32) -> PhysicsSubstep {
        assert!(seconds.is_finite() && seconds > 0.0);

        let inv_2 = seconds.recip().powi(2);

        PhysicsSubstep { seconds, inv_2 }
    }

    /// Returns the duration of the current physics substep in seconds.
    ///
    /// # Example
    ///
    /// ```
    /// use mechanica::bevy_::PhysicsSubstep;
    ///
    /// let substep = PhysicsSubstep::new(0.0625);
    /// assert_eq!(substep.seconds(), 0.0625);
    /// ```
    #[inline]
    pub fn seconds(&self) -> f32 {
        self.seconds
    }

    /// Returns the squared inverse of the current physics substep (i.e. `seconds.recip().powi(2)`).
    ///
    /// # Example
    ///
    /// ```
    /// use mechanica::bevy_::PhysicsSubstep;
    ///
    /// let substep = PhysicsSubstep::new(0.0625);
    /// assert_eq!(substep.seconds().recip().powi(2), substep.inverse_squared());
    /// ```
    #[inline]
    pub fn inverse_squared(&self) -> f32 {
        self.inv_2
    }
}

#[derive(WorldQuery)]
#[world_query(mutable)]
struct TransformRecorderQuery {
    transform: &'static Transform,
    prev_transform: &'static mut PrevTransform,
}

impl TransformRecorder for TransformRecorderQueryItem<'_> {
    fn position(&self) -> Vec3A {
        self.transform.translation.into()
    }

    #[inline]
    fn set_prev_position(&mut self, position: Vec3A) {
        self.prev_transform.0.translation = position.into();
    }

    #[inline]
    fn orientation(&self) -> Quat {
        self.transform.rotation
    }

    #[inline]
    fn set_prev_orientation(&mut self, orientation: Quat) {
        self.prev_transform.0.rotation = orientation;
    }
}

fn record_transform(mut query: Query<TransformRecorderQuery>) {
    crate::rigid::record_transform(query.iter_mut());
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
struct IgnoredPair {
    a: Entity,
    b: Entity,
}

impl IgnoredPair {
    fn new(a: Entity, b: Entity) -> Option<IgnoredPair> {
        if a == b {
            return None;
        }

        let (a, b) = if a < b { (a, b) } else { (b, a) };

        Some(IgnoredPair { a, b })
    }
}

#[derive(Resource)]
pub struct PhysicsIgnoredPairs {
    // Per-entity sorted list of paired entities.
    //
    // Storing at most 3 entities inline makes the SmallVec 32 bytes wide.
    keys: HashMap<Entity, SmallVec<[Entity; 3]>>,

    pairs: HashSet<IgnoredPair>,
}

impl PhysicsIgnoredPairs {
    fn with_capacity(cap: usize) -> PhysicsIgnoredPairs {
        PhysicsIgnoredPairs {
            keys: HashMap::with_capacity(cap),
            pairs: HashSet::with_capacity(cap),
        }
    }

    fn add_key_to_key(&mut self, src: Entity, dst: Entity) {
        match self.keys.entry(src) {
            Entry::Occupied(mut o) => {
                let v = o.get_mut();
                if let Err(idx) = v.binary_search(&dst) {
                    v.insert(idx, dst);
                }
            }

            Entry::Vacant(v) => {
                v.insert(smallvec![dst]);
            }
        };
    }

    fn remove_key_to_key(&mut self, src: Entity, dst: Entity) {
        let v = &mut self.keys.get_mut(&src).unwrap();
        let idx = v.binary_search(&dst).unwrap();
        v.remove(idx);
    }

    /// Inserts a pair of entities into the set of ignored pairs.
    ///
    /// The order of the arguments does not matter.
    pub fn insert(&mut self, a: Entity, b: Entity) {
        let Some(pair) = IgnoredPair::new(a, b) else {
            return;
        };

        self.add_key_to_key(a, b);
        self.add_key_to_key(b, a);

        self.pairs.insert(pair);
    }

    /// Removes a pair of entities from the set of ignored pairs.
    ///
    /// The order of the arguments does not matter.
    pub fn remove(&mut self, a: Entity, b: Entity) {
        let Some(pair) = IgnoredPair::new(a, b) else {
            return;
        };

        self.remove_key_to_key(a, b);
        self.remove_key_to_key(b, a);

        self.pairs.remove(&pair);
    }

    /// Removes all ignored pairs containing `entity`.
    pub fn remove_all(&mut self, entity: Entity) {
        let v = self.keys.remove(&entity).expect("entity not present");

        for other in v {
            let pair = IgnoredPair::new(entity, other).unwrap();

            self.remove_key_to_key(other, entity);

            self.pairs.remove(&pair);
        }
    }
}

/// Defines global acceleration layers.
#[doc(alias = "GravityLayers")]
#[derive(Resource)]
pub struct PhysicsGlobalAccelLayers {
    pub layers: [Vec3; 32],
}

/// Selects which global acceleration layers apply to an entity.
#[doc(alias = "GravityMask")]
#[derive(Component)]
pub struct PhysicsGlobalAccelMask {
    mask: u32,
}

impl PhysicsGlobalAccelMask {
    pub fn new(mask: u32) -> PhysicsGlobalAccelMask {
        PhysicsGlobalAccelMask { mask }
    }
}

fn apply_global_acceleration(
    substep: Res<PhysicsSubstep>,
    layers: Res<PhysicsGlobalAccelLayers>,
    mut query: Query<(&PhysicsGlobalAccelMask, &mut PhysicsVelocity)>,
) {
    let delta_t = substep.seconds();
    let mut accel = Vec3A::ZERO;

    for (mask, mut vel) in query.iter_mut() {
        let start = mask.mask.trailing_zeros();
        let end = u32::BITS - mask.mask.leading_zeros();

        for idx in start..end {
            accel += Vec3A::from(layers.layers[idx as usize]);
        }

        let new_vel = vel.get() + delta_t * accel;
        vel.set(new_vel);
    }
}

#[derive(Bundle)]
pub struct PhysicsBundle {
    pub transform: TransformBundle,
    pub prev_transform: PrevTransform,
    pub velocity: PhysicsVelocity,
    pub collider: PhysicsCollider,
    pub aabb: PhysicsAabb,
}

#[derive(Bundle)]
pub struct RigidBodyBundle {
    pub physics: PhysicsBundle,
    pub mass: Mass,
    pub inertia: Inertia,
    pub ang_vel: PhysicsAngVel,
}

impl RigidBodyBundle {
    pub fn solid_sphere(radius: f32, mass: f32) -> RigidBodyBundle {
        RigidBodyBundle {
            physics: PhysicsBundle {
                collider: PhysicsCollider {
                    shape: ColliderShape::Sphere(Sphere {
                        center: Vec3::ZERO,
                        radius,
                    }),
                },
                transform: default(),
                prev_transform: default(),
                velocity: default(),
                aabb: default(),
            },
            mass: Mass::new(mass),
            inertia: Inertia::solid_sphere(radius, mass).unwrap(),
            ang_vel: default(),
        }
    }

    pub fn solid_cuboid(half_extents: [f32; 3], mass: f32) -> RigidBodyBundle {
        RigidBodyBundle {
            physics: PhysicsBundle {
                collider: PhysicsCollider {
                    shape: ColliderShape::Hull(Hull::cuboid(half_extents.into())),
                },
                transform: default(),
                prev_transform: default(),
                velocity: default(),
                aabb: default(),
            },
            mass: Mass::new(mass),
            inertia: Inertia::solid_cuboid(half_extents, mass).unwrap(),
            ang_vel: default(),
        }
    }

    pub fn solid_capsule(radius: f32, height: f32, mass: f32) -> RigidBodyBundle {
        let radius = radius.abs();
        let height = height.abs();

        RigidBodyBundle {
            physics: PhysicsBundle {
                collider: PhysicsCollider {
                    shape: ColliderShape::Capsule(Capsule {
                        segment: Segment {
                            a: 0.5 * height * Vec3A::Y,
                            b: -0.5 * height * Vec3A::Y,
                        },
                        radius,
                    }),
                },
                transform: default(),
                prev_transform: default(),
                velocity: default(),
                aabb: default(),
            },
            mass: Mass::new(mass),
            inertia: Inertia::solid_capsule(radius, height, mass).unwrap(),
            ang_vel: default(),
        }
    }
}

#[derive(Default, Component)]
pub struct PrevTransform(pub Transform);

/// The bounding-volume hierarchy (BVH) used for broad-phase collision detection.
///
/// Entities with a [`PhysicsAabb`] component are automatically registered with the BVH at every
/// fixed timestep.
#[derive(Resource)]
pub struct PhysicsBvh(pub Bvh<Entity>);

/// The computed AABB of a physics object for the current physics step.
#[derive(Default, Component)]
pub struct PhysicsAabb {
    pub aabb: Aabb,
}

#[derive(WorldQuery)]
#[world_query(mutable)]
struct VelocityIntegratorQuery {
    transform: &'static mut Transform,
    velocity: &'static PhysicsVelocity,
    ang_vel: &'static PhysicsAngVel,
}

impl VelocityIntegrator for VelocityIntegratorQueryItem<'_> {
    #[inline]
    fn position(&self) -> Vec3A {
        self.transform.translation.into()
    }

    #[inline]
    fn set_position(&mut self, position: Vec3A) {
        self.transform.translation = position.into();
    }

    #[inline]
    fn orientation(&self) -> Quat {
        self.transform.rotation
    }

    #[inline]
    fn set_orientation(&mut self, orientation: Quat) {
        self.transform.rotation = orientation;
    }

    #[inline]
    fn velocity(&self) -> Vec3A {
        self.velocity.get()
    }

    #[inline]
    fn ang_vel(&self) -> PhysicsAngVel {
        *self.ang_vel
    }
}

fn integrate_velocity(substep: Res<PhysicsSubstep>, mut query: Query<VelocityIntegratorQuery>) {
    crate::rigid::integrate_velocity(substep.seconds(), query.iter_mut());
}

#[derive(WorldQuery)]
#[world_query(mutable)]
struct ConstraintElementQuery {
    transform: &'static mut Transform,
    prev_transform: &'static mut PrevTransform,
    mass: &'static Mass,
    inertia: &'static Inertia,
}

// Orphan rule :(
struct ConstraintElementQueryWrapper<'world, 'state>(Query<'world, 'state, ConstraintElementQuery>);

impl<'world, 'state> ConstraintSolver<Entity> for ConstraintElementQueryWrapper<'world, 'state> {
    type Element<'e> = ConstraintElementQueryItem<'e> where Self: 'e;

    fn elements<C, const N: usize>(&mut self, keys: [Entity; N]) -> [Self::Element<'_>; N]
    where
        C: Constraint<Entity, N>,
    {
        self.0.get_many_mut(keys).unwrap()
    }
}

impl<'elem> ConstraintElement for ConstraintElementQueryItem<'elem> {
    #[inline]
    fn mass(&self) -> Mass {
        *self.mass
    }

    #[inline]
    fn transform(&self) -> Isometry {
        Isometry::from_transform(*self.transform)
    }

    #[inline]
    fn set_transform(&mut self, transform: Isometry) {
        *self.transform = transform.into();
    }

    #[inline]
    fn inertia(&self) -> Option<Inertia> {
        Some(*self.inertia)
    }
}

#[derive(WorldQuery)]
pub struct ColliderConstraintQuery {
    collider: &'static PhysicsCollider,
    transform: &'static Transform,
}

struct ColliderConstraintQueryWrapper<'a, 'w, 's>(&'a Query<'w, 's, ColliderConstraintQuery>);

impl<'w, 's> ColliderMap for ColliderConstraintQueryWrapper<'_, 'w, 's> {
    type Key = Entity;

    type Element<'elem> = ColliderConstraintQueryItem<'elem> where Self: 'elem;

    fn get(&self, key: Self::Key) -> Self::Element<'_> {
        self.0.get(key).unwrap()
    }
}

impl<'elem> CandidateCollider for ColliderConstraintQueryItem<'elem> {
    #[inline]
    fn collider(&self) -> &ColliderShape {
        &self.collider.shape
    }

    #[inline]
    fn transform(&self) -> Isometry {
        Isometry::from_transform(*self.transform)
    }
}

#[derive(Resource)]
pub struct ContactConstraints(pub Vec<ContactConstraint<Entity>>);

impl ContactConstraints {
    pub fn with_capacity(cap: usize) -> ContactConstraints {
        ContactConstraints(Vec::with_capacity(cap))
    }
}

#[derive(Component)]
struct PhysicsBvhKey(BvhKey);

/// Initializes the `PhysicsBvhKey` component on physics entities that do not yet have one.
fn init_bvh_key(
    mut commands: Commands,
    mut bvh: ResMut<PhysicsBvh>,
    query: Query<(Entity, &PhysicsAabb), Without<PhysicsBvhKey>>,
) {
    for (ent, aabb) in query.iter() {
        let key = bvh.0.insert(aabb.aabb, ent);
        commands.entity(ent).insert(PhysicsBvhKey(key));
    }
}

#[derive(WorldQuery)]
#[world_query(mutable)]
struct UpdatePhysicsAabbQuery {
    entity: Entity,
    transform: &'static Transform,
    velocity: &'static PhysicsVelocity,
    collider: &'static PhysicsCollider,
    aabb: &'static mut PhysicsAabb,
    bvh_key: &'static mut PhysicsBvhKey,
}

impl BvhUpdate for UpdatePhysicsAabbQueryItem<'_> {
    type Key = Entity;

    #[inline]
    fn key(&self) -> Self::Key {
        self.entity
    }

    #[inline]
    fn transform(&self) -> Isometry {
        Isometry::from_transform(*self.transform)
    }

    #[inline]
    fn velocity(&self) -> PhysicsVelocity {
        *self.velocity
    }

    #[inline]
    fn collider(&self) -> &ColliderShape {
        &self.collider.shape
    }

    #[inline]
    fn bvh_key(&self) -> BvhKey {
        self.bvh_key.0
    }

    #[inline]
    fn set_bvh_key(&mut self, key: BvhKey) {
        self.bvh_key.0 = key;
    }
}

fn update_bvh(
    time: Res<Time>,
    mut bvh: ResMut<PhysicsBvh>,
    mut query: Query<UpdatePhysicsAabbQuery>,
) {
    crate::rigid::update_bvh(time.delta_seconds(), &mut bvh.0, query.iter_mut())
}

#[derive(WorldQuery)]
struct BroadPhaseColliderQuery {
    entity: Entity,
    bvh_key: &'static PhysicsBvhKey,
}

impl BroadPhaseCollider for BroadPhaseColliderQueryItem<'_> {
    type Key = Entity;

    #[inline]
    fn key(&self) -> Self::Key {
        self.entity
    }

    #[inline]
    fn bvh_key(&self) -> BvhKey {
        self.bvh_key.0
    }
}

/// Stores the list of collision candidate pairs for the current physics step.
#[derive(Resource)]
pub struct CollisionCandidates {
    candidates: Vec<CollisionCandidate<Entity>>,
}

impl CollisionCandidates {
    pub fn with_capacity(cap: usize) -> CollisionCandidates {
        CollisionCandidates {
            candidates: Vec::with_capacity(cap),
        }
    }
}

fn gather_collision_candidates(
    bvh: Res<PhysicsBvh>,
    mut candidates: ResMut<CollisionCandidates>,
    colliders: Query<BroadPhaseColliderQuery>,
) {
    candidates.candidates.clear();

    candidates
        .candidates
        .extend(crate::rigid::generate_collision_candidates(
            &bvh.0,
            colliders.iter(),
        ));
}

fn generate_collision_constraints(
    mut gizmos: Gizmos,
    candidates: Res<CollisionCandidates>,
    query: Query<ColliderConstraintQuery>,
    mut contacts: ResMut<ContactConstraints>,
) {
    contacts.0.clear();
    contacts.0.extend(
        crate::rigid::generate_contact_constraints(
            candidates.candidates.iter().copied(),
            ColliderConstraintQueryWrapper(&query),
        )
        .inspect(|cc| {
            let elem = query.get(cc.keys[0]).unwrap();
            gizmos.sphere(
                Isometry::from_transform(*elem.transform) * cc.local_contact_points[0],
                Quat::IDENTITY,
                0.1,
                Color::RED,
            );
        }),
    )
}

fn solve_explicit_constraints<C, const N: usize>(
    substep: Res<PhysicsSubstep>,
    explicit_constraints: Query<&C>,
    elements: Query<ConstraintElementQuery>,
) where
    C: Component + Constraint<Entity, N>,
{
    crate::rigid::solve_constraints(
        substep.seconds(),
        explicit_constraints.iter(),
        ConstraintElementQueryWrapper(elements),
    );
}

fn solve_contact_constraints(
    substep: Res<PhysicsSubstep>,
    contact_constraints: Res<ContactConstraints>,
    elements: Query<ConstraintElementQuery>,
) {
    crate::rigid::solve_constraints(
        substep.seconds(),
        contact_constraints.0.iter(),
        ConstraintElementQueryWrapper(elements),
    );
}

#[derive(WorldQuery)]
#[world_query(mutable)]
struct VelocitySolverQuery {
    transform: &'static Transform,
    prev_transform: &'static PrevTransform,
    velocity: &'static mut PhysicsVelocity,
    ang_vel: &'static mut PhysicsAngVel,
}

impl VelocitySolver for VelocitySolverQueryItem<'_> {
    fn transform(&self) -> Isometry {
        Isometry::from_transform(*self.transform)
    }

    fn prev_transform(&self) -> Isometry {
        Isometry::from_transform(self.prev_transform.0)
    }

    fn set_velocity(&mut self, velocity: PhysicsVelocity) {
        *self.velocity = velocity;
    }

    fn set_ang_vel(&mut self, ang_vel: PhysicsAngVel) {
        *self.ang_vel = ang_vel;
    }
}

fn solve_velocity(substep: Res<PhysicsSubstep>, mut query: Query<VelocitySolverQuery>) {
    crate::rigid::solve_velocity(substep.seconds(), query.iter_mut());
}
