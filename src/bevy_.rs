use bevy::{
    ecs::{query::WorldQuery, schedule::ScheduleLabel},
    prelude::*,
};
use glam::Vec3A;

use crate::{
    bvh::{Bvh, BvhKey},
    collider::ColliderShape,
    constraint::{
        Constraint, ConstraintElement, ConstraintSolver, ContactConstraint, PositionalConstraint,
    },
    rigid::{
        BroadPhaseCollider, BvhUpdate, CandidateCollider, ColliderMap, CollisionCandidate, Mass,
        PhysicsAngVel, PhysicsCollider, PhysicsVelocity, RigidBodyInertia, TransformRecorder,
        VelocityIntegrator, VelocitySolver,
    },
    Aabb, Isometry,
};

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

/// Core physics plugin.
pub struct PhysicsPlugin;

impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        let mut physics_substep_schedule = Schedule::new();

        physics_substep_schedule.add_systems(
            (
                record_transform,
                apply_global_acceleration,
                integrate_velocity,
                generate_collision_constraints,
                resolve_positional_constraints,
                resolve_contact_constraints,
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
    }
}

/// The schedule which runs physics simulation substeps.
#[derive(Clone, Debug, PartialEq, Eq, Hash, ScheduleLabel)]
struct PhysicsSubstepSchedule;

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
    // TODO: un-pub
    seconds: f32,
    inv_2: f32,
}

impl PhysicsSubstep {
    pub fn new(seconds: f32) -> PhysicsSubstep {
        assert!(seconds.is_finite() && seconds > 0.0);

        let inv_2 = seconds.recip().powi(2);

        PhysicsSubstep { seconds, inv_2 }
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

/// Defines global acceleration layers.
#[derive(Resource)]
pub struct PhysicsGlobalAccelLayers {
    pub layers: [Vec3; 32],
}

/// Selects which global acceleration layers apply to an entity.
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
    pub collider: PhysicsCollider,
    pub velocity: PhysicsVelocity,
}

#[derive(Bundle)]
pub struct RigidBodyBundle {
    pub physics: PhysicsBundle,
    pub mass: Mass,
    pub inertia: RigidBodyInertia,
    pub ang_vel: PhysicsAngVel,
}

#[derive(Default, Component)]
pub struct PrevTransform(pub Transform);

#[derive(Resource)]
pub struct PhysicsBvh(pub Bvh<Entity>);

/// The computed AABB of a physics object for the current physics step.
#[derive(Component)]
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
    inertia: &'static RigidBodyInertia,
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
    fn inertia(&self) -> Option<RigidBodyInertia> {
        Some(*self.inertia)
    }
}

#[derive(WorldQuery)]
pub struct ColliderConstraintQuery {
    collider: &'static PhysicsCollider,
    transform: &'static Transform,
}

struct ColliderConstraintQueryWrapper<'w, 's>(Query<'w, 's, ColliderConstraintQuery>);

impl<'w, 's> ColliderMap for ColliderConstraintQueryWrapper<'w, 's> {
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
    candidates: Res<CollisionCandidates>,
    query: Query<ColliderConstraintQuery>,
    mut contacts: ResMut<ContactConstraints>,
) {
    contacts.0.clear();
    contacts
        .0
        .extend(crate::rigid::generate_collision_constraints(
            candidates.candidates.iter().copied(),
            ColliderConstraintQueryWrapper(query),
        ))
}

fn resolve_positional_constraints(
    substep: Res<PhysicsSubstep>,
    explicit_constraints: Query<&PositionalConstraint<Entity>>,
    elements: Query<ConstraintElementQuery>,
) {
    crate::rigid::solve_constraints(
        substep.seconds(),
        explicit_constraints.iter(),
        ConstraintElementQueryWrapper(elements),
    );
}

fn resolve_contact_constraints(
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
    fn position(&self) -> Vec3A {
        self.transform.translation.into()
    }

    fn prev_position(&self) -> Vec3A {
        self.prev_transform.0.translation.into()
    }

    fn orientation(&self) -> Quat {
        self.transform.rotation
    }

    fn prev_orientation(&self) -> Quat {
        self.prev_transform.0.rotation
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
