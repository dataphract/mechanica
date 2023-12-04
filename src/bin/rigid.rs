use bevy::{
    ecs::{
        query::{QueryIter, WorldQuery},
        schedule::ScheduleLabel,
    },
    prelude::*,
};
use glam::Vec3A;
use mechanica::{
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
    testbench::TestbenchPlugins,
    Aabb, Isometry, Sphere,
};

fn main() {
    let mut physics_substep_schedule = Schedule::new();

    physics_substep_schedule.add_systems(
        (
            record_transform,
            apply_gravity,
            integrate_velocity,
            generate_collision_constraints,
            resolve_positional_constraints,
            resolve_contact_constraints,
            solve_velocity,
        )
            .chain(),
    );

    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(TestbenchPlugins)
        .add_schedule(PhysicsSubstepSchedule, physics_substep_schedule)
        .add_systems(Startup, setup)
        .add_systems(
            FixedUpdate,
            (
                init_bvh_key,
                update_bvh,
                gather_collision_candidates,
                run_physics_substep_loop,
            )
                .chain(),
        )
        .add_systems(Update, draw_physics_bvh)
        .run();
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

#[derive(Resource)]
pub struct PhysicsSubstep {
    // TODO: un-pub
    pub seconds: f32,
}

impl PhysicsSubstep {
    pub fn seconds(&self) -> f32 {
        self.seconds
    }
}

#[derive(Default, Component)]
struct PrevTransform(Transform);

#[derive(Clone, Debug, PartialEq, Eq, Hash, ScheduleLabel)]
struct PhysicsSubstepSchedule;

#[derive(Clone, Debug, PartialEq, Eq, Hash, ScheduleLabel)]
struct PhysicsSchedule;

// Based on bevy_time::fixed::run_fixed_update_schedule
pub fn run_physics_substep_loop(world: &mut World) {
    let timestep = world.resource::<FixedTime>().period.as_secs_f32();
    let substep_count = world.resource::<PhysicsSubstepCount>().get();
    let substep = timestep / substep_count as f32;

    world.insert_resource(PhysicsSubstep { seconds: substep });

    let _ = world.try_schedule_scope(PhysicsSubstepSchedule, |world, schedule| {
        for _ in 0..substep_count {
            schedule.run(world);
        }
    });
}

#[derive(Resource)]
pub struct PhysicsBvh(Bvh<Entity>);

/// The computed AABB of a physics object for the current physics step.
#[derive(Component)]
pub struct PhysicsAabb {
    aabb: Aabb,
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.insert_resource(CollisionCandidates::with_capacity(1024));
    commands.insert_resource(ContactConstraints::with_capacity(1024));
    commands.insert_resource(FixedTime::new_from_secs(1.0 / 64.0));
    commands.insert_resource(PhysicsBvh(Bvh::with_capacity(1024)));
    commands.insert_resource(PhysicsSubstepCount::new(20));

    let sphere_radius = 0.1;

    let rigid_sphere = |mass: f32| RigidBodyBundle {
        physics: PhysicsBundle {
            collider: PhysicsCollider {
                shape: ColliderShape::Sphere(Sphere {
                    center: Vec3::ZERO,
                    radius: sphere_radius,
                }),
            },
            transform: default(),
            velocity: default(),
        },
        mass: Mass::new(mass),
        inertia: RigidBodyInertia::solid_sphere(sphere_radius, mass).unwrap(),
        ang_vel: default(),
    };

    let pbr_sphere =
        |meshes: &mut Assets<Mesh>, materials: &mut Assets<StandardMaterial>| PbrBundle {
            mesh: meshes.add(
                shape::UVSphere {
                    radius: sphere_radius,
                    ..default()
                }
                .into(),
            ),
            material: materials.add(Color::GRAY.into()),
            ..default()
        };

    let test_sphere = commands.spawn(pbr_sphere(&mut meshes, &mut materials));

    let anchor = commands
        .spawn(rigid_sphere(f32::INFINITY))
        .insert(PhysicsAabb {
            aabb: Aabb::new(Vec3::ZERO, Vec3::ONE),
        })
        .insert(pbr_sphere(&mut meshes, &mut materials))
        .insert(Transform::from_xyz(0.0, 1.0, 0.0))
        .insert(PrevTransform(Transform::from_xyz(0.0, 1.0, 0.0)))
        .id();
    let pendulum = commands
        .spawn(rigid_sphere(1.0))
        .insert(PhysicsAabb {
            aabb: Aabb::new(Vec3::ZERO, Vec3::ONE),
        })
        .insert(pbr_sphere(&mut meshes, &mut materials))
        .insert(Transform::from_xyz(0.05, 1.4, 0.0))
        .insert(PrevTransform(Transform::from_xyz(0.0, 1.5, 0.0)))
        .insert(Gravity)
        .id();

    commands.spawn(PositionalConstraint {
        keys: [anchor, pendulum],
        local_anchors: [Vec3::ZERO, Vec3::ZERO],
        lower_bound: 0.0,
        upper_bound: 0.5,
        compliance: 0.00001,
    });
}

#[derive(Component)]
struct Gravity;

#[derive(Component)]
struct PhysicsBvhKey(BvhKey);

fn init_bvh_key(
    mut commands: Commands,
    mut bvh: ResMut<PhysicsBvh>,
    query: Query<(Entity, &PhysicsAabb), Without<PhysicsBvhKey>>,
) {
    for (ent, aabb) in query.iter() {
        let key = bvh.0.insert(aabb.aabb, ent);
        println!("inserted key {key:?}");
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

    fn key(&self) -> Self::Key {
        self.entity
    }

    fn transform(&self) -> Isometry {
        Isometry::from_transform(*self.transform)
    }

    fn velocity(&self) -> PhysicsVelocity {
        *self.velocity
    }

    fn collider(&self) -> &ColliderShape {
        &self.collider.shape
    }

    fn bvh_key(&self) -> BvhKey {
        self.bvh_key.0
    }

    fn set_bvh_key(&mut self, key: BvhKey) {
        self.bvh_key.0 = key;
    }
}

fn update_bvh(
    time: Res<Time>,
    mut bvh: ResMut<PhysicsBvh>,
    mut query: Query<UpdatePhysicsAabbQuery>,
) {
    mechanica::rigid::update_bvh(time.delta_seconds(), &mut bvh.0, query.iter_mut())
}

fn draw_physics_bvh(bvh: Res<PhysicsBvh>, mut gizmos: Gizmos) {
    bvh.0.draw(&mut gizmos);
}

fn apply_gravity(
    substep: Res<PhysicsSubstep>,
    mut query: Query<&mut PhysicsVelocity, With<Gravity>>,
) {
    for mut vel in query.iter_mut() {
        let v = vel.get() + substep.seconds() * -9.8 * Vec3A::Y;
        vel.set(v);
    }
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

#[derive(Resource)]
struct CollisionCandidates {
    candidates: Vec<CollisionCandidate<Entity>>,
}

impl CollisionCandidates {
    fn with_capacity(cap: usize) -> CollisionCandidates {
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
        .extend(mechanica::rigid::generate_collision_candidates(
            &bvh.0,
            colliders.iter(),
        ));
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
    mechanica::rigid::record_transform(query.iter_mut());
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
    mechanica::rigid::integrate_velocity(substep.seconds(), query.iter_mut());
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
pub struct ContactConstraints(Vec<ContactConstraint<Entity>>);

impl ContactConstraints {
    fn with_capacity(cap: usize) -> ContactConstraints {
        ContactConstraints(Vec::with_capacity(cap))
    }
}

fn generate_collision_constraints(
    candidates: Res<CollisionCandidates>,
    query: Query<ColliderConstraintQuery>,
    mut contacts: ResMut<ContactConstraints>,
) {
    contacts.0.clear();
    contacts
        .0
        .extend(mechanica::rigid::generate_collision_constraints(
            candidates.candidates.iter().copied(),
            ColliderConstraintQueryWrapper(query),
        ))
}

fn resolve_positional_constraints(
    substep: Res<PhysicsSubstep>,
    explicit_constraints: Query<&PositionalConstraint<Entity>>,
    elements: Query<ConstraintElementQuery>,
) {
    mechanica::rigid::solve_constraints(
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
    mechanica::rigid::solve_constraints(
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
    mechanica::rigid::solve_velocity(substep.seconds(), query.iter_mut());
}
