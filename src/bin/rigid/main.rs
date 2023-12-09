use bevy::prelude::*;
use glam::Vec3A;
use mechanica::{
    bevy_::{
        CollisionCandidates, ContactConstraints, PhysicsAabb, PhysicsBundle, PhysicsBvh,
        PhysicsGlobalAccelLayers, PhysicsGlobalAccelMask, PhysicsPlugin, PhysicsSubstepCount,
        PrevTransform, RigidBodyBundle,
    },
    bvh::Bvh,
    collider::ColliderShape,
    constraint::PositionalConstraint,
    rigid::{Mass, PhysicsCollider, RigidBodyInertia},
    testbench::TestbenchPlugins,
    Aabb, Isometry, Sphere,
};
use tracing_subscriber::layer::SubscriberExt;

const GRAVITY_LAYER: u32 = 0;

fn main() {
    // tracing::subscriber::set_global_default(
    //     tracing_subscriber::registry()
    //         .with(tracing_tracy::TracyLayer::new())
    //         .with(tracing_subscriber::filter::LevelFilter::INFO),
    // )
    // .unwrap();

    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(TestbenchPlugins)
        .add_plugins(PhysicsPlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, draw_physics_bvh)
        .run();
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

    let mut layers = [Vec3::ZERO; 32];
    layers[GRAVITY_LAYER as usize] = -9.8 * Vec3::Y;
    commands.insert_resource(PhysicsGlobalAccelLayers { layers });

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

    // let anchor = commands
    //     .spawn(RigidBodyBundle::solid_sphere(sphere_radius, f32::INFINITY))
    //     .insert(pbr_sphere(&mut meshes, &mut materials))
    //     .insert(Transform::from_xyz(0.0, 1.0, 0.0))
    //     .id();
    let pendulum = commands
        .spawn(RigidBodyBundle::solid_sphere(sphere_radius, 1.0))
        .insert(pbr_sphere(&mut meshes, &mut materials))
        .insert(Transform::from_xyz(0.05, 1.4, 0.0))
        .insert(PhysicsGlobalAccelMask::new(1 << GRAVITY_LAYER))
        .id();
    let _floor = commands
        .spawn(RigidBodyBundle::solid_cuboid(
            [10.0, 0.5, 10.0],
            f32::INFINITY,
        ))
        .insert(PbrBundle {
            mesh: meshes.add(shape::Box::new(10.0, 0.5, 10.0).into()),
            material: materials.add(Color::GRAY.into()),
            transform: Transform::default(),
            ..default()
        });

    // commands.spawn(PositionalConstraint {
    //     keys: [anchor, pendulum],
    //     local_anchors: [Vec3::ZERO, Vec3::ZERO],
    //     lower_bound: 0.0,
    //     upper_bound: 0.5,
    //     compliance: 0.00001,
    // });
}

fn draw_physics_bvh(bvh: Res<PhysicsBvh>, mut gizmos: Gizmos) {
    bvh.0.draw(&mut gizmos);
}
