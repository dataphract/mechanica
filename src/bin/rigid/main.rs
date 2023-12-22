use bevy::prelude::*;
use mechanica::{
    bevy_::{
        PhysicsBvh, PhysicsGlobalAccelLayers, PhysicsGlobalAccelMask, PhysicsPlugin,
        RigidBodyBundle,
    },
    testbench::TestbenchPlugins,
};

const GRAVITY_LAYER: u32 = 0;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(TestbenchPlugins)
        .add_plugins(PhysicsPlugin::default())
        .add_systems(Startup, setup)
        .add_systems(Update, draw_physics_bvh)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut accel_layers: ResMut<PhysicsGlobalAccelLayers>,
) {
    commands.insert_resource(FixedTime::new_from_secs(1.0 / 64.0));

    accel_layers.layers[GRAVITY_LAYER as usize] = -9.8 * Vec3::Y;

    let sphere_radius = 0.1;

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
