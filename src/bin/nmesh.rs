use bevy::{
    input::mouse::{MouseMotion, MouseWheel},
    prelude::*,
};
use bevy_infinite_grid::{InfiniteGridBundle, InfiniteGridPlugin};
use bevy_mod_picking::prelude::*;
use bevy_transform_gizmo::GizmoPickSource;
use cg3::nmesh::{
    bevy_::{create_nmesh, NMeshPlugins},
    NMesh,
};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(InfiniteGridPlugin)
        .add_plugins(NMeshPlugins)
        .add_systems(Startup, setup)
        .add_systems(Update, control_camera)
        .run();
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut nmeshes: ResMut<Assets<NMesh>>,
) {
    commands.spawn(InfiniteGridBundle::default());
    commands
        .spawn(Camera3dBundle {
            projection: Projection::Perspective(default()),
            transform: Transform::from_xyz(5.0, 1.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        })
        .insert(EditorCamera { focus: Vec3::ZERO })
        .insert(RaycastPickCamera::default())
        .insert(GizmoPickSource::default());
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 4000.0,
            ..default()
        },
        transform: Transform::from_xyz(10.0, 10.0, 10.0),
        ..default()
    });

    create_nmesh(
        commands,
        &mut materials,
        &mut meshes,
        &mut nmeshes,
        NMesh::cube(),
    );
}

#[derive(Component)]
struct EditorCamera {
    pub focus: Vec3,
}

fn control_camera(
    mut mouse_motion: EventReader<MouseMotion>,
    mut mouse_wheel: EventReader<MouseWheel>,
    mouse_button: Res<Input<MouseButton>>,
    mut camera: Query<(&mut EditorCamera, &mut Transform)>,
) {
    let orbit_button = MouseButton::Middle;

    let mut orbit_motion = Vec2::ZERO;

    if mouse_button.pressed(orbit_button) {
        for evt in mouse_motion.iter() {
            orbit_motion += 0.01 * evt.delta;
        }
    }

    let (mut cam, mut transform) = camera.get_single_mut().unwrap();

    if orbit_motion.length_squared() > 0.0 {
        let yaw = Quat::from_rotation_y(-orbit_motion.x);
        let pitch = Quat::from_rotation_x(-orbit_motion.y);

        // Rotate around global Y-axis, then local X-axis.
        transform.rotation = (yaw * transform.rotation) * pitch;
    }

    transform.translation = cam.focus + transform.rotation * Vec3::new(0.0, 0.0, 5.0);
}
