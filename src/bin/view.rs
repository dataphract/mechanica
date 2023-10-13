use bevy::{asset::Assets, prelude::*};
use cg3::mesh_edit::MeshEdit;

fn main() {
    let app = App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, setup)
        .add_systems(Update, update)
        .run();
}

#[derive(Component)]
struct CMeshEdit(MeshEdit);

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let cube_edit = MeshEdit::cube();
    let cube = commands
        .spawn(PbrBundle {
            mesh: meshes.add(cube_edit.to_mesh()),
            material: materials.add(Color::rgb(0.5, 0.5, 0.5).into()),
            transform: Transform::from_xyz(4.0, 0.0, 0.0),
            ..default()
        })
        .insert(CMeshEdit(cube_edit));

    let camera = commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(10.0, 5.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 1500.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        ..default()
    });
}

fn update(cube: Query<(&CMeshEdit, &GlobalTransform)>, mut gizmos: Gizmos) {
    let (cube, gt) = cube.get_single().unwrap();
    let vert_lists = cube.0.face_verts();
    let mat = gt.compute_matrix();
    for list in vert_lists {
        gizmos.linestrip(
            list.into_iter().map(|v| (mat * v.extend(1.0)).truncate()),
            Color::BLACK,
        );
    }
}
