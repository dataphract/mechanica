use bevy::prelude::*;
use bevy_mod_picking::{prelude::*, DefaultPickingPlugins, PickableBundle};
use bevy_transform_gizmo::TransformGizmoPlugin;

use super::{EdgeKey, FaceKey, NMesh, VertKey};

struct NMeshPlugin;
impl Plugin for NMeshPlugin {
    fn build(&self, app: &mut App) {
        app.add_asset::<NMesh>();
    }
}

pub struct NMeshPlugins;

impl PluginGroup for NMeshPlugins {
    fn build(self) -> bevy::app::PluginGroupBuilder {
        DefaultPickingPlugins
            .build()
            .add(TransformGizmoPlugin::default())
            .add(NMeshPlugin)
    }
}

/// Marker component for features of an NMesh.
#[derive(Component)]
pub struct NMeshFeature;

/// Marker component for the root entity of an NMesh.
#[derive(Component)]
pub struct NMeshRoot;

#[derive(Component)]
pub struct NMeshVert(VertKey);

#[derive(Component)]
pub struct NMeshEdge(EdgeKey);

#[derive(Component)]
pub struct NMeshFace(FaceKey);

#[derive(Bundle)]
pub struct NMeshFeatureBundle {
    pub feature: NMeshFeature,
    pub pickable: PickableBundle,
    pub target: RaycastPickTarget,
    pub pbr: PbrBundle,
    pub nmesh: Handle<NMesh>,
}

#[derive(Bundle)]
pub struct VertHandleBundle {
    pub handle: NMeshFeatureBundle,
    pub vert: NMeshVert,
}

#[derive(Bundle)]
pub struct EdgeHandleBundle {
    pub handle: NMeshFeatureBundle,
    pub edge: NMeshEdge,
}

#[derive(Bundle)]
pub struct FaceHandleBundle {
    pub handle: NMeshFeatureBundle,
    pub vert: NMeshFace,
}

#[derive(Bundle)]
pub struct NMeshBundle {
    pub root: NMeshRoot,
    pub nmesh: Handle<NMesh>,
    pub spatial: SpatialBundle,
    pub pickable: PickableBundle,
    pub target: RaycastPickTarget,
}

fn capsule_from_line_segment(a: Vec3, b: Vec3) -> (shape::Capsule, Quat) {
    let ab = b - a;
    let len = ab.length();
    let dir = ab.normalize();

    let axis = dir.cross(Vec3::Y);
    let angle = axis.dot(dir).acos();

    (
        shape::Capsule {
            radius: 0.025,
            depth: len,
            ..default()
        },
        Quat::from_axis_angle(axis, angle),
    )
}

pub fn create_nmesh(
    mut commands: Commands,
    materials: &mut Assets<StandardMaterial>,
    meshes: &mut Assets<Mesh>,
    nmeshes: &mut Assets<NMesh>,
    nmesh: NMesh,
) {
    let handle = nmeshes.add(nmesh);
    let nmesh = nmeshes.get_mut(&handle).unwrap();

    let material = materials.add(Color::SILVER.into());

    let vert_mesh = meshes.add(
        shape::UVSphere {
            radius: 0.05,
            ..default()
        }
        .into(),
    );

    commands
        .spawn(NMeshBundle {
            root: NMeshRoot,
            nmesh: handle.clone(),
            spatial: default(),
            pickable: PickableBundle::default(),
            target: RaycastPickTarget::default(),
        })
        .insert(On::<Pointer<Over>>::run(|| {
            println!("hovered");
        }))
        .with_children(|builder| {
            for (vk, vert) in &nmesh.verts {
                builder.spawn(VertHandleBundle {
                    handle: NMeshFeatureBundle {
                        feature: NMeshFeature,
                        pickable: PickableBundle::default(),
                        target: RaycastPickTarget::default(),
                        pbr: PbrBundle {
                            mesh: vert_mesh.clone(),
                            material: material.clone(),
                            transform: Transform::from_translation(vert.coord),
                            ..default()
                        },
                        nmesh: handle.clone(),
                    },
                    vert: NMeshVert(vk),
                });
            }

            for (ek, edge) in &nmesh.edges {
                let a = nmesh.verts[edge.v0].coord;
                let b = nmesh.verts[edge.v1].coord;

                let (capsule, rotation) = capsule_from_line_segment(a, b);

                builder.spawn(EdgeHandleBundle {
                    handle: NMeshFeatureBundle {
                        feature: NMeshFeature,
                        pickable: PickableBundle::default(),
                        target: RaycastPickTarget::default(),
                        pbr: PbrBundle {
                            mesh: meshes.add(capsule.into()),
                            material: material.clone(),
                            transform: Transform::from_translation(a + 0.5 * (b - a))
                                .with_rotation(rotation),
                            ..default()
                        },
                        nmesh: handle.clone(),
                    },
                    edge: NMeshEdge(ek),
                });
            }
        });
}
