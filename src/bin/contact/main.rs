use std::iter;

use arrayvec::ArrayVec;
use bevy::prelude::*;
use bevy_mod_picking::{prelude::*, PickableBundle};
use bevy_transform_gizmo::GizmoTransformable;
use mechanica::{
    collider::ColliderShape,
    contact::{
        contact_capsule_capsule, contact_capsule_hull, contact_capsule_sphere,
        contact_collider_collider, contact_hull_hull, contact_hull_sphere, contact_sphere_sphere,
        Contact,
    },
    testbench::TestbenchPlugins,
    Isometry, Line, Plane, Sphere,
};

mod ui;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(TestbenchPlugins)
        .add_systems(Startup, setup)
        .add_systems(Update, init_physobj)
        .add_systems(Update, init_contact_vis)
        .add_systems(Update, ui::render_ui)
        .add_systems(Update, update_physobj)
        .add_systems(Update, update_contact_vis)
        //.add_systems(Update, draw_line)
        .run();
}

#[derive(Component)]
struct ObjA;

#[derive(Component)]
struct ObjB;

#[derive(Component)]
struct Collider {
    shape: ColliderShape,
}

fn setup(mut commands: Commands) {
    let obj1 = commands
        .spawn_empty()
        .insert(PhysObj {
            shape: ColliderShape::Sphere(Sphere {
                center: Vec3::ZERO,
                radius: 0.5,
            }),
        })
        .insert(Transform::from_xyz(-2.0, 1.0, 0.0))
        .insert(ObjA)
        .id();

    let obj2 = commands
        .spawn_empty()
        .insert(PhysObj {
            shape: ColliderShape::Sphere(Sphere {
                center: Vec3::ZERO,
                radius: 0.5,
            }),
        })
        .insert(Transform::from_xyz(2.0, 1.0, 0.0))
        .insert(ObjB)
        .id();

    commands.spawn_empty().insert(ContactPair {
        on_a: obj1,
        on_b: obj2,
    });
}

#[derive(Component)]
struct PhysObj {
    shape: ColliderShape,
}

fn mesh_for_collider(collider: &ColliderShape) -> Mesh {
    match collider {
        ColliderShape::Sphere(s) => shape::UVSphere {
            radius: s.radius,
            ..default()
        }
        .into(),

        ColliderShape::Capsule(c) => {
            let ab = c.segment.b - c.segment.a;
            assert_eq!(ab.x, 0.0);
            assert_eq!(ab.z, 0.0);

            shape::Capsule {
                radius: c.radius,
                depth: ab.y.abs(),
                ..default()
            }
            .into()
        }

        ColliderShape::Hull(h) => h.to_mesh(),
    }
}

fn init_physobj(
    mut commands: Commands,
    mut objs: Query<(Entity, &PhysObj, &Transform), Without<Handle<Mesh>>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for (ent, obj, &xf) in objs.iter_mut() {
        bevy::log::info!("init_physobj");
        let mut ec = commands.entity(ent);
        ec.insert(PbrBundle {
            mesh: meshes.add(mesh_for_collider(&obj.shape)),
            material: materials.add(Color::WHITE.with_a(0.5).into()),
            transform: xf,
            ..default()
        })
        .insert(Collider {
            shape: obj.shape.clone(),
        })
        .insert(PickableBundle::default())
        .insert(RaycastPickTarget::default())
        .insert(GizmoTransformable);
    }
}

fn update_physobj(
    mut objs: Query<(&PhysObj, &mut Handle<Mesh>), Changed<PhysObj>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    for (obj, mut mesh) in objs.iter_mut() {
        *mesh = meshes.add(mesh_for_collider(&obj.shape));
    }
}

#[derive(Debug, Component)]
struct ContactPair {
    on_a: Entity,
    on_b: Entity,
}

#[derive(Debug, Component)]
struct ContactVis {
    a_vis: Entity,
    b_vis: Entity,
    contact_points: ArrayVec<Entity, 4>,
    axis: Vec3,
}

#[derive(Debug, Component)]
struct VisTag;

fn init_contact_vis(
    mut commands: Commands,
    query: Query<Entity, (With<ContactPair>, Without<ContactVis>)>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for ent in query.iter() {
        let mut vis = || -> PbrBundle {
            PbrBundle {
                mesh: meshes.add(
                    shape::UVSphere {
                        radius: 0.1,
                        ..default()
                    }
                    .into(),
                ),
                material: materials.add(StandardMaterial {
                    unlit: true,
                    ..Color::RED.into()
                }),
                ..default()
            }
        };

        let a_vis = commands.spawn(vis()).insert(VisTag).id();
        let b_vis = commands.spawn(vis()).insert(VisTag).id();
        let contact_points = ArrayVec::from_iter(
            iter::repeat_with(|| commands.spawn(vis()).insert(VisTag).id()).take(4),
        );

        let mut ec = commands.entity(ent);
        ec.insert(ContactVis {
            a_vis,
            b_vis,
            contact_points,
            axis: Vec3::ZERO,
        });
    }
}

fn update_contact_vis(
    query: Query<(&ContactPair, &ContactVis)>,
    objs: Query<(&PhysObj, &Transform), Without<VisTag>>,
    mut vis: Query<(&mut Transform, &mut Visibility), With<VisTag>>,
    mut gizmos: Gizmos,
) {
    for (closest, closest_vis) in query.iter() {
        let (a_obj, a_xf) = objs.get(closest.on_a).unwrap();
        let (b_obj, b_xf) = objs.get(closest.on_b).unwrap();
        let iso_a = Isometry::from_transform(*a_xf);
        let iso_b = Isometry::from_transform(*b_xf);

        match contact_collider_collider(&a_obj.shape, iso_a, &b_obj.shape, iso_b) {
            Contact::Disjoint(d) => {
                let (mut a_xf, mut a_vis) = vis.get_mut(closest_vis.a_vis).unwrap();
                *a_xf = Transform::from_translation(d.on_a);
                *a_vis = Visibility::Visible;

                let (mut b_xf, mut b_vis) = vis.get_mut(closest_vis.b_vis).unwrap();
                *b_xf = Transform::from_translation(d.on_b);
                *b_vis = Visibility::Visible;

                for &pt in &closest_vis.contact_points {
                    let (_xf, mut vis) = vis.get_mut(pt).unwrap();
                    *vis = Visibility::Hidden;
                }
            }

            Contact::Penetrating(p) => {
                let (_a_xf, mut a_vis) = vis.get_mut(closest_vis.a_vis).unwrap();
                *a_vis = Visibility::Hidden;

                let (_b_xf, mut b_vis) = vis.get_mut(closest_vis.b_vis).unwrap();
                *b_vis = Visibility::Hidden;

                for &pt in &closest_vis.contact_points {
                    let (_xf, mut vis) = vis.get_mut(pt).unwrap();
                    *vis = Visibility::Hidden;
                }

                for (i, &pt) in p.points.iter().enumerate() {
                    gizmos.ray(pt, p.axis, Color::BLACK);

                    let (mut xf, mut vis) = vis.get_mut(closest_vis.contact_points[i]).unwrap();
                    *xf = Transform::from_translation(pt);
                    *vis = Visibility::Visible;
                }
            }
        }
    }
}

// struct ZOrderPoints {
//     points: Vec<Vec3A>,
// }

// impl Default for ZOrderPoints {
//     fn default() -> Self {
//         let mut v = Vec::new();

//         const SEED: [u8; 32] = [
//             0x4c, 0x64, 0x43, 0x4e, 0xb8, 0x8d, 0x56, 0xf7, 0x73, 0xb2, 0x6a, 0xb0, 0x52, 0xcc,
//             0x0b, 0xac, 0x11, 0x42, 0x1b, 0xad, 0xfe, 0xb4, 0x67, 0xa7, 0x05, 0x70, 0x0f, 0xe0,
//             0x35, 0x1b, 0xbe, 0x2b,
//         ];

//         use rand::SeedableRng;
//         let mut rng = rand::rngs::StdRng::from_seed(SEED);

//         for _ in 0..(128 * 1024) {
//             v.push(Vec3A::new(
//                 16.0 * (-0.5 + rng.gen::<f32>()),
//                 16.0 * (-0.5 + rng.gen::<f32>()),
//                 16.0 * (-0.5 + rng.gen::<f32>()),
//             ))
//         }

//         mechanica::zorder::sort_z_order(&mut v);

//         Self { points: v }
//     }
// }

// fn draw_zorder(points: Local<ZOrderPoints>, mut gizmos: Gizmos) {
//     let len = points.points.len();
//     gizmos.linestrip_gradient(points.points.iter().enumerate().map(|(idx, &pt)| {
//         let hue = 360.0 * (idx as f32 / len as f32);
//         (
//             pt.into(),
//             Color::Hsla {
//                 hue,
//                 saturation: 1.0,
//                 lightness: 0.5,
//                 alpha: 1.0,
//             },
//         )
//     }))
// }

fn draw_line(mut gizmos: Gizmos) {
    let a = Vec3::new(-0.88, 1.5, 0.71);
    let b = Vec3::new(-1.56, 1.5, -0.03);

    let line = Line::from_points(a, b).unwrap();

    gizmos.sphere(a, Quat::IDENTITY, 0.1, Color::BLUE);
    gizmos.sphere(b, Quat::IDENTITY, 0.1, Color::BLUE);

    gizmos.sphere(line.ray.origin, Quat::IDENTITY, 0.1, Color::CYAN);
    gizmos.ray(line.ray.origin, 100.0 * line.ray.dir, Color::CYAN);
    gizmos.ray(line.ray.origin, -100.0 * line.ray.dir, Color::CYAN);

    let pt = line
        .intersect_plane(Plane::from_point_normal(Vec3::ZERO, -Vec3::Z).unwrap())
        .unwrap();
    gizmos.sphere(pt.into(), Quat::IDENTITY, 0.1, Color::ORANGE_RED);
}
