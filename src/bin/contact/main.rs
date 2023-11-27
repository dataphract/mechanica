use std::iter;

use arrayvec::ArrayVec;
use bevy::{
    ecs::system::SystemParam,
    input::mouse::{MouseMotion, MouseScrollUnit, MouseWheel},
    prelude::*,
    window::{CursorGrabMode, PrimaryWindow},
};
use bevy_egui::EguiPlugin;
use bevy_infinite_grid::{InfiniteGridBundle, InfiniteGridPlugin};
use bevy_mod_picking::{prelude::*, PickableBundle};
use bevy_transform_gizmo::{GizmoPickSource, GizmoTransformable};
use cg3::{
    collider::ColliderShape,
    contact::{
        contact_capsule_capsule, contact_capsule_hull, contact_capsule_sphere, contact_hull_hull,
        contact_hull_sphere, contact_sphere_sphere, Contact,
    },
    Isometry, Line, Plane, Sphere,
};
use glam::Vec3A;
use rand::Rng;

mod ui;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(InfiniteGridPlugin)
        .add_plugins(EguiPlugin)
        .add_plugins(bevy_mod_picking::DefaultPickingPlugins)
        .insert_resource(State::new(
            bevy_mod_picking::debug::DebugPickingMode::Disabled,
        ))
        .add_plugins(bevy_transform_gizmo::TransformGizmoPlugin::default())
        .add_systems(Startup, setup)
        .add_systems(PostStartup, post_startup)
        .add_systems(Update, init_physobj)
        .add_systems(Update, init_contact_vis)
        .add_systems(Update, control_camera)
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

fn post_startup(
    mut materials: ResMut<Assets<StandardMaterial>>,
    highlight: ResMut<GlobalHighlight<StandardMaterial>>,
) {
    let mut make_translucent = |handle| {
        let mat = materials.get_mut(handle).unwrap();
        mat.alpha_mode = AlphaMode::Blend;
        mat.base_color.set_a(0.5);
    };

    make_translucent(&highlight.hovered);
    make_translucent(&highlight.pressed);
    make_translucent(&highlight.selected);
}

fn setup(mut commands: Commands) {
    commands.spawn(InfiniteGridBundle::default());

    commands
        .spawn(Camera3dBundle {
            projection: Projection::Perspective(default()),
            transform: Transform::from_xyz(0.0, 2.0, 8.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        })
        .insert(EditorCamera {
            focus: Vec3::ZERO,
            distance: 5.0,
        })
        .insert(RaycastPickCamera::default())
        .insert(GizmoPickSource::default());

    commands
        .spawn(DirectionalLightBundle {
            directional_light: DirectionalLight {
                illuminance: 10_000.0,
                shadows_enabled: false,
                ..default()
            },
            ..default()
        })
        .insert(SolidShadingLight);

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
        let a_iso = Isometry::from_transform(*a_xf);
        let b_iso = Isometry::from_transform(*b_xf);

        let contact = match (&a_obj.shape, &b_obj.shape) {
            (ColliderShape::Capsule(c1), ColliderShape::Capsule(c2)) => {
                contact_capsule_capsule(c1, a_iso, c2, b_iso)
            }

            (ColliderShape::Capsule(c), ColliderShape::Hull(h)) => {
                contact_capsule_hull(c, a_iso, h, b_iso, &mut gizmos)
            }

            (ColliderShape::Hull(h), ColliderShape::Capsule(c)) => {
                contact_capsule_hull(c, b_iso, h, a_iso, &mut gizmos).reverse()
            }

            (ColliderShape::Sphere(s1), ColliderShape::Sphere(s2)) => {
                contact_sphere_sphere(s1, a_iso, s2, b_iso)
            }

            (ColliderShape::Capsule(c), ColliderShape::Sphere(s)) => {
                contact_capsule_sphere(c, a_iso, s, b_iso)
            }

            (ColliderShape::Sphere(s), ColliderShape::Capsule(c)) => {
                contact_capsule_sphere(c, b_iso, s, a_iso).reverse()
            }

            (ColliderShape::Hull(h), ColliderShape::Sphere(s)) => {
                contact_hull_sphere(h, a_iso, s, b_iso, &mut gizmos)
            }

            (ColliderShape::Sphere(s), ColliderShape::Hull(h)) => {
                contact_hull_sphere(h, b_iso, s, a_iso, &mut gizmos).reverse()
            }

            (ColliderShape::Hull(h1), ColliderShape::Hull(h2)) => {
                contact_hull_hull(h1, a_iso, h2, b_iso, &mut gizmos)
            }
        };

        match contact {
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

#[derive(Component)]
struct SolidShadingLight;

#[derive(Component)]
struct EditorCamera {
    pub focus: Vec3,
    pub distance: f32,
}

#[derive(SystemParam)]
struct CameraInput<'w, 's> {
    mouse_motion: EventReader<'w, 's, MouseMotion>,

    mouse_wheel: EventReader<'w, 's, MouseWheel>,

    key_input: Res<'w, Input<KeyCode>>,
    mouse_button: Res<'w, Input<MouseButton>>,
}

fn control_camera(
    mut window: Query<&mut Window, With<PrimaryWindow>>,
    mut camera_input: CameraInput,
    mut camera: Query<(&mut EditorCamera, &mut Transform, &Projection), Without<SolidShadingLight>>,
    mut solid_shading_light: Query<&mut Transform, With<SolidShadingLight>>,
) {
    const ORBIT_SENSITIVITY: f32 = 4.0;
    const PAN_SENSITIVITY: f32 = 2.0;

    let mut window = window.get_single_mut().unwrap();
    let win_phys_resolution = Vec2::new(
        window.resolution.physical_width() as f32,
        window.resolution.physical_height() as f32,
    );
    let win_scale_factor = win_phys_resolution.recip();

    enum OrbitOrPan {
        Orbit,
        Pan,
    }

    let mut orbit_or_pan = OrbitOrPan::Orbit;
    let mut mouse_motion = Vec2::ZERO;

    if camera_input.mouse_button.pressed(MouseButton::Middle)
        || camera_input.key_input.pressed(KeyCode::Space)
    {
        window.cursor.grab_mode = CursorGrabMode::Confined;

        for evt in camera_input.mouse_motion.iter() {
            mouse_motion += win_scale_factor * evt.delta;
        }

        if camera_input.key_input.pressed(KeyCode::ShiftLeft) {
            orbit_or_pan = OrbitOrPan::Pan;
        }

        // TODO: this doesn't work on Wayland.
        //
        // Wayland doesn't allow the cursor position to be directly set programmatically. Blender
        // uses an elaborate workaround (see `GHOST_SystemWayland::window_cursor_grab_set`) which
        // involves locking and hiding the hardware cursor, using a software cursor (which can be
        // wrapped programatically) until the grab is released, and maintaining a cursor hint (via
        // zwp_locked_pointer_v1) so that the hardware cursor warps to the correct location on unlock.
        if let Some(cursor_pos) = window.physical_cursor_position() {
            let new_x = if cursor_pos[0] == 0.0 && mouse_motion[0] < 0.0 {
                win_phys_resolution[0]
            } else if cursor_pos[0] >= win_phys_resolution[0] - 1.0 && mouse_motion[0] > 0.0 {
                0.0
            } else {
                cursor_pos[0]
            };

            let new_y = if cursor_pos[1] == 0.0 && mouse_motion[1] < 0.0 {
                win_phys_resolution[1]
            } else if cursor_pos[1] >= win_phys_resolution[1] - 1.0 && mouse_motion[1] > 0.0 {
                0.0
            } else {
                cursor_pos[1]
            };

            window.set_physical_cursor_position(Some(Vec2::new(new_x, new_y).into()));
        };
    } else {
        window.cursor.grab_mode = CursorGrabMode::None;
    }

    let scroll: f32 = camera_input
        .mouse_wheel
        .iter()
        .fold(0.0, |acc, evt| match evt.unit {
            MouseScrollUnit::Line => acc + evt.y,
            MouseScrollUnit::Pixel => acc + evt.y / 32.0,
        });

    let (mut cam, mut transform, proj) = camera.get_single_mut().unwrap();

    if mouse_motion.length_squared() > 0.0 {
        match orbit_or_pan {
            OrbitOrPan::Orbit => {
                let yaw = Quat::from_rotation_y(ORBIT_SENSITIVITY * -mouse_motion.x);
                let pitch = Quat::from_rotation_x(ORBIT_SENSITIVITY * -mouse_motion.y);

                // Rotate around global Y-axis, then local X-axis.
                transform.rotation = (yaw * transform.rotation) * pitch;
            }

            OrbitOrPan::Pan => {
                // TODO: pan by raycasting against the plane parallel to the camera which contains
                // the focus point.

                let mut pan = mouse_motion;

                if let Projection::Perspective(proj) = proj {
                    pan *= Vec2::new(proj.fov * proj.aspect_ratio, proj.fov);
                }

                let right = transform.rotation * Vec3::X * -pan.x;
                let up = transform.rotation * Vec3::Y * pan.y;
                let translation = (right + up) * cam.distance;

                cam.focus += PAN_SENSITIVITY * translation;
            }
        }
    }

    // TODO: zoom smoothly.
    if scroll.abs() > 0.0 {
        cam.distance -= scroll * cam.distance * 0.2;
        cam.distance = cam.distance.clamp(0.05, 4096.0);
    }

    transform.translation = cam.focus + transform.rotation * Vec3::new(0.0, 0.0, cam.distance);

    let mut solid_light = solid_shading_light.get_single_mut().unwrap();
    *solid_light = Transform::from_rotation(transform.rotation);
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

//         cg3::zorder::sort_z_order(&mut v);

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
