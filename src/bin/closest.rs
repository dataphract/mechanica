use std::ops::RangeInclusive;

use bevy::{
    ecs::system::SystemParam,
    input::mouse::{MouseMotion, MouseWheel},
    prelude::*,
    window::{CursorGrabMode, PrimaryWindow},
};
use bevy_egui::{
    egui::{self, emath},
    EguiContexts, EguiPlugin,
};
use bevy_infinite_grid::{InfiniteGridBundle, InfiniteGridPlugin};
use bevy_mod_picking::{prelude::*, PickableBundle};
use bevy_transform_gizmo::{GizmoPickSource, GizmoTransformable};
use cg3::{
    closest::{
        closest_capsule_capsule, closest_capsule_hull, closest_capsule_sphere, closest_hull_hull,
        closest_hull_sphere, closest_sphere_sphere,
    },
    hull::Hull,
    Capsule, Isometry, Segment, Sphere,
};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(InfiniteGridPlugin)
        .add_plugins(EguiPlugin)
        .add_plugins(bevy_mod_picking::DefaultPickingPlugins)
        .add_plugins(bevy_transform_gizmo::TransformGizmoPlugin::default())
        .add_systems(Startup, spawn.chain())
        .add_systems(Update, init_physobj)
        .add_systems(Update, init_closest_points)
        .add_systems(Update, control_camera)
        .add_systems(Update, render_ui)
        .add_systems(Update, update_physobj)
        .add_systems(Update, update_closest_points)
        .run();
}

#[derive(Component)]
struct Collider {
    shape: ColliderShape,
}

#[derive(Clone)]
enum ColliderShape {
    Sphere(Sphere),
    Capsule(Capsule),
    Hull(Hull),
}

fn spawn(mut commands: Commands) {
    commands.spawn(InfiniteGridBundle::default());

    commands
        .spawn(Camera3dBundle {
            projection: Projection::Perspective(default()),
            transform: Transform::from_xyz(5.0, 1.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
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
                radius: 1.0,
            }),
        })
        .insert(Transform::from_xyz(2.0, 1.0, 0.0))
        .id();

    let obj2 = commands
        .spawn_empty()
        .insert(PhysObj {
            shape: ColliderShape::Sphere(Sphere {
                center: Vec3::ZERO,
                radius: 1.0,
            }),
        })
        .insert(Transform::from_xyz(-2.0, 1.0, 0.0))
        .id();

    commands
        .spawn_empty()
        .insert(ClosestPoints { a: obj1, b: obj2 });
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
        // ColliderShape::Cuboid(c) => {
        //     let extents = 2.0 * c.half_extents;
        //     shape::Box::new(extents.x, extents.y, extents.z).into()
        // }
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
            material: materials.add(Color::WHITE.into()),
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
    mut objs: Query<(&PhysObj, &mut Handle<Mesh>, &mut Handle<StandardMaterial>), Changed<PhysObj>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    for (obj, mut mesh, mut mat) in objs.iter_mut() {
        *mesh = meshes.add(mesh_for_collider(&obj.shape));
    }
}

#[derive(Debug, Component)]
struct ClosestPoints {
    a: Entity,
    b: Entity,
}

#[derive(Debug, Component)]
struct ClosestPointsVis {
    a_vis: Entity,
    b_vis: Entity,
}

#[derive(Debug, Component)]
struct VisTag;

fn init_closest_points(
    mut commands: Commands,
    query: Query<Entity, (With<ClosestPoints>, Without<ClosestPointsVis>)>,
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

        let mut ec = commands.entity(ent);
        ec.insert(ClosestPointsVis { a_vis, b_vis });
    }
}

fn update_closest_points(
    query: Query<(&ClosestPoints, &ClosestPointsVis)>,
    objs: Query<(&PhysObj, &Transform), Without<VisTag>>,
    mut vis: Query<&mut Transform, With<VisTag>>,
) {
    for (closest, closest_vis) in query.iter() {
        let (a_obj, a_xf) = objs.get(closest.a).unwrap();
        let (b_obj, b_xf) = objs.get(closest.b).unwrap();
        let a_iso = Isometry::from_transform(*a_xf);
        let b_iso = Isometry::from_transform(*b_xf);

        let opt_points = match (&a_obj.shape, &b_obj.shape) {
            (ColliderShape::Capsule(c1), ColliderShape::Capsule(c2)) => {
                closest_capsule_capsule(c1, a_iso, c2, b_iso)
            }

            (ColliderShape::Capsule(c), ColliderShape::Hull(h)) => {
                closest_capsule_hull(c, a_iso, h, b_iso)
            }

            (ColliderShape::Hull(h), ColliderShape::Capsule(c)) => {
                closest_capsule_hull(c, b_iso, h, a_iso).map(|(b, a)| (a, b))
            }

            (ColliderShape::Sphere(s1), ColliderShape::Sphere(s2)) => {
                closest_sphere_sphere(s1, a_iso, s2, b_iso)
            }

            (ColliderShape::Capsule(c), ColliderShape::Sphere(s)) => {
                closest_capsule_sphere(c, a_iso, s, b_iso)
            }

            (ColliderShape::Sphere(s), ColliderShape::Capsule(c)) => {
                closest_capsule_sphere(c, b_iso, s, a_iso).map(|(b, a)| (a, b))
            }

            (ColliderShape::Hull(h), ColliderShape::Sphere(s)) => {
                closest_hull_sphere(h, a_iso, s, b_iso)
            }

            (ColliderShape::Sphere(s), ColliderShape::Hull(h)) => {
                closest_hull_sphere(h, b_iso, s, a_iso).map(|(b, a)| (a, b))
            }

            (ColliderShape::Hull(h1), ColliderShape::Hull(h2)) => {
                closest_hull_hull(h1, a_iso, h2, b_iso)
            }
        };

        let Some((on_a, on_b)) = opt_points else {
            continue;
        };

        let mut a_xf = vis.get_mut(closest_vis.a_vis).unwrap();
        a_xf.translation = on_a;
        let mut b_xf = vis.get_mut(closest_vis.b_vis).unwrap();
        b_xf.translation = on_b;
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum ShapeSel {
    Sphere,
    Capsule,
    Hull,
    // Box,
}

fn label_slider<N: emath::Numeric>(
    ui: &mut egui::Ui,
    label: &str,
    var: &mut N,
    range: RangeInclusive<N>,
) {
    ui.horizontal(|ui| {
        ui.add(egui::widgets::Label::new(label));
        ui.add(egui::widgets::Slider::new(var, range));
    });
}

fn render_ui(
    mut egui_cx: EguiContexts,
    selected: Query<(Entity, &PickSelection), With<PhysObj>>,
    mut objs: Query<&mut PhysObj>,
    mut last_selected: Local<Option<Entity>>,
) {
    for (ent, sel) in selected.iter() {
        if sel.is_selected {
            last_selected.replace(ent);
            break;
        }
    }

    let mut obj = match last_selected.and_then(|sel| objs.get_mut(sel).ok()) {
        Some(o) => o,
        None => {
            egui::Window::new("No objects selected").show(egui_cx.ctx_mut(), |ui| {
                ui.label("Select an object to modify it!");
            });
            return;
        }
    };

    let mut shape_sel = match obj.shape {
        ColliderShape::Sphere(_) => ShapeSel::Sphere,
        ColliderShape::Capsule(_) => ShapeSel::Capsule,
        ColliderShape::Hull(_) => ShapeSel::Hull,
        // ColliderShape::Cuboid(_) => ShapeSel::Box,
        // ColliderShape::Polyhedron(_) => ShapeSel::Polyhedron,
    };

    egui::Window::new("Physics object").show(egui_cx.ctx_mut(), |ui| {
        egui::ComboBox::from_label("Shape")
            .selected_text(format!("{shape_sel:?}"))
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut shape_sel, ShapeSel::Sphere, "Sphere");
                ui.selectable_value(&mut shape_sel, ShapeSel::Capsule, "Capsule");
                ui.selectable_value(&mut shape_sel, ShapeSel::Hull, "Hull");
                // ui.selectable_value(&mut shape_sel, ShapeSel::Box, "Box");
            });

        ui.separator();

        match shape_sel {
            ShapeSel::Sphere => {
                let mut radius = if let ColliderShape::Sphere(s) = &obj.shape {
                    s.radius
                } else {
                    1.0
                };

                label_slider(ui, "Radius", &mut radius, 0.1..=5.0);

                if !matches!(&obj.shape, ColliderShape::Sphere(_)) {
                    obj.shape = ColliderShape::Sphere(Sphere {
                        center: Vec3::ZERO,
                        radius,
                    });
                }
            } // ShapeSel::Box => {
            //     if !matches!(&obj.shape, ColliderShape::Cuboid(_)) {
            //         obj.shape = ColliderShape::Cuboid(Obb {
            //             world_center: Vec3::ZERO,
            //             local_to_world: Mat3::IDENTITY,
            //             half_extents: Vec3::splat(1.0),
            //         });
            //     }
            // }
            ShapeSel::Capsule => {
                if !matches!(&obj.shape, ColliderShape::Capsule(_)) {
                    obj.shape = ColliderShape::Capsule(Capsule {
                        segment: Segment {
                            a: 0.5 * Vec3::Y,
                            b: -0.5 * Vec3::Y,
                        },
                        radius: 0.5,
                    });
                }
            }

            ShapeSel::Hull => {
                if !matches!(&obj.shape, ColliderShape::Hull(_)) {
                    obj.shape = ColliderShape::Hull(Hull::tetrahedron());
                }
            }
        }
    });
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

    let orbit_button = MouseButton::Middle;

    enum OrbitOrPan {
        Orbit,
        Pan,
    }

    let mut orbit_or_pan = OrbitOrPan::Orbit;
    let mut mouse_motion = Vec2::ZERO;

    if camera_input.mouse_button.pressed(orbit_button) {
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

    let scroll: f32 = camera_input.mouse_wheel.iter().map(|evt| evt.y).sum();

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
                // TODO: Mouse acceleration can cause the cursor to move relative to the focal
                // point. Blender avoids this somehow.

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
