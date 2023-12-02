use bevy::{
    app::PluginGroupBuilder,
    ecs::system::SystemParam,
    input::mouse::{MouseMotion, MouseScrollUnit, MouseWheel},
    prelude::*,
    window::{CursorGrabMode, PrimaryWindow},
};
use bevy_egui::EguiPlugin;
use bevy_infinite_grid::{InfiniteGridBundle, InfiniteGridPlugin};
use bevy_mod_picking::prelude::{GlobalHighlight, RaycastPickCamera};
use bevy_transform_gizmo::{GizmoPickSource, TransformGizmoPlugin};

pub struct TestbenchPlugins;

impl PluginGroup for TestbenchPlugins {
    fn build(self) -> PluginGroupBuilder {
        PluginGroupBuilder::start::<Self>()
            .add(InfiniteGridPlugin)
            .add(EguiPlugin)
            .add(bevy_mod_picking::picking_core::CorePlugin)
            .add(bevy_mod_picking::picking_core::InteractionPlugin)
            .add(bevy_mod_picking::input::InputPlugin)
            .add(bevy_mod_picking::highlight::DefaultHighlightingPlugin)
            .add(bevy_mod_picking::selection::SelectionPlugin)
            .add(bevy_mod_picking::backends::raycast::RaycastBackend)
            .add(bevy_mod_picking::backends::bevy_ui::BevyUiBackend)
            .add(TransformGizmoPlugin::default())
            .add(TestbenchPlugin)
    }

    fn name() -> String {
        "TestbenchPlugins".into()
    }
}

pub struct TestbenchPlugin;

impl Plugin for TestbenchPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup)
            .add_systems(PostStartup, post_startup)
            .add_systems(Update, control_camera);
    }
}

#[derive(Component)]
pub struct SolidShadingLight;

#[derive(Component)]
pub struct EditorCamera {
    pub focus: Vec3,
    pub distance: f32,
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

#[derive(SystemParam)]
pub struct CameraInput<'w, 's> {
    mouse_motion: EventReader<'w, 's, MouseMotion>,

    mouse_wheel: EventReader<'w, 's, MouseWheel>,

    key_input: Res<'w, Input<KeyCode>>,
    mouse_button: Res<'w, Input<MouseButton>>,
}

pub fn control_camera(
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
