use std::{hash::Hash, ops::RangeInclusive};

use bevy::prelude::*;
use bevy_egui::{
    egui::{self, emath},
    EguiContexts,
};
use glam::{Vec3, Vec3A};
use mechanica::{collider::ColliderShape, hull::Hull, Capsule, Segment, Sphere};

use crate::{ObjA, ObjB, PhysObj};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum ShapeSel {
    Sphere,
    Capsule,
    Hull,
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

#[derive(Copy, Clone, Debug, PartialEq)]
struct SphereParams {
    radius: f32,
}

impl Default for SphereParams {
    fn default() -> Self {
        Self { radius: 0.5 }
    }
}

impl SphereParams {
    fn ui(&mut self, ui: &mut egui::Ui, id_source: impl Hash) {
        egui::Grid::new(id_source).show(ui, |ui| {
            label_slider(ui, "Radius", &mut self.radius, 0.1..=2.0);
            ui.end_row();
        });
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
struct CapsuleParams {
    length: f32,
    radius: f32,
}

impl Default for CapsuleParams {
    fn default() -> Self {
        Self {
            length: 1.0,
            radius: 0.5,
        }
    }
}

impl CapsuleParams {
    fn ui(&mut self, ui: &mut egui::Ui, id_source: impl Hash) {
        egui::Grid::new(id_source).show(ui, |ui| {
            label_slider(ui, "Length", &mut self.length, 0.1..=2.0);
            ui.end_row();

            label_slider(ui, "Radius", &mut self.radius, 0.1..=2.0);
            ui.end_row();
        });
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
struct BoxParams {
    dims: Vec3,
}

impl Default for BoxParams {
    fn default() -> Self {
        Self {
            dims: Vec3::splat(1.0),
        }
    }
}

impl BoxParams {
    fn ui(&mut self, ui: &mut egui::Ui, id_source: impl Hash) {
        egui::Grid::new(id_source).show(ui, |ui| {
            label_slider(ui, "Width", &mut self.dims[0], 0.1..=2.0);
            ui.end_row();

            label_slider(ui, "Height", &mut self.dims[1], 0.1..=2.0);
            ui.end_row();

            label_slider(ui, "Depth", &mut self.dims[2], 0.1..=2.0);
            ui.end_row();
        });
    }
}

#[derive(Debug, Default)]
pub struct ShapeParams {
    sphere: SphereParams,
    capsule: CapsuleParams,
    box_: BoxParams,
}

fn update_obj(ui: &mut egui::Ui, obj: &mut PhysObj, params: &mut ShapeParams, name: &str) {
    let mut shape_sel = match obj.shape {
        ColliderShape::Sphere(_) => ShapeSel::Sphere,
        ColliderShape::Capsule(_) => ShapeSel::Capsule,
        ColliderShape::Hull(_) => ShapeSel::Hull,
    };

    egui::ComboBox::new((name, "shape"), "Shape")
        .selected_text(format!("{shape_sel:?}"))
        .show_ui(ui, |ui| {
            ui.selectable_value(&mut shape_sel, ShapeSel::Sphere, "Sphere");
            ui.selectable_value(&mut shape_sel, ShapeSel::Capsule, "Capsule");
            ui.selectable_value(&mut shape_sel, ShapeSel::Hull, "Hull");
        });

    ui.separator();

    match shape_sel {
        ShapeSel::Sphere => {
            let old_params = params.sphere;

            params.sphere.ui(ui, (name, "sphere"));

            if !matches!(&obj.shape, ColliderShape::Sphere(_)) || params.sphere != old_params {
                obj.shape = ColliderShape::Sphere(Sphere {
                    center: Vec3::ZERO,
                    radius: params.sphere.radius,
                });
            }
        }

        ShapeSel::Capsule => {
            let old_params = params.capsule;

            params.capsule.ui(ui, (name, "capsule"));

            if params.capsule != old_params || !matches!(&obj.shape, ColliderShape::Capsule(_)) {
                obj.shape = ColliderShape::Capsule(Capsule {
                    segment: Segment {
                        a: 0.5 * params.capsule.length * Vec3A::Y,
                        b: -0.5 * params.capsule.length * Vec3A::Y,
                    },
                    radius: params.capsule.radius,
                });
            }
        }

        ShapeSel::Hull => {
            let old_params = params.box_;

            params.box_.ui(ui, (name, "box"));

            if params.box_ != old_params || !matches!(&obj.shape, ColliderShape::Hull(_)) {
                //obj.shape = ColliderShape::Hull(Hull::tetrahedron(2.0));
                obj.shape = ColliderShape::Hull(Hull::cuboid(params.box_.dims));
            }
        }
    }
}

pub fn render_ui(
    mut egui_cx: EguiContexts,
    mut query_a: Query<&mut PhysObj, (With<ObjA>, Without<ObjB>)>,
    mut params_a: Local<ShapeParams>,
    mut query_b: Query<&mut PhysObj, (With<ObjB>, Without<ObjA>)>,
    mut params_b: Local<ShapeParams>,
) {
    let ctx = egui_cx.ctx_mut();
    egui::TopBottomPanel::bottom("bottom_panel").show(ctx, |ui| {
        ui.label("Contact testbed");
        ui.separator();

        ui.vertical(|ui| {
            let (left, right) = ui.max_rect().split_left_right_at_fraction(0.5);

            ui.allocate_ui_at_rect(left, |ui| {
                let mut obj_a = query_a.get_single_mut().unwrap();
                ui.horizontal(|ui| {
                    update_obj(ui, &mut obj_a, &mut params_a, "obj_a");
                });
            });

            ui.allocate_ui_at_rect(right, |ui| {
                let mut obj_b = query_b.get_single_mut().unwrap();
                ui.horizontal(|ui| {
                    update_obj(ui, &mut obj_b, &mut params_b, "obj_b");
                });
            });
        });
    });
}
