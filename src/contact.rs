//! Computation of contact manifolds between convex sets.

use std::iter;

use arrayvec::ArrayVec;
use glam::{Vec3, Vec3A};

use crate::{
    collider::ColliderShape,
    gjk::{self, Support},
    glam_ext::Vec3AExt,
    hull::{Edge, Face, Hull},
    Capsule, Isometry, Line, Plane, Segment, Sphere,
};

/// The result of a collision test between two colliders.
pub enum Contact {
    /// The colliders are disjoint.
    Disjoint(Disjoint),
    /// The colliders are penetrating.
    Penetrating(Penetrating),
}

impl Contact {
    pub fn reverse(self) -> Contact {
        match self {
            Contact::Disjoint(d) => Contact::Disjoint(d.reverse()),
            Contact::Penetrating(p) => Contact::Penetrating(p.reverse()),
        }
    }
}

/// The result of a collision test between two disjoint colliders.
pub struct Disjoint {
    /// The closest point on the first collider to the second collider.
    pub on_a: Vec3,
    /// The closest point on the second collider to the first collider.
    pub on_b: Vec3,
    /// The shortest distance between the two colliders.
    pub dist: f32,
}

impl Disjoint {
    pub fn reverse(self) -> Disjoint {
        Disjoint {
            on_a: self.on_b,
            on_b: self.on_a,
            ..self
        }
    }
}

/// The result of a collision test between two penetrating colliders.
#[derive(Debug)]
pub struct Penetrating {
    pub points: ArrayVec<Vec3, 4>,
    /// The axis of minimum penetration between the two colliders.
    pub axis: Vec3,
    /// The penetration depth along `self.axis`.
    pub depth: f32,
}

impl Penetrating {
    pub fn reverse(self) -> Penetrating {
        Penetrating {
            axis: -self.axis,
            ..self
        }
    }
}

/// Computes the contact between two capsules.
pub fn contact_capsule_capsule(
    capsule_a: &Capsule,
    iso_a: Isometry,
    capsule_b: &Capsule,
    iso_b: Isometry,
) -> Contact {
    let a1 = iso_a * capsule_a.segment.a;
    let b1 = iso_a * capsule_a.segment.b;
    let a2 = iso_b * capsule_b.segment.a;
    let b2 = iso_b * capsule_b.segment.b;

    let radii = capsule_a.radius + capsule_b.radius;

    let v1 = b1 - a1;
    let v2 = b2 - a2;

    // Check to see if the segments are parallel.
    let cos = v1.dot(v2) / (v1.length_squared() * v2.length_squared()).sqrt();
    if cos.abs() > 1.0 - f32::EPSILON {
        // If segments are parallel, find the parts that overlap.
        let dir = v1.normalize();

        // Sort the endpoints of both segments along the direction vector.
        let ta1 = a1.dot(dir);
        let tb1 = b1.dot(dir);
        let ta2 = a2.dot(dir);
        let tb2 = b2.dot(dir);

        let (min1, tmin1, max1, tmax1) = if ta1 < tb1 {
            (a1, ta1, b1, tb1)
        } else {
            (b1, tb1, a1, ta1)
        };

        let (min2, tmin2, max2, tmax2) = if ta2 < tb2 {
            (a2, ta2, b2, tb2)
        } else {
            (b2, tb2, a2, ta2)
        };

        let collide_disjoint = |inner_a: Vec3A, inner_b: Vec3A| {
            let to_b = inner_b - inner_a;
            let inner_dist = to_b.length();
            let axis = to_b / inner_dist;

            if inner_dist > radii {
                Contact::Disjoint(Disjoint {
                    on_a: (inner_a + capsule_a.radius * axis).into(),
                    on_b: (inner_b - capsule_b.radius * axis).into(),
                    dist: inner_dist - radii,
                })
            } else {
                Contact::Penetrating(Penetrating {
                    points: ArrayVec::from_iter([Vec3::lerp(
                        inner_a.into(),
                        inner_b.into(),
                        capsule_b.radius / radii,
                    )]),
                    axis: axis.into(),
                    depth: radii - inner_dist,
                })
            }
        };

        // If the projections of the segments on the direction vector are disjoint or tangent, the
        // capsules are either disjoint or only penetrating at the ends.
        if tmin1 >= tmax2 {
            return collide_disjoint(min1, max2);
        } else if tmin2 >= tmax1 {
            return collide_disjoint(min2, max1);
        }

        // The projections of the segments are not disjoint, so there are two contact points. Clip
        // both segments based on the closest endpoints.
        let (cmin1, cmin2) = if tmin1 > tmin2 {
            (min1, min2 + (tmin1 - tmin2) * dir)
        } else {
            (min1 + (tmin2 - tmin1) * dir, min2)
        };

        let delta = cmin2 - cmin1;
        let inner_dist = delta.length();
        let dist = inner_dist - radii;
        let axis = delta / inner_dist;

        if dist > 0.0 {
            return Contact::Disjoint(Disjoint {
                on_a: (cmin1 + capsule_a.radius * axis).into(),
                on_b: (cmin2 - capsule_b.radius * axis).into(),
                dist,
            });
        }

        let (cmax1, cmax2) = if tmax1 < tmax2 {
            (max1, max2 + (tmax1 - tmax2) * dir)
        } else {
            (max1 + (tmax2 - tmax1) * dir, max2)
        };

        let factor = capsule_b.radius / (capsule_a.radius + capsule_b.radius);
        let contact_min = Vec3A::lerp(cmin1, cmin2, factor);
        let contact_max = Vec3A::lerp(cmax1, cmax2, factor);

        return Contact::Penetrating(Penetrating {
            points: ArrayVec::from_iter([contact_min.into(), contact_max.into()]),
            axis: axis.into(),
            depth: dist.abs(),
        });
    }

    // The segments aren't parallel, so they're either penetrating at a single point or disjoint.
    let s1 = Segment::new(a1, b1);
    let s2 = Segment::new(a2, b2);

    let pair = s1.closest_point_to_segment(&s2);
    let inner_a = pair.first.point;
    let inner_b = pair.second.point;

    let to_b = inner_b - inner_a;
    let inner_dist = to_b.length();

    if inner_dist == 0.0 {
        // The inner segments are intersecting, so an axis of minimum penetration can't be
        // calculated. Just use the cross product.

        return Contact::Penetrating(Penetrating {
            points: ArrayVec::from_iter([inner_a.into()]),
            axis: (b1 - a1).cross(b2 - a2).into(),
            depth: radii,
        });
    }

    let axis = to_b / inner_dist;
    let dist = inner_dist - radii;

    let on_a = inner_a + capsule_a.radius * axis;
    let on_b = inner_b - capsule_b.radius * axis;

    if dist > 0.0 {
        Contact::Disjoint(Disjoint {
            on_a: on_a.into(),
            on_b: on_b.into(),
            dist,
        })
    } else {
        Contact::Penetrating(Penetrating {
            points: ArrayVec::from_iter([(0.5 * (on_a + on_b)).into()]),
            axis: axis.into(),
            depth: dist.abs(),
        })
    }
}

/// Computes the contact between a capsule and a hull.
pub fn contact_capsule_hull(
    capsule: &Capsule,
    iso_a: Isometry,
    hull: &Hull,
    iso_b: Isometry,
) -> Contact {
    if let Some((on_segment, on_hull)) = gjk::closest(&capsule.segment, iso_a, hull, iso_b) {
        // If GJK succeeds, the colliders are either disjoint or in shallow penetration.
        let to_hull = on_hull - on_segment;
        let inner_dist = to_hull.length();
        let dist = inner_dist - capsule.radius;
        let axis = to_hull / inner_dist;

        if dist <= 0.0 {
            // TODO: test for any parallel faces and generate a second contact to allow stacking.

            return Contact::Penetrating(Penetrating {
                points: ArrayVec::from_iter([on_hull.into()]),
                axis: axis.into(),
                depth: dist.abs(),
            });
        } else {
            return Contact::Disjoint(Disjoint {
                on_a: (on_segment + capsule.radius * axis).into(),
                on_b: on_hull.into(),
                dist,
            });
        }
    }

    // GJK failed, meaning the capsule segment is penetrating the hull. The axis of minimum
    // penetration is either a face normal or the cross product between the capsule segment and one
    // of the hull edges.

    enum Feature {
        Face {
            clip_a: Vec3A,
            clip_b: Vec3A,
            plane: Plane,
        },
        Edge {
            on_cap_seg: Vec3A,
            on_hull: Vec3A,
            normal: Vec3A,
        },
    }

    let mut min_depth = f32::INFINITY;

    let capsule_a = iso_a * capsule.segment.a;
    let capsule_b = iso_a * capsule.segment.b;

    let mut best_feature = None;

    // Find the face normal that minimizes the penetration depth.
    for face in hull.iter_faces() {
        // Walk the perimeter of the face and clip the capsule segment.
        let first_edge = face.first_edge();
        let mut edge = first_edge;

        let mut clip_a = capsule_a;
        let mut clip_b = capsule_b;

        loop {
            let clip_plane = edge.clip_plane(iso_b);

            clip_a = clip_plane.clamp_point(clip_a);
            clip_b = clip_plane.clamp_point(clip_b);

            edge = edge.next();
            if edge == first_edge {
                break;
            }
        }

        let plane = face.plane(iso_b);

        let a_depth = -plane.distance_to_point(clip_a);
        let b_depth = -plane.distance_to_point(clip_b);

        let face_depth = a_depth.max(b_depth);

        if face_depth < min_depth {
            best_feature = Some(Feature::Face {
                clip_a,
                clip_b,
                plane,
            });
            min_depth = face_depth;
        }
    }

    // Test whether any cross product between the capsule segment and a hull edge gives a smaller
    // penetration depth.
    for edge in hull.iter_edges() {
        let edge_a = edge.start(iso_b);
        let edge_b = edge.end(iso_b);

        // Find the closest points on the capsule segment and the edge.
        let capsule_segment = Segment::new(capsule_a, capsule_b);
        let edge_segment = Segment::new(edge_a, edge_b);
        let points = capsule_segment.closest_point_to_segment(&edge_segment);
        let on_cap_seg = points.first.point;
        let on_edge_seg = points.second.point;

        // Compute the candidate contact normal.
        let contact_normal = (on_cap_seg - on_edge_seg).normalize();

        // The closest point on the segment is interior to the hull if the dot product of the hull's
        // face normal and the capsule's contact normal is negative.
        if edge.face_normal(iso_b.rotation).dot(contact_normal) >= 0.0 {
            continue;
        }

        let inner_depth = points.distance_squared.sqrt();
        if inner_depth < min_depth {
            min_depth = inner_depth;
            best_feature = Some(Feature::Edge {
                on_cap_seg,
                on_hull: on_edge_seg,
                normal: contact_normal,
            });
        }
    }

    match best_feature.unwrap() {
        Feature::Face {
            clip_a,
            clip_b,
            plane,
        } => Contact::Penetrating(Penetrating {
            points: ArrayVec::from_iter([
                plane.project_point(clip_a).into(),
                plane.project_point(clip_b).into(),
            ]),
            axis: -plane.normal,
            depth: min_depth + capsule.radius,
        }),

        Feature::Edge {
            on_cap_seg,
            on_hull,
            normal,
        } => {
            // If the capsule is penetrating across an edge, there's a single contact point on the
            // separating axis.
            Contact::Penetrating(Penetrating {
                points: ArrayVec::from_iter([(0.5 * (on_hull + on_cap_seg)).into()]),
                axis: normal.into(),
                depth: min_depth + capsule.radius,
            })
        }
    }
}

/// Computes the contact between a capsule and a sphere.
pub fn contact_capsule_sphere(
    capsule: &Capsule,
    iso_a: Isometry,
    sphere: &Sphere,
    iso_b: Isometry,
) -> Contact {
    let a = iso_a * capsule.segment.a;
    let b = iso_a * capsule.segment.b;
    let c = iso_b * Vec3A::from(sphere.center);

    // Compute the projection of the sphere center on the capsule's inner segment.
    let ab = b - a;
    let t = (c - a).dot(ab) / ab.length_squared();
    let t_clamp = t.clamp(0.0, 1.0);
    let on_segment = a + t_clamp * ab;

    // Compute the distance from the sphere center to the capsule segment.
    let dir = c - on_segment;
    let inner_dist = dir.length();
    let radii = capsule.radius + sphere.radius;

    // If the sphere center is on the segment, a normal can't be computed, so an arbitrary normal is
    // returned. The penetration depth is the sum of the radii.
    if inner_dist == 0.0 {
        return Contact::Penetrating(Penetrating {
            // TODO: possibly add a parameter allowing a user-specified default axis, or return Err
            // here. The arbitrary choice of axis may cause unexpected behavior.
            points: ArrayVec::from_iter([on_segment.into()]),
            axis: ab.normalize().any_orthonormal_vector().into(),
            depth: radii,
        });
    }

    let axis = dir / inner_dist;
    let dist = inner_dist - radii;

    let on_a = on_segment + capsule.radius * axis;
    let on_b = c - sphere.radius * axis;

    if dist > 0.0 {
        Contact::Disjoint(Disjoint {
            on_a: on_a.into(),
            on_b: on_b.into(),
            dist,
        })
    } else {
        Contact::Penetrating(Penetrating {
            points: ArrayVec::from_iter([(0.5 * (on_a + on_b)).into()]),
            axis: axis.into(),
            depth: dist.abs(),
        })
    }
}

fn gauss_arcs_intersect(e1: Edge, iso1: Isometry, e2: Edge, iso2: Isometry) -> bool {
    // Three conditions must hold in order for the arcs to intersect:
    //
    // - A and B must lie on opposite sides of the plane through CD.
    // - C and D must lie on opposite sides of the plane through AB.
    // - A and D must lie on the same side of the plane through B and C.
    //
    // The first two conditions are true if the scalar triple products have opposite signs:
    //
    // - [c · (b ⨯ a)] * [d · (b ⨯ a)] < 0
    // - [a · (d ⨯ c)] * [b · (d ⨯ c)] < 0
    //
    // The third condition is true if the scalar triple products have the same sign:
    //
    // - [a · (c ⨯ b)] * [d · (c ⨯ b)] > 0
    //
    // As an optimization, the scalar triple product identity can be applied to the third test
    // to use terms from the first two tests:
    //
    // - [c · (b ⨯ a)] * [b · (d ⨯ c)] > 0

    let c = -e2.face_normal(iso2.rotation);
    let d = -e2.reverse().face_normal(iso2.rotation);

    let bxa = -e1.dir(iso1.rotation);

    let cba = c.dot(bxa);
    let dba = d.dot(bxa);

    if cba * dba >= 0.0 {
        return false;
    }

    let a = e1.face_normal(iso1.rotation);
    let b = e1.reverse().face_normal(iso1.rotation);

    let dxc = -e2.dir(iso2.rotation);

    let adc = a.dot(dxc);
    let bdc = b.dot(dxc);

    if adc * bdc >= 0.0 {
        return false;
    }

    cba * bdc > 0.0
}

// One round of Sutherland-Hodgman clipping.
fn clip_vert_loop(
    clip_plane: Plane,
    mut input: impl Iterator<Item = Vec3A>,
    output: &mut Vec<Vec3A>,
) {
    let first = input.next().unwrap();
    let input = input.chain(iter::once(first));

    let mut start = first;
    let mut start_inside = clip_plane.distance_to_point(start) >= 0.0;

    for end in input {
        let end_inside = clip_plane.distance_to_point(end) >= 0.0;

        if start_inside {
            output.push(start);

            if !end_inside {
                let clip_v = Line::from_points(start.into(), end.into())
                    .unwrap()
                    .intersect_plane(clip_plane)
                    .unwrap();

                output.push(clip_v);

                start_inside = false;
            }
        } else if end_inside {
            let clip_v = Line::from_points(start.into(), end.into())
                .unwrap()
                .intersect_plane(clip_plane)
                .unwrap();

            output.push(clip_v);

            start_inside = true;
        }

        start = end;
    }
}

/// Computes the contact between two hulls.
pub fn contact_hull_hull(
    hull_a: &Hull,
    iso_a: Isometry,
    hull_b: &Hull,
    iso_b: Isometry,
) -> Contact {
    if let Some((on_a, on_b)) = gjk::closest(hull_a, iso_a, hull_b, iso_b) {
        return Contact::Disjoint(Disjoint {
            on_a: on_a.into(),
            on_b: on_b.into(),
            dist: on_a.distance(on_b),
        });
    }

    enum Feature<'a> {
        Face {
            ref_face: Face<'a>,
            ref_iso: Isometry,
            inc_hull: &'a Hull,
            inc_iso: Isometry,
        },
        Edge {
            edge_a: Edge<'a>,
            edge_b: Edge<'a>,
        },
    }

    // If GJK fails, try to locate a separating axis by testing all face normals of both polygons
    // and the cross products of all edge pairs. If the hulls are penetrating along all such axes,
    // select the axis of minimum penetration and return a contact.
    let mut best_feature = None;
    let mut min_depth = f32::INFINITY;
    let mut contact_normal = Vec3::ZERO;

    // Test all face normals of A.
    for face in hull_a.iter_faces() {
        let plane = face.plane(iso_a);
        let on_b = hull_b.support(iso_b, (-plane.normal).into());

        let dist = plane.distance_to_point(on_b);
        if dist > 0.0 {
            return Contact::Disjoint(Disjoint {
                on_a: (on_b - dist * Vec3A::from(plane.normal)).into(),
                on_b: on_b.into(),
                dist,
            });
        }

        let depth = -dist;
        if depth < min_depth {
            best_feature = Some(Feature::Face {
                ref_face: face,
                ref_iso: iso_a,
                inc_hull: hull_b,
                inc_iso: iso_b,
            });
            min_depth = depth;
            contact_normal = plane.normal;
        }
    }

    // Test all face normals of B.
    for face in hull_b.iter_faces() {
        let plane = face.plane(iso_b);
        let on_a = hull_a.support(iso_a, (-plane.normal).into());

        let dist = plane.distance_to_point(on_a);
        if dist > 0.0 {
            return Contact::Disjoint(Disjoint {
                on_a: on_a.into(),
                on_b: (on_a - dist * Vec3A::from(plane.normal)).into(),
                dist: -dist,
            });
        }

        let depth = -dist;
        if depth < min_depth {
            best_feature = Some(Feature::Face {
                ref_face: face,
                ref_iso: iso_b,
                inc_hull: hull_a,
                inc_iso: iso_a,
            });
            min_depth = depth;
            contact_normal = plane.normal;
        }
    }

    // A pair of edges can only produce a separating axis if the edges build a face on the Minkowski
    // difference. An edge pair builds such a face if and only if the arcs produced by those edges
    // on the hulls' respective Gauss maps intersect.
    for edge_a in hull_a.iter_edges() {
        let centroid = Vec3A::from(edge_a.face().centroid(iso_a));

        let start_a = edge_a.start(iso_a);
        let va = edge_a.dir(iso_a.rotation);

        let out = edge_a.end(iso_a) - centroid;

        for edge_b in hull_b.iter_edges() {
            if !gauss_arcs_intersect(edge_a, iso_a, edge_b, iso_b) {
                continue;
            }

            let start_b = edge_b.start(iso_b);
            let vb = edge_b.dir(iso_b.rotation);

            if va.is_parallel_to(vb) {
                // Can't build a face on the Minkowski difference.
                continue;
            }

            let cross = va.cross(vb);

            // Orient the axis from A to B.
            let axis = if cross.dot(out) > 0.0 { cross } else { -cross };
            let sep_plane =
                Plane::from_point_normal(start_a.into(), axis.normalize().into()).unwrap();
            let dist = sep_plane.distance_to_point(start_b);

            if dist > 0.0 {
                let seg_a = edge_a.segment(iso_a);
                let seg_b = edge_b.segment(iso_b);

                let points = seg_a.closest_point_to_segment(&seg_b);
                return Contact::Disjoint(Disjoint {
                    on_a: points.first.point.into(),
                    on_b: points.second.point.into(),
                    dist,
                });
            }

            let depth = -dist;

            if depth < min_depth {
                best_feature = Some(Feature::Edge { edge_a, edge_b });
                min_depth = depth;
                contact_normal = axis.into();
            }
        }
    }

    match best_feature.unwrap() {
        Feature::Face {
            ref_face,
            ref_iso,
            inc_hull,
            inc_iso,
        } => {
            let ref_plane = ref_face.plane(ref_iso);
            let ref_normal = ref_plane.normal;

            // Find the most antiparallel face on the incident hull.
            //
            // TODO: The supporting point on B found earlier can be used here, as the incident face
            // has to be adjacent to it.
            let mut inc_face = None;
            let mut min_dot = f32::INFINITY;
            for face in inc_hull.iter_faces() {
                let normal = face.plane(inc_iso).normal;
                let dot = ref_normal.dot(normal);

                if dot < min_dot {
                    min_dot = dot;
                    inc_face = Some(face);
                }
            }

            assert!(min_dot < 0.0);

            let inc_face = inc_face.unwrap();

            // Clip the incident face using the reference face.
            let mut input = Vec::from_iter(inc_face.iter_edges().map(|edge| edge.start(inc_iso)));
            let mut output = Vec::with_capacity(2 * input.len());
            for ref_edge in ref_face.iter_edges() {
                let clip_plane = ref_edge.clip_plane(ref_iso);

                clip_vert_loop(clip_plane, input.drain(..), &mut output);
                (output, input) = (input, output);
            }

            // Discard all vertices that aren't in contact with the reference hull.
            output.extend(
                input
                    .iter()
                    .copied()
                    .filter(|&v| ref_plane.distance_to_point(v) <= 0.0),
            );

            let clipped = output;

            // Find the point on the clipped incident face with the greatest penetration depth.
            let mut deepest = None;
            let mut max_depth = f32::NEG_INFINITY;
            for (i, &v) in clipped.iter().enumerate() {
                let depth = -ref_plane.distance_to_point(v);
                if depth > max_depth {
                    max_depth = depth;
                    deepest = Some(i);
                }
            }
            let a = deepest.unwrap();

            if clipped.len() <= 4 {
                return Contact::Penetrating(Penetrating {
                    points: ArrayVec::from_iter(clipped.into_iter().map(Vec3::from)),
                    axis: contact_normal,
                    depth: max_depth,
                });
            }

            // Find the furthest point from the deepest point.
            let mut furthest = None;
            let mut max_dist = f32::NEG_INFINITY;
            for (i, &v) in clipped.iter().enumerate().filter(|&(i, _)| i != a) {
                let dist = v.distance_squared(clipped[a]);
                if dist > max_dist {
                    max_dist = dist;
                    furthest = Some(i);
                }
            }
            let b = furthest.unwrap();

            // Find two points that produce triangles with the largest area, one in each winding
            // order (negative area for CW, positive area for CCW).
            let mut c = None;
            let mut d = None;
            let mut max_area = f32::NEG_INFINITY;
            let mut min_area = f32::INFINITY;
            for (i, &v) in clipped
                .iter()
                .enumerate()
                .filter(|&(i, _)| i != a && i != b)
            {
                let va = clipped[a] - v;
                let vb = clipped[b] - v;

                let area = va.cross(vb).dot(ref_normal.into());
                if area > max_area {
                    max_area = area;
                    c = Some(i);
                }

                if area < min_area {
                    min_area = area;
                    d = Some(i);
                }
            }
            let (c, d) = (c.unwrap(), d.unwrap());

            Contact::Penetrating(Penetrating {
                points: ArrayVec::from_iter(
                    [a, b, c, d].map(|i| Vec3::from(ref_plane.project_point(clipped[i]))),
                ),

                // TODO: do this via pointer comparison (i.e. is inc_hull == hull_b)?
                axis: if ref_plane.normal.dot(contact_normal) > 0.0 {
                    contact_normal
                } else {
                    -contact_normal
                },

                depth: max_depth,
            })
        }

        Feature::Edge {
            edge_a: on_a,
            edge_b: on_b,
        } => {
            let seg_a = on_a.segment(iso_a);

            let seg_b = on_b.segment(iso_b);

            let points = seg_a.closest_point_to_segment(&seg_b);
            let mid = 0.5 * (points.first.point + points.second.point);

            Contact::Penetrating(Penetrating {
                points: ArrayVec::from_iter([mid.into()]),
                axis: contact_normal,
                depth: min_depth,
            })
        }
    }
}

/// Computes the contact between a hull and a sphere.
pub fn contact_hull_sphere(
    hull: &Hull,
    iso_a: Isometry,
    sphere: &Sphere,
    iso_b: Isometry,
) -> Contact {
    // If the sphere center is not interior to the convex hull, GJK suffices to find the contact.
    if let Some((on_hull, center)) = gjk::closest(hull, iso_a, &sphere.center, iso_b) {
        let to_center = center - on_hull;
        let inner_dist = to_center.length();
        let axis = to_center / inner_dist;
        let dist = inner_dist - sphere.radius;
        let on_sphere = center - sphere.radius * axis;

        if dist <= 0.0 {
            return Contact::Penetrating(Penetrating {
                points: ArrayVec::from_iter([(0.5 * (on_hull + on_sphere)).into()]),
                axis: axis.into(),
                depth: dist.abs(),
            });
        } else {
            return Contact::Disjoint(Disjoint {
                on_a: on_hull.into(),
                on_b: on_sphere.into(),
                dist,
            });
        }
    }

    let center = iso_b * Vec3A::from(sphere.center);

    // Find the face closest to the sphere center.
    //
    // Note that GJK can fail even if the sphere center is outside the hull due to numerical
    // instability, so the signed distance from a face to the sphere center may be positive.
    let mut max_dist = f32::NEG_INFINITY;
    let mut closest_plane = None;
    for face in hull.iter_faces() {
        let plane = face.plane(iso_a);

        assert!(plane.normal.is_normalized());

        let dist = plane.distance_to_point(center);

        if dist > max_dist {
            closest_plane = Some(plane);
            max_dist = dist;
        }
    }

    let closest_plane = closest_plane.unwrap();
    let axis = closest_plane.normal;

    let on_hull = closest_plane.project_point(center);
    let on_sphere = center - sphere.radius * Vec3A::from(axis);

    if max_dist > 0.0 {
        Contact::Penetrating(Penetrating {
            points: ArrayVec::from_iter([(0.5 * (on_hull + on_sphere)).into()]),
            axis,
            depth: sphere.radius - max_dist,
        })
    } else {
        Contact::Penetrating(Penetrating {
            points: ArrayVec::from_iter([on_hull.into()]),
            axis,
            depth: -max_dist + sphere.radius,
        })
    }
}

/// Computes the contact between two spheres.
pub fn contact_sphere_sphere(
    sphere_a: &Sphere,
    iso_a: Isometry,
    sphere_b: &Sphere,
    iso_b: Isometry,
) -> Contact {
    assert!(!iso_a.translation.is_nan());
    assert!(!iso_b.translation.is_nan());

    let ca = iso_a * Vec3A::from(sphere_a.center);
    let cb = iso_b * Vec3A::from(sphere_b.center);
    let ab = cb - ca;

    let center_dist = ab.length();
    assert!(!center_dist.is_nan());

    if center_dist == 0.0 {
        return Contact::Penetrating(Penetrating {
            // TODO: possibly add a parameter allowing a user-specified default axis, or return Err
            // here. The arbitrary choice of axis may cause unexpected behavior.
            points: ArrayVec::from_iter([ca.into()]),
            axis: Vec3::Y,
            depth: sphere_a.radius + sphere_b.radius,
        });
    }

    let dist = center_dist - (sphere_a.radius + sphere_b.radius);
    let axis = ab / center_dist;

    assert!(!axis.is_nan());

    let on_a = ca + sphere_a.radius * axis;
    let on_b = cb - sphere_b.radius * axis;

    if dist > 0.0 {
        Contact::Disjoint(Disjoint {
            on_a: on_a.into(),
            on_b: on_b.into(),
            dist,
        })
    } else {
        Contact::Penetrating(Penetrating {
            points: ArrayVec::from_iter([(0.5 * (on_a + on_b)).into()]),
            axis: axis.into(),
            depth: dist.abs(),
        })
    }
}

pub fn contact_collider_collider(
    collider_a: &ColliderShape,
    iso_a: Isometry,
    collider_b: &ColliderShape,
    iso_b: Isometry,
) -> Contact {
    match (collider_a, collider_b) {
        (ColliderShape::Capsule(c1), ColliderShape::Capsule(c2)) => {
            contact_capsule_capsule(c1, iso_a, c2, iso_b)
        }

        (ColliderShape::Capsule(c), ColliderShape::Hull(h)) => {
            contact_capsule_hull(c, iso_a, h, iso_b)
        }

        (ColliderShape::Hull(h), ColliderShape::Capsule(c)) => {
            contact_capsule_hull(c, iso_b, h, iso_a).reverse()
        }

        (ColliderShape::Sphere(s1), ColliderShape::Sphere(s2)) => {
            contact_sphere_sphere(s1, iso_a, s2, iso_b)
        }

        (ColliderShape::Capsule(c), ColliderShape::Sphere(s)) => {
            contact_capsule_sphere(c, iso_a, s, iso_b)
        }

        (ColliderShape::Sphere(s), ColliderShape::Capsule(c)) => {
            contact_capsule_sphere(c, iso_b, s, iso_a).reverse()
        }

        (ColliderShape::Hull(h), ColliderShape::Sphere(s)) => {
            contact_hull_sphere(h, iso_a, s, iso_b)
        }

        (ColliderShape::Sphere(s), ColliderShape::Hull(h)) => {
            contact_hull_sphere(h, iso_b, s, iso_a).reverse()
        }

        (ColliderShape::Hull(h1), ColliderShape::Hull(h2)) => {
            contact_hull_hull(h1, iso_a, h2, iso_b)
        }
    }
}
