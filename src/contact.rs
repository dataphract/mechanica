//! Contact

use std::ptr;

use arrayvec::ArrayVec;
use bevy::prelude::{Color, Gizmos};
use glam::{Vec3, Vec3A};

use crate::{
    gjk,
    hull::{self, Hull},
    Capsule, Isometry, Plane, Segment, SegmentClosestPair, Sphere,
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
    let a1 = iso_a * Vec3A::from(capsule_a.segment.a);
    let b1 = iso_a * Vec3A::from(capsule_a.segment.b);
    let a2 = iso_b * Vec3A::from(capsule_b.segment.a);
    let b2 = iso_b * Vec3A::from(capsule_b.segment.b);

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
    let s1 = Segment::new(a1.into(), b1.into());
    let s2 = Segment::new(a2.into(), b2.into());

    let pair = s1.closest_point_to_segment(&s2);
    let inner_a = Vec3A::from(pair.first.point);
    let inner_b = Vec3A::from(pair.second.point);

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
    gizmos: &mut Gizmos,
) -> Contact {
    if let Some((on_segment, on_hull)) = gjk::closest(&capsule.segment, iso_a, hull, iso_b, gizmos)
    {
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

    let capsule_a = iso_a * Vec3A::from(capsule.segment.a);
    let capsule_b = iso_a * Vec3A::from(capsule.segment.b);

    let mut best_feature = None;

    // Find the face normal that minimizes the penetration depth.
    for face in hull.iter_faces() {
        // Walk the perimeter of the face and clip the capsule segment.
        let first_edge = face.first_edge();
        let mut edge = first_edge;

        let mut clip_a = capsule_a;
        let mut clip_b = capsule_b;

        loop {
            let clip_plane = iso_b * edge.clip_plane();

            clip_a = clip_plane.clamp_point(clip_a);
            clip_b = clip_plane.clamp_point(clip_b);

            edge = edge.next();
            if edge == first_edge {
                break;
            }
        }

        let plane = iso_b * face.plane();

        let a_depth = -plane.distance_to_point(clip_a);
        let b_depth = -plane.distance_to_point(clip_b);

        if a_depth >= 0.0 {
            gizmos.ray(clip_a.into(), a_depth * plane.normal, Color::BLUE);
        }

        if b_depth >= 0.0 {
            gizmos.ray(clip_b.into(), b_depth * plane.normal, Color::BLUE);
        }

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
        let edge_a = iso_b * edge.start();
        let edge_b = iso_b * edge.end();

        // Find the closest points on the capsule segment and the edge.
        let capsule_segment = Segment::new(capsule_a.into(), capsule_b.into());
        let edge_segment = Segment::new(edge_a.into(), edge_b.into());
        let points = capsule_segment.closest_point_to_segment(&edge_segment);
        let on_cap_seg = Vec3A::from(points.first.point);
        let on_edge_seg = Vec3A::from(points.second.point);

        // Compute the candidate contact normal.
        let contact_normal = (on_cap_seg - on_edge_seg).normalize();

        // The closest point on the segment is interior to the hull if the dot product of the hull's
        // face normal and the capsule's contact normal is negative.
        if edge.face_normal().dot(contact_normal) >= 0.0 {
            continue;
        }

        let inner_depth = points.distance_squared.sqrt();
        gizmos.ray(points.second.point, contact_normal.into(), Color::GREEN);

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
    let a = iso_a * Vec3A::from(capsule.segment.a);
    let b = iso_a * Vec3A::from(capsule.segment.b);
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

/// Computes the contact between two hulls.
fn contact_hull_hull(hull_a: &Hull, iso_a: Isometry, hull_b: &Hull, iso_b: Isometry) -> Contact {
    todo!()
}

/// Computes the contact between a hull and a sphere.
pub fn contact_hull_sphere(
    hull: &Hull,
    iso_a: Isometry,
    sphere: &Sphere,
    iso_b: Isometry,
    gizmos: &mut Gizmos,
) -> Contact {
    // If the sphere center is not interior to the convex hull, GJK suffices to find the contact.
    if let Some((on_hull, center)) = gjk::closest(hull, iso_a, &sphere.center, iso_b, gizmos) {
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
    let mut min_depth = f32::INFINITY;
    let mut closest_plane = Plane::new(Vec3::X, 0.0).unwrap();
    for face in hull.iter_faces() {
        let plane = iso_a * face.plane();

        assert!(plane.normal.is_normalized());

        let depth = plane.distance_to_point(center).abs();

        if depth < min_depth {
            closest_plane = plane;
            min_depth = depth;
        }
    }

    let on_hull = closest_plane.project_point(center);

    Contact::Penetrating(Penetrating {
        points: ArrayVec::from_iter([on_hull.into()]),
        axis: closest_plane.normal,
        depth: min_depth + sphere.radius,
    })
}

/// Computes the contact between two spheres.
pub fn contact_sphere_sphere(
    sphere_a: &Sphere,
    iso_a: Isometry,
    sphere_b: &Sphere,
    iso_b: Isometry,
) -> Contact {
    let ca = iso_a * Vec3A::from(sphere_a.center);
    let cb = iso_b * Vec3A::from(sphere_b.center);
    let ab = cb - ca;

    let center_dist = ab.length();
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
