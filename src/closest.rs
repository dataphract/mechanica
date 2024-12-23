use glam::{Vec3, Vec3A};

use crate::{gjk, hull::Hull, Capsule, Isometry, Segment, Sphere};

/// Computes the closest points between two capsules.
pub fn closest_capsule_capsule(
    capsule_a: &Capsule,
    iso_a: Isometry,
    capsule_b: &Capsule,
    iso_b: Isometry,
) -> Option<(Vec3, Vec3)> {
    let a1 = iso_a * Vec3A::from(capsule_a.segment.a);
    let b1 = iso_a * Vec3A::from(capsule_a.segment.b);
    let a2 = iso_b * Vec3A::from(capsule_b.segment.a);
    let b2 = iso_b * Vec3A::from(capsule_b.segment.b);

    let s1 = Segment::new(a1.into(), b1.into());
    let s2 = Segment::new(a2.into(), b2.into());

    let pair = s1.closest_point_to_segment(&s2);
    let on_a = Vec3A::from(pair.first.point);
    let on_b = Vec3A::from(pair.second.point);

    let to_b = on_b - on_a;
    let dist = to_b.length();

    if dist <= capsule_a.radius + capsule_b.radius {
        return None;
    }

    let dir = to_b / dist;
    let on_a = capsule_a.radius * dir + on_a;
    let on_b = -capsule_b.radius * dir + on_b;

    Some((on_a.into(), on_b.into()))
}

/// Computes the closest points between a capsule and a convex hull.
pub fn closest_capsule_hull(
    capsule: &Capsule,
    iso_a: Isometry,
    hull: &Hull,
    iso_b: Isometry,
) -> Option<(Vec3, Vec3)> {
    let (on_segment, on_hull) = gjk::closest(&capsule.segment, iso_a, hull, iso_b)?;

    let to_hull = on_hull - on_segment;
    let length = to_hull.length();

    if length <= capsule.radius {
        return None;
    }

    let dir = to_hull / length;

    Some(((capsule.radius * dir + on_segment).into(), on_hull.into()))
}

/// Computes the closest points between a capsule and a sphere.
pub fn closest_capsule_sphere(
    capsule: &Capsule,
    iso_a: Isometry,
    sphere: &Sphere,
    iso_b: Isometry,
) -> Option<(Vec3, Vec3)> {
    let a = iso_a * Vec3A::from(capsule.segment.a);
    let b = iso_a * Vec3A::from(capsule.segment.b);
    let c = iso_b * Vec3A::from(sphere.center);

    let ab = b - a;
    let t = (c - a).dot(ab) / ab.length_squared();
    let t_clamp = t.clamp(0.0, 1.0);

    let on_segment = a + t_clamp * ab;
    let dir = on_segment - c;
    let dist = dir.length();
    let dir = dir / dist;

    if dist <= sphere.radius + capsule.radius {
        return None;
    }

    let on_capsule = on_segment - capsule.radius * dir;
    let on_sphere = c + sphere.radius * dir;

    Some((on_capsule.into(), on_sphere.into()))
}

/// Computes the closest points between two convex hulls.
pub fn closest_hull_hull(
    hull_a: &Hull,
    iso_a: Isometry,
    hull_b: &Hull,
    iso_b: Isometry,
) -> Option<(Vec3, Vec3)> {
    let (on_a, on_b) = gjk::closest(hull_a, iso_a, hull_b, iso_b)?;
    Some((on_a.into(), on_b.into()))
}

/// Computes the closest points between a convex hull and a sphere.
pub fn closest_hull_sphere(
    hull: &Hull,
    iso_a: Isometry,
    sphere: &Sphere,
    iso_b: Isometry,
) -> Option<(Vec3, Vec3)> {
    let (on_hull, center) = gjk::closest(hull, iso_a, &sphere.center, iso_b)?;

    let to_hull = on_hull - center;
    let length = to_hull.length();

    if length <= sphere.radius {
        return None;
    }

    let dir = to_hull / length;

    Some((on_hull.into(), (sphere.radius * dir + center).into()))
}

/// Computes the closest points between two spheres.
pub fn closest_sphere_sphere(
    sphere_a: &Sphere,
    iso_a: Isometry,
    sphere_b: &Sphere,
    iso_b: Isometry,
) -> Option<(Vec3, Vec3)> {
    let ca = iso_a * Vec3A::from(sphere_a.center);
    let cb = iso_b * Vec3A::from(sphere_b.center);
    let ab = cb - ca;

    if ab.length() <= sphere_a.radius + sphere_b.radius {
        // Spheres are intersecting.
        return None;
    }

    let dir = ab.try_normalize()?;

    let on_a = ca + sphere_a.radius * dir;
    let on_b = cb - sphere_b.radius * dir;

    Some((on_a.into(), on_b.into()))
}
