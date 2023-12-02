use glam::{Quat, Vec3, Vec3A};

use crate::Isometry;

/// An axis-aligned bounding box.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Aabb {
    origin: Vec3,
    half_extents: Vec3,
}

impl Aabb {
    /// Constructs an AABB given its origin and half-extents (or radii) along each axis.
    pub fn new(origin: Vec3, half_extents: Vec3) -> Aabb {
        Aabb {
            origin,
            half_extents,
        }
    }

    pub fn of_vertices<I>(vertices: I) -> Aabb
    where
        I: IntoIterator<Item = Vec3>,
    {
        let mut mins = Vec3A::splat(f32::INFINITY);
        let mut maxs = Vec3A::splat(f32::NEG_INFINITY);

        for v in vertices.into_iter() {
            mins = mins.min(v.into());
            maxs = maxs.max(v.into());
        }

        let origin = 0.5 * (mins + maxs);
        let half_extents = 0.5 * (maxs - mins).abs();

        Aabb {
            origin: origin.into(),
            half_extents: half_extents.into(),
        }
    }

    /// Returns `true` _iff_ `self` intersects `b`.
    #[inline]
    pub fn intersects_aabb(&self, b: &Aabb) -> bool {
        (self.origin_a() - b.origin_a())
            .abs()
            .cmplt(self.half_extents_a() + b.half_extents_a())
            .all()
    }

    /// Returns `true` _iff_ `self` fully contains the AABB `b`.
    #[inline]
    pub fn contains_aabb(&self, b: &Aabb) -> bool {
        (self.mins_a() - b.mins_a())
            .cmple(Vec3A::splat(f32::EPSILON))
            .all()
            && (self.maxs_a() - b.maxs_a())
                .cmpge(Vec3A::splat(-f32::EPSILON))
                .all()
    }

    /// Returns `true` _iff_ `self` fully contains the point `point`.
    #[inline]
    pub fn contains_point(&self, point: Vec3) -> bool {
        let point = Vec3A::from(point);
        self.mins_a().cmple(point).all() && self.maxs_a().cmpge(point).all()
    }

    /// Returns the closest point on the AABB to the point `point`.
    #[inline]
    pub fn closest_point_to(&self, point: Vec3) -> Vec3 {
        let mins = self.origin - self.half_extents;
        let maxs = self.origin + self.half_extents;
        point.clamp(mins, maxs)
    }

    /// Returns the origin of the AABB.
    #[inline]
    pub fn origin(&self) -> Vec3 {
        self.origin
    }

    #[inline]
    fn origin_a(&self) -> Vec3A {
        self.origin.into()
    }

    /// Returns the half-extents of the AABB.
    #[inline]
    pub fn half_extents(&self) -> Vec3 {
        self.half_extents
    }

    #[inline]
    pub(crate) fn half_extents_a(&self) -> Vec3A {
        self.half_extents.into()
    }

    /// Returns the minimum extent of the AABB along each axis.
    #[inline]
    pub fn mins(&self) -> Vec3 {
        self.origin - self.half_extents
    }

    #[inline]
    pub(crate) fn mins_a(&self) -> Vec3A {
        Vec3A::from(self.origin) - Vec3A::from(self.half_extents)
    }

    /// Returns the maximum extent of the AABB along each axis.
    #[inline]
    pub fn maxs(&self) -> Vec3 {
        self.origin + self.half_extents
    }

    #[inline]
    pub(crate) fn maxs_a(&self) -> Vec3A {
        Vec3A::from(self.origin) + Vec3A::from(self.half_extents)
    }

    /// Returns the union of `self` with the AABB `b`.
    pub fn union(&self, b: &Aabb) -> Aabb {
        let mins = self.mins_a().min(b.mins_a());
        let maxs = self.maxs_a().max(b.maxs_a());

        let origin = 0.5 * (mins + maxs);
        let half_extents = 0.5 * (maxs - mins);

        Aabb {
            origin: origin.into(),
            half_extents: half_extents.into(),
        }
    }

    /// Returns the surface area of the AABB.
    pub fn surface_area(&self) -> f32 {
        let [w, h, d] = (2.0 * self.half_extents).into();

        2.0 * (w * h + w * d + h * d)
    }

    /// Returns a new AABB which bounds the rotation of this AABB by `quat`.
    ///
    /// The rotation is performed about the world origin.
    #[must_use = "this returns the rotated AABB, without modifying the original"]
    pub fn rotate(&self, quat: Quat) -> Aabb {
        Aabb {
            origin: (quat * self.origin_a()).into(),
            half_extents: (quat * self.half_extents_a()).abs().into(),
        }
    }

    /// Returns a new AABB which bounds the local rotation of this AABB by `quat`.
    ///
    /// The rotation is performed about the AABB's origin rather than the world origin.
    #[must_use = "this returns the rotated AABB, without modifying the original"]
    pub fn rotate_local(&self, quat: Quat) -> Aabb {
        Aabb {
            origin: self.origin,
            half_extents: (quat * self.half_extents_a()).abs().into(),
        }
    }

    /// Returns a new AABB which bounds the transformation of this AABB by `iso`.
    ///
    /// This is equivalent to computing the oriented bounding box produced by transforming `self`
    /// by `iso`, then computing an AABB for that OBB.
    #[must_use = "this returns the transformed AABB, without modifying the original"]
    pub fn transform(&self, iso: Isometry) -> Aabb {
        let origin = Vec3A::from(iso.translation) + iso.rotation * Vec3A::from(self.origin);
        let half_extents = (iso.rotation * self.half_extents_a()).abs();

        Aabb {
            origin: origin.into(),
            half_extents: half_extents.into(),
        }
    }

    /// Increases the extents of the AABB along each axis by `dims` in the negative and positive
    /// directions.
    #[must_use = "this returns the dilated AABB, without modifying the original"]
    pub fn dilate(&self, dims: Vec3) -> Aabb {
        let dims = Vec3A::from(dims).abs();

        Aabb {
            half_extents: (self.half_extents_a() + dims).into(),
            ..*self
        }
    }
}
