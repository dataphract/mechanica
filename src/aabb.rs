use glam::{Quat, Vec3, Vec3A};

use crate::Isometry;

#[derive(Clone, Debug, PartialEq)]
pub struct Aabb {
    origin: Vec3,
    half_extents: Vec3,
}

impl Aabb {
    pub fn new(origin: Vec3, half_extents: Vec3) -> Aabb {
        Aabb {
            origin,
            half_extents,
        }
    }

    #[inline]
    pub fn intersects_aabb(&self, b: &Aabb) -> bool {
        (self.origin_a() - b.origin_a())
            .abs()
            .cmplt(self.half_extents_a() + b.half_extents_a())
            .all()
    }

    #[inline]
    pub fn contains_aabb(&self, b: &Aabb) -> bool {
        self.mins_a().cmple(b.mins_a()).all() && self.maxs_a().cmpge(b.maxs_a()).all()
    }

    #[inline]
    pub fn contains_point(&self, point: Vec3) -> bool {
        let point = Vec3A::from(point);
        self.mins_a().cmple(point).all() && self.maxs_a().cmpge(point).all()
    }

    #[inline]
    pub fn closest_point_to(&self, point: Vec3) -> Vec3 {
        let mins = self.origin - self.half_extents;
        let maxs = self.origin + self.half_extents;
        point.clamp(mins, maxs)
    }

    #[inline]
    pub fn origin(&self) -> Vec3 {
        self.origin
    }

    #[inline]
    fn origin_a(&self) -> Vec3A {
        self.origin.into()
    }

    #[inline]
    pub fn half_extents(&self) -> Vec3 {
        self.half_extents
    }

    #[inline]
    pub(crate) fn half_extents_a(&self) -> Vec3A {
        self.half_extents.into()
    }

    /// Returns the minimum extents of the AABB along each axis.
    #[inline]
    pub fn mins(&self) -> Vec3 {
        self.origin - self.half_extents
    }

    #[inline]
    pub(crate) fn mins_a(&self) -> Vec3A {
        Vec3A::from(self.origin) - Vec3A::from(self.half_extents)
    }

    /// Returns the maximum extents of the AABB along each axis.
    #[inline]
    pub fn maxs(&self) -> Vec3 {
        self.origin + self.half_extents
    }

    #[inline]
    pub(crate) fn maxs_a(&self) -> Vec3A {
        Vec3A::from(self.origin) + Vec3A::from(self.half_extents)
    }

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

    pub fn surface_area(&self) -> f32 {
        let [w, h, d] = (2.0 * self.half_extents).into();

        2.0 * (w * h + w * d + h * d)
    }

    /// Returns a new AABB which bounds the rotation of this AABB by `quat`.
    ///
    /// The rotation is performed about the world origin.
    pub fn rotate(&self, quat: Quat) -> Aabb {
        Aabb {
            origin: (quat * self.origin_a()).into(),
            half_extents: (quat * self.half_extents_a()).abs().into(),
        }
    }

    /// Returns a new AABB which bounds the local rotation of this AABB by `quat`.
    ///
    /// The rotation is performed about the AABB's origin rather than the world origin.
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
    pub fn transform(&self, iso: Isometry) -> Aabb {
        let origin = Vec3A::from(iso.translation) + iso.rotation * Vec3A::from(self.origin);
        let half_extents = (iso.rotation * self.half_extents_a()).abs();

        Aabb {
            origin: origin.into(),
            half_extents: half_extents.into(),
        }
    }
}
