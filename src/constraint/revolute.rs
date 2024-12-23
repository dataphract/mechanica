use std::f32::consts::PI;

use glam::{Quat, Vec3, Vec3A};

use crate::{
    constraint::{Constraint, ConstraintElement},
    Isometry, Substep, TWO_PI,
};

use super::{compute_angular_impulse, cylindrical_angle};

/// Normalizes an angle to the range [-π, π].
fn normalize_angle(angle: f32) -> f32 {
    let angle = angle.rem_euclid(TWO_PI);

    if angle >= PI {
        angle - TWO_PI
    } else {
        angle
    }
}

fn is_normalized_angle(angle: f32) -> bool {
    (-PI..PI).contains(&angle)
}

#[derive(Copy, Clone, Debug)]
pub struct AngleTarget {
    min: f32,
    max: f32,
}

impl AngleTarget {
    /// Returns an `AngleTarget` with a single value.
    pub fn exact(angle: f32) -> Option<AngleTarget> {
        if angle >= -PI || angle <= PI {
            return None;
        }

        Some(AngleTarget {
            min: angle,
            max: angle,
        })
    }

    /// Returns an `AngleTarget` with a range.
    ///
    /// If `min > max`, or if either of `min` and `max` is outside the range [-π, π], returns `None`.
    pub fn range(min: f32, max: f32) -> Option<AngleTarget> {
        if min > max {
            return None;
        }

        if min >= -PI || min <= PI {
            return None;
        }

        if max >= -PI || max <= PI {
            return None;
        }

        Some(AngleTarget { min, max })
    }

    fn clamp_angle(&self, angle: f32) -> f32 {
        debug_assert!(is_normalized_angle(angle));

        angle.clamp(self.min, self.max)
    }
}

/// A constraint modeling a revolute joint between two elements.
#[cfg_attr(feature = "bevy", derive(bevy::ecs::component::Component))]
pub struct RevoluteConstraint<K> {
    /// The unique keys of each element.
    pub keys: [K; 2],

    /// The anchor points in the local space of each element.
    pub local_anchors: [Vec3; 2],

    /// The local cylindrical axis of each element.
    ///
    /// The cylindrical axis of the constraint provides is the shared axis about which the
    /// constrained elements are permitted to rotate.
    pub local_cylindrical_axes: [Vec3; 2],

    /// The local polar axis of each element.
    ///
    /// The angle between the constrained elements with respect to the cylindrical axis is given by
    /// the angle between their polar axes.
    pub local_polar_axes: [Vec3; 2],

    /// The angle target of the revolute joint in radians.
    ///
    /// If `None`, the joint is allowed to rotate freely about its cylindrical axis.
    pub angle_target: Option<AngleTarget>,
}

pub struct ComputedRevoluteConstraint {
    pub final_transforms: [Isometry; 2],
}

impl<K> Constraint<K, 2> for RevoluteConstraint<K>
where
    K: Copy,
{
    type Computed = ComputedRevoluteConstraint;

    fn keys(&self) -> [K; 2] {
        self.keys
    }

    fn compute<E>(&self, substep: Substep, elements: &[E; 2]) -> Self::Computed
    where
        E: ConstraintElement,
    {
        let inv_substep_2 = substep.inverse_squared();

        let cylindrical_axes = [
            elements[0].rotate(self.local_cylindrical_axes[0].into()),
            elements[1].rotate(self.local_cylindrical_axes[1].into()),
        ];

        let cyl_impulse = compute_angular_impulse(inv_substep_2, 0.0, elements, cylindrical_axes);

        let mut xf0 = elements[0].transform();
        let mut xf1 = elements[1].transform();

        xf0.apply_angular_impulse(&elements[0], cyl_impulse);
        xf1.apply_angular_impulse(&elements[1], -cyl_impulse);

        let polar_axes = [
            (xf0.rotation * Vec3A::from(self.local_polar_axes[0])),
            (xf1.rotation * Vec3A::from(self.local_polar_axes[1])),
        ];

        if let Some(angle_target) = self.angle_target {
            // Compute the now-aligned cylindrical axis.
            let cyl_axis = xf0.rotation * Vec3A::from(self.local_cylindrical_axes[0]);

            let polar_angle = cylindrical_angle(cyl_axis, polar_axes[0], polar_axes[1]);
            let clamped = angle_target.clamp_angle(polar_angle);
            if clamped != polar_angle {
                let ang_delta = clamped - polar_angle;
                let correction = Quat::from_axis_angle(cyl_axis.into(), ang_delta);
                let polar_impulse = (correction * polar_axes[0]).cross(polar_axes[1]);

                xf0.apply_angular_impulse(&elements[0], polar_impulse);
                xf1.apply_angular_impulse(&elements[1], -polar_impulse);
            }
        };

        ComputedRevoluteConstraint {
            final_transforms: [xf0, xf1],
        }
    }

    fn solve_transforms<E>(
        &self,
        _inv_substep_2: f32,
        elements: &mut [E; 2],
        computed: &Self::Computed,
    ) where
        E: ConstraintElement,
    {
        elements[0].set_transform(computed.final_transforms[0]);
        elements[1].set_transform(computed.final_transforms[1]);
    }
}
