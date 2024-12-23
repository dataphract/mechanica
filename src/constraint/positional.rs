use glam::{Vec3, Vec3A};

use crate::{
    constraint::{compute_positional_delta, Constraint, ConstraintElement},
    Isometry, Substep,
};

use super::positional_gen_inv_mass;

/// A constraint governing the distance between two elements.
#[cfg_attr(feature = "bevy", derive(bevy::ecs::component::Component))]
pub struct PositionalConstraint<K> {
    /// The unique keys of the elements of the constraint.
    pub keys: [K; 2],

    /// The anchor points on the constraint elements.
    pub local_anchors: [Vec3; 2],

    /// The minimum distance in meters between the anchor points which satisfies the constraint.
    pub lower_bound: f32,

    /// The maximum distance in meters between the anchor points which satisfies the constraint.
    pub upper_bound: f32,

    /// The compliance of the constraint in meters per newton.
    pub compliance: f32,
}

/// The computed state of a [`PositionalConstraint`].
pub struct ComputedPositionalConstraint {
    /// The transforms of the constrained elements.
    pub transforms: [Isometry; 2],

    /// The anchors of the constrained elements in world space.
    pub anchors: [Vec3A; 2],

    /// The constraint gradient.
    pub gradient: Vec3A,

    /// The generalized inverse masses of the constrained elements.
    pub inv_masses: [f32; 2],

    /// The constraint error.
    pub error: f32,
}

impl<K> Constraint<K, 2> for PositionalConstraint<K>
where
    K: Copy,
{
    type Computed = ComputedPositionalConstraint;

    fn keys(&self) -> [K; 2] {
        self.keys
    }

    fn compute<E>(&self, substep: Substep, elements: &[E; 2]) -> ComputedPositionalConstraint
    where
        E: ConstraintElement,
    {
        let transforms = [elements[0].transform(), elements[1].transform()];

        let anchors = [
            elements[0].anchor(self.local_anchors[0].into()),
            elements[1].anchor(self.local_anchors[1].into()),
        ];

        let delta = anchors[1] - anchors[0];
        let dist = delta.length();

        let error = (dist - self.lower_bound).min(0.0) + (dist - self.upper_bound).max(0.0);

        let gradient = if dist != 0.0 {
            delta / dist
        } else {
            // With a distance of zero and nonzero error, there's no way to derive a constraint
            // gradient. Just pick one.
            Vec3A::Y
        };

        let inv_masses = [
            positional_gen_inv_mass(&elements[0], Some(self.local_anchors[0].into()), gradient),
            positional_gen_inv_mass(&elements[1], Some(self.local_anchors[1].into()), gradient),
        ];

        ComputedPositionalConstraint {
            transforms,
            anchors,
            gradient,
            inv_masses,
            error,
        }
    }

    fn solve_transforms<E>(
        &self,
        inv_substep_2: f32,
        elements: &mut [E; 2],
        computed: &Self::Computed,
    ) where
        E: ConstraintElement,
    {
        assert!(!elements[0].transform().translation.is_nan());
        assert!(!elements[1].transform().translation.is_nan());

        let lagrange_denom =
            computed.inv_masses.iter().sum::<f32>() + inv_substep_2 * self.compliance;
        let lagrange = -computed.error / lagrange_denom;
        let impulse = lagrange * computed.gradient;

        let mut xf1 = elements[0].transform();
        let mut xf2 = elements[1].transform();

        let (dx1, dq1) = compute_positional_delta(
            &elements[0],
            Some(self.local_anchors[0].into()),
            impulse,
            computed.inv_masses[0],
        );

        let (dx2, dq2) = compute_positional_delta(
            &elements[1],
            Some(self.local_anchors[1].into()),
            impulse,
            computed.inv_masses[1],
        );

        xf1.translation = (Vec3A::from(xf1.translation) + dx1).into();
        xf2.translation = (Vec3A::from(xf2.translation) - dx2).into();

        xf1.rotation = (xf1.rotation + dq1).normalize();
        xf2.rotation = (xf2.rotation - dq2).normalize();

        elements[0].set_transform(xf1);
        elements[1].set_transform(xf2);
    }
}
