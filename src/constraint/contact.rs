use glam::{Vec3, Vec3A};

use crate::{
    constraint::{Constraint, ConstraintElement},
    Isometry, Substep,
};

use super::{compute_positional_delta, positional_gen_inv_mass};

/// A constraint resolving a single point of contact between two elements.
#[derive(Debug)]
#[cfg_attr(feature = "bevy", derive(bevy::ecs::component::Component))]
pub struct ContactConstraint<K> {
    /// The unique keys of the elements in contact.
    pub keys: [K; 2],

    /// The contact points in the local space of each element.
    pub local_contact_points: [Vec3; 2],

    // /// The transforms of each element at the previous substep.
    // TODO: move to ConstraintElement?
    // pub prev_transforms: [Isometry; 2],

    // /// The friction properties of each element.
    // pub frictions: [Friction; 2],
    /// The contact normal from the first element to the second.
    pub normal: Vec3,

    /// The penetration depth of the contact.
    pub penetration_depth: f32,
}

/// The computed state of a [`ContactConstraint`].
pub struct ComputedContactConstraint {
    /// The transforms of each element at the current substep.
    pub transforms: [Isometry; 2],

    pub anchors: [Vec3A; 2],
    pub inv_masses: [f32; 2],
}

impl<K> Constraint<K, 2> for ContactConstraint<K>
where
    K: Copy,
{
    type Computed = ComputedContactConstraint;

    fn keys(&self) -> [K; 2] {
        self.keys
    }

    fn compute<E>(&self, _substep: Substep, elements: &[E; 2]) -> Self::Computed
    where
        E: ConstraintElement,
    {
        let transforms = [elements[0].transform(), elements[1].transform()];
        let anchors = [
            elements[0].anchor(self.local_contact_points[0].into()),
            elements[1].anchor(self.local_contact_points[1].into()),
        ];

        let normal = Vec3A::from(self.normal);
        let inv_masses = [
            positional_gen_inv_mass(
                &elements[0],
                Some(self.local_contact_points[0].into()),
                normal,
            ),
            positional_gen_inv_mass(
                &elements[1],
                Some(self.local_contact_points[1].into()),
                normal,
            ),
        ];

        ComputedContactConstraint {
            transforms,
            anchors,
            inv_masses,
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
        // Not used: contact constraints have zero compliance.
        let _ = inv_substep_2;

        // Compliance term is omitted for rigid body contacts.
        let lagrange = -self.penetration_depth / computed.inv_masses.iter().sum::<f32>();
        let impulse = lagrange * Vec3A::from(self.normal);

        let mut xf1 = elements[0].transform();
        let mut xf2 = elements[1].transform();

        let (dx1, dq1) = compute_positional_delta(
            &elements[0],
            Some(self.local_contact_points[0].into()),
            impulse,
            computed.inv_masses[0],
        );

        let (dx2, dq2) = compute_positional_delta(
            &elements[1],
            Some(self.local_contact_points[1].into()),
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
