//! Constraints on simulation elements.
//!
//! A constraint governs the relative positions and orientations of a set of simulation elements.
//!
//! Simple constraints:
//!   - Positional
//!   - Angular
//! Joints:
//!   -

use bevy::ecs::component::Component;
use glam::{Mat3A, Quat, Vec3, Vec3A};

use crate::{
    rigid::{Friction, Mass, RigidBodyInertia},
    Isometry,
};

/// Represents a constraint between a system of `N` simulation elements.
pub trait Constraint<K, const N: usize>
where
    K: Copy,
{
    /// The type which contains concrete information about the state of the constraint.
    type Computed;

    /// Returns the keys associated with the elements of the constraint.
    fn keys(&self) -> [K; N];

    /// Computes the state of the constraint for the current state of the system.
    fn compute<E>(&self, elements: &[E; N]) -> Self::Computed
    where
        E: ConstraintElement;

    /// Solves the constraint based on the computed solution.
    fn solve_positions<E>(
        &self,
        inv_substep_2: f32,
        elements: &mut [E; N],
        computed: &Self::Computed,
    ) where
        E: ConstraintElement;
}

/// Provides access to simulation elements in order to solve constraints.
pub trait ConstraintSolver<K>
where
    K: Copy,
{
    /// The type of the elements governed by the constraint.
    type Element<'e>: ConstraintElement
    where
        Self: 'e;

    /// Returns the elements associated with `keys`.
    fn elements<C, const N: usize>(&mut self, keys: [K; N]) -> [Self::Element<'_>; N]
    where
        C: Constraint<K, N>;

    /// Solves the constraint and applies the solution to the simulation elements.
    fn solve_positions<C, const N: usize>(&mut self, inv_timestep_2: f32, constraint: &C)
    where
        C: Constraint<K, N>,
    {
        let mut elements = self.elements::<C, N>(constraint.keys());
        let computed = constraint.compute(&elements);
        constraint.solve_positions(inv_timestep_2, &mut elements, &computed);
    }
}

/// A datatype that represents the state of a constraint element.
pub trait ConstraintElement {
    /// Returns the mass of the element.
    fn mass(&self) -> Mass;

    /// Returns the transform of the element.
    fn transform(&self) -> Isometry;

    /// Sets the transform of the element.
    fn set_transform(&mut self, transform: Isometry);

    /// Returns the moment of inertia of the element, if any.
    ///
    /// The default implementation returns `None` to represent a particle. Constraint elements
    /// representing rigid bodies should return `Some` with the appropriate moment of inertia.
    #[inline]
    fn inertia(&self) -> Option<RigidBodyInertia> {
        None
    }

    fn anchor(&self, local_anchor: Option<Vec3A>) -> Vec3A {
        let xf = self.transform();
        let pos = Vec3A::from(xf.translation);
        let orient = xf.rotation;

        match local_anchor {
            Some(anchor) => pos + orient * anchor,
            None => pos,
        }
    }

    fn positional_generalized_inverse_mass(
        &self,
        local_anchor: Option<Vec3A>,
        gradient: Vec3A,
    ) -> f32 {
        let inv_mass = self.mass().inverse();

        match self.inertia() {
            Some(inertia) => {
                let axis = self.anchor(local_anchor).cross(gradient);
                inv_mass + axis.dot(inertia.inverse() * axis)
            }

            None => inv_mass,
        }
    }
}

/// Additional data provided by rigid-body simulation elements.
pub struct RigidElement {
    pub inertia: RigidBodyInertia,
    pub local_anchor: Vec3A,
}

/// A constraint governing the distance between two elements.
#[derive(Component)]
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

fn bikeshed_impulse_transform<E>(
    element: &E,
    local_anchor: Option<Vec3A>,
    impulse: Vec3A,
    inv_mass: f32,
) -> Isometry
where
    E: ConstraintElement,
{
    let transform = element.transform();
    let prev_pos = Vec3A::from(transform.translation);
    let prev_orient = transform.rotation;

    let translation = prev_pos + inv_mass * impulse;
    let rotation = match element.inertia() {
        Some(inertia) => {
            let axis = 0.5 * inertia.inverse() * local_anchor.unwrap().cross(impulse);
            (prev_orient + Quat::from_vec4(axis.extend(0.0)) * prev_orient).normalize()
        }

        None => prev_orient,
    };

    Isometry {
        translation: translation.into(),
        rotation,
    }
}

impl<K> Constraint<K, 2> for PositionalConstraint<K>
where
    K: Copy,
{
    type Computed = ComputedPositionalConstraint;

    fn keys(&self) -> [K; 2] {
        self.keys
    }

    fn compute<E>(&self, elements: &[E; 2]) -> ComputedPositionalConstraint
    where
        E: ConstraintElement,
    {
        let transforms = [elements[0].transform(), elements[1].transform()];

        let anchors = [
            elements[0].anchor(Some(self.local_anchors[0].into())),
            elements[1].anchor(Some(self.local_anchors[1].into())),
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
            elements[0]
                .positional_generalized_inverse_mass(Some(self.local_anchors[0].into()), gradient),
            elements[1]
                .positional_generalized_inverse_mass(Some(self.local_anchors[1].into()), gradient),
        ];

        ComputedPositionalConstraint {
            transforms,
            anchors,
            gradient,
            inv_masses,
            error,
        }
    }

    fn solve_positions<E>(
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

        let new_transforms = [
            bikeshed_impulse_transform(
                &elements[0],
                Some(self.local_anchors[0].into()),
                impulse,
                computed.inv_masses[0],
            ),
            bikeshed_impulse_transform(
                &elements[1],
                Some(self.local_anchors[1].into()),
                impulse,
                computed.inv_masses[1],
            ),
        ];

        elements[0].set_transform(new_transforms[0]);
        elements[1].set_transform(new_transforms[1]);
    }
}

/// A constraint resolving a single point of contact between two elements.
#[derive(Component)]
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

    fn compute<E>(&self, elements: &[E; 2]) -> Self::Computed
    where
        E: ConstraintElement,
    {
        let transforms = [elements[0].transform(), elements[1].transform()];
        let anchors = [
            elements[0].anchor(Some(self.local_contact_points[0].into())),
            elements[1].anchor(Some(self.local_contact_points[1].into())),
        ];

        let normal = Vec3A::from(self.normal);
        let inv_masses = [
            elements[0].positional_generalized_inverse_mass(
                Some(self.local_contact_points[0].into()),
                normal,
            ),
            elements[1].positional_generalized_inverse_mass(
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

    fn solve_positions<E>(
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
        let impulse = lagrange * -self.normal;

        let new_transforms = [
            bikeshed_impulse_transform(
                &elements[0],
                Some(self.local_contact_points[0].into()),
                impulse.into(),
                computed.inv_masses[0],
            ),
            bikeshed_impulse_transform(
                &elements[1],
                Some(self.local_contact_points[1].into()),
                impulse.into(),
                computed.inv_masses[1],
            ),
        ];

        elements[0].set_transform(new_transforms[0]);
        elements[1].set_transform(new_transforms[1]);
    }
}
