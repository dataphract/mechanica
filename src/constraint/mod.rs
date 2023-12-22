//! Constraints on simulation elements.
//!
//! A constraint governs the relative positions and orientations of a set of simulation elements.

use std::f32::consts::PI;

use glam::{Quat, Vec3A};

use crate::{
    rigid::{Inertia, Mass},
    Isometry, Substep,
};

mod contact;
mod positional;
mod prismatic;
mod revolute;

pub use self::{
    contact::ContactConstraint, positional::PositionalConstraint, revolute::RevoluteConstraint,
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
    fn compute<E>(&self, substep: Substep, elements: &[E; N]) -> Self::Computed
    where
        E: ConstraintElement;

    /// Solves the constraint based on the computed solution.
    fn solve_transforms<E>(
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
    fn solve_positions<C, const N: usize>(&mut self, substep: Substep, constraint: &C)
    where
        C: Constraint<K, N>,
    {
        let mut elements = self.elements::<C, N>(constraint.keys());
        let computed = constraint.compute(substep, &elements);
        constraint.solve_transforms(substep.inverse_squared(), &mut elements, &computed);
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
    fn inertia(&self) -> Option<Inertia> {
        None
    }

    #[inline]
    fn anchor(&self, local_anchor: Vec3A) -> Vec3A {
        self.transform() * local_anchor
    }

    #[inline]
    fn position(&self) -> Vec3A {
        self.transform().translation.into()
    }

    #[inline]
    fn orientation(&self) -> Quat {
        self.transform().rotation
    }

    #[inline]
    fn rotate(&self, v: Vec3A) -> Vec3A {
        self.transform().rotation * v
    }
}

/// Computes the positional generalized inverse mass.
pub fn positional_gen_inv_mass<E>(elem: &E, local_anchor: Option<Vec3A>, gradient: Vec3A) -> f32
where
    E: ConstraintElement,
{
    let inv_mass = elem.mass().inverse();

    match elem.inertia() {
        Some(inertia) => {
            let axis = elem.anchor(local_anchor.unwrap()).cross(gradient);
            inv_mass + axis.dot(inertia.inverse_mul(axis))
        }

        None => inv_mass,
    }
}

/// Computes the angular generalized inverse mass.
pub fn angular_gen_inv_mass<E>(elem: &E, axis: Vec3A) -> f32
where
    E: ConstraintElement,
{
    match elem.inertia() {
        Some(inertia) => axis.dot(inertia.inverse_mul(axis)),

        // TODO: this is really a logic error, since angular constraints only apply to rigid bodies.
        None => elem.mass().inverse(),
    }
}

/// Additional data provided by rigid-body simulation elements.
pub struct RigidElement {
    pub inertia: Inertia,
    pub local_anchor: Vec3A,
}

fn compute_positional_delta<E>(
    element: &E,
    local_anchor: Option<Vec3A>,
    impulse: Vec3A,
    inv_mass: f32,
) -> (Vec3A, Quat)
where
    E: ConstraintElement,
{
    let transform = element.transform();
    let prev_orient = transform.rotation;

    let translation = inv_mass * impulse;
    let rotation = match element.inertia() {
        Some(inertia) => {
            let axis = 0.5 * inertia.inverse_mul(local_anchor.unwrap().cross(impulse));
            Quat::from_vec4(axis.extend(0.0)) * prev_orient
        }

        None => prev_orient,
    };

    (translation, rotation)
}

/// Computes the angular impulse necessary to align two elements along some common axis.
fn compute_angular_impulse<E>(
    inv_substep_2: f32,
    compliance: f32,
    elements: &[E; 2],
    axes: [Vec3A; 2],
) -> Vec3A
where
    E: ConstraintElement,
{
    let axial_correction = Vec3A::from(axes[0]).cross(Vec3A::from(axes[1]));
    let theta = axial_correction.length();
    let axis = axial_correction / theta;

    let inv_masses = [
        angular_gen_inv_mass(&elements[0], axis),
        angular_gen_inv_mass(&elements[1], axis),
    ];

    let axial_lagrange = -theta / (inv_masses[0] + inv_masses[1] + inv_substep_2 * compliance);

    axial_lagrange * axis
}

fn compute_angular_delta<E>(element: &E, impulse: Vec3A) -> Quat
where
    E: ConstraintElement,
{
    let transform = element.transform();

    let prev_orient = transform.rotation;
    let inertia = element.inertia().unwrap();

    // TODO: fold the 0.5 into inverse inertia tensor?
    let axis = 0.5 * inertia.inverse_mul(impulse);
    Quat::from_vec4(axis.extend(0.0)) * prev_orient
}

/// Computes the signed angle of the projection of `a` and `b` onto the plane orthogonal to `axis`.
fn cylindrical_angle(axis: Vec3A, a: Vec3A, b: Vec3A) -> f32 {
    // The angle between the vectors can be computed with arccos(a ⋅ b), but this doesn't respect
    // orientation. The cross product respects orientation but has a restricted range of [-π/2, π/2].
    // Both are necessary to compute the correctly signed angle.

    let mut phi = a.cross(b).dot(axis).asin();

    if a.dot(b) < 0.0 {
        // If dot product is negative, map quadrants 1 and 4 to 2 and 3.
        phi = PI - phi;

        if phi > PI {
            // Normalize angles in quadrant 3.
            phi -= 2.0 * PI;
        }
    }

    phi
}

impl Isometry {
    #[inline]
    fn apply_angular_impulse<E>(&mut self, elem: &E, impulse: Vec3A)
    where
        E: ConstraintElement,
    {
        self.rotation = (self.rotation + compute_angular_delta(elem, impulse)).normalize();
    }
}
