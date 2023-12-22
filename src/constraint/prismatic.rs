use glam::{Vec3, Vec3A};

use crate::{
    constraint::{Constraint, ConstraintElement},
    Substep,
};

use super::{angular_gen_inv_mass, compute_angular_delta};

pub struct PrismaticConstraint<K> {
    /// The unique keys of each element.
    pub keys: [K; 2],

    /// The anchor points in the local space of each element.
    pub local_anchors: [Vec3; 2],

    /// The local x-axis of each element.
    pub local_x: [Vec3; 2],

    /// The local z-axis of each element.
    pub local_z: [Vec3; 2],
}

pub struct ComputedPrismaticConstraint {
    pub x_axis: [Vec3A; 2],
    pub z_axis: [Vec3A; 2],
}

impl<K> Constraint<K, 2> for PrismaticConstraint<K>
where
    K: Copy,
{
    type Computed = ComputedPrismaticConstraint;

    fn keys(&self) -> [K; 2] {
        self.keys
    }

    fn compute<E>(&self, _substep: Substep, elements: &[E; 2]) -> Self::Computed
    where
        E: ConstraintElement,
    {
        // 1. Apply angular corrections so that the bodies' local x- and z-axes are parallel.
        // 2. Apply positional correction so that the bodies' anchors are collinear along the z-axis.

        let z_axis = [
            elements[0].rotate(self.local_z[0].into()).into(),
            elements[1].rotate(self.local_z[1].into()).into(),
        ];

        let x_axis = [
            elements[0].rotate(self.local_x[0].into()).into(),
            elements[1].rotate(self.local_x[1].into()).into(),
        ];

        ComputedPrismaticConstraint { x_axis, z_axis }
    }

    fn solve_transforms<E>(
        &self,
        inv_substep_2: f32,
        elements: &mut [E; 2],
        computed: &Self::Computed,
    ) where
        E: ConstraintElement,
    {
        let axial_correction = computed.z_axis[0].cross(computed.z_axis[1]);
        let theta = axial_correction.length();
        let correction_axis = axial_correction / theta;

        todo!()

        // let sum_inv_masses =
    }
}
