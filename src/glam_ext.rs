use glam::{Mat3, Vec3A};

pub trait Vec3AExt {
    fn cosine_between(&self, rhs: Vec3A) -> f32;

    /// Returns `true` iff `rhs` is parallel to `self`.
    ///
    /// In practice, this returns `true` if the cosine of the angle between `self` and `rhs` is
    /// within 1 ulp of `1.0`.
    fn is_parallel_to(&self, rhs: Vec3A) -> bool;
}

impl Vec3AExt for Vec3A {
    #[inline]
    fn cosine_between(&self, rhs: Vec3A) -> f32 {
        self.dot(rhs) / (self.length_squared() * rhs.length_squared()).sqrt()
    }

    #[inline]
    fn is_parallel_to(&self, rhs: Vec3A) -> bool {
        self.cosine_between(rhs).abs() > 1.0 - f32::EPSILON
    }
}

pub trait Mat3Ext {
    fn transpose_mul_mat3(&self, m: Mat3) -> Mat3;
    // fn transpose_mul_mat3a(&self, m: Mat3) -> Mat3A;
    fn transpose_mul_vec3a(&self, v: Vec3A) -> Vec3A;
}

impl Mat3Ext for Mat3 {
    fn transpose_mul_mat3(&self, m: Mat3) -> Mat3 {
        let xx = self.x_axis.dot(m.x_axis);
        let xy = self.x_axis.dot(m.y_axis);
        let xz = self.x_axis.dot(m.z_axis);
        let yx = self.y_axis.dot(m.x_axis);
        let yy = self.y_axis.dot(m.y_axis);
        let yz = self.y_axis.dot(m.z_axis);
        let zx = self.z_axis.dot(m.x_axis);
        let zy = self.z_axis.dot(m.y_axis);
        let zz = self.z_axis.dot(m.z_axis);

        Mat3 {
            x_axis: [xx, xy, xz].into(),
            y_axis: [yx, yy, yz].into(),
            z_axis: [zx, zy, zz].into(),
        }
    }

    fn transpose_mul_vec3a(&self, v: Vec3A) -> Vec3A {
        let v: Vec3A = v.into();
        let col0 = Vec3A::from(self.col(0));
        let col1 = Vec3A::from(self.col(1));
        let col2 = Vec3A::from(self.col(2));
        Vec3A::new(col0.dot(v), col1.dot(v), col2.dot(v))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn is_parallel_to() {
        assert!(Vec3A::X.is_parallel_to(Vec3A::X));
        assert!(Vec3A::X.is_parallel_to(-Vec3A::X));
        assert!(!Vec3A::X.is_parallel_to(Vec3A::Y));
        assert!(!Vec3A::X.is_parallel_to(Vec3A::new(1.0, 0.001, 0.0)));
    }
}
