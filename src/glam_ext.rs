use glam::{Mat3, Vec3, Vec3A};

pub trait Mat3Ext {
    fn transpose_mul_mat3(&self, m: Mat3) -> Mat3;
    // fn transpose_mul_mat3a(&self, m: Mat3) -> Mat3A;
    fn transpose_mul_vec3(&self, v: Vec3) -> Vec3;
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

    fn transpose_mul_vec3(&self, v: Vec3) -> Vec3 {
        Vec3::new(self.col(0).dot(v), self.col(1).dot(v), self.col(2).dot(v))
    }

    fn transpose_mul_vec3a(&self, v: Vec3A) -> Vec3A {
        let v: Vec3 = v.into();
        Vec3A::new(self.col(0).dot(v), self.col(1).dot(v), self.col(2).dot(v))
    }
}
