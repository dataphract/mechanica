use glam::{Vec3, Vec3A};

struct Arc {
    a: Vec3,
    b: Vec3,
}

impl Arc {
    fn intersects(&self, other: &Arc) -> bool {
        let a = Vec3A::from(self.a);
        let b = Vec3A::from(self.b);
        let c = Vec3A::from(other.a);
        let d = Vec3A::from(other.b);

        // Three conditions must hold in order for the arcs to intersect:
        //
        // - A and B must lie on opposite sides of the plane through CD.
        // - C and D must lie on opposite sides of the plane through AB.
        // - A and D must lie on the same side of the plane through B and C.
        //
        // The first two conditions are true if the scalar triple products have opposite signs:
        //
        // - [c · (b ⨯ a)] * [d · (b ⨯ a)] < 0
        // - [a · (d ⨯ c)] * [b · (d ⨯ c)] < 0
        //
        // The third condition is true if the scalar triple products have the same sign:
        //
        // - [a · (c ⨯ b)] * [d · (c ⨯ b)] > 0
        //
        // As an optimization, the scalar triple product identity can be applied to the third test
        // to use terms from the first two tests:
        //
        // - [c · (b ⨯ a)] * [b · (d ⨯ c)] > 0

        let bxa = b.cross(a);
        let dxc = d.cross(c);

        let cba = c.dot(bxa);
        let dba = d.dot(bxa);

        if cba * dba >= 0.0 {
            return false;
        }

        let adc = a.dot(dxc);
        let bdc = b.dot(dxc);

        if adc * bdc >= 0.0 {
            return false;
        }

        cba * bdc > 0.0
    }
}
