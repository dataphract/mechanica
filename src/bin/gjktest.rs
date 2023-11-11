use cg3::{gjk, hull::Hull, Isometry, Segment};
use glam::Vec3;

fn main() {
    // let t1 = Segment {
    //     a: -Vec3::X,
    //     b: Vec3::X,
    // };
    let t1 = Vec3::ZERO;
    let t2 = Segment {
        a: Vec3::new(0.0, 1.0, -1.0),
        b: Vec3::new(0.0, 1.0, 1.0),
    };

    let (p1, p2) = gjk::closest(&t1, Isometry::default(), &t2, Isometry::default(), None);

    println!("closest points: {p1}, {p2}");
}
