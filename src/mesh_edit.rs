// Mesh editing data structure using half-edge representation.
//
// The half-edge structure (aka doubly-connected edge list) cannot represent non-manifold or
// non-orientable geometry; all edges must be shared by exactly two faces, and all faces must be
// wound in the same direction.

use bitvec::vec::BitVec;
use glam::Vec3;

index!(pub struct EdgeId(u32): Edge);
index!(pub struct VertId(u32): Vec3);
index!(pub struct FaceId(u32): EdgeId);

// Directed edge.
struct Edge {
    /// Index of the reversed edge.
    reverse: EdgeId,
    /// Index of the next edge, walking counter-clockwise.
    next: EdgeId,
    /// Index of the adjacent face.
    face: FaceId,
    /// Index of the originating vertex.
    vert: VertId,
}

pub struct MeshEdit {
    verts: Vec<Vec3>,

    // For each vert, list of outgoing edges, walking counter-clockwise.
    vert_edges: Vec<Vec<EdgeId>>,

    edges: Vec<Edge>,

    faces: Vec<EdgeId>,
    face_normals: Vec<Vec3>,
}

impl MeshEdit {
    pub fn cube() -> MeshEdit {
        let verts = vec![
            Vec3::new(0.0, 0.0, 0.0), // A
            Vec3::new(1.0, 0.0, 0.0), // B
            Vec3::new(1.0, 0.0, 1.0), // C
            Vec3::new(0.0, 0.0, 1.0), // D
            Vec3::new(0.0, 1.0, 1.0), // E
            Vec3::new(1.0, 1.0, 1.0), // F
            Vec3::new(1.0, 1.0, 0.0), // G
            Vec3::new(0.0, 1.0, 0.0), // H
        ];

        let vert_edges = vec![
            vec![EdgeId(0), EdgeId(13), EdgeId(23)],
            vec![EdgeId(1), EdgeId(22), EdgeId(4)],
            vec![EdgeId(2), EdgeId(7), EdgeId(18)],
            vec![EdgeId(3), EdgeId(17), EdgeId(14)],
            vec![EdgeId(16), EdgeId(10), EdgeId(15)],
            vec![EdgeId(6), EdgeId(11), EdgeId(19)],
            vec![EdgeId(21), EdgeId(8), EdgeId(5)],
            vec![EdgeId(12), EdgeId(9), EdgeId(20)],
        ];

        #[rustfmt::skip]
        let edges = vec! [
            Edge { reverse: EdgeId(22), next: EdgeId(1), face: FaceId(0), vert: VertId(0) },  //  0: AB
            Edge { reverse: EdgeId(7), next: EdgeId(2), face: FaceId(0), vert: VertId(1) },   //  1: BC
            Edge { reverse: EdgeId(17), next: EdgeId(3), face: FaceId(0), vert: VertId(2) },  //  2: CD
            Edge { reverse: EdgeId(13), next: EdgeId(0), face: FaceId(0), vert: VertId(3) },  //  3: DA
            Edge { reverse: EdgeId(21), next: EdgeId(5), face: FaceId(1), vert: VertId(1) },  //  4: BG
            Edge { reverse: EdgeId(11), next: EdgeId(6), face: FaceId(1), vert: VertId(6) },  //  5: GF
            Edge { reverse: EdgeId(6), next: EdgeId(7), face: FaceId(1), vert: VertId(5) },   //  6: FC
            Edge { reverse: EdgeId(1), next: EdgeId(4), face: FaceId(1), vert: VertId(2) },   //  7: CB
            Edge { reverse: EdgeId(20), next: EdgeId(9), face: FaceId(2), vert: VertId(6) },  //  8: GH
            Edge { reverse: EdgeId(15), next: EdgeId(10), face: FaceId(2), vert: VertId(7) }, //  9: HE
            Edge { reverse: EdgeId(19), next: EdgeId(11), face: FaceId(2), vert: VertId(4) }, // 10: EF
            Edge { reverse: EdgeId(5), next: EdgeId(8), face: FaceId(2), vert: VertId(5) },   // 11: FG
            Edge { reverse: EdgeId(23), next: EdgeId(13), face: FaceId(3), vert: VertId(7) }, // 12: HA
            Edge { reverse: EdgeId(3), next: EdgeId(14), face: FaceId(3), vert: VertId(0) },  // 13: AD
            Edge { reverse: EdgeId(16), next: EdgeId(15), face: FaceId(3), vert: VertId(3) }, // 14: DE
            Edge { reverse: EdgeId(9), next: EdgeId(12), face: FaceId(3), vert: VertId(4) },  // 15: EH
            Edge { reverse: EdgeId(14), next: EdgeId(17), face: FaceId(4), vert: VertId(4) }, // 16: ED
            Edge { reverse: EdgeId(2), next: EdgeId(18), face: FaceId(4), vert: VertId(3) },  // 17: DC
            Edge { reverse: EdgeId(6), next: EdgeId(19), face: FaceId(4), vert: VertId(2) },  // 18: CF
            Edge { reverse: EdgeId(10), next: EdgeId(16), face: FaceId(4), vert: VertId(5) }, // 19: FE
            Edge { reverse: EdgeId(8), next: EdgeId(21), face: FaceId(5), vert: VertId(7) },  // 20: HG
            Edge { reverse: EdgeId(4), next: EdgeId(22), face: FaceId(5), vert: VertId(6) },  // 21: GB
            Edge { reverse: EdgeId(0), next: EdgeId(23), face: FaceId(5), vert: VertId(1) },  // 22: BA
            Edge { reverse: EdgeId(12), next: EdgeId(20), face: FaceId(5), vert: VertId(0) }, // 23: AH
        ];

        let faces = vec![
            EdgeId(0),  // ABCD
            EdgeId(4),  // BGFC
            EdgeId(8),  // GHEF
            EdgeId(12), // HADE
            EdgeId(16), // EDCF
            EdgeId(20), // HGBA
        ];

        MeshEdit {
            verts,
            vert_edges,
            edges,
            faces,
            face_normals: vec![-Vec3::Y, Vec3::X, Vec3::Y, -Vec3::X, Vec3::Z, -Vec3::Z],
        }
    }

    pub fn face_verts(&self) -> Vec<Vec<Vec3>> {
        let mut out = Vec::new();

        for &face in &self.faces {
            let mut face_verts = Vec::new();

            let mut edge = face;
            loop {
                face_verts.push(self.verts[self.edges[edge].vert]);
                edge = self.edges[edge].next;

                if edge == face {
                    break;
                }
            }

            out.push(face_verts);
        }

        out
    }

    pub fn to_mesh(&self) -> bevy::render::mesh::Mesh {
        use bevy::{
            prelude::*,
            render::{mesh::Indices, render_resource::PrimitiveTopology},
        };

        let mut positions = Vec::new();
        let mut normals = Vec::new();
        let mut indices = Vec::new();
        for (face_id, &face) in self.faces.iter().enumerate() {
            let normal = self.face_normals[face_id].to_array();
            let a = face;
            let a_idx = positions.len();
            positions.push(self.verts[self.edges[a].vert].to_array());
            normals.push(normal);

            let mut b = self.edges[a].next;
            let mut c = self.edges[b].next;

            while c != a {
                let b_idx = positions.len();
                positions.push(self.verts[self.edges[b].vert].to_array());
                normals.push(normal);

                let c_idx = positions.len();
                positions.push(self.verts[self.edges[c].vert].to_array());
                normals.push(normal);

                indices.push(a_idx as u32);
                indices.push(b_idx as u32);
                indices.push(c_idx as u32);

                b = c;
                c = self.edges[c].next;
            }
        }

        let mut mesh = Mesh::new(PrimitiveTopology::TriangleList);
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
        mesh.set_indices(Some(Indices::U32(indices)));

        mesh
    }
}

struct MeshSel {
    verts: BitVec,
    edges: BitVec,
}

macro_rules! index {
    ($v:vis struct $wrapper:ident($inner:ident): $element:ident) => {
        #[derive(Copy, Clone, Debug, PartialEq, Eq)]
        $v struct $wrapper($inner);

        impl std::ops::Index<$wrapper> for [$element] {
            type Output = $element;

            fn index(&self, index: $wrapper) -> &Self::Output {
                &self[index.0 as usize]
            }
        }

        impl std::ops::Index<$wrapper> for Vec<$element> {
            type Output = $element;

            fn index(&self, index: $wrapper) -> &Self::Output {
                &self[index.0 as usize]
            }
        }
    };
}
use index;

#[cfg(test)]
mod tests {
    use approx::assert_ulps_eq;

    use super::*;

    // fn validate_normals(cube: &MeshEdit) {
    //     for vert_a in 0..8 {
    //         let a = cube.verts[vert_a];

    //         let num_edges = cube.vert_edges[vert_a].len();
    //         for (i, j) in (0..num_edges).map(|i| (i, (i + 1) % num_edges)) {
    //             let edge_b = cube.vert_edges[vert_a][i];
    //             let edge_c = cube.vert_edges[vert_a][j];

    //             let vert_b = cube.next_vert(edge_b);
    //             let vert_c = cube.next_vert(edge_c);

    //             let b = cube.verts[vert_b];
    //             let c = cube.verts[vert_c];

    //             assert_ulps_eq!(
    //                 (b - a).cross(c - a).normalize(),
    //                 cube.face_normals[cube.edges[edge_b].face.0 as usize],
    //             );
    //         }
    //     }
    // }

    // #[test]
    // fn cube_vert_edge_normals() {
    //     validate_normals(&MeshEdit::cube());
    // }
}
