use std::{
    ops::{Index, IndexMut},
    ptr,
};

use bevy::{
    prelude::*,
    render::{mesh::Indices, render_resource::PrimitiveTopology},
};
use glam::{Vec3, Vec3A};
use hashbrown::HashSet;

use crate::{Isometry, Plane, Segment};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct VertIdx(u32);

impl From<u32> for VertIdx {
    #[inline]
    fn from(value: u32) -> Self {
        VertIdx(value)
    }
}

impl Index<VertIdx> for Vec<Vec3> {
    type Output = Vec3;

    fn index(&self, index: VertIdx) -> &Self::Output {
        &self[index.0 as usize]
    }
}

impl IndexMut<VertIdx> for Vec<Vec3> {
    #[inline]
    fn index_mut(&mut self, index: VertIdx) -> &mut Self::Output {
        &mut self[index.0 as usize]
    }
}

/// An index into the edge list.
pub(crate) type EdgeIndex = u32;
/// An index into the face list.
pub(crate) type FaceIndex = u32;
/// An index into the per-face vertex list.
pub(crate) type FaceVertIndex = u32;

#[derive(Copy, Clone, Debug)]
pub(crate) struct ListSlice {
    pub first: u32,
    pub len: u32,
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct EdgeData {
    /// Index of the next edge, walking counter-clockwise.
    pub(crate) next: EdgeIndex,
    /// Index of the adjacent face.
    pub(crate) face: FaceIndex,
    /// Index of the originating vertex.
    pub(crate) vertex: VertIdx,
}

/// A convex polyhedron.
#[derive(Clone, Debug)]
pub struct Hull {
    pub(crate) vertices: Vec<Vec3>,

    // TODO: all adjacency information can be stored in a single Vec<u32>.

    // Mapping from vertex index to slice of the `vert_edges` array.
    pub(crate) vert_edge_slices: Vec<ListSlice>,
    // Concatenated per-vertex edge lists.
    pub(crate) vert_edges: Vec<EdgeIndex>,

    pub(crate) edges: Vec<EdgeData>,

    // Faces are defined implicitly by their first directed edge.
    pub(crate) faces: Vec<EdgeIndex>,
    // Cached face normals.
    pub(crate) face_normals: Vec<Vec3>,
}

impl Hull {
    // Iterates over the edges that contain this vertex.
    fn iter_vertex_edges(&self, vertex_id: VertIdx) -> impl Iterator<Item = EdgeData> + '_ {
        let ListSlice { first, len } = self.vert_edge_slices[vertex_id.0 as usize];
        self.vert_edges
            .iter()
            .skip(first as usize)
            .take(len as usize)
            .map(|&ei| self.edges[ei as usize])
    }

    pub(crate) fn iter_faces(&self) -> impl Iterator<Item = Face<'_>> + '_ {
        self.faces
            .iter()
            .zip(self.face_normals.iter())
            .map(|(&first_edge, &normal)| Face {
                hull: self,
                first_edge,
                normal,
            })
    }

    pub(crate) fn iter_edges(&self) -> impl Iterator<Item = Edge<'_>> + '_ {
        (0..self.edges.len()).step_by(2).map(|i| Edge {
            hull: self,
            idx: i as u32,
        })
    }

    fn assert_invariants(&self) {
        // Euler characteristic.
        assert_eq!(self.vertices.len() - self.edges.len() + self.faces.len(), 2);

        assert_eq!(self.edges.len() % 2, 0);

        // Ensure src(A) = dst(rev(A)).
        for (edge_idx, edge) in self.edges.iter().enumerate() {
            let reverse = &self.edges[edge_idx ^ 1];
            let reverse_dst = self.edges[reverse.next as usize].vertex;
            assert_eq!(edge.vertex, reverse_dst);
        }

        let mut edge_set = HashSet::new();

        for &face_first in self.faces.iter() {
            let mut edge = &self.edges[face_first as usize];

            edge_set.insert(face_first);

            while edge.next != face_first {
                let first_seen = edge_set.insert(edge.next);
                assert!(first_seen, "duplicate edge in face {face_first:?}");

                edge = &self.edges[edge.next as usize];
            }
        }
    }

    // TODO: consider building a Dobkin-Kirkpatrick hierarchy for large hulls.
    pub(crate) fn compute_supporting_point(&self, dir: Vec3A) -> Vec3A {
        let mut vid = VertIdx(0);
        'next_vert: loop {
            let v: Vec3A = self.vertices[vid].into();

            for edge in self.iter_vertex_edges(vid) {
                let other_vid = self.edges[edge.next as usize].vertex;
                let other: Vec3A = self.vertices[other_vid].into();

                // Walk toward the extreme vertex.
                if (other - v).dot(dir) > f32::EPSILON {
                    vid = other_vid;
                    continue 'next_vert;
                }
            }

            break;
        }

        self.vertices[vid].into()
    }

    pub fn to_mesh(&self) -> Mesh {
        let mut positions = Vec::new();
        let mut normals = Vec::new();
        let mut indices = Vec::new();
        for (face_id, &face) in self.faces.iter().enumerate() {
            let normal = self.face_normals[face_id].to_array();
            let a = face as usize;
            let a_idx = positions.len();
            positions.push(self.vertices[self.edges[a].vertex].to_array());
            normals.push(normal);

            let mut b = self.edges[a].next as usize;
            let mut c = self.edges[b].next as usize;

            while c != a {
                let b_idx = positions.len();
                positions.push(self.vertices[self.edges[b].vertex].to_array());
                normals.push(normal);

                let c_idx = positions.len();
                positions.push(self.vertices[self.edges[c].vertex].to_array());
                normals.push(normal);

                indices.push(a_idx as u32);
                indices.push(b_idx as u32);
                indices.push(c_idx as u32);

                b = c;
                c = self.edges[c].next as usize;
            }
        }

        let mut mesh = Mesh::new(PrimitiveTopology::TriangleList);
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
        mesh.set_indices(Some(Indices::U32(indices)));

        mesh
    }

    /// Constructs a regular tetrahedron with edge lengths of `1.0`.
    pub fn tetrahedron(scale: f32) -> Hull {
        // Top-down:
        //
        //   2.
        //   |\'-.
        //   | \  '-.
        //   |  \    '-.
        //   |   3-------0
        //   |  /    .-'
        //   | /  .-'
        //   |/.-'
        //   1'

        let vertices = vec![
            scale * Vec3::new((8.0_f32 / 9.0).sqrt(), -1.0 / 3.0, 0.0),
            scale * Vec3::new(-(2.0_f32 / 9.0).sqrt(), -1.0 / 3.0, (2.0_f32 / 3.0).sqrt()),
            scale * Vec3::new(-(2.0_f32 / 9.0).sqrt(), -1.0 / 3.0, -(2.0_f32 / 3.0).sqrt()),
            scale * Vec3::new(0.0, 1.0, 0.0),
        ];

        let vert_edge_slices = vec![
            ListSlice { first: 0, len: 3 },
            ListSlice { first: 3, len: 3 },
            ListSlice { first: 6, len: 3 },
            ListSlice { first: 9, len: 3 },
        ];

        #[rustfmt::skip]
        let vert_edges = vec![
            // 0
            0, 5, 6,
            // 1
            1, 2, 8,
            // 2
            3, 4, 10,
            // 3
            7, 9, 11,
        ];

        #[rustfmt::skip]
        let edges = vec![
            EdgeData {vertex: 0.into(), next: 2, face: 0 },
            EdgeData {vertex: 1.into(), next: 6, face: 3,},
            EdgeData {vertex: 1.into(), next: 4, face: 0,},
            EdgeData {vertex: 2.into(), next: 8, face: 2,},
            EdgeData {vertex: 2.into(), next: 0, face: 0,},
            EdgeData {vertex: 0.into(), next: 10, face: 1,},
            EdgeData {vertex: 0.into(), next: 9, face: 3,},
            EdgeData {vertex: 3.into(), next: 5, face: 1,},
            EdgeData {vertex: 1.into(), next: 11, face: 2,},
            EdgeData {vertex: 3.into(), next: 1, face: 3,},
            EdgeData {vertex: 2.into(), next: 7, face: 1,},
            EdgeData {vertex: 3.into(), next: 3, face: 2,},
        ];

        let faces = vec![0, 5, 3, 1];

        let [p0, p1, p2, p3]: [Vec3; 4] = vertices.clone().try_into().unwrap();

        let face_normals = vec![
            (p1 - p0).cross(p2 - p1).normalize(),
            (p2 - p0).cross(p3 - p2).normalize(),
            (p1 - p2).cross(p3 - p1).normalize(),
            (p0 - p1).cross(p3 - p0).normalize(),
        ];

        Hull {
            vertices,
            vert_edge_slices,
            vert_edges,
            edges,
            faces,
            face_normals,
        }
    }

    pub fn cuboid(half_extents: Vec3) -> Hull {
        let hx = 0.5 * half_extents.x;
        let hy = 0.5 * half_extents.y;
        let hz = 0.5 * half_extents.z;

        #[rustfmt::skip]
        let vertices: Vec<Vec3> = vec![
            [ hx,  hy,  hz].into(),
            [-hx,  hy,  hz].into(),
            [-hx, -hy,  hz].into(),
            [ hx, -hy,  hz].into(),
            [ hx,  hy, -hz].into(),
            [-hx,  hy, -hz].into(),
            [-hx, -hy, -hz].into(),
            [ hx, -hy, -hz].into(),
        ];

        #[rustfmt::skip]
        let vert_edge_slices = vec![
            ListSlice { first: 0, len: 3 },
            ListSlice { first: 3, len: 3 },
            ListSlice { first: 6, len: 3 },
            ListSlice { first: 9, len: 3 },
            ListSlice { first: 12, len: 3 },
            ListSlice { first: 15, len: 3 },
            ListSlice { first: 18, len: 3 },
            ListSlice { first: 21, len: 3 },
        ];

        #[rustfmt::skip]
        let vert_edges = vec![
            0, 7, 8,
            2, 1, 13,
            4, 3, 17,
            6, 5, 21,
            10, 9, 23,
            12, 11, 14,
            16, 15, 18,
            20, 19, 22,
        ];

        #[rustfmt::skip]
        let edges = vec![
            EdgeData { vertex:  0.into(), next:  2, face: 0 }, // AB
            EdgeData { vertex:  1.into(), next:  8, face: 1 }, // BA
            EdgeData { vertex:  1.into(), next:  4, face: 0 }, // BC
            EdgeData { vertex:  2.into(), next: 13, face: 2 }, // CB
            EdgeData { vertex:  2.into(), next:  6, face: 0 }, // CD
            EdgeData { vertex:  3.into(), next: 17, face: 3 }, // DC
            EdgeData { vertex:  3.into(), next:  0, face: 0 }, // DA
            EdgeData { vertex:  0.into(), next: 21, face: 4 }, // AD

            EdgeData { vertex:  0.into(), next: 10, face: 1 }, // AE
            EdgeData { vertex:  4.into(), next:  7, face: 4 }, // EA
            EdgeData { vertex:  4.into(), next: 12, face: 1 }, // EF
            EdgeData { vertex:  5.into(), next: 23, face: 5 }, // FE
            EdgeData { vertex:  5.into(), next:  1, face: 1 }, // FB
            EdgeData { vertex:  1.into(), next: 14, face: 2 }, // BF
            EdgeData { vertex:  5.into(), next: 16, face: 2 }, // FG
            EdgeData { vertex:  6.into(), next: 11, face: 5 }, // GF

            EdgeData { vertex:  6.into(), next:  3, face: 2 }, // GC
            EdgeData { vertex:  2.into(), next: 18, face: 3 }, // CG
            EdgeData { vertex:  6.into(), next: 20, face: 3 }, // GH
            EdgeData { vertex:  7.into(), next: 15, face: 5 }, // HG
            EdgeData { vertex:  7.into(), next:  5, face: 3 }, // HD
            EdgeData { vertex:  3.into(), next: 22, face: 4 }, // DH
            EdgeData { vertex:  7.into(), next:  9, face: 4 }, // HE
            EdgeData { vertex:  4.into(), next: 19, face: 5 }, // EH
        ];

        let faces = vec![0, 1, 3, 5, 7, 11];
        let face_normals = vec![Vec3::Z, Vec3::Y, -Vec3::X, -Vec3::Y, Vec3::X, -Vec3::Z];

        Hull {
            vertices,
            vert_edge_slices,
            vert_edges,
            edges,
            faces,
            face_normals,
        }
    }

    // pub fn dodecahedron() -> Hull {
    //     const PHI: f32 = 1.618034;
    //     const FRAC_1_PHI: f32 = 0.618034;

    //     #[rustfmt::skip]
    //     let vertices: Vec<Vec3> = vec![
    //         [        0.0,         PHI,  FRAC_1_PHI].into(),
    //         [       -1.0,         1.0,         1.0].into(),
    //         [-FRAC_1_PHI,         0.0,         PHI].into(),
    //         [ FRAC_1_PHI,         0.0,         PHI].into(),
    //         [        1.0,         1.0,         1.0].into(),
    //         [        0.0,         PHI, -FRAC_1_PHI].into(),
    //         [       -PHI,  FRAC_1_PHI,        -0.0].into(),
    //         [       -1.0,        -1.0,         1.0].into(),
    //         [        1.0,        -1.0,         1.0].into(),
    //         [        PHI,  FRAC_1_PHI,        -0.0].into(),
    //         [       -1.0,         1.0,        -1.0].into(),
    //         [       -PHI, -FRAC_1_PHI,        -0.0].into(),
    //         [        0.0,        -PHI,  FRAC_1_PHI].into(),
    //         [        PHI, -FRAC_1_PHI,        -0.0].into(),
    //         [        1.0,         1.0,        -1.0].into(),
    //         [-FRAC_1_PHI,         0.0,        -PHI].into(),
    //         [       -1.0,        -1.0,        -1.0].into(),
    //         [        0.0,        -PHI, -FRAC_1_PHI].into(),
    //         [        1.0,        -1.0,        -1.0].into(),
    //         [ FRAC_1_PHI,         0.0,        -PHI].into(),
    //     ];

    //     let vert_edge_slices = vec![
    //         ListSlice { first: 0, len: 3 },
    //         ListSlice { first: 3, len: 3 },
    //         ListSlice { first: 6, len: 3 },
    //         ListSlice { first: 9, len: 3 },
    //         ListSlice { first: 12, len: 3 },
    //         ListSlice { first: 15, len: 3 },
    //         ListSlice { first: 18, len: 3 },
    //         ListSlice { first: 21, len: 3 },
    //         ListSlice { first: 24, len: 3 },
    //         ListSlice { first: 27, len: 3 },
    //         ListSlice { first: 30, len: 3 },
    //         ListSlice { first: 33, len: 3 },
    //         ListSlice { first: 36, len: 3 },
    //         ListSlice { first: 39, len: 3 },
    //         ListSlice { first: 42, len: 3 },
    //         ListSlice { first: 45, len: 3 },
    //         ListSlice { first: 48, len: 3 },
    //         ListSlice { first: 51, len: 3 },
    //         ListSlice { first: 54, len: 3 },
    //         ListSlice { first: 57, len: 3 },
    //     ];

    //     #[rustfmt::skip]
    //     let vert_edges = vec![
    //          0,  5, 25,
    //          1,  9, 10,
    //          2, 14, 15,
    //          3, 19, 20,
    //          4, 24, 26,
    //          6, 29, 51,
    //          8, 11, 30,
    //         13, 16, 35,
    //         18, 21, 40,
    //         23, 27, 45,
    //          7, 31, 50,
    //         12, 34, 36,
    //         17, 39, 41,
    //         22, 44, 46,
    //         28, 49, 52,
    //         32, 54, 55,
    //         37, 38, 59,
    //         38, 42, 58,
    //         43, 47, 57,
    //         48, 53, 56,
    //     ];

    //     #[rustfmt::skip]
    //     let edges = vec![
    //         // 0
    //         Edge { reverse: 9, next: 1, face: 0, vertex: 0.into(), },
    //         Edge { reverse: 14, next: 2, face: 0, vertex: 1.into(), },
    //         Edge { reverse: 19, next: 3, face: 0, vertex: 2.into(), },
    //         Edge { reverse: 24, next: 4, face: 0, vertex: 3.into(), },
    //         Edge { reverse: 25, next: 0, face: 0, vertex: 4.into(), },

    //         // 1
    //         Edge { reverse: 29, next: 6, face: 1, vertex: 0.into(), },
    //         Edge { reverse: 50, next: 7, face: 1, vertex: 5.into(), },
    //         Edge { reverse: 30, next: 8, face: 1, vertex: 10.into(), },
    //         Edge { reverse: 10, next: 9, face: 1, vertex: 6.into(), },
    //         Edge { reverse: 0, next: 5, face: 1, vertex: 1.into(), },

    //         // 2
    //         Edge { reverse: 8, next: 11, face: 2, vertex: 1.into(), },
    //         Edge { reverse: 34, next: 12, face: 2, vertex: 6.into(), },
    //         Edge { reverse: 35, next: 13, face: 2, vertex: 11.into(), },
    //         Edge { reverse: 15, next: 14, face: 2, vertex: 7.into(), },
    //         Edge { reverse: 1, next: 10, face: 2, vertex: 2.into(), },

    //         // 3
    //         Edge { reverse: 2, next: 16, face: 3, vertex: 2.into(), },
    //         Edge { reverse: 13, next: 17, face: 3, vertex: 7.into(), },
    //         Edge { reverse: 39, next: 18, face: 3, vertex: 12.into(), },
    //         Edge { reverse: 40, next: 19, face: 3, vertex: 8.into(), },
    //         Edge { reverse: 20, next: 15, face: 3, vertex: 3.into(), },

    //         // 4
    //         Edge { reverse: 18, next: 21, face: 4, vertex: 3.into(), },
    //         Edge { reverse: 44, next: 22, face: 4, vertex: 8.into(), },
    //         Edge { reverse: 45, next: 23, face: 4, vertex: 13.into(), },
    //         Edge { reverse: 26, next: 24, face: 4, vertex: 9.into(), },
    //         Edge { reverse: 3, next: 20, face: 4, vertex: 4.into(), },

    //         // 5
    //         Edge { reverse: 4, next: 26, face: 5, vertex: 0.into(), },
    //         Edge { reverse: 23, next: 27, face: 5, vertex: 4.into(), },
    //         Edge { reverse: 49, next: 28, face: 5, vertex: 9.into(), },
    //         Edge { reverse: 51, next: 29, face: 5, vertex: 14.into(), },
    //         Edge { reverse: 5, next: 25, face: 5, vertex: 5.into(), },

    //         // 6
    //         Edge { reverse: 7, next: 31, face: 6, vertex: 6.into(), },
    //         Edge { reverse: 54, next: 32, face: 6, vertex: 10.into(), },
    //         Edge { reverse: 59, next: 33, face: 6, vertex: 15.into(), },
    //         Edge { reverse: 36, next: 34, face: 6, vertex: 16.into(), },
    //         Edge { reverse: 11, next: 30, face: 6, vertex: 11.into(), },

    //         // 7
    //         Edge { reverse: 12, next: 36, face: 7, vertex: 7.into(), },
    //         Edge { reverse: 33, next: 37, face: 7, vertex: 11.into(), },
    //         Edge { reverse: 58, next: 38, face: 7, vertex: 16.into(), },
    //         Edge { reverse: 41, next: 39, face: 7, vertex: 17.into(), },
    //         Edge { reverse: 16, next: 35, face: 7, vertex: 12.into(), },

    //         // 8
    //         Edge { reverse: 17, next: 41, face: 8, vertex: 8.into(), },
    //         Edge { reverse: 38, next: 42, face: 8, vertex: 12.into(), },
    //         Edge { reverse: 57, next: 43, face: 8, vertex: 17.into(), },
    //         Edge { reverse: 46, next: 44, face: 8, vertex: 18.into(), },
    //         Edge { reverse: 21, next: 40, face: 8, vertex: 13.into(), },

    //         // 9
    //         Edge { reverse: 22, next: 46, face: 9, vertex: 9.into(), },
    //         Edge { reverse: 43, next: 47, face: 9, vertex: 13.into(), },
    //         Edge { reverse: 56, next: 48, face: 9, vertex: 18.into(), },
    //         Edge { reverse: 52, next: 49, face: 9, vertex: 19.into(), },
    //         Edge { reverse: 27, next: 45, face: 9, vertex: 14.into(), },

    //         // 10
    //         Edge { reverse: 6, next: 51, face: 10, vertex: 10.into(), },
    //         Edge { reverse: 28, next: 52, face: 10, vertex: 5.into(), },
    //         Edge { reverse: 48, next: 53, face: 10, vertex: 14.into(), },
    //         Edge { reverse: 55, next: 54, face: 10, vertex: 19.into(), },
    //         Edge { reverse: 31, next: 50, face: 10, vertex: 15.into(), },

    //         // 11
    //         Edge { reverse: 53, next: 56, face: 11, vertex: 15.into(), },
    //         Edge { reverse: 47, next: 57, face: 11, vertex: 19.into(), },
    //         Edge { reverse: 42, next: 58, face: 11, vertex: 18.into(), },
    //         Edge { reverse: 37, next: 59, face: 11, vertex: 17.into(), },
    //         Edge { reverse: 32, next: 55, face: 11, vertex: 16.into(), },
    //     ];

    //     let faces: Vec<EdgeIndex> = vec![0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55];

    //     let mut normals = Vec::new();
    //     for face in faces.iter().copied() {
    //         let a = edges[face as usize];
    //         let b = edges[a.next as usize];
    //         let c = edges[b.next as usize];

    //         let ab = vertices[b.vertex] - vertices[a.vertex];
    //         let bc = vertices[c.vertex] - vertices[b.vertex];
    //         normals.push(ab.cross(bc).normalize());
    //     }

    //     Hull {
    //         vertices,
    //         vert_edge_slices,
    //         vert_edges,
    //         edges,
    //         faces,
    //         face_normals: normals,
    //     }
    // }
}

pub(crate) struct Face<'hull> {
    hull: &'hull Hull,
    first_edge: EdgeIndex,
    normal: Vec3,
}

impl<'hull> Face<'hull> {
    pub(crate) fn first_edge(&self) -> Edge<'hull> {
        Edge {
            hull: self.hull,
            idx: self.first_edge,
        }
    }

    pub(crate) fn iter_edges(&self) -> FaceEdges<'hull> {
        FaceEdges {
            hull: self.hull,
            first_edge: self.first_edge,
            cur_edge: Some(self.first_edge),
        }
    }

    pub(crate) fn centroid(&self) -> Vec3 {
        let mut edge = &self.hull.edges[self.first_edge as usize];
        let mut sum = Vec3A::ZERO;
        let mut count = 0;
        loop {
            sum += Vec3A::from(self.hull.vertices[edge.vertex]);
            count += 1;

            if edge.next == self.first_edge {
                break;
            }

            edge = &self.hull.edges[edge.next as usize];
        }

        (sum / count as f32).into()
    }

    pub(crate) fn plane(&self) -> Plane {
        let edge = &self.hull.edges[self.first_edge as usize];
        let vert = self.hull.vertices[edge.vertex];

        // TODO: add unchecked version, check verts and normals at hull construction time
        Plane::from_point_normal(vert, self.normal).unwrap()
    }
}

pub(crate) struct FaceEdges<'hull> {
    hull: &'hull Hull,
    first_edge: EdgeIndex,
    cur_edge: Option<EdgeIndex>,
}

impl<'hull> Iterator for FaceEdges<'hull> {
    type Item = Edge<'hull>;

    fn next(&mut self) -> Option<Self::Item> {
        let cur_edge = self.cur_edge?;

        let next_edge = self.hull.edges[cur_edge as usize].next;

        self.cur_edge = (next_edge != self.first_edge).then_some(next_edge);

        Some(Edge {
            hull: self.hull,
            idx: cur_edge,
        })
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct Edge<'hull> {
    hull: &'hull Hull,
    idx: EdgeIndex,
}

impl<'hull> PartialEq for Edge<'hull> {
    fn eq(&self, other: &Self) -> bool {
        ptr::eq(self.hull, other.hull) && self.idx == other.idx
    }
}

impl<'hull> Eq for Edge<'hull> {}

impl<'hull> Edge<'hull> {
    pub fn face(&self) -> Face<'hull> {
        Face {
            hull: self.hull,
            first_edge: self.hull.faces[self.hull.edges[self.idx as usize].face as usize],
            normal: self.face_normal().into(),
        }
    }

    pub fn face_normal(&self) -> Vec3A {
        self.hull.face_normals[self.hull.edges[self.idx as usize].face as usize].into()
    }

    /// Returns the directed edge in the opposite direction.
    #[inline]
    pub fn reverse(&self) -> Edge {
        Edge {
            hull: self.hull,
            idx: self.idx ^ 1,
        }
    }

    #[inline]
    fn start_local(&self) -> Vec3A {
        let vi = self.hull.edges[self.idx as usize].vertex;
        Vec3A::from(self.hull.vertices[vi])
    }

    /// Returns the position of the edge's starting vertex, transformed by `iso`.
    #[inline]
    pub fn start(&self, iso: Isometry) -> Vec3A {
        iso * self.start_local()
    }

    /// Returns the position of the edge's end vertex in local space.
    #[inline]
    pub fn end_local(&self) -> Vec3A {
        self.reverse().start_local()
    }

    /// Returns the position of the edge's end vertex, transformed by `iso`.
    #[inline]
    pub fn end(&self, iso: Isometry) -> Vec3A {
        iso * self.end_local()
    }

    /// Returns the direction vector of the edge in local space.
    #[inline]
    pub fn dir_local(&self) -> Vec3A {
        self.end_local() - self.start_local()
    }

    /// Returns the direction vector of the edge, rotated by `rotation`.
    #[inline]
    pub fn dir(&self, rotation: Quat) -> Vec3A {
        rotation * self.dir_local()
    }

    #[inline]
    pub fn segment_local(&self) -> Segment {
        Segment::new(self.start_local(), self.end_local())
    }

    #[inline]
    pub fn segment(&self, iso: Isometry) -> Segment {
        Segment::new(self.start(iso), self.end(iso))
    }

    #[inline]
    pub fn next(&self) -> Edge<'hull> {
        Edge {
            hull: self.hull,
            idx: self.hull.edges[self.idx as usize].next,
        }
    }

    #[inline]
    pub fn clip_plane(&self, iso: Isometry) -> Plane {
        let start = self.start(iso);

        // TODO: precompute and store the clip plane normal?
        let normal = iso.rotation * self.face_normal().cross(self.dir_local());
        let normal = normal.normalize();

        Plane::from_point_normal(start.into(), normal.into()).unwrap()
    }
}
