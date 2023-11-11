use glam::Vec3;

/// An index into the vertex list.
pub(crate) type VertIndex = u32;
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
pub(crate) struct Edge {
    /// Index of the reversed edge.
    pub(crate) reverse: EdgeIndex,
    /// Index of the next edge, walking counter-clockwise.
    pub(crate) next: EdgeIndex,
    /// Index of the adjacent face.
    pub(crate) face: FaceIndex,
    /// Index of the originating vertex.
    pub(crate) vertex: VertIndex,
}

#[derive(Clone, Debug)]
pub struct ConvexPolyhedron {
    pub(crate) vertices: Vec<Vec3>,

    // TODO: all adjacency information can be stored in a single Vec<u32>.

    // Mapping from vertex index to slice of the `vert_edges` array.
    pub(crate) vert_edge_slices: Vec<ListSlice>,
    // Concatenated per-vertex edge lists.
    pub(crate) vert_edges: Vec<EdgeIndex>,

    pub(crate) edges: Vec<Edge>,

    // Faces are defined implicitly by their first directed edge.
    pub(crate) faces: Vec<EdgeIndex>,
    // Cached face normals.
    pub(crate) face_normals: Vec<Vec3>,
}

impl ConvexPolyhedron {
    // Iterates over the edges that contain this vertex.
    fn iter_vertex_edges(&self, vertex_id: u32) -> impl Iterator<Item = Edge> + '_ {
        let ListSlice { first, len } = self.vert_edge_slices[vertex_id as usize];
        self.vert_edges
            .iter()
            .skip(first as usize)
            .take(len as usize)
            .map(|&ei| self.edges[ei as usize])
    }

    // TODO: consider using a Dobkin-Kirkpatrick hierarchy.
    pub fn compute_supporting_point(&self, dir: Vec3) -> Vec3 {
        let mut vid = 0_u32;
        'next_vert: loop {
            let v = self.vertices[vid as usize];

            for edge in self.iter_vertex_edges(vid) {
                let other_vid = self.edges[edge.next as usize].vertex;
                let other = self.vertices[other_vid as usize];

                // Walk toward the extreme vertex.
                if (other - v).dot(dir) > f32::EPSILON {
                    vid = other_vid;
                    continue 'next_vert;
                }
            }

            break;
        }

        self.vertices[vid as usize]
    }

    pub fn tetrahedron() -> ConvexPolyhedron {
        let vertices = vec![
            Vec3::new((8.0_f32 / 9.0).sqrt(), -1.0 / 3.0, 0.0),
            Vec3::new(-(2.0_f32 / 9.0).sqrt(), -1.0 / 3.0, (2.0_f32 / 3.0).sqrt()),
            Vec3::new(-(2.0_f32 / 9.0).sqrt(), -1.0 / 3.0, -(2.0_f32 / 3.0).sqrt()),
            Vec3::new(0.0, 1.0, 0.0),
        ];

        // Faces are:
        // - 0, 1, 2 => 01, 12, 20
        // - 0, 3, 1 => 03, 31, 10
        // - 1, 3, 2 => 13, 32, 21
        // - 2, 3, 0 => 23, 30, 02

        let vert_edge_slices = vec![
            ListSlice { first: 0, len: 3 },
            ListSlice { first: 3, len: 3 },
            ListSlice { first: 6, len: 3 },
            ListSlice { first: 9, len: 3 },
        ];

        #[rustfmt::skip]
        let vert_edges = vec![
            // 0
            0, 3, 11,
            // 1
            1, 5, 6,
            // 2
            2, 8, 9,
            // 3
            4, 7, 10,
        ];

        #[rustfmt::skip]
        let edges = vec![
            Edge { reverse: 5, next: 1, face: 0, vertex: 0 }, // 0: 01
            Edge { reverse: 8, next: 2, face: 0, vertex: 1 }, // 1: 12
            Edge { reverse: 11, next: 0, face: 0, vertex: 2 }, // 2: 20
            Edge { reverse: 10, next: 4, face: 1, vertex: 0 }, // 3: 03
            Edge { reverse: 6, next: 5, face: 1, vertex: 3 }, // 4: 31
            Edge { reverse: 0, next: 3, face: 1, vertex: 1 }, // 5: 10

            Edge { reverse: 4, next: 7, face: 2, vertex: 1 }, // 6: 13
            Edge { reverse: 9, next: 8, face: 2, vertex: 3 }, // 7: 32
            Edge { reverse: 1, next: 6, face: 2, vertex: 2 }, // 8: 21
            Edge { reverse: 7, next: 10, face: 3, vertex: 2 }, // 9: 23
            Edge { reverse: 3, next: 11, face: 3, vertex: 3 }, // 10: 30
            Edge { reverse: 2, next: 9, face: 3, vertex: 0 }, // 11: 02
        ];

        let faces = vec![0, 3, 6, 9];

        let [p0, p1, p2, p3]: [Vec3; 4] = vertices.clone().try_into().unwrap();

        let face_normals = vec![
            (p1 - p0).cross(p2 - p1).normalize(),
            (p3 - p0).cross(p1 - p3).normalize(),
            (p3 - p1).cross(p2 - p3).normalize(),
            (p3 - p2).cross(p0 - p3).normalize(),
        ];

        ConvexPolyhedron {
            vertices,
            vert_edge_slices,
            vert_edges,
            edges,
            faces,
            face_normals,
        }
    }

    pub fn dodecahedron() -> ConvexPolyhedron {
        const PHI: f32 = 1.618034;
        const FRAC_1_PHI: f32 = 0.618034;

        #[rustfmt::skip]
        let vertices: Vec<Vec3> = vec![
            [        0.0,         PHI,  FRAC_1_PHI].into(),
            [       -1.0,         1.0,         1.0].into(),
            [-FRAC_1_PHI,         0.0,         PHI].into(),
            [ FRAC_1_PHI,         0.0,         PHI].into(),
            [        1.0,         1.0,         1.0].into(),
            [        0.0,         PHI, -FRAC_1_PHI].into(),
            [       -PHI,  FRAC_1_PHI,        -0.0].into(),
            [       -1.0,        -1.0,         1.0].into(),
            [        1.0,        -1.0,         1.0].into(),
            [        PHI,  FRAC_1_PHI,        -0.0].into(),
            [       -1.0,         1.0,        -1.0].into(),
            [       -PHI, -FRAC_1_PHI,        -0.0].into(),
            [        0.0,        -PHI,  FRAC_1_PHI].into(),
            [        PHI, -FRAC_1_PHI,        -0.0].into(),
            [        1.0,         1.0,        -1.0].into(),
            [-FRAC_1_PHI,         0.0,        -PHI].into(),
            [       -1.0,        -1.0,        -1.0].into(),
            [        0.0,        -PHI, -FRAC_1_PHI].into(),
            [        1.0,        -1.0,        -1.0].into(),
            [ FRAC_1_PHI,         0.0,        -PHI].into(),
        ];

        let vert_edge_slices = vec![
            ListSlice { first: 0, len: 3 },
            ListSlice { first: 3, len: 3 },
            ListSlice { first: 6, len: 3 },
            ListSlice { first: 9, len: 3 },
            ListSlice { first: 12, len: 3 },
            ListSlice { first: 15, len: 3 },
            ListSlice { first: 18, len: 3 },
            ListSlice { first: 21, len: 3 },
            ListSlice { first: 24, len: 3 },
            ListSlice { first: 27, len: 3 },
            ListSlice { first: 30, len: 3 },
            ListSlice { first: 33, len: 3 },
            ListSlice { first: 36, len: 3 },
            ListSlice { first: 39, len: 3 },
            ListSlice { first: 42, len: 3 },
            ListSlice { first: 45, len: 3 },
            ListSlice { first: 48, len: 3 },
            ListSlice { first: 51, len: 3 },
            ListSlice { first: 54, len: 3 },
            ListSlice { first: 57, len: 3 },
        ];

        #[rustfmt::skip]
        let vert_edges = vec![
             0,  5, 25,
             1,  9, 10,
             2, 14, 15,
             3, 19, 20,
             4, 24, 26,
             6, 29, 51,
             8, 11, 30,
            13, 16, 35,
            18, 21, 40,
            23, 27, 45,
             7, 31, 50,
            12, 34, 36,
            17, 39, 41,
            22, 44, 46,
            28, 49, 52,
            32, 54, 55,
            37, 38, 59,
            38, 42, 58,
            43, 47, 57,
            48, 53, 56,
        ];

        #[rustfmt::skip]
        let edges = vec![
            // 0
            Edge { reverse: 9, next: 1, face: 0, vertex: 0, },
            Edge { reverse: 14, next: 2, face: 0, vertex: 1, },
            Edge { reverse: 19, next: 3, face: 0, vertex: 2, },
            Edge { reverse: 24, next: 4, face: 0, vertex: 3, },
            Edge { reverse: 25, next: 0, face: 0, vertex: 4, },

            // 1
            Edge { reverse: 29, next: 6, face: 1, vertex: 0, },
            Edge { reverse: 50, next: 7, face: 1, vertex: 5, },
            Edge { reverse: 30, next: 8, face: 1, vertex: 10, },
            Edge { reverse: 10, next: 9, face: 1, vertex: 6, },
            Edge { reverse: 0, next: 5, face: 1, vertex: 1, },

            // 2
            Edge { reverse: 8, next: 11, face: 2, vertex: 1, },
            Edge { reverse: 34, next: 12, face: 2, vertex: 6, },
            Edge { reverse: 35, next: 13, face: 2, vertex: 11, },
            Edge { reverse: 15, next: 14, face: 2, vertex: 7, },
            Edge { reverse: 1, next: 10, face: 2, vertex: 2, },

            // 3
            Edge { reverse: 2, next: 16, face: 3, vertex: 2, },
            Edge { reverse: 13, next: 17, face: 3, vertex: 7, },
            Edge { reverse: 39, next: 18, face: 3, vertex: 12, },
            Edge { reverse: 40, next: 19, face: 3, vertex: 8, },
            Edge { reverse: 20, next: 15, face: 3, vertex: 3, },

            // 4
            Edge { reverse: 18, next: 21, face: 4, vertex: 3, },
            Edge { reverse: 44, next: 22, face: 4, vertex: 8, },
            Edge { reverse: 45, next: 23, face: 4, vertex: 13, },
            Edge { reverse: 26, next: 24, face: 4, vertex: 9, },
            Edge { reverse: 3, next: 20, face: 4, vertex: 4, },

            // 5
            Edge { reverse: 4, next: 26, face: 5, vertex: 0, },
            Edge { reverse: 23, next: 27, face: 5, vertex: 4, },
            Edge { reverse: 49, next: 28, face: 5, vertex: 9, },
            Edge { reverse: 51, next: 29, face: 5, vertex: 14, },
            Edge { reverse: 5, next: 25, face: 5, vertex: 5, },

            // 6
            Edge { reverse: 7, next: 31, face: 6, vertex: 6, },
            Edge { reverse: 54, next: 32, face: 6, vertex: 10, },
            Edge { reverse: 59, next: 33, face: 6, vertex: 15, },
            Edge { reverse: 36, next: 34, face: 6, vertex: 16, },
            Edge { reverse: 11, next: 30, face: 6, vertex: 11, },

            // 7
            Edge { reverse: 12, next: 36, face: 7, vertex: 7, },
            Edge { reverse: 33, next: 37, face: 7, vertex: 11, },
            Edge { reverse: 58, next: 38, face: 7, vertex: 16, },
            Edge { reverse: 41, next: 39, face: 7, vertex: 17, },
            Edge { reverse: 16, next: 35, face: 7, vertex: 12, },

            // 8
            Edge { reverse: 17, next: 41, face: 8, vertex: 8, },
            Edge { reverse: 38, next: 42, face: 8, vertex: 12, },
            Edge { reverse: 57, next: 43, face: 8, vertex: 17, },
            Edge { reverse: 46, next: 44, face: 8, vertex: 18, },
            Edge { reverse: 21, next: 40, face: 8, vertex: 13, },

            // 9
            Edge { reverse: 22, next: 46, face: 9, vertex: 9, },
            Edge { reverse: 43, next: 47, face: 9, vertex: 13, },
            Edge { reverse: 56, next: 48, face: 9, vertex: 18, },
            Edge { reverse: 52, next: 49, face: 9, vertex: 19, },
            Edge { reverse: 27, next: 45, face: 9, vertex: 14, },

            // 10
            Edge { reverse: 6, next: 51, face: 10, vertex: 10, },
            Edge { reverse: 28, next: 52, face: 10, vertex: 5, },
            Edge { reverse: 48, next: 53, face: 10, vertex: 14, },
            Edge { reverse: 55, next: 54, face: 10, vertex: 19, },
            Edge { reverse: 31, next: 50, face: 10, vertex: 15, },

            // 11
            Edge { reverse: 53, next: 56, face: 11, vertex: 15, },
            Edge { reverse: 47, next: 57, face: 11, vertex: 19, },
            Edge { reverse: 42, next: 58, face: 11, vertex: 18, },
            Edge { reverse: 37, next: 59, face: 11, vertex: 17, },
            Edge { reverse: 32, next: 55, face: 11, vertex: 16, },
        ];

        let faces: Vec<EdgeIndex> = vec![0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55];

        let mut normals = Vec::new();
        for face in faces.iter().copied() {
            let a = edges[face as usize];
            let b = edges[a.next as usize];
            let c = edges[b.next as usize];

            let ab = vertices[b.vertex as usize] - vertices[a.vertex as usize];
            let bc = vertices[c.vertex as usize] - vertices[b.vertex as usize];
            normals.push(ab.cross(bc).normalize());
        }

        ConvexPolyhedron {
            vertices,
            vert_edge_slices,
            vert_edges,
            edges,
            faces,
            face_normals: normals,
        }
    }
}