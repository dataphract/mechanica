//! A non-manifold mesh structure similar to Blender's BMesh.

use std::collections::{hash_map::Entry, HashMap, HashSet};

use bevy::reflect::{TypePath, TypeUuid};
use glam::Vec3;
use slotmap::{new_key_type, SlotMap};

pub mod bevy_;

#[derive(Debug, Default)]
struct Links<T> {
    prev: T,
    next: T,
}

new_key_type! {
    pub struct VertKey;
    pub struct EdgeKey;
    pub struct LoopKey;
    pub struct FaceKey;
}

#[derive(Debug, Default)]
pub struct NVert {
    coord: Vec3,

    // First edge
    edge: Option<EdgeKey>,
}

#[derive(Debug, Default)]
pub struct NEdge {
    v0: VertKey,
    v0_disk: Links<EdgeKey>,

    v1: VertKey,
    v1_disk: Links<EdgeKey>,

    loop_: Option<LoopKey>,
}

#[derive(Debug, Default)]
pub struct NFace {
    len: u32,

    normal: Vec3,
}

#[derive(Debug, Default)]
pub struct NLoop {
    face_links: Links<LoopKey>,
    radial_links: Links<LoopKey>,

    vert: VertKey,
    edge: EdgeKey,
    face: FaceKey,
}

#[derive(Debug, TypeUuid, TypePath)]
#[uuid = "dbcb459d-3d09-4de7-afa8-61b85ae9e46f"]
pub struct NMesh {
    verts: SlotMap<VertKey, NVert>,
    edges: SlotMap<EdgeKey, NEdge>,
    loops: SlotMap<LoopKey, NLoop>,
    faces: SlotMap<FaceKey, NFace>,
}

impl NMesh {
    fn new() -> NMesh {
        NMesh {
            verts: SlotMap::default(),
            edges: SlotMap::default(),
            loops: SlotMap::default(),
            faces: SlotMap::default(),
        }
    }

    pub fn cube() -> NMesh {
        let mut mesh = NMesh::new();

        let a = mesh.vert_create(Vec3::ZERO);
        let b = mesh.vert_create(Vec3::new(1.0, 0.0, 0.0));
        let c = mesh.vert_create(Vec3::new(1.0, 0.0, 1.0));
        let d = mesh.vert_create(Vec3::new(0.0, 0.0, 1.0));
        let e = mesh.vert_create(Vec3::new(0.0, 1.0, 1.0));
        let f = mesh.vert_create(Vec3::new(1.0, 1.0, 1.0));
        let g = mesh.vert_create(Vec3::new(1.0, 1.0, 0.0));
        let h = mesh.vert_create(Vec3::new(0.0, 1.0, 0.0));

        let ab = mesh.edge_create(a, b);
        let bc = mesh.edge_create(b, c);
        let cd = mesh.edge_create(c, d);
        let da = mesh.edge_create(d, a);
        let ef = mesh.edge_create(e, f);
        let fg = mesh.edge_create(f, g);
        let gh = mesh.edge_create(g, h);
        let he = mesh.edge_create(h, e);
        let ah = mesh.edge_create(a, h);
        let bg = mesh.edge_create(b, g);
        let cf = mesh.edge_create(c, f);
        let de = mesh.edge_create(d, e);

        let abcd = mesh.face_create([(a, ab), (b, bc), (c, cd), (d, da)]);
        let efgh = mesh.face_create([(e, ef), (f, fg), (g, gh), (h, he)]);
        let adeh = mesh.face_create([(a, da), (d, de), (e, he), (h, ah)]);
        let gfcb = mesh.face_create([(g, fg), (f, cf), (c, bc), (b, bg)]);
        let hgba = mesh.face_create([(h, gh), (g, bg), (b, ab), (a, ah)]);
        let fedc = mesh.face_create([(f, ef), (e, de), (d, cd), (c, cf)]);

        mesh
    }

    pub fn validate(&self) {
        // Properties to check:
        // - Verts:
        //   - Edges pointed to by a vert are incident to that vert
        //   - Disk cycles cover the same set of edges forward and in reverse
        // - Edges:
        //   - Edges have two distinct vertices
        //   - Edges are contained by the disks of both their vertices
        //   - Any two vertices are connected by at most one edge
        // - Faces:
        //   - Boundary loops/verts/edges are unique
        //   - Boundary loops point to face

        let mut vert_edge_map = HashMap::new();

        for (vk, vert) in self.verts.iter() {
            let Some(ek) = vert.edge else {
                continue;
            };

            let edge = &self.edges[ek];
            assert!(
                edge.v0 == vk || edge.v1 == vk,
                "{ek:?} pointed to by {vk:?} but not incident to it"
            );

            let mut vert_edge_set = HashSet::new();
            for disk_edge in self.vert_disk_iter(vk) {
                vert_edge_set.insert(disk_edge);
            }

            let mut rev_count = 0;
            for disk_edge in self.vert_disk_iter(vk).rev() {
                if !vert_edge_set.contains(&disk_edge) {
                    panic!("{disk_edge:?} present in forward disk but missing in reverse");
                }

                rev_count += 1;
            }

            assert_eq!(
                vert_edge_set.len(),
                rev_count,
                "{ek:?} reverse disk contains more edges than forward disk"
            );

            vert_edge_map.insert(vk, vert_edge_set);
        }

        let mut edge_set = HashMap::new();

        for (ek, edge) in self.edges.iter() {
            assert_ne!(edge.v0, edge.v1, "{ek:?} is a self-loop");

            let v0 = edge.v0.min(edge.v1);
            let v1 = edge.v0.max(edge.v1);
            match edge_set.entry((v0, v1)) {
                Entry::Occupied(o) => {
                    panic!("duplicate edge [{v0:?} -> {v1:?}]: {:?}, {ek:?}", o.get())
                }
                Entry::Vacant(v) => {
                    v.insert(ek);
                }
            }

            assert!(
                vert_edge_map[&v0].contains(&ek),
                "{ek:?} missing from disk of {v0:?}"
            );
            assert!(
                vert_edge_map[&v1].contains(&ek),
                "{ek:?} missing from disk of {v1:?}"
            );

            let mut edge_radial_set = HashSet::new();

            for loop_ in self.edge_iter_radial(ek) {
                edge_radial_set.insert(loop_);
            }

            let mut rev_count = 0;
            for loop_ in self.edge_iter_radial(ek).rev() {
                if !edge_radial_set.contains(&loop_) {
                    panic!("{loop_:?} present in forward radial cycle but missing in reverse");
                }

                rev_count += 1;
            }

            assert_eq!(
                edge_radial_set.len(),
                rev_count,
                "{ek:?} reverse radial cycle contains more loops than forward radial cycle"
            );
        }
    }

    // vert methods ============================================================

    /// Creates a new vertex, returning its key.
    ///
    /// The vertex is not connected to the rest of the mesh.
    pub fn vert_create(&mut self, coord: Vec3) -> VertKey {
        self.verts.insert(NVert { coord, edge: None })
    }

    /// Returns an iterator over the disk cycle of a vertex.
    pub fn vert_disk_iter(&self, vert: VertKey) -> DiskIter<'_> {
        DiskIter {
            mesh: self,
            vert,
            edges: self.verts[vert].edge.map(|e| DiskIterEdges {
                next: e,
                next_back: self.edge_disk_prev(e, vert),
            }),
        }
    }

    /// Adds an edge to the disk cycle of a vertex.
    pub fn vert_disk_add_edge(&mut self, vert: VertKey, new_edge: EdgeKey) {
        match self.verts[vert].edge {
            Some(first_edge) => {
                let last_edge = self.edge_disk_prev(first_edge, vert);

                self.edge_disk_set_next(last_edge, vert, new_edge);
                self.edge_disk_set_prev(new_edge, vert, last_edge);
                self.edge_disk_set_next(new_edge, vert, first_edge);
                self.edge_disk_set_prev(first_edge, vert, new_edge);
            }

            None => {
                self.edge_disk_set_prev(new_edge, vert, new_edge);
                self.edge_disk_set_next(new_edge, vert, new_edge);
                self.verts[vert].edge = Some(new_edge);
            }
        }
    }

    /// Removes an edge from the disk cycle of a vertex.
    pub fn vert_disk_remove_edge(&mut self, vert: VertKey, edge: EdgeKey) {
        let prev = self.edge_disk_prev(edge, vert);
        let next = self.edge_disk_next(edge, vert);

        if prev != edge {
            debug_assert_ne!(next, edge);
            self.edge_disk_set_next(prev, vert, next);
            self.edge_disk_set_prev(next, vert, prev);

            debug_assert!(self.verts[vert].edge.is_some());
            if self.verts[vert].edge == Some(edge) {
                self.verts[vert].edge = Some(next);
            }
        } else {
            self.verts[vert].edge = None;
        }
    }

    fn vert_edge_count_incident_faces(&self, vert: VertKey, edge: EdgeKey) -> usize {
        let Some(first_loop) = self.edges[edge].loop_ else {
            return 0;
        };

        let mut count = 0;
        let mut loop_ = first_loop;
        loop {
            if self.loops[loop_].vert == vert {
                count += 1;
            }

            loop_ = self.loops[loop_].radial_links.next;

            if loop_ == first_loop {
                break;
            }
        }

        count
    }

    /// Counts the number of faces incident to a vertex.
    pub fn vert_count_incident_faces(&self, vert: VertKey) -> usize {
        let Some(first_edge) = self.verts[vert].edge else {
            return 0;
        };

        let mut count = 0;
        let mut edge = first_edge;
        loop {
            count += self.vert_edge_count_incident_faces(vert, edge);

            edge = self.edge_disk_next(edge, vert);

            if edge == first_edge {
                break;
            }
        }

        count
    }

    // edge methods ============================================================

    fn edge_create(&mut self, v0: VertKey, v1: VertKey) -> EdgeKey {
        let edge = self.edges.insert_with_key(|k| NEdge {
            v0,
            v0_disk: Links { prev: k, next: k },
            v1,
            v1_disk: Links { prev: k, next: k },
            loop_: None,
        });

        self.vert_disk_add_edge(v0, edge);
        self.vert_disk_add_edge(v1, edge);

        edge
    }

    fn loop_create(&mut self, vert: VertKey, edge: EdgeKey, face: FaceKey) -> LoopKey {
        self.loops.insert_with_key(|k| NLoop {
            face_links: Links { prev: k, next: k },
            radial_links: Links { prev: k, next: k },
            vert,
            edge,
            face,
        })
    }

    #[inline]
    fn loop_face_prev(&self, loop_: LoopKey) -> LoopKey {
        self.loops[loop_].face_links.prev
    }

    #[inline]
    fn loop_face_next(&self, loop_: LoopKey) -> LoopKey {
        self.loops[loop_].face_links.next
    }

    #[inline]
    fn loop_face_set_next(&mut self, loop_: LoopKey, next: LoopKey) {
        self.loops[loop_].face_links.next = next;
    }

    #[inline]
    fn loop_face_set_prev(&mut self, loop_: LoopKey, prev: LoopKey) {
        self.loops[loop_].face_links.prev = prev;
    }

    #[inline]
    fn loop_radial_prev(&self, loop_: LoopKey) -> LoopKey {
        self.loops[loop_].radial_links.prev
    }

    #[inline]
    fn loop_radial_next(&self, loop_: LoopKey) -> LoopKey {
        self.loops[loop_].radial_links.next
    }

    #[inline]
    fn loop_radial_set_next(&mut self, loop_: LoopKey, next: LoopKey) {
        self.loops[loop_].radial_links.next = next;
    }

    #[inline]
    fn loop_radial_set_prev(&mut self, loop_: LoopKey, prev: LoopKey) {
        self.loops[loop_].radial_links.prev = prev;
    }

    fn radial_add(&mut self, edge: EdgeKey, loop_: LoopKey) {
        debug_assert_eq!(
            self.loops[loop_].edge, edge,
            "{loop_:?} cannot be assigned to {edge:?} as it already belongs to {:?}",
            self.loops[loop_].edge,
        );

        let Some(first_loop) = self.edges[edge].loop_ else {
            self.loop_radial_set_prev(loop_, loop_);
            self.loop_radial_set_next(loop_, loop_);
            self.edges[edge].loop_ = Some(loop_);
            return;
        };

        let last_loop = self.loop_radial_prev(first_loop);

        self.loop_radial_set_next(last_loop, loop_);
        self.loop_radial_set_prev(loop_, last_loop);
        self.loop_radial_set_next(loop_, first_loop);
        self.loop_radial_set_prev(first_loop, loop_);
    }

    fn face_boundary_add(&mut self, face: FaceKey, vert: VertKey, edge: EdgeKey) {
        let loop_ = self.loop_create(vert, edge, face);

        self.radial_add(edge, loop_);
    }

    fn face_create<I>(&mut self, verts_edges: I) -> FaceKey
    where
        I: IntoIterator<Item = (VertKey, EdgeKey)>,
    {
        let face = self.faces.insert(NFace {
            len: 0,
            normal: Vec3::ZERO,
        });

        for (vert, edge) in verts_edges {
            self.faces[face].len += 1;

            let loop_ = self.loop_create(vert, edge, face);
            self.radial_add(edge, loop_);
        }

        if self.faces[face].len < 3 {
            panic!("faces must have at least 3 verts");
        }

        face
    }

    fn edge_disk_links(&self, edge: EdgeKey, vert: VertKey) -> &Links<EdgeKey> {
        if self.edges[edge].v0 == vert {
            &self.edges[edge].v0_disk
        } else if self.edges[edge].v1 == vert {
            &self.edges[edge].v1_disk
        } else {
            panic!("{vert:?} does not belong to {edge:?}");
        }
    }

    fn edge_disk_links_mut(&mut self, edge: EdgeKey, vert: VertKey) -> &mut Links<EdgeKey> {
        if self.edges[edge].v0 == vert {
            &mut self.edges[edge].v0_disk
        } else if self.edges[edge].v1 == vert {
            &mut self.edges[edge].v1_disk
        } else {
            panic!("{vert:?} does not belong to {edge:?}");
        }
    }

    fn edge_disk_prev(&self, edge: EdgeKey, vert: VertKey) -> EdgeKey {
        self.edge_disk_links(edge, vert).prev
    }

    fn edge_disk_next(&self, edge: EdgeKey, vert: VertKey) -> EdgeKey {
        self.edge_disk_links(edge, vert).next
    }

    fn edge_disk_set_prev(&mut self, edge: EdgeKey, vert: VertKey, prev: EdgeKey) {
        self.edge_disk_links_mut(edge, vert).prev = prev;
    }

    fn edge_disk_set_next(&mut self, edge: EdgeKey, vert: VertKey, next: EdgeKey) {
        self.edge_disk_links_mut(edge, vert).next = next;
    }

    fn edge_iter_radial(&self, edge: EdgeKey) -> RadialIter<'_> {
        RadialIter {
            mesh: self,
            loops: self.edges[edge].loop_.map(|l| RadialIterLoops {
                next: l,
                next_back: self.loop_radial_prev(l),
            }),
        }
    }

    fn edge_count_incident_faces(&self, edge: EdgeKey, vert: VertKey) -> usize {
        let Some(first_loop) = self.edges[edge].loop_ else {
            return 0;
        };

        let mut count = 0;
        let mut loop_ = first_loop;
        loop {
            if self.loops[loop_].vert == vert {
                count += 1;
            }

            loop_ = self.loops[loop_].radial_links.next;

            if loop_ == first_loop {
                break;
            }
        }

        count
    }
}

pub struct DiskIter<'mesh> {
    mesh: &'mesh NMesh,
    vert: VertKey,
    edges: Option<DiskIterEdges>,
}

pub struct DiskIterEdges {
    next: EdgeKey,
    next_back: EdgeKey,
}

impl<'mesh> Iterator for DiskIter<'mesh> {
    type Item = EdgeKey;

    fn next(&mut self) -> Option<Self::Item> {
        let edges = self.edges.as_mut()?;

        let edge = edges.next;
        if edges.next == edges.next_back {
            self.edges = None;
            return Some(edge);
        }

        edges.next = self.mesh.edge_disk_next(edge, self.vert);

        Some(edge)
    }
}

impl<'mesh> DoubleEndedIterator for DiskIter<'mesh> {
    fn next_back(&mut self) -> Option<Self::Item> {
        let edges = self.edges.as_mut()?;

        let edge = edges.next_back;
        if edges.next == edges.next_back {
            self.edges = None;
            return Some(edge);
        }

        edges.next_back = self.mesh.edge_disk_prev(edge, self.vert);

        Some(edge)
    }
}

struct RadialIter<'mesh> {
    mesh: &'mesh NMesh,
    loops: Option<RadialIterLoops>,
}

struct RadialIterLoops {
    next: LoopKey,
    next_back: LoopKey,
}

impl<'mesh> Iterator for RadialIter<'mesh> {
    type Item = LoopKey;

    fn next(&mut self) -> Option<Self::Item> {
        let loops = self.loops.as_mut()?;

        let loop_ = loops.next;
        if loops.next == loops.next_back {
            self.loops = None;
            return Some(loop_);
        }

        loops.next = self.mesh.loop_radial_next(loop_);

        Some(loop_)
    }
}

impl<'mesh> DoubleEndedIterator for RadialIter<'mesh> {
    fn next_back(&mut self) -> Option<Self::Item> {
        let loops = self.loops.as_mut()?;

        let loop_ = loops.next_back;
        if loops.next == loops.next_back {
            self.loops = None;
            return Some(loop_);
        }

        loops.next_back = self.mesh.loop_radial_prev(loop_);

        Some(loop_)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn validate_cube() {
        let cube = NMesh::cube();

        assert_eq!(cube.verts.len(), 8);
        assert_eq!(cube.edges.len(), 12);
        assert_eq!(cube.faces.len(), 6);

        cube.validate();

        for (vk, _) in cube.verts.iter() {
            assert_eq!(cube.vert_count_incident_faces(vk), 3);
        }
    }
}
