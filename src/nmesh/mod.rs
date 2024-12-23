//! A non-manifold mesh structure similar to Blender's BMesh.

use std::collections::{hash_map::Entry, HashMap, HashSet};

#[cfg(feature = "bevy")]
use bevy::reflect::{TypePath, TypeUuid};
use glam::{Mat3, Vec3};
use slotmap::{new_key_type, SlotMap};

#[cfg(all(feature = "bevy", feature = "bevy_mod_picking"))]
pub mod bevy_;
pub mod edit;

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

#[derive(Debug)]
#[cfg_attr(feature = "bevy", derive(TypePath, TypeUuid))]
#[cfg_attr(feature = "bevy", uuid = "dbcb459d-3d09-4de7-afa8-61b85ae9e46f")]
pub struct NMesh {
    // TODO: don't pub these
    pub verts: SlotMap<VertKey, NVert>,
    pub edges: SlotMap<EdgeKey, NEdge>,
    pub loops: SlotMap<LoopKey, NLoop>,
    pub faces: SlotMap<FaceKey, NFace>,
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

        let a = mesh.vert_create(Vec3::new(-0.5, -0.5, -0.5));
        let b = mesh.vert_create(Vec3::new(0.5, -0.5, -0.5));
        let c = mesh.vert_create(Vec3::new(0.5, -0.5, 0.5));
        let d = mesh.vert_create(Vec3::new(-0.5, -0.5, 0.5));
        let e = mesh.vert_create(Vec3::new(-0.5, 0.5, 0.5));
        let f = mesh.vert_create(Vec3::new(0.5, 0.5, 0.5));
        let g = mesh.vert_create(Vec3::new(0.5, 0.5, -0.5));
        let h = mesh.vert_create(Vec3::new(-0.5, 0.5, -0.5));

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

        let _abcd = mesh.face_create([(a, ab), (b, bc), (c, cd), (d, da)]);
        let _efgh = mesh.face_create([(e, ef), (f, fg), (g, gh), (h, he)]);
        let _adeh = mesh.face_create([(a, da), (d, de), (e, he), (h, ah)]);
        let _gfcb = mesh.face_create([(g, fg), (f, cf), (c, bc), (b, bg)]);
        let _fedc = mesh.face_create([(f, ef), (e, de), (d, cd), (c, cf)]);
        let _hgba = mesh.face_create([(h, gh), (g, bg), (b, ab), (a, ah)]);

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
                let is_new = vert_edge_set.insert(disk_edge);

                assert!(is_new, "{disk_edge:?} seen twice in disk of {vk:?}");
            }

            let mut rev_count = 0;
            for disk_edge in self.vert_disk_iter(vk).rev() {
                if !vert_edge_set.contains(&disk_edge) {
                    panic!("{disk_edge:?} present in forward disk but missing in reverse");
                }

                rev_count += 1;

                if rev_count > vert_edge_set.len() {
                    panic!("{ek:?} reverse disk contains more edges than forward disk");
                }
            }

            if rev_count < vert_edge_set.len() {
                panic!("{ek:?} reverse disk contains fewer edges than forward disk");
            }

            vert_edge_map.insert(vk, vert_edge_set);
        }

        let mut edge_set = HashMap::new();

        for (it, (ek, edge)) in self.edges.iter().enumerate() {
            println!("{ek:?}");

            assert!(it <= 1_000);

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
                println!("{loop_:?}");

                let is_new = edge_radial_set.insert(loop_);

                assert!(is_new, "{loop_:?} seen twice in radial cycle of {ek:?}");
            }

            let mut rev_count = 0;
            for (it, loop_) in self.edge_iter_radial(ek).rev().enumerate() {
                assert!(it < 1000);

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

    pub fn transform_by(&mut self, mat: Mat3) {
        for (_, vert) in self.verts.iter_mut() {
            vert.coord = mat * vert.coord;
        }
    }

    pub fn translate_by(&mut self, delta: Vec3) {
        for (_, vert) in self.verts.iter_mut() {
            vert.coord += delta;
        }
    }

    // vert methods ============================================================

    /// Creates a new vertex, returning its key.
    ///
    /// The vertex is not connected to the rest of the mesh.
    pub fn vert_create(&mut self, coord: Vec3) -> VertKey {
        self.verts.insert(NVert { coord, edge: None })
    }

    pub fn vert_set_coord(&mut self, vert: VertKey, coord: Vec3) {
        self.verts[vert].coord = coord;
    }

    fn vert_disk_contains(&self, vert: VertKey, edge: EdgeKey) -> bool {
        self.vert_disk_iter(vert).any(|e| e == edge)
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

    /// Create an edge between `v0` and `v1`, adding the new edge to both disks.
    ///
    /// Returns the key of the newly created edge.
    fn edge_create(&mut self, v0: VertKey, v1: VertKey) -> EdgeKey {
        let edge = self.edges.insert_with_key(|k| NEdge {
            v0,
            v0_disk: Links { prev: k, next: k },
            v1,
            v1_disk: Links { prev: k, next: k },
            loop_: None,
        });

        debug_assert!(!self.vert_disk_contains(v0, edge));
        self.vert_disk_add_edge(v0, edge);

        debug_assert!(!self.vert_disk_contains(v1, edge));
        self.vert_disk_add_edge(v1, edge);

        edge
    }

    fn edge_contains_vert(&self, edge: EdgeKey, vert: VertKey) -> bool {
        let edge = &self.edges[edge];
        edge.v0 == vert || edge.v1 == vert
    }

    fn edge_other_vert(&self, edge: EdgeKey, vert: VertKey) -> VertKey {
        let edge = &self.edges[edge];

        if edge.v0 == vert {
            edge.v1
        } else if edge.v1 == vert {
            edge.v0
        } else {
            panic!("{vert:?} is not a vertex of {edge:?}");
        }
    }

    /// Splits an edge, creating a new edge and a new vertex.
    pub fn edge_split(&mut self, edge: EdgeKey) -> (EdgeKey, VertKey) {
        // To split an edge:

        // Splitting an edge creates:
        // - One new vertex.
        //   - This can have any coordinates, but it's easiest to think about it being the midpoint
        //     of the original edge's vertices.
        // - One new edge.
        //   - The original edge goes from v0 to the new vertex. The new edge goes from the new
        //     vertex to v1.
        // - One new loop for every face in the original edge's radial cycle.
        //   - The new loops inherit the winding of the corresponding face.
        //   - The existing loop may need to be transferred to the new edge depending on the winding
        //     of the corresponding face.

        let v0 = self.edges[edge].v0;
        let v1 = self.edges[edge].v1;

        println!("v0 = {v0:?}");
        println!("v1 = {v1:?}");

        let midpoint = 0.5 * (self.verts[v0].coord + self.verts[v1].coord);
        let new_vert = self.vert_create(midpoint);

        println!("new_vert = {new_vert:?}");

        self.vert_disk_remove_edge(v1, edge);
        self.edges[edge].v1 = new_vert;
        self.vert_disk_add_edge(new_vert, edge);

        let new_edge = self.edge_create(new_vert, v1);

        println!("new_edge = {new_edge:?}");
        println!("===");

        let Some(mut old_loop) = self.edges[edge].loop_ else {
            return (new_edge, new_vert);
        };

        let first_loop = old_loop;

        loop {
            let radial_next = self.loop_radial_next(old_loop);

            // Determine the direction of the edge for this face.
            let (first_edge, second_edge);
            if self.loops[old_loop].vert == v0 {
                first_edge = edge;
                second_edge = new_edge;
            } else {
                debug_assert!(self.loops[old_loop].vert == v1);

                first_edge = new_edge;
                second_edge = edge;
            }

            let face = self.loops[old_loop].face;
            let face_next = self.loop_face_next(old_loop);

            let new_loop = self.loop_create(new_vert, second_edge, face);
            self.loops[old_loop].edge = first_edge;

            // Relink edges into the face.
            self.loop_face_set_next(old_loop, new_loop);
            self.loop_face_set_prev(new_loop, old_loop);
            self.loop_face_set_next(new_loop, face_next);
            self.loop_face_set_prev(face_next, new_loop);

            self.edge_radial_remove(first_edge, old_loop);

            // Relink edges into the appropriate radial loops.
            self.edge_radial_add(first_edge, old_loop);
            self.edge_radial_add(second_edge, new_loop);

            let new_radial_next = self.loop_radial_next(old_loop);

            println!("first_edge = {first_edge:?}");
            println!("second_edge = {second_edge:?}");
            println!("old_loop = {old_loop:?}");
            println!("new_loop = {new_loop:?}");
            println!("radial_next = {radial_next:?}");
            println!("new_radial_next = {new_radial_next:?}");
            println!("---");

            old_loop = radial_next;

            if old_loop == first_loop {
                break;
            }
        }

        (new_edge, new_vert)
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

    #[track_caller]
    fn edge_radial_add(&mut self, edge: EdgeKey, loop_: LoopKey) {
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

        println!("  last_loop = {last_loop:?}");
        println!("  loop_ = {loop_:?}");
        println!("  first_loop = {first_loop:?}");
    }

    fn edge_radial_remove(&mut self, edge: EdgeKey, loop_: LoopKey) {
        let prev = self.loop_radial_prev(loop_);
        let next = self.loop_radial_next(loop_);

        if prev == loop_ {
            debug_assert_eq!(next, loop_);
            debug_assert_eq!(self.edges[edge].loop_, Some(loop_));
            self.edges[edge].loop_ = None;
            return;
        }

        self.loop_radial_set_next(prev, next);
        self.loop_radial_set_prev(next, prev);
        debug_assert!(self.edges[edge].loop_.is_some());

        if self.edges[edge].loop_ == Some(loop_) {
            self.edges[edge].loop_ = Some(next);
        }
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

    fn face_boundary_add(&mut self, face: FaceKey, vert: VertKey, edge: EdgeKey) {
        let loop_ = self.loop_create(vert, edge, face);

        self.edge_radial_add(edge, loop_);
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
            self.edge_radial_add(edge, loop_);
        }

        if self.faces[face].len < 3 {
            panic!("faces must have at least 3 verts");
        }

        face
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

#[derive(Debug)]
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
        println!("{loop_:?}->next = {:?}", loops.next);

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
        let mut cube = NMesh::cube();

        assert_eq!(cube.verts.len(), 8);
        assert_eq!(cube.edges.len(), 12);
        assert_eq!(cube.faces.len(), 6);

        cube.validate();

        for (vk, _) in cube.verts.iter() {
            assert_eq!(cube.vert_count_incident_faces(vk), 3);
        }

        let (ek, _) = cube.edges.iter().next().unwrap();
        let loop_ = cube.edges[ek].loop_;
        let v0 = cube.edges[ek].v0;
        let v1 = cube.edges[ek].v1;

        cube.edge_split(ek);

        assert_eq!(cube.edges[ek].v0, v0);
        assert_ne!(cube.edges[ek].v1, v1);

        cube.validate();
    }
}
