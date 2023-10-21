use bitflags::bitflags;
use hashbrown::HashMap;

use super::{EdgeKey, FaceKey, NMesh, VertKey};

bitflags! {
    struct EditFlags: u32 {
        const SELECTED = 0x0001;
        const VISIBLE  = 0x0002;
    }
}

impl Default for EditFlags {
    fn default() -> Self {
        EditFlags::VISIBLE
    }
}

pub struct NMeshEdit {
    mesh: NMesh,

    faces: HashMap<FaceKey, EditFlags>,
    edges: HashMap<EdgeKey, EditFlags>,
    verts: HashMap<VertKey, EditFlags>,
}

impl NMeshEdit {
    pub fn select_all(&mut self) {
        self.faces
            .values_mut()
            .for_each(|flags| flags.insert(EditFlags::SELECTED));
        self.edges
            .values_mut()
            .for_each(|flags| flags.insert(EditFlags::SELECTED));
        self.verts
            .values_mut()
            .for_each(|flags| flags.insert(EditFlags::SELECTED));
    }

    pub fn deselect_all(&mut self) {
        self.faces
            .values_mut()
            .for_each(|flags| flags.remove(EditFlags::SELECTED));
        self.edges
            .values_mut()
            .for_each(|flags| flags.remove(EditFlags::SELECTED));
        self.verts
            .values_mut()
            .for_each(|flags| flags.remove(EditFlags::SELECTED));
    }
}
