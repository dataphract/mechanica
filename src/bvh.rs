// Implementation based on Erin Catto's 2019 GDC talk on Dynamic AABB Trees:
// https://box2d.org/files/ErinCatto_DynamicBVH_GDC2019.pdf

use std::{
    cmp::Ordering,
    collections::{BinaryHeap, VecDeque},
    hash::Hash,
    ops::{Index, IndexMut, Not},
};

use glam::{Vec3, Vec3A};
use hashbrown::HashMap;
use slotmap::{new_key_type, SlotMap};

use crate::{aabb::Aabb, Ray};

new_key_type! {
    struct NodeKey;
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Dir {
    Left = 0,
    Right = 1,
}

impl Not for Dir {
    type Output = Dir;

    #[inline]
    fn not(self) -> Self::Output {
        match self {
            Dir::Left => Dir::Right,
            Dir::Right => Dir::Left,
        }
    }
}

impl Index<Dir> for [NodeKey; 2] {
    type Output = NodeKey;

    #[inline]
    fn index(&self, index: Dir) -> &Self::Output {
        &self[index as usize]
    }
}

impl IndexMut<Dir> for [NodeKey; 2] {
    fn index_mut(&mut self, index: Dir) -> &mut Self::Output {
        &mut self[index as usize]
    }
}

/// A dynamic bounding-volume hierarchy implemented as an AABB tree.
#[derive(Default)]
pub struct Bvh<T> {
    root: Option<NodeKey>,
    keys: HashMap<T, NodeKey>,
    nodes: SlotMap<NodeKey, Node<T>>,
}

struct Swap {
    cost: f32,
    child: Dir,
    grandchild: Dir,
    new_aabb: CachedAabb,
    new_child_aabb: CachedAabb,
}

impl<T> Bvh<T>
where
    T: Copy + Hash + Eq,
{
    /// Constructs an empty BVH.
    pub fn new() -> Bvh<T> {
        Bvh {
            root: None,
            keys: HashMap::default(),
            nodes: SlotMap::with_key(),
        }
    }

    /// Constructs an empty BVH, with space for at least `cap` elements.
    pub fn with_capacity(cap: usize) -> Bvh<T> {
        Bvh {
            root: None,
            keys: HashMap::with_capacity(cap),
            nodes: SlotMap::with_capacity_and_key(2 * cap - 1),
        }
    }

    /// Returns `true` if the BVH contains no elements.
    #[inline]
    pub fn is_empty(&self) -> bool {
        #[cfg(debug)]
        {
            assert_eq!(self.keys.is_empty(), self.nodes.is_empty());
            assert_eq!(self.keys.is_empty(), self.root.is_none());
        }

        self.keys.is_empty()
    }

    /// Returns the number of items in the BVH.
    #[inline]
    pub fn len(&self) -> usize {
        #[cfg(debug)]
        {
            assert_eq!(self.keys.len(), self.nodes.len());
        }

        self.keys.len()
    }

    /// Returns the AABB associated with `key`.
    #[inline]
    pub fn get(&self, key: T) -> Option<&Aabb> {
        let &node = self.keys.get(&key)?;
        Some(&self.nodes[node].aabb.aabb)
    }

    /// Removes all items from the BVH.
    #[inline]
    pub fn clear(&mut self) {
        self.root = None;
        self.keys.clear();
        self.nodes.clear();
    }

    fn aabb(&self, node: NodeKey) -> &CachedAabb {
        &self.nodes[node].aabb
    }

    /// Computes the lower bound of the cost of choosing `sibling` as a sibling node for `new_aabb`.
    ///
    /// The lower bound is the cost that would be incurred if `sibling` were to be entirely
    /// contained by `new_aabb`. This is equivalent to the surface area of `new_aabb` plus the
    /// increase in surface area caused by refitting all ancestors of `sibling` to fit `new_aabb`.
    fn compute_sibling_cost_lower_bound(&self, sibling: NodeKey, new_aabb: &CachedAabb) -> f32 {
        // TODO: lower bound inherited cost can be memoized for all ancestor nodes.

        let mut cost = new_aabb.surface_area;

        let mut parent = self.nodes[sibling].parent;
        let mut refit = new_aabb.clone();

        while let Some(par) = parent {
            refit = refit.union(&self.nodes[par].aabb);
            cost += refit.surface_area - self.nodes[par].aabb.surface_area;
            parent = self.nodes[par].parent;
        }

        cost
    }

    /// Computes the cost of choosing `sibling` as a sibling node for `new_aabb`.
    fn compute_sibling_cost(&self, sibling: NodeKey, new_aabb: &CachedAabb) -> f32 {
        // The direct cost is the surface area of the potential parent AABB.
        let sibling_aabb = &self.nodes[sibling].aabb;
        let union = sibling_aabb.union(new_aabb);
        let direct_cost = union.surface_area;

        // The inherited cost is the additional surface area after refitting all ancestors.
        let mut inherited_cost = 0.0;
        let mut parent = self.nodes[sibling].parent;
        let mut refit = union;

        while let Some(par) = parent {
            refit = refit.union(&self.nodes[par].aabb);
            inherited_cost += refit.surface_area - self.nodes[par].aabb.surface_area;

            parent = self.nodes[par].parent;
        }

        direct_cost + inherited_cost
    }

    fn refit_node(&mut self, node: NodeKey) {
        let children = match &mut self.nodes[node].kind {
            NodeKind::Branch(b) => b.children,

            // Nothing to refit.
            NodeKind::Leaf(_) => return,
        };

        let left = &self.nodes[children[0]].aabb;
        let right = &self.nodes[children[1]].aabb;
        self.nodes[node].aabb = left.union(right);
    }

    /// Computes a rotation of the subtree rooted at `node`.
    fn compute_rotation(&self, node: NodeKey, child_dir: Dir, grandchild_dir: Dir) -> Option<Swap> {
        let children = self.branch(node).children;

        let swap_child = children[child_dir];
        let stay_child = children[!child_dir];

        let NodeKind::Branch(stay_branch) = &self.nodes[stay_child].kind else {
            // Opposite child has no children, can't swap.
            return None;
        };

        let swap_grandchild = stay_branch.children[grandchild_dir];
        let stay_grandchild = stay_branch.children[!grandchild_dir];

        let new_child_aabb = self.nodes[stay_grandchild]
            .aabb
            .union(&self.nodes[swap_child].aabb);

        let new_aabb = self.nodes[swap_grandchild].aabb.union(&new_child_aabb);

        let cost = new_child_aabb.surface_area - self.nodes[stay_child].aabb.surface_area
            + new_aabb.surface_area
            - self.nodes[node].aabb.surface_area;

        Some(Swap {
            cost,
            child: child_dir,
            grandchild: grandchild_dir,
            new_aabb,
            new_child_aabb,
        })
    }

    /// Applies the best possible rotation for the subtree rooted at `node`.
    ///
    /// If all possible rotations would increase the tree's cost, no rotation is applied.
    fn apply_best_rotation(&mut self, node: NodeKey) {
        let mut best_cost = 0.0;
        let mut best_swap: Option<Swap> = None;

        for child_dir in [Dir::Left, Dir::Right] {
            for grandchild_dir in [Dir::Left, Dir::Right] {
                let Some(swap) = self.compute_rotation(node, child_dir, grandchild_dir) else {
                    continue;
                };

                if swap.cost < best_cost {
                    best_cost = swap.cost;
                    best_swap = Some(swap);
                }
            }
        }

        if let Some(swap) = best_swap {
            let swap_child = self.branch(node).children[swap.child];
            let stay_child = self.branch(node).children[!swap.child];
            let swap_grandchild = self.branch(stay_child).children[swap.grandchild];

            self.branch_mut(node).children[swap.child] = swap_grandchild;
            self.branch_mut(stay_child).children[swap.grandchild] = swap_child;

            self.nodes[stay_child].aabb = swap.new_child_aabb;
            self.nodes[node].aabb = swap.new_aabb;
        }
    }

    /// Ascends the tree from `node` to the root, refitting and applying rotations.
    fn refit_and_rotate(&mut self, node: NodeKey) {
        self.refit_node(node);
        self.apply_best_rotation(node);

        let mut ancestor = self.nodes[node].parent;

        while let Some(anc) = ancestor {
            self.refit_node(anc);
            self.apply_best_rotation(anc);

            ancestor = self.nodes[anc].parent;
        }
    }

    /// Inserts `item` into the BVH, associated with the axis-aligned bounding box `aabb`.
    ///
    /// If `item` was already an element of the BVH, the previous AABB is removed and returned.
    pub fn insert(&mut self, item: T, aabb: Aabb) -> Option<Aabb> {
        let aabb = CachedAabb::new(aabb);

        let old_aabb = self.remove(&item);

        let node_key = self.nodes.insert(Node {
            aabb: aabb.clone(),
            parent: None,
            kind: NodeKind::Leaf(Leaf { item }),
        });

        self.keys.insert(item, node_key);

        // If the tree is empty, set `item` as root and return.
        let Some(root) = self.root else {
            self.root = Some(node_key);
            return old_aabb;
        };

        let root_sibling_aabb = self.nodes[root].aabb.union(&aabb);
        let root_sibling_cost = root_sibling_aabb.surface_area;

        // Priority queue of nodes, ordered by lower bound on insertion cost.
        //
        // TODO: cache this in the struct to avoid reallocating on every insertion.
        let mut queue = BinaryHeap::with_capacity(self.keys.len());

        if let NodeKind::Branch(b) = &self.nodes[root].kind {
            for &child in b.children.iter() {
                let lower_bound = self.compute_sibling_cost_lower_bound(child, &aabb);
                if lower_bound < root_sibling_cost {
                    queue.push(NodeCost {
                        lower_bound,
                        key: child,
                    });
                }
            }
        }

        let mut best_sibling = root;
        let mut best_sibling_cost = root_sibling_cost;

        // Search for the optimal insertion location via branch and bound.
        while let Some(cand) = queue.pop() {
            if cand.lower_bound >= best_sibling_cost {
                // The best cost may have decreased since `cand` was pushed onto the queue. If it
                // decreased to be lower than the candidate's lower bound, the optimal solution has
                // been found,
                break;
            }

            let cost = self.compute_sibling_cost(cand.key, &aabb);
            if cost < best_sibling_cost {
                best_sibling = cand.key;
                best_sibling_cost = cost;
            }

            if let NodeKind::Branch(b) = &self.nodes[cand.key].kind {
                for &child in b.children.iter() {
                    let lower_bound = self.compute_sibling_cost_lower_bound(child, &aabb);
                    if lower_bound < best_sibling_cost {
                        queue.push(NodeCost {
                            lower_bound,
                            key: child,
                        });
                    }
                }
            }
        }

        let sibling = best_sibling;

        // Create the new parent node.
        let new_parent_aabb = aabb.union(&self.nodes[best_sibling].aabb);
        let mut new_parent_node = Node {
            aabb: new_parent_aabb,
            parent: None,
            kind: NodeKind::Branch(Branch {
                children: [node_key, sibling],
            }),
        };

        let old_parent = self.nodes[sibling].parent;
        let Some(old_parent) = old_parent else {
            // If the sibling had no parent, the new node becomes the root.
            let new_parent = self.nodes.insert(new_parent_node);

            self.root = Some(new_parent);
            self.nodes[node_key].parent = Some(new_parent);

            return old_aabb;
        };

        // Link the new parent node into the tree.
        new_parent_node.parent = Some(old_parent);
        let new_parent = self.nodes.insert(new_parent_node);

        let dir = self.which_child(old_parent, sibling);
        self.branch_mut(old_parent).children[dir] = new_parent;
        self.nodes[node_key].parent = Some(new_parent);
        self.nodes[sibling].parent = Some(new_parent);

        self.refit_and_rotate(new_parent);

        old_aabb
    }

    pub fn remove(&mut self, item: &T) -> Option<Aabb> {
        let node_key = self.keys.remove(item)?;
        let removed_aabb = self.nodes.remove(node_key).unwrap().aabb.aabb;

        // Get the removed node's parent.
        let Some(parent) = self.nodes[node_key].parent else {
            // If the removed node was the root, the tree becomes empty.
            debug_assert_eq!(self.root, Some(node_key));
            debug_assert!(self.keys.is_empty());
            debug_assert!(self.nodes.is_empty());
            self.root = None;
            return Some(removed_aabb);
        };

        // Get the sibling of the removed node.
        let parent_branch = self.branch_mut(parent);
        let sibling = if parent_branch.children[0] == node_key {
            parent_branch.children[1]
        } else {
            debug_assert_eq!(parent_branch.children[1], node_key);
            parent_branch.children[0]
        };

        // Remove the parent node.
        let new_parent = self.nodes[parent].parent;
        self.nodes.remove(parent);

        let Some(new_parent) = new_parent else {
            // If the removed parent was the root, the sibling becomes the root.
            self.nodes[sibling].parent = None;

            debug_assert_eq!(self.root, Some(parent));
            self.root = Some(sibling);
            return Some(removed_aabb);
        };

        // Reparent the sibling node.
        self.nodes[sibling].parent = Some(new_parent);
        let new_parent_branch = self.branch_mut(new_parent);
        if new_parent_branch.children[0] == parent {
            new_parent_branch.children[0] = sibling;
        } else {
            debug_assert_eq!(new_parent_branch.children[1], parent);
            new_parent_branch.children[1] = sibling;
        }

        // Refit all ancestors.
        self.refit_and_rotate(new_parent);

        Some(removed_aabb)
    }

    /// Returns an iterator over every key whose associated AABB contains `point`.
    pub fn point_query(&self, point: Vec3) -> BvhQuery<'_, T, PointAabbQuery> {
        BvhQuery {
            tree: self,
            node: self.root,
            query: PointAabbQuery { point },
            came_from: CameFrom::Parent,
        }
    }

    /// Returns an iterator over every key whose associated AABB is intersected by `ray`.
    pub fn ray_query(&self, ray: Ray) -> BvhQuery<'_, T, RayAabbQuery> {
        BvhQuery {
            tree: self,
            node: self.root,
            query: RayAabbQuery::new(ray),
            came_from: CameFrom::Parent,
        }
    }

    fn which_child(&self, parent: NodeKey, child: NodeKey) -> Dir {
        let NodeKind::Branch(b) = &self.nodes[parent].kind else {
            panic!("{parent:?} is not a branch");
        };

        if b.children[Dir::Left] == child {
            Dir::Left
        } else if b.children[Dir::Right] == child {
            Dir::Right
        } else {
            panic!("{child:?} is not a child of {parent:?}");
        }
    }

    fn parent_and_dir(&self, child: NodeKey) -> Option<(NodeKey, Dir)> {
        let parent = self.nodes[child].parent?;
        Some((parent, self.which_child(parent, child)))
    }

    fn branch(&self, key: NodeKey) -> &Branch {
        match &self.nodes[key].kind {
            NodeKind::Branch(b) => b,
            NodeKind::Leaf(_) => panic!("not a branch"),
        }
    }

    fn branch_mut(&mut self, key: NodeKey) -> &mut Branch {
        match &mut self.nodes[key].kind {
            NodeKind::Branch(b) => b,
            NodeKind::Leaf(_) => panic!("not a branch"),
        }
    }

    fn assert_invariants(&self) {
        let Some(node) = self.root else {
            assert!(self.keys.is_empty());
            assert!(self.nodes.is_empty());
            return;
        };

        let mut queue = VecDeque::new();
        queue.push_back(node);

        while let Some(node) = queue.pop_front() {
            match &self.nodes[node].kind {
                NodeKind::Branch(b) => {
                    let [left, right] = b.children;

                    queue.push_back(left);
                    queue.push_back(right);

                    assert!(self.aabb(node).aabb.contains_aabb(&self.aabb(left).aabb));
                    assert!(self.aabb(node).aabb.contains_aabb(&self.aabb(right).aabb));
                }

                NodeKind::Leaf(l) => {
                    assert!(self.keys.contains_key(&l.item));
                }
            }
        }
    }
}

struct Node<T> {
    /// The bounding box of the node.
    aabb: CachedAabb,

    /// The parent node, if any.
    parent: Option<NodeKey>,

    /// Branch or leaf data, depending on the node kind.
    kind: NodeKind<T>,
}

/// An AABB with surface area precomputed.
#[derive(Clone, Debug)]
struct CachedAabb {
    aabb: Aabb,
    surface_area: f32,
}

impl CachedAabb {
    fn new(aabb: Aabb) -> CachedAabb {
        CachedAabb {
            surface_area: aabb.surface_area(),
            aabb,
        }
    }

    #[inline]
    fn union(&self, other: &CachedAabb) -> CachedAabb {
        CachedAabb::new(self.aabb.union(&other.aabb))
    }

    #[inline]
    fn contains_point(&self, point: Vec3) -> bool {
        self.aabb.contains_point(point)
    }
}

enum NodeKind<T> {
    Branch(Branch),
    Leaf(Leaf<T>),
}

struct Branch {
    children: [NodeKey; 2],
}

struct Leaf<T> {
    item: T,
}

struct NodeCost {
    lower_bound: f32,
    key: NodeKey,
}

impl PartialEq for NodeCost {
    fn eq(&self, other: &Self) -> bool {
        self.lower_bound == other.lower_bound
    }
}

impl Eq for NodeCost {}

impl PartialOrd for NodeCost {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for NodeCost {
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(other).unwrap()
    }
}

#[derive(Copy, Clone, Debug)]
enum CameFrom {
    Left,
    Right,
    Parent,
}

impl From<Dir> for CameFrom {
    fn from(dir: Dir) -> Self {
        match dir {
            Dir::Left => CameFrom::Left,
            Dir::Right => CameFrom::Right,
        }
    }
}

pub trait AabbQuery {
    fn aabb_query(&self, aabb: &Aabb) -> bool;
}

pub struct PointAabbQuery {
    point: Vec3,
}

impl AabbQuery for PointAabbQuery {
    #[inline]
    fn aabb_query(&self, aabb: &Aabb) -> bool {
        aabb.contains_point(self.point)
    }
}

pub struct RayAabbQuery {
    /// Ray origin.
    origin: Vec3A,
    /// Component-wise reciprocal of ray direction.
    dir_inv: Vec3A,
}

impl RayAabbQuery {
    fn new(ray: Ray) -> RayAabbQuery {
        RayAabbQuery {
            origin: ray.origin.into(),
            dir_inv: ray.dir.recip().into(),
        }
    }
}

impl AabbQuery for RayAabbQuery {
    fn aabb_query(&self, aabb: &Aabb) -> bool {
        // Line-plane intersection:
        //
        //     t = -(plane.normal · ray.origin + plane.dist) / (plane.normal · ray.dir)
        //
        // Since each plane is orthogonal to an axis, we have
        //
        //     t[k] = -(1 * ray.origin[k] + plane.dist) / (1 * ray.dir[k])
        //          = plane.dist - ray.origin[k] * (ray.dir[k] ^ -1)
        //
        // Since all dot products are removed, this can be vectorized.
        let t0 = (aabb.mins_a() - self.origin) * self.dir_inv;
        let t1 = (aabb.maxs_a() - self.origin) * self.dir_inv;

        // Minimum and maximum t-values for each axis.
        let tmins = t0.min(t1);
        let tmaxs = t0.max(t1);

        // Clamp tnear to minimum of zero to prevent intersection with boxes behind the ray.
        let tnear = tmins.max_element().max(0.0);
        let tfar = tmaxs.min_element();

        tfar >= tnear
    }
}

pub struct BvhQuery<'tree, T, Q> {
    tree: &'tree Bvh<T>,
    node: Option<NodeKey>,
    query: Q,
    came_from: CameFrom,
}

impl<'tree, T, Q> BvhQuery<'tree, T, Q>
where
    T: Copy + Hash + Eq,
    Q: AabbQuery,
{
    fn ascend(&mut self) {
        let Some(node) = self.node else {
            return;
        };

        let Some((parent, dir)) = self.tree.parent_and_dir(node) else {
            self.node = None;
            return;
        };

        self.came_from = dir.into();
        self.node = Some(parent);
    }

    fn descend(&mut self, dir: Dir) {
        let Some(node) = self.node else {
            return;
        };

        self.came_from = CameFrom::Parent;
        self.node = Some(self.tree.branch(node).children[dir]);
    }
}

macro_rules! aabb_tree_query {
    ($outer:ident : $inner:ident) => {
        pub struct $outer<'tree, T> {
            inner: BvhQuery<'tree, T, $inner>,
        }

        impl<'tree, T> Iterator for $outer<'tree, T>
        where
            T: Copy + Eq + Hash,
        {
            type Item = T;

            #[inline]
            fn next(&mut self) -> Option<Self::Item> {
                self.inner.next()
            }
        }
    };
}

aabb_tree_query!(PointBvhQuery: PointAabbQuery);
aabb_tree_query!(RayBvhQuery: RayAabbQuery);

impl<'tree, T, Q> Iterator for BvhQuery<'tree, T, Q>
where
    T: Copy + Hash + Eq,
    Q: AabbQuery,
{
    type Item = T;

    fn next(&mut self) -> Option<Self::Item> {
        while let Some(node) = self.node {
            match self.came_from {
                CameFrom::Left => {
                    self.descend(Dir::Right);
                    continue;
                }

                CameFrom::Right => {
                    self.ascend();
                    continue;
                }

                CameFrom::Parent => (),
            }

            if !self.query.aabb_query(&self.tree.aabb(node).aabb) {
                self.ascend();
                continue;
            }

            match &self.tree.nodes[node].kind {
                NodeKind::Branch(_) => self.descend(Dir::Left),
                NodeKind::Leaf(l) => {
                    self.came_from = CameFrom::Right;
                    return Some(l.item);
                }
            }
        }

        None
    }
}

#[cfg(test)]
mod tests {
    use glam::Vec3;
    use hashbrown::HashSet;

    use super::*;

    fn unit_box(origin: Vec3) -> Aabb {
        Aabb::new(origin, Vec3::splat(0.5))
    }

    #[test]
    fn is_empty_and_len() {
        let mut tree = Bvh::new();
        tree.assert_invariants();
        assert!(tree.is_empty());

        tree.insert("a", Aabb::new(Vec3::new(2.0, 0.0, 0.0), Vec3::ONE));
        tree.assert_invariants();
        assert_eq!(tree.len(), 1);

        tree.insert("b", Aabb::new(Vec3::new(-2.0, 0.0, 0.0), Vec3::ONE));
        tree.assert_invariants();
        assert_eq!(tree.len(), 2);
    }

    #[test]
    fn single_box() {
        let mut tree = Bvh::new();

        tree.insert("a", Aabb::new(Vec3::ZERO, Vec3::ONE));
        assert_eq!(tree.point_query(Vec3::ZERO).collect::<Vec<_>>(), vec!["a"]);
        assert_eq!(tree.point_query(2.0 * Vec3::ONE).count(), 0);
    }

    #[test]
    fn two_boxes() {
        let mut tree = Bvh::new();

        // Disjoint boxes
        tree.insert("a", unit_box(-Vec3::X));
        tree.insert("b", unit_box(Vec3::X));
        tree.assert_invariants();

        assert_eq!(tree.point_query(-Vec3::X).collect::<Vec<_>>(), vec!["a"]);
        assert_eq!(tree.point_query(Vec3::X).collect::<Vec<_>>(), vec!["b"]);
        assert_eq!(tree.point_query(Vec3::ZERO).count(), 0);

        tree.clear();

        // Intersecting boxes
        tree.insert("c", unit_box(-0.25 * Vec3::X));
        tree.insert("d", unit_box(0.25 * Vec3::X));
        tree.assert_invariants();

        assert_eq!(
            tree.point_query(-0.5 * Vec3::X).collect::<Vec<_>>(),
            vec!["c"]
        );
        assert_eq!(
            tree.point_query(0.5 * Vec3::X).collect::<Vec<_>>(),
            vec!["d"]
        );
        assert_eq!(
            tree.point_query(Vec3::ZERO).collect::<HashSet<_>>(),
            HashSet::from_iter(["c", "d"])
        );
    }

    #[test]
    #[ignore]
    fn sorted_inputs_not_degenerate() {
        let mut tree = Bvh::new();

        // Without rotation, this produces a degenerate tree like
        //
        //  x-x-x-x-4
        //  | | | |
        //  0 1 2 3
        //
        for x in 0..5 {
            let y = 2.0 * x as f32;

            tree.insert(x, unit_box(y * Vec3::X));
        }
    }
}
