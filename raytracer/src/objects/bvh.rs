#![allow(dead_code)]
use crate::objects::aabb::AABB;
use crate::objects::hit::{HitRecord, Hitable};
use crate::Ray;
use rand::Rng;
use std::cmp::Ordering;
use std::sync::Arc;

/// This BVHNode should be constructed statically.
/// You should use procedural macro to generate code like this:
/// ```
/// let bvh = BVHNode::construct(
///     box BVHNode::construct(
///         box Sphere { .. }
///         box Sphere { .. }
///     ),
///     box BVHNode::construct(
///         box Sphere { .. }
///         box Sphere { .. }
///     )
/// )
/// ```
/// And you can put that `bvh` into your `HittableList`.
// pub struct BVHNode<L: Hitable, R: Hitable> {
//     pub left: L,
//     pub right: R,
//     pub bounding_box: AABB,
// }
pub struct BVHNode {
    pub left: Arc<dyn Hitable>,
    pub right: Arc<dyn Hitable>,
    pub bounding_box: AABB,
}
fn box_compare(a: &Arc<dyn Hitable>, b: &Arc<dyn Hitable>, axis: usize) -> Ordering {
    let box_a = a.bounding_box();
    let box_b = b.bounding_box();
    if let (Some(box_a), Some(box_b)) = (box_a, box_b) {
        return match box_a.minimum[axis] < box_b.minimum[axis] {
            true => Ordering::Less,
            _ => Ordering::Greater,
        };
    }
    panic!("No bounding box in bvh_node constructor.\n")
}

impl BVHNode {
    pub fn construct(left: Arc<dyn Hitable>, right: Arc<dyn Hitable>) -> Self {
        let box_left = left.bounding_box();
        let box_right = right.bounding_box();
        if let (Some(box_left), Some(box_right)) = (box_left, box_right) {
            return Self {
                bounding_box: AABB::surrounding_box(&box_left, &box_right),
                left,
                right,
            };
        }
        panic!("No bounding box in bvh_node constructor.\n")
    }

    pub fn new(src_objects: Vec<Arc<dyn Hitable>>) -> Self {
        let mut rng = rand::thread_rng();
        let axis = rng.gen_range(0..3);
        let comparator = |a: &Arc<dyn Hitable>, b: &Arc<dyn Hitable>| box_compare(a, b, axis);
        let object_span = src_objects.len();
        match object_span {
            1 => Self::construct(Arc::clone(&src_objects[0]), Arc::clone(&src_objects[0])),
            2 => {
                if comparator(&src_objects[0], &src_objects[1]).is_le() {
                    Self::construct(Arc::clone(&src_objects[0]), Arc::clone(&src_objects[1]))
                } else {
                    Self::construct(Arc::clone(&src_objects[1]), Arc::clone(&src_objects[0]))
                }
            }
            _ => {
                let mut objects = src_objects;
                objects.sort_by(comparator);
                let mid = object_span / 2;
                let left = Self::new(objects[0..mid].to_vec());
                let right = Self::new(objects[mid..].to_vec());
                Self::construct(Arc::new(left), Arc::new(right))
            }
        }
    }
}

impl Hitable for BVHNode {
    fn hit(&self, r: &Ray, t_min: f64, t_max: f64) -> Option<HitRecord> {
        if !self.bounding_box.hit(r, t_min, t_max) {
            return None;
        }
        let hit_left = self.left.hit(r, t_min, t_max);
        let left_t_max = if hit_left.is_some() {
            hit_left.as_ref().unwrap().t
        } else {
            t_max
        };
        let hit_right = self.right.hit(r, t_min, left_t_max);
        match hit_right.is_some() {
            true => hit_right,
            _ => hit_left,
        }
    }
    fn bounding_box(&self) -> Option<AABB> {
        Some(self.bounding_box)
    }
}
