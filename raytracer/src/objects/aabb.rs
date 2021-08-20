use crate::{Ray, Vec3};
use std::mem::swap;

pub struct AABB {
    pub minimum: Vec3,
    pub maximum: Vec3,
}

impl Clone for AABB {
    fn clone(&self) -> Self {
        Self {
            minimum: self.minimum,
            maximum: self.maximum,
        }
    }
}

impl AABB {
    pub fn new(minimum: Vec3, maximum: Vec3) -> Self {
        Self { minimum, maximum }
    }
    pub fn hit(&self, r: &Ray, t_min: f64, t_max: f64) -> bool {
        let mut min = t_min;
        let mut max = t_max;
        for a in 0..3 {
            let inv_d = 1.0 / r.dir[a];
            let mut t0 = (self.minimum[a] - r.orig[a]) * inv_d;
            let mut t1 = (self.maximum[a] - r.orig[a]) * inv_d;
            if inv_d < 0.0 {
                swap(&mut t0, &mut t1);
            }
            min = min.max(t0);
            max = max.min(t1);
            if max <= min {
                return false;
            }
        }
        true
    }
    pub fn surrounding_box(box0: &AABB, box1: &AABB) -> AABB {
        let small = Vec3::new(
            box0.minimum.x.min(box1.minimum.x),
            box0.minimum.y.min(box1.minimum.y),
            box0.minimum.z.min(box1.minimum.z),
        );
        let big = Vec3::new(
            box0.maximum.x.max(box1.maximum.x),
            box0.maximum.y.max(box1.maximum.y),
            box0.maximum.z.max(box1.maximum.z),
        );
        AABB::new(small, big)
    }
}
