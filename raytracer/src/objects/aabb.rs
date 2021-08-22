#![allow(dead_code)]
use crate::{Ray, Vec3};
use std::mem::swap;

#[derive(Copy, Clone, Default)]
pub struct AABB {
    pub minimum: Vec3,
    pub maximum: Vec3,
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
    pub fn is_intersect_box(&self, other: &AABB) -> bool {
        (self.minimum.x <= other.maximum.x && self.maximum.x >= other.minimum.x)
            && (self.minimum.y <= other.maximum.y && self.maximum.y >= other.minimum.y)
            && (self.minimum.z <= other.maximum.z && self.maximum.z >= other.minimum.z)
    }
    pub fn is_intersect_point(&self, other: &Vec3) -> bool {
        (other.x >= self.minimum.x && other.x <= self.maximum.x)
            && (other.y >= self.minimum.y && other.y <= self.maximum.y)
            && (other.z >= self.minimum.z && other.z <= self.maximum.z)
    }
    pub fn oct_sub_box(&self) -> Vec<AABB> {
        let (min_x, min_y, min_z) = self.minimum.xyz();
        let (max_x, max_y, max_z) = self.maximum.xyz();
        let (lx, ly, lz) = (
            (max_x - min_x) / 2.,
            (max_y - min_y) / 2.,
            (max_z - min_z) / 2.,
        );
        let maximum = |minimum: Vec3| Vec3::new(minimum.x + lx, minimum.y + ly, minimum.z + lz);
        let minimums = [
            self.minimum,
            Vec3::new(min_x + lx, min_y, min_z),
            Vec3::new(min_x, min_y + ly, min_z),
            Vec3::new(min_x, min_y, min_z + lz),
            Vec3::new(min_x + lx, min_y + ly, min_z),
            Vec3::new(min_x + lx, min_y, min_z + lz),
            Vec3::new(min_x, min_y + ly, min_z + lz),
            Vec3::new(min_x + lx, min_y + ly, min_z + lz),
        ];
        minimums
            .iter()
            .map(|minimum| Self::new(*minimum, maximum(*minimum)))
            .take(8)
            .collect::<Vec<AABB>>()
    }
}
