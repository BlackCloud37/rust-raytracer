use crate::material::Material;
use crate::objects::aabb::AABB;
use crate::objects::hit::{HitRecord, Hitable};
use crate::{Ray, Vec3};
use std::sync::Arc;

pub struct XYRectangle {
    pub xy0: (f64, f64),
    pub xy1: (f64, f64),
    pub z: f64,
    pub material: Arc<dyn Material>,
}

impl Hitable for XYRectangle {
    fn hit(&self, r: &Ray, t_min: f64, t_max: f64) -> Option<HitRecord> {
        let (x0, y0) = self.xy0;
        let (x1, y1) = self.xy1;

        let t = (self.z - r.orig.z) / r.dir.z;
        if t < t_min || t > t_max {
            return None;
        }
        let Vec3 { x, y, z: _ } = r.at(t);
        if x < x0 || x > x1 || y < y0 || y > y1 {
            return None;
        }
        Some(HitRecord::new(
            t,
            Vec3::new(0., 0., 1.),
            r,
            Arc::clone(&self.material),
            ((x - x0) / (x1 - x0), (y - y0) / (y1 - y0)),
        ))
    }
    fn bounding_box(&self) -> Option<AABB> {
        const BIAS: f64 = 0.0001;
        Some(AABB::new(
            Vec3::new(self.xy0.0, self.xy0.1, self.z - BIAS),
            Vec3::new(self.xy1.0, self.xy1.1, self.z + BIAS),
        ))
    }
}

pub struct XZRectangle {
    pub xz0: (f64, f64),
    pub xz1: (f64, f64),
    pub y: f64,
    pub material: Arc<dyn Material>,
}

impl Hitable for XZRectangle {
    fn hit(&self, r: &Ray, t_min: f64, t_max: f64) -> Option<HitRecord> {
        let (x0, z0) = self.xz0;
        let (x1, z1) = self.xz1;

        let t = (self.y - r.orig.y) / r.dir.y;
        if t < t_min || t > t_max {
            return None;
        }
        let Vec3 { x, y: _, z } = r.at(t);
        if x < x0 || x > x1 || z < z0 || z > z1 {
            return None;
        }
        Some(HitRecord::new(
            t,
            Vec3::new(0., 1., 0.),
            r,
            Arc::clone(&self.material),
            ((x - x0) / (x1 - x0), (z - z0) / (z1 - z0)),
        ))
    }
    fn bounding_box(&self) -> Option<AABB> {
        const BIAS: f64 = 0.0001;
        Some(AABB::new(
            Vec3::new(self.xz0.0, self.y - BIAS, self.xz0.1),
            Vec3::new(self.xz1.0, self.y + BIAS, self.xz1.1),
        ))
    }
}

pub struct YZRectangle {
    pub yz0: (f64, f64),
    pub yz1: (f64, f64),
    pub x: f64,
    pub material: Arc<dyn Material>,
}

impl Hitable for YZRectangle {
    fn hit(&self, r: &Ray, t_min: f64, t_max: f64) -> Option<HitRecord> {
        let (y0, z0) = self.yz0;
        let (y1, z1) = self.yz1;

        let t = (self.x - r.orig.x) / r.dir.x;
        if t < t_min || t > t_max {
            return None;
        }
        let Vec3 { x: _, y, z } = r.at(t);
        if y < y0 || y > y1 || z < z0 || z > z1 {
            return None;
        }
        Some(HitRecord::new(
            t,
            Vec3::new(1., 0., 0.),
            r,
            Arc::clone(&self.material),
            ((y - y0) / (y1 - y0), (z - z0) / (z1 - z0)),
        ))
    }
    fn bounding_box(&self) -> Option<AABB> {
        const BIAS: f64 = 0.0001;
        Some(AABB::new(
            Vec3::new(self.x - BIAS, self.yz0.0, self.yz0.1),
            Vec3::new(self.x + BIAS, self.yz1.0, self.yz1.1),
        ))
    }
}
