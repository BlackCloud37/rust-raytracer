use crate::material::Material;
use crate::objects::hit::{HitRecord, Hitable};
use crate::{Ray, Vec3};
use std::sync::Arc;

pub struct Sphere {
    pub center: Vec3,
    pub radius: f64,
    pub material: Arc<dyn Material + Send + Sync>,
}

impl Hitable for Sphere {
    fn hit(&self, r: &Ray, t_min: f64, t_max: f64) -> Option<HitRecord> {
        let oc = r.orig - self.center;
        let a = r.dir.squared_length();
        let half_b = oc * r.dir;
        let c = oc.squared_length() - self.radius * self.radius;
        let discriminant = half_b.powf(2.0) - a * c;
        if discriminant < 0. {
            return None;
        }
        let sqrt_d = discriminant.sqrt();

        // Find the nearest root that lies in the acceptable range.
        let in_range = |root| root >= t_min && root <= t_max;
        let mut root = (-half_b - sqrt_d) / a;
        if !in_range(root) {
            root = (-half_b + sqrt_d) / a;
        }
        if !in_range(root) {
            return None;
        }

        let p = r.at(root);
        let outward_normal = (p - self.center) / self.radius;
        let rec = HitRecord::new(root, outward_normal, r, Arc::clone(&self.material));
        Some(rec)
    }
}

pub struct MovingSphere {
    pub center0: Vec3,
    pub center1: Vec3,
    pub time0: f64,
    pub time1: f64,
    pub radius: f64,
    pub material: Arc<dyn Material + Send + Sync>,
}

impl MovingSphere {
    pub fn center(&self, time: f64) -> Vec3 {
        self.center0
            + ((time - self.time0) / (self.time1 - self.time0)) * (self.center1 - self.center0)
    }
}

impl Hitable for MovingSphere {
    fn hit(&self, r: &Ray, t_min: f64, t_max: f64) -> Option<HitRecord> {
        let oc = r.orig - self.center(r.time);
        let a = r.dir.squared_length();
        let half_b = oc * r.dir;
        let c = oc.squared_length() - self.radius * self.radius;
        let discriminant = half_b.powf(2.0) - a * c;
        if discriminant < 0. {
            return None;
        }
        let sqrt_d = discriminant.sqrt();

        // Find the nearest root that lies in the acceptable range.
        let in_range = |root| root >= t_min && root <= t_max;
        let mut root = (-half_b - sqrt_d) / a;
        if !in_range(root) {
            root = (-half_b + sqrt_d) / a;
        }
        if !in_range(root) {
            return None;
        }

        let p = r.at(root);
        let outward_normal = (p - self.center(r.time)) / self.radius;
        let rec = HitRecord::new(root, outward_normal, r, Arc::clone(&self.material));
        Some(rec)
    }
}
