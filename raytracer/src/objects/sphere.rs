use crate::material::Material;
use crate::objects::aabb::AABB;
use crate::objects::hit::{HitRecord, Hitable};
use crate::{Ray, Vec3};
use std::f64::consts::{FRAC_1_PI, PI};
use std::sync::Arc;

#[derive(Clone)]
pub struct Sphere {
    pub center: Vec3,
    pub radius: f64,
    pub material: Arc<dyn Material>,
}

impl Sphere {
    fn get_uv(&self, p: &Vec3) -> (f64, f64) {
        let theta = f64::acos(-p.y);
        let phi = f64::atan2(-p.z, p.x) + PI;
        (phi * FRAC_1_PI * 0.5, theta * FRAC_1_PI)
    }
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
        let rec = HitRecord::new(
            root,
            outward_normal,
            r,
            Arc::clone(&self.material),
            self.get_uv(&outward_normal),
        );
        Some(rec)
    }
    fn bounding_box(&self) -> Option<AABB> {
        Some(AABB::new(
            self.center - Vec3::all(self.radius),
            self.center + Vec3::all(self.radius),
        ))
    }
}
