#![allow(dead_code)]
use crate::material::Material;
use crate::objects::aabb::AABB;
use crate::objects::hit::{HitRecord, Hitable};
use crate::{Ray, Vec3};
use rand::Rng;
use std::sync::Arc;

pub struct ConstantMedium {
    pub boundary: Arc<dyn Hitable>,
    pub phase_function: Arc<dyn Material>,
    pub neg_inv_density: f64,
}

impl ConstantMedium {
    pub fn new(d: f64, boundary: Arc<dyn Hitable>, phase_function: Arc<dyn Material>) -> Self {
        Self {
            boundary,
            neg_inv_density: -1. / d,
            phase_function,
        }
    }
}

impl Hitable for ConstantMedium {
    fn hit(&self, r: &Ray, t_min: f64, t_max: f64) -> Option<HitRecord> {
        if let Some(mut rec1) = self.boundary.hit(r, -f64::INFINITY, f64::INFINITY) {
            if let Some(mut rec2) = self.boundary.hit(r, rec1.t + 0.0001, f64::INFINITY) {
                rec1.t = rec1.t.max(t_min);
                rec2.t = rec2.t.min(t_max);
                if rec1.t >= rec2.t {
                    return None;
                }
                rec1.t = rec1.t.max(0.);
                let ray_length = r.dir.length();
                let distance_inside_boundary = (rec2.t - rec1.t) * ray_length;
                let mut rng = rand::thread_rng();
                let hit_distance = self.neg_inv_density * rng.gen::<f64>().ln();
                if hit_distance > distance_inside_boundary {
                    return None;
                }
                let t = rec1.t + hit_distance / ray_length;
                return Some(HitRecord::new(
                    t,
                    Vec3::new(1., 0., 0.),
                    r,
                    Arc::clone(&self.phase_function),
                    (0.0, 0.0),
                ));
            }
        }
        None
    }
    fn bounding_box(&self) -> Option<AABB> {
        self.boundary.bounding_box()
    }
}
