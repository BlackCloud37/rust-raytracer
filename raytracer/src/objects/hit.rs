use crate::material::Material;
use crate::vec3::Vec3;
use crate::Ray;
use std::sync::Arc;

pub struct HitRecord {
    pub p: Vec3,
    pub normal: Vec3,
    pub t: f64,
    pub front_face: bool,
    pub mat: Arc<dyn Material + Send + Sync>,
}

impl HitRecord {
    pub fn new(
        t: f64,
        outward_normal: Vec3,
        r: &Ray,
        mat: Arc<dyn Material + Send + Sync>,
    ) -> Self {
        let p = r.at(t);
        let front_face = r.dir * outward_normal < 0.;
        let normal = if front_face {
            outward_normal
        } else {
            -outward_normal
        };
        Self {
            p,
            normal: normal.unit(),
            t,
            front_face,
            mat,
        }
    }
}

pub trait Hitable {
    fn hit(&self, r: &Ray, t_min: f64, t_max: f64) -> Option<HitRecord>;
}

impl Hitable for Vec<Box<dyn Hitable + Send + Sync>> {
    fn hit(&self, r: &Ray, t_min: f64, t_max: f64) -> Option<HitRecord> {
        let mut closest_so_far = t_max;
        let mut res = None;
        for object in self {
            if let Some(temp_rec) = object.hit(r, t_min, closest_so_far) {
                closest_so_far = temp_rec.t;
                res = Some(temp_rec);
            }
        }
        res
    }
}
