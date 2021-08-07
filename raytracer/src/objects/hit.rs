use crate::material::Material;
use crate::objects::aabb::AABB;
use crate::vec3::Vec3;
use crate::Ray;
use std::sync::Arc;

pub struct HitRecord {
    pub p: Vec3,
    pub normal: Vec3,
    pub t: f64,
    pub front_face: bool,
    pub mat: Arc<dyn Material>,
    pub uv: (f64, f64),
}

impl HitRecord {
    pub fn new(
        t: f64,
        outward_normal: Vec3,
        r: &Ray,
        mat: Arc<dyn Material>,
        uv: (f64, f64),
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
            uv,
        }
    }
}

pub trait Hitable: Sync + Send {
    fn hit(&self, r: &Ray, t_min: f64, t_max: f64) -> Option<HitRecord>;
    fn bounding_box(&self, time0: f64, time1: f64) -> Option<AABB>;
}

impl Hitable for Vec<Arc<dyn Hitable>> {
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

    fn bounding_box(&self, time0: f64, time1: f64) -> Option<AABB> {
        if self.is_empty() {
            return None;
        }

        let mut first_box = true;
        let mut output_box: AABB = AABB {
            minimum: Vec3::zero(),
            maximum: Vec3::zero(),
        };
        for object in self {
            if let Some(temp_box) = object.bounding_box(time0, time1) {
                output_box = if first_box {
                    temp_box
                } else {
                    AABB::surrounding_box(&output_box, &temp_box)
                };
                first_box = false;
            } else {
                return None;
            }
        }
        Some(output_box)
    }
}
