use crate::camera::Camera;
use crate::light::{AllLights, Light};
use crate::objects::bvh::BVHNode;
use crate::objects::hit::{HitRecord, Hitable};
use crate::Ray;
use std::sync::Arc;

pub struct World {
    pub bvh: BVHNode,
    pub cam: Camera,
    pub lights: AllLights,
}

impl World {
    pub fn new(
        hitable_list: Vec<Arc<dyn Hitable>>,
        cam: Camera,
        lights: Vec<Arc<dyn Light>>,
    ) -> Self {
        Self {
            bvh: BVHNode::new(hitable_list),
            cam,
            lights: AllLights::new(lights),
        }
    }

    pub fn hit(&self, r: &Ray, t_min: f64, t_max: f64) -> Option<HitRecord> {
        self.bvh.hit(r, t_min, t_max)
    }
}
