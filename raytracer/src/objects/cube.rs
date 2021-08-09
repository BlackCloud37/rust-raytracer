use crate::material::Material;
use crate::objects::aabb::AABB;
use crate::objects::hit::{HitRecord, Hitable};
use crate::objects::rectangle::{XYRectangle, XZRectangle, YZRectangle};
use crate::{Ray, Vec3};
use std::sync::Arc;

pub struct Cube {
    pub box_min: Vec3,
    pub box_max: Vec3,
    pub sides: Vec<Arc<dyn Hitable>>,
}

impl Cube {
    pub fn new(box_min: Vec3, box_max: Vec3, mat: Arc<dyn Material>) -> Self {
        let sides: Vec<Arc<dyn Hitable>> = vec![
            Arc::new(XYRectangle {
                xy0: box_min.xy(),
                xy1: box_max.xy(),
                z: box_min.z,
                material: Arc::clone(&mat),
            }),
            Arc::new(XYRectangle {
                xy0: box_min.xy(),
                xy1: box_max.xy(),
                z: box_max.z,
                material: Arc::clone(&mat),
            }),
            Arc::new(XZRectangle {
                xz0: box_min.xz(),
                xz1: box_max.xz(),
                y: box_min.y,
                material: Arc::clone(&mat),
            }),
            Arc::new(XZRectangle {
                xz0: box_min.xz(),
                xz1: box_max.xz(),
                y: box_max.y,
                material: Arc::clone(&mat),
            }),
            Arc::new(YZRectangle {
                yz0: box_min.yz(),
                yz1: box_max.yz(),
                x: box_min.x,
                material: Arc::clone(&mat),
            }),
            Arc::new(YZRectangle {
                yz0: box_min.yz(),
                yz1: box_max.yz(),
                x: box_max.x,
                material: Arc::clone(&mat),
            }),
        ];
        Self {
            box_min,
            box_max,
            sides,
        }
    }
}

impl Hitable for Cube {
    fn hit(&self, r: &Ray, t_min: f64, t_max: f64) -> Option<HitRecord> {
        self.sides.hit(r, t_min, t_max)
    }
    fn bounding_box(&self, _time0: f64, _time1: f64) -> Option<AABB> {
        Some(AABB::new(self.box_min, self.box_max))
    }
}
