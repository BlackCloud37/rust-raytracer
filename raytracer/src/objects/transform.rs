#![allow(dead_code)]
use crate::objects::aabb::AABB;
use crate::objects::hit::{HitRecord, Hitable};
use crate::vec3::degrees_to_radians;
use crate::{Ray, Vec3};
use nalgebra::Matrix4;
use std::sync::Arc;

pub struct Transform {
    obj: Arc<dyn Hitable>,
    trans: Matrix4<f64>,
    inverse_trans: Matrix4<f64>,
    bounding_box: Option<AABB>,
}

impl Transform {
    pub(crate) fn new(
        rotate_in_degree: Vec3,
        scale: Vec3,
        translate: Vec3,
        obj: Arc<dyn Hitable>,
    ) -> Self {
        let (rx, ry, rz) = (
            degrees_to_radians(rotate_in_degree.x),
            degrees_to_radians(rotate_in_degree.y),
            degrees_to_radians(rotate_in_degree.z),
        );
        let mut trans = Matrix4::new(
            scale.x, 0., 0., 0., 0., scale.y, 0., 0., 0., 0., scale.z, 0., 0., 0., 0., 1.,
        );
        trans *= Matrix4::new(
            1.,
            0.,
            0.,
            translate.x,
            0.,
            1.,
            0.,
            translate.y,
            0.,
            0.,
            1.,
            translate.z,
            0.,
            0.,
            0.,
            1.,
        );
        trans *= Matrix4::new(
            // rotate X
            1.,
            0.,
            0.,
            0.,
            0.,
            f64::cos(rx),
            -f64::sin(rx),
            0.,
            0.,
            f64::sin(rx),
            f64::cos(rx),
            0.,
            0.,
            0.,
            0.,
            1.,
        );
        trans *= Matrix4::new(
            // rotate Y
            f64::cos(ry),
            0.,
            f64::sin(ry),
            0.,
            0.,
            1.,
            0.,
            0.,
            -f64::sin(ry),
            0.,
            f64::cos(ry),
            0.,
            0.,
            0.,
            0.,
            1.,
        );
        trans *= Matrix4::new(
            // rotate Z
            f64::cos(rz),
            -f64::sin(rz),
            0.,
            0.,
            f64::sin(rz),
            f64::cos(rz),
            0.,
            0.,
            0.,
            0.,
            1.,
            0.,
            0.,
            0.,
            0.,
            1.,
        );

        //print!("{}", trans);
        // compute bounding box
        let mut bounding_box = None;
        if let Some(bbox) = obj.bounding_box() {
            let mut min = [f64::INFINITY, f64::INFINITY, f64::INFINITY];
            let mut max = [-f64::INFINITY, -f64::INFINITY, -f64::INFINITY];
            for i in 0..2 {
                for j in 0..2 {
                    for k in 0..2 {
                        let (i, j, k) = (i as f64, j as f64, k as f64);
                        let tester = Vec3::new(
                            i * bbox.maximum.x + (1. - i) * bbox.minimum.x,
                            j * bbox.maximum.y + (1. - j) * bbox.minimum.y,
                            k * bbox.maximum.z + (1. - k) * bbox.minimum.z,
                        )
                        .transform_point(&trans);

                        for c in 0..3 {
                            min[c] = min[c].min(tester[c]);
                            max[c] = max[c].max(tester[c]);
                        }
                    }
                }
            }
            bounding_box = Some(AABB::new(
                Vec3::new(min[0], min[1], min[2]),
                Vec3::new(max[0], max[1], max[2]),
            ));
        }

        if let Some(inverse_trans) = trans.try_inverse() {
            Self {
                obj,
                trans,
                inverse_trans,
                bounding_box,
            }
        } else {
            panic!("Invalid transform matrix")
        }
    }
}

impl Hitable for Transform {
    fn hit(&self, r: &Ray, t_min: f64, t_max: f64) -> Option<HitRecord> {
        let trans_r = Ray::new(
            r.orig.transform_point(&self.inverse_trans),
            r.dir.transform_dir(&self.inverse_trans),
        );
        if let Some(mut trans_rec) = self.obj.hit(&trans_r, t_min, t_max) {
            let outward_normal = trans_rec.normal.transform_dir(&self.trans);
            trans_rec.p = trans_rec.p.transform_point(&self.trans); // ok
            trans_rec.set_face_normal(&trans_r, outward_normal);
            Some(trans_rec)
        } else {
            None
        }
    }
    fn bounding_box(&self) -> Option<AABB> {
        Some(self.bounding_box.as_ref().unwrap().clone())
    }
}
