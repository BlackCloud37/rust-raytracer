use crate::material::{ConstantTexture, DiffuseLight};
use crate::material::{HitRecord, Hitable};
use crate::Ray;
use crate::Vec3;
use image::Rgb;
use raytracer_codegen::make_spheres_impl;

// Call the procedural macro, which will become `make_spheres` function.
make_spheres_impl! {}

pub struct Sphere {
    pub center: Vec3,
    pub radius: f64,
    pub material: DiffuseLight,
}

impl Hitable for Sphere {
    fn hit(&self, r: &Ray, t_min: f64, t_max: f64) -> Option<HitRecord> {
        let oc = r.orig - self.center;
        let a = r.dir.squared_length();
        let half_b = oc * r.dir;
        let c = oc.squared_length() - self.radius * self.radius;
        let discriminant = half_b * half_b - a * c;
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
        let rec = HitRecord::new(root, outward_normal, r);
        return Some(rec);
    }
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

pub struct World {
    pub hitable_list: Vec<Box<dyn Hitable + Send + Sync>>,
}

impl World {
    pub fn ray_color(&self, r: Ray) -> Rgb<u8> {
        if let Some(rec) = self.hitable_list.hit(&r, 0., f64::INFINITY) {
            (0.5 * Vec3::new(rec.normal.x + 1., rec.normal.y + 1., rec.normal.z + 1.)).to_color()
        } else {
            let unit_direction = r.dir.unit();
            let t = 0.5 * unit_direction.y + 1.;
            ((1.0 - t) * Vec3::new(1., 1., 1.) + t * Vec3::new(0.5, 0.7, 1.0)).to_color()
        }
    }
}

pub fn example_scene() -> World {
    let mut hittable_list: Vec<Box<dyn Hitable + Send + Sync>> = vec![];
    hittable_list.push(Box::new(Sphere {
        center: Vec3::new(0., 0., -1.),
        radius: 0.5,
        material: DiffuseLight(ConstantTexture(Vec3::zero())),
    }));
    hittable_list.push(Box::new(Sphere {
        center: Vec3::new(0., -100.5, -1.),
        radius: 100.,
        material: DiffuseLight(ConstantTexture(Vec3::zero())),
    }));
    World {
        hitable_list: hittable_list,
    }
}
