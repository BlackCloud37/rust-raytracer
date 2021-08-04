use crate::material::{ConstantTexture, Dielectric, DiffuseLight, Lambertian, Material, Metal};
use crate::material::{HitRecord, Hitable};
use crate::Ray;
use crate::Vec3;
use raytracer_codegen::make_spheres_impl;
use std::sync::Arc;

// Call the procedural macro, which will become `make_spheres` function.
// make_spheres_impl! {}

pub struct Camera {
    pub origin: Vec3,
    pub lower_left_corner: Vec3,
    pub horizontal: Vec3,
    pub vertical: Vec3,
}

impl Camera {
    pub fn default() -> Self {
        let aspect_ratio = 16. / 9.;
        let viewport_height = 2.;
        let viewport_width = aspect_ratio * viewport_height;
        let focal_length = 1.;

        let origin = Vec3::zero();
        let horizontal = Vec3::new(viewport_width, 0., 0.);
        let vertical = Vec3::new(0., viewport_height, 0.);
        Self {
            origin,
            horizontal,
            vertical,
            lower_left_corner: origin
                - horizontal / 2.
                - vertical / 2.
                - Vec3::new(0., 0., focal_length),
        }
    }

    pub fn get_ray(&self, u: f64, v: f64) -> Ray {
        Ray::new(
            self.origin,
            self.lower_left_corner + u * self.horizontal + v * self.vertical - self.origin,
        )
    }
}

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
        let mut rec = HitRecord::new(root, outward_normal, r, Arc::clone(&self.material));
        Some(rec)
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
    pub cam: Camera,
}

impl World {
    pub fn ray_color(&self, r: Ray, depth: i32) -> Vec3 {
        if depth <= 0 {
            return Vec3::zero();
        }

        if let Some(rec) = self.hitable_list.hit(&r, 0.001, f64::INFINITY) {
            if let Some((attenuation, scattered)) = rec.mat.scatter(&r, &rec) {
                Vec3::elemul(attenuation, self.ray_color(scattered, depth - 1))
            } else {
                Vec3::zero()
            }
        } else {
            let unit_direction = r.dir.unit();
            let t = 0.5 * unit_direction.y + 1.;
            (1.0 - t) * Vec3::new(1., 1., 1.) + t * Vec3::new(0.5, 0.7, 1.0)
        }
    }
}

pub fn example_scene() -> World {
    let hitable_list: Vec<Box<dyn Hitable + Send + Sync>> = vec![
        // ground
        Box::new(Sphere {
            center: Vec3::new(0., -100.5, -1.),
            radius: 100.,
            material: Arc::new(Lambertian::new(ConstantTexture(Vec3::new(0.8, 0.8, 0.0)))),
        }),
        // center
        Box::new(Sphere {
            center: Vec3::new(0., 0., -1.),
            radius: 0.5,
            material: Arc::new(Lambertian::new(ConstantTexture(Vec3::new(0.1, 0.2, 0.5)))),
            // material: Arc::new(Dielectric::new(1.5)),
        }),
        //right
        Box::new(Sphere {
            center: Vec3::new(-1.0, 0.0, -1.0),
            radius: 0.5,
            // material: Arc::new(Metal::new(ConstantTexture(Vec3::new(0.8, 0.8, 0.8)), 0.)),
            material: Arc::new(Dielectric::new(1.5)),
        }),
        //left
        Box::new(Sphere {
            center: Vec3::new(1.0, 0.0, -1.0),
            radius: 0.5,
            material: Arc::new(Metal::new(ConstantTexture(Vec3::new(0.8, 0.6, 0.2)), 0.)),
        }),
    ];
    World {
        hitable_list,
        cam: Camera::default(),
    }
}
