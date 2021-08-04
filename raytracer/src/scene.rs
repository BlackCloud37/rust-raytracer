use crate::material::{ConstantTexture, Dielectric, DiffuseLight, Lambertian, Material, Metal};
use crate::material::{HitRecord, Hitable};
use crate::Ray;
use crate::Vec3;
use raytracer_codegen::make_spheres_impl;
use std::f64::consts::PI;
use std::sync::Arc;

// Call the procedural macro, which will become `make_spheres` function.
// make_spheres_impl! {}

pub struct Camera {
    pub origin: Vec3,
    pub lower_left_corner: Vec3,
    pub horizontal: Vec3,
    pub vertical: Vec3,
    pub u: Vec3,
    pub v: Vec3,
    pub w: Vec3,
    pub lens_radius: f64,
}

pub fn degrees_to_radians(degrees: f64) -> f64 {
    degrees * PI / 180.
}

impl Camera {
    pub fn new(
        look_from: Vec3,
        look_at: Vec3,
        vup: Vec3,
        vfov: f64,
        aspect_ratio: f64,
        aperture: f64,
        focus_dist: f64,
    ) -> Self {
        let theta = degrees_to_radians(vfov);
        let h = f64::tan(theta / 2.);
        let viewport_height = 2.0 * h;
        let viewport_width = aspect_ratio * viewport_height;

        let w = (look_from - look_at).unit();
        let u = Vec3::cross(vup, w).unit();
        let v = Vec3::cross(w, u);

        let origin = look_from;
        let horizontal = focus_dist * viewport_width * u;
        let vertical = focus_dist * viewport_height * v;
        Self {
            origin,
            horizontal,
            vertical,
            lower_left_corner: origin - horizontal / 2. - vertical / 2. - focus_dist * w,
            u,
            v,
            w,
            lens_radius: aperture / 2.,
        }
    }
    // pub fn default() -> Self {
    //     let aspect_ratio = 16. / 9.;
    //     let viewport_height = 2.;
    //     let viewport_width = aspect_ratio * viewport_height;
    //     let focal_length = 1.;
    //
    //     let origin = Vec3::zero();
    //     let horizontal = Vec3::new(viewport_width, 0., 0.);
    //     let vertical = Vec3::new(0., viewport_height, 0.);
    //     Self {
    //         origin,
    //         horizontal,
    //         vertical,
    //         lower_left_corner: origin
    //             - horizontal / 2.
    //             - vertical / 2.
    //             - Vec3::new(0., 0., focal_length),
    //     }
    // }

    pub fn get_ray(&self, s: f64, t: f64) -> Ray {
        let rd: Vec3 = self.lens_radius * Vec3::random_in_unit_disk();
        let offset: Vec3 = self.u * rd.x + self.v * rd.y;
        Ray::new(
            self.origin + offset,
            self.lower_left_corner + s * self.horizontal + t * self.vertical - self.origin - offset,
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
        cam: Camera::new(
            Vec3::new(3., 3., 2.),
            Vec3::new(0., 0., -1.),
            Vec3::new(0., 1., 0.),
            20.0,
            16. / 9.,
            1.,
            Vec3::new(3., 3., 3.).length(),
        ),
    }
}
