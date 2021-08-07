use crate::material::{CheckerTexture, ConstantTexture, Dielectric, Lambertian, Metal};
use crate::objects::bvh::BVHNode;
use crate::objects::hit::Hitable;
use crate::objects::sphere::Sphere;
use crate::{Ray, Vec3};
use rand::Rng;
use std::f64::consts::PI;
use std::sync::Arc;

pub struct Camera {
    pub origin: Vec3,
    pub lower_left_corner: Vec3,
    pub horizontal: Vec3,
    pub vertical: Vec3,
    pub u: Vec3,
    pub v: Vec3,
    pub w: Vec3,
    pub lens_radius: f64,
    pub time0: f64,
    pub time1: f64,
}

pub fn degrees_to_radians(degrees: f64) -> f64 {
    degrees * PI / 180.
}

impl Camera {
    pub fn new(
        look_from_to: (Vec3, Vec3),
        vup: Vec3,
        vfov: f64,
        aspect_ratio: f64,
        aperture: f64,
        focus_dist: f64,
        time_range: (f64, f64),
    ) -> Self {
        let (look_from, look_at) = look_from_to;
        let (time0, time1) = time_range;
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
            time0,
            time1,
        }
    }

    pub fn get_ray(&self, s: f64, t: f64) -> Ray {
        let rd: Vec3 = self.lens_radius * Vec3::random_in_unit_disk();
        let offset: Vec3 = self.u * rd.x + self.v * rd.y;
        let mut rng = rand::thread_rng();
        Ray::new(
            self.origin + offset,
            self.lower_left_corner + s * self.horizontal + t * self.vertical - self.origin - offset,
            rng.gen_range(self.time0..self.time1),
        )
    }
}

pub struct World {
    // pub hitable_list: Vec<Arc<dyn Hitable>>,
    pub bvh: BVHNode,
    pub cam: Camera,
}

impl World {
    pub fn new(hitable_list: Vec<Arc<dyn Hitable>>, cam: Camera) -> Self {
        Self {
            bvh: BVHNode::new(hitable_list, 0.0, 1.0),
            cam,
            // hitable_list, // bvh: bvh(hitable_list, 0, hitable_list.len(), 0.0, 1.0),
        }
    }

    pub fn ray_color(&self, r: Ray, depth: i32) -> Vec3 {
        if depth <= 0 {
            return Vec3::zero();
        }

        if let Some(rec) = self.bvh.hit(&r, 0.001, f64::INFINITY) {
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

pub fn random_scene() -> World {
    let mut hitable_list: Vec<Arc<dyn Hitable>> = vec![
        Arc::new(Sphere {
            center: Vec3::new(0., -1000., 0.),
            radius: 1000.,
            material: Arc::new(Lambertian::new(CheckerTexture(
                ConstantTexture(Vec3::new(0.2, 0.3, 0.1)),
                ConstantTexture(Vec3::new(0.9, 0.9, 0.9)),
            ))),
        }),
        Arc::new(Sphere {
            center: Vec3::new(0., 1., 0.),
            radius: 1.,
            material: Arc::new(Dielectric::new(1.5)),
        }),
        Arc::new(Sphere {
            center: Vec3::new(-4., 1., 0.),
            radius: 1.,
            material: Arc::new(Lambertian::new(ConstantTexture(Vec3::new(0.4, 0.2, 0.1)))),
        }),
        Arc::new(Sphere {
            center: Vec3::new(4., 1., 0.),
            radius: 1.,
            material: Arc::new(Metal::new(ConstantTexture(Vec3::new(0.7, 0.6, 0.5)), 0.)),
        }),
    ];
    let mut rng = rand::thread_rng();
    for a in -11..12 {
        for b in -11..12 {
            let choose_mat: f64 = rng.gen();
            let center = Vec3::new(
                f64::from(a) + 0.9_f64 * rng.gen::<f64>(),
                0.2,
                f64::from(b) + 0.9_f64 * rng.gen::<f64>(),
            );
            if (center - Vec3::new(4., 0.2, 0.)).length() > 0.9 {
                if choose_mat < 0.8 {
                    // diffuse
                    let albedo = Vec3::random_in_range(0., 1.);
                    hitable_list.push(Arc::new(Sphere {
                        center,
                        radius: 0.2,
                        material: Arc::new(Lambertian::new(ConstantTexture(albedo))),
                    }));
                } else if choose_mat < 0.95 {
                    // metal
                    let albedo = Vec3::random_in_range(0.5, 1.);
                    let fuzz: f64 = rng.gen_range(0.0..0.5);
                    hitable_list.push(Arc::new(Sphere {
                        center,
                        radius: 0.2,
                        material: Arc::new(Metal::new(ConstantTexture(albedo), fuzz)),
                    }))
                } else {
                    // glass
                    hitable_list.push(Arc::new(Sphere {
                        center,
                        radius: 0.2,
                        material: Arc::new(Dielectric::new(1.5)),
                    }))
                }
            }
        }
    }
    World::new(
        hitable_list,
        Camera::new(
            (Vec3::new(13., 2., 3.), Vec3::new(0., 0., 0.)),
            Vec3::new(0., 1., 0.),
            20.,
            16. / 9.,
            0.1,
            10.,
            (0.0, 1.0),
        ),
    )
}
