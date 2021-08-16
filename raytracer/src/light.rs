use crate::objects::hit::{HitRecord, Hitable};
// use crate::objects::rectangle::XZRectangle;
use crate::material::{ConstantTexture, DiffuseLight};
use crate::objects::aabb::AABB;
use crate::objects::sphere::Sphere;
use crate::scene::World;
use crate::vec3::polar_direction;
use crate::{Ray, Vec3};
use kd_tree::KdPoint;
use rand::distributions::Distribution;
use rand::distributions::WeightedIndex;
use rand::thread_rng;
use std::f64::consts::PI;
use std::sync::Arc;

pub struct Photon {
    position: Vec3,
    phi: u8,
    theta: u8,
    power: Vec3,
}

impl Photon {
    pub fn new(position: Vec3, power: Vec3, direction: Vec3) -> Self {
        let phi = (255. * (f64::atan2(direction.y, direction.x) + PI) / (2. * PI)).floor() as u8;
        let theta = (255. * f64::acos(direction.x) / PI).floor() as u8;
        Self {
            position,
            phi,
            theta,
            power,
        }
    }
    pub fn position(&self) -> Vec3 {
        self.position
    }
    pub fn power(&self) -> Vec3 {
        self.power
    }
    pub fn direction(&self) -> Vec3 {
        polar_direction(self.theta, self.phi)
    }
}

impl Default for Photon {
    fn default() -> Self {
        Photon::new(Vec3::zero(), Vec3::zero(), Vec3::zero())
    }
}

impl KdPoint for Photon {
    type Scalar = f64;
    type Dim = typenum::U3;
    fn at(&self, i: usize) -> Self::Scalar {
        self.position[i]
    }
}

pub trait Light: Hitable {
    fn emit(&self) -> (Ray, Vec3); // ray, power
    fn power(&self) -> Vec3; // total power
    fn sample_li(&self, rec: &HitRecord, world: &World, sample_cnt: usize) -> Vec3;
}

#[derive(Clone)]
pub struct SphereDiffuseLight {
    sphere: Sphere,
    pub flux: Vec3,
    pub scale: f64,
}

impl SphereDiffuseLight {
    pub fn new(center: Vec3, radius: f64, flux: Vec3, scale: f64) -> Self {
        Self {
            sphere: Sphere {
                center,
                radius,
                material: Arc::new(DiffuseLight::new(ConstantTexture(flux))),
            },
            flux,
            scale,
        }
    }
}

impl Hitable for SphereDiffuseLight {
    fn hit(&self, r: &Ray, t_min: f64, t_max: f64) -> Option<HitRecord> {
        self.sphere.hit(r, t_min, t_max)
    }
    fn bounding_box(&self, time0: f64, time1: f64) -> Option<AABB> {
        self.sphere.bounding_box(time0, time1)
    }
}

impl Light for SphereDiffuseLight {
    fn emit(&self) -> (Ray, Vec3) {
        let dir = Vec3::random_in_unit_sphere();
        (
            Ray::new(
                self.sphere.center + dir * (self.sphere.radius + 0.0001),
                dir,
                0.,
            ),
            self.flux * self.scale,
        )
    }
    fn power(&self) -> Vec3 {
        self.scale * self.flux
    }
    fn sample_li(&self, rec: &HitRecord, world: &World, sample_cnt: usize) -> Vec3 {
        // todo!(maybe bug)
        let mut sample_li = Vec3::zero();
        for _ in 0..sample_cnt {
            let center_to_p = (rec.p - self.sphere.center).unit();
            let point_on_sphere =
                self.sphere.center + Vec3::random_in_hemisphere(&center_to_p) * self.sphere.radius;
            let direct_to_light = (point_on_sphere - rec.p).unit();
            let shadow_ray = Ray::new(rec.p, direct_to_light, 0.);
            let t = (rec.p - point_on_sphere).length();
            if world.bvh.hit(&shadow_ray, 0.0001, t - 0.0001).is_none() {
                if let (_, _, Some(f)) = rec.mat.scatter(&shadow_ray, rec) {
                    sample_li +=
                        Vec3::elemul(self.flux, f) * (rec.normal.unit() * direct_to_light).max(0.0);
                    // / (self.position - rec.p).length();
                }
            }
        }
        sample_li / sample_cnt as f64
    }
}

// pub struct XZRectLight {
//     area: XZRectangle,
//     pub flux: Vec3,
//     pub scale: f64,
// }
//
// impl Light for XZRectLight {
//     fn emit(&self) -> (Ray, Vec3) {
//         let mut rng = rand::thread_rng();
//         let (u, v) = (rng.gen::<f64>(), rng.gen::<f64>());
//         let (x0, z0) = self.area.xz0;
//         let (x1, z1) = self.area.xz1;
//         let x = x0 + (x1 - x0) * u;
//         let z = z0 + (z1 - z0) * v;
//         let orig = Vec3::new(x, self.area.y, z);
//         let w = Vec3::random_in_hemisphere(&Vec3::new(0., -1., 0.));
//         (Ray::new(orig, w, 0.), self.flux * self.scale * (Vec3::new(0., -1., 0.) * w).max(0.))
//     }
//     fn power(&self) -> Vec3 {
//         self.flux * self.scale
//     }
//     fn sample_li(&self, rec: &HitRecord, world: &World) -> Vec3 {
//
//     }
// }

pub struct AllLights {
    lights: Vec<Arc<dyn Light>>,
    lights_prob: Vec<f64>,
}

impl AllLights {
    pub fn new(lights: Vec<Arc<dyn Light>>) -> Self {
        let light_powers = lights
            .iter()
            .map(|l| l.power().length())
            .collect::<Vec<f64>>();
        let tot_power: f64 = light_powers.iter().sum();
        let lights_prob = light_powers
            .iter()
            .map(|p| p / tot_power)
            .collect::<Vec<f64>>();
        Self {
            lights,
            lights_prob,
        }
    }
}

impl AllLights {
    pub fn emit(&self) -> (Ray, Vec3) {
        let mut rng = thread_rng();
        let light_dist = WeightedIndex::new(&self.lights_prob).unwrap();
        let idx: usize = light_dist.sample(&mut rng);
        self.lights[idx].emit()
    }
    // pub fn power(&self) -> Vec3 {
    //     self.lights.iter().map(|l| l.power()).sum()
    // }
    pub fn sample_li(&self, rec: &HitRecord, world: &World, sample_cnt: usize) -> Vec3 {
        self.lights
            .iter()
            .map(|l| l.sample_li(rec, world, sample_cnt))
            .sum()
    }
}
