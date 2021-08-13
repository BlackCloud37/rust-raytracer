// This file shows necessary examples of how to complete Track 4 and 5.
use crate::material::Interaction::{Absorb, Diffuse, Reflect, Refract, Specular};
use crate::objects::hit::HitRecord;
use crate::Ray;
use crate::Vec3;
use image::{DynamicImage, GenericImageView};
use rand::Rng;

pub enum Interaction {
    Diffuse,
    Specular,
    Absorb,
    Reflect,
    Refract,
}

pub trait Texture: Send + Sync {
    fn get_color(&self, rec: &HitRecord) -> Vec3;
}
pub trait Material: Send + Sync {
    // fn f(&self, wi: Vec3, wo: Vec3) -> Vec3;
    fn scatter(&self, r: &Ray, rec: &HitRecord) -> (Interaction, Option<Ray>, Option<Vec3>);
    fn emitted(&self, _rec: &HitRecord) -> Vec3 {
        Vec3::zero()
    }
    fn scatter_photon(
        &self,
        r: &Ray,
        rec: &HitRecord,
        power: Vec3,
    ) -> (Interaction, Option<Ray>, Option<Vec3>) {
        // inter, next ray, f
        if let (interaction, out_ray, Some(f)) = self.scatter(r, rec) {
            // R.R.
            let h = f.max();
            if rand::thread_rng().gen::<f64>() > h {
                (Absorb, None, None)
            } else {
                (interaction, out_ray, Some(Vec3::elemul(power, f / f.max())))
            }
        } else {
            (Absorb, None, None)
        }
    }
}

pub struct ConstantTexture(pub Vec3);
pub struct CheckerTexture(pub ConstantTexture, pub ConstantTexture);
pub struct ImageTexture(pub DynamicImage);

impl Texture for ConstantTexture {
    fn get_color(&self, _rec: &HitRecord) -> Vec3 {
        self.0
    }
}

impl Texture for CheckerTexture {
    fn get_color(&self, rec: &HitRecord) -> Vec3 {
        let p = rec.p;
        let sines = f64::sin(10. * p.x) * f64::sin(10. * p.y) * f64::sin(10. * p.z);
        if sines < 0. {
            self.0.get_color(rec)
        } else {
            self.1.get_color(rec)
        }
    }
}

impl Texture for ImageTexture {
    fn get_color(&self, rec: &HitRecord) -> Vec3 {
        let image = &self.0;
        let (u, v) = rec.uv;
        let (u, v) = (u.clamp(0., 1.), 1. - v.clamp(0., 1.));
        let (width, height) = (image.width(), image.height());
        let (x, y) = (
            (width as f64 * u).floor() as u32,
            (height as f64 * v).floor() as u32,
        );

        let rgb = image.get_pixel(x, y);
        Vec3::from(rgb)
    }
}

/// `Lambertian` now takes a generic parameter `T`.
/// This reduces the overhead of using `Box<dyn Texture>`
#[derive(Clone)]
pub struct Lambertian<T: Texture> {
    pub albedo: T,
}

impl<T: Texture> Lambertian<T> {
    pub fn new(albedo: T) -> Self {
        Self { albedo }
    }

    fn scattered_direction(&self, n: Vec3) -> Vec3 {
        let mut scatter_direction = n + Vec3::random_unit_vector();
        if scatter_direction.is_near_zero() {
            scatter_direction = n;
        }
        scatter_direction
    }
}

impl<T: Texture> Material for Lambertian<T> {
    fn scatter(&self, r: &Ray, rec: &HitRecord) -> (Interaction, Option<Ray>, Option<Vec3>) {
        let scattered = Ray::new(rec.p, self.scattered_direction(rec.normal), r.time);
        let attenutaion = self.albedo.get_color(rec);
        (Diffuse, Some(scattered), Some(attenutaion))
    }

    // fn scatter_photon(
    //     &self,
    //     r: &Ray,
    //     rec: &HitRecord,
    //     power: Vec3,
    // ) -> (Interaction, Option<Ray>, Option<Vec3>) {
    //     let diffuse_color = self.albedo.get_color(rec);
    //     let max_diffuse = diffuse_color.max();
    //
    //     // R.R., to decide if absorbed
    //     if rand::thread_rng().gen::<f64>() > max_diffuse {
    //         return (Absorb, None, None);
    //     }
    //
    //     let change_emit = diffuse_color / max_diffuse;
    //
    //     let scatter_direction = self.scattered_direction(rec.normal);
    //
    //     (
    //         Diffuse,
    //         Some(Ray::new(rec.p, scatter_direction, r.time)), // out ray
    //         Some(Vec3::elemul(power, change_emit)),
    //     ) // new power
    // }
}

pub struct Metal<T: Texture> {
    pub albedo: T,
    pub fuzz: f64,
}

impl<T: Texture> Metal<T> {
    pub fn new(albedo: T, fuzz: f64) -> Self {
        Self { albedo, fuzz }
    }
}

impl<T: Texture> Material for Metal<T> {
    fn scatter(&self, r: &Ray, rec: &HitRecord) -> (Interaction, Option<Ray>, Option<Vec3>) {
        let reflected = Vec3::reflect(r.dir.unit(), rec.normal);
        let scattered = Ray::new(
            rec.p,
            reflected + self.fuzz * Vec3::random_in_unit_sphere(),
            r.time,
        );
        if scattered.dir * rec.normal > 0. {
            let attenuation = self.albedo.get_color(rec);
            (Specular, Some(scattered), Some(attenuation))
        } else {
            (Absorb, None, None)
        }
    }
}

pub struct Dielectric {
    pub ir: f64,
}

impl Dielectric {
    pub fn new(ir: f64) -> Self {
        Self { ir }
    }
    pub fn reflectance(cosine: f64, ref_idx: f64) -> f64 {
        // Use Schlick's approximation for reflectance
        let r0 = ((1. - ref_idx) / (1. + ref_idx)).powf(2.);
        r0 + (1. - r0) * (1. - cosine).powf(5.)
    }
}

impl Material for Dielectric {
    fn scatter(&self, r: &Ray, rec: &HitRecord) -> (Interaction, Option<Ray>, Option<Vec3>) {
        let attenuation = Vec3::ones();
        let refraction_ratio = if rec.front_face {
            1.0 / self.ir
        } else {
            self.ir
        };
        let unit_direction = r.dir.unit();
        let cos_theta = (-unit_direction * rec.normal).min(1.0);
        let sin_theta = (1.0 - cos_theta.powf(2.0)).sqrt();
        let cannot_refract = refraction_ratio * sin_theta > 1.0;
        let mut rng = rand::thread_rng();
        let (direction, interaction) =
            if cannot_refract || Self::reflectance(cos_theta, refraction_ratio) > rng.gen() {
                (Vec3::reflect(unit_direction, rec.normal), Reflect)
            } else {
                (
                    Vec3::refract(unit_direction, rec.normal, refraction_ratio),
                    Refract,
                )
            };
        (
            interaction,
            Some(Ray::new(rec.p, direction, r.time)),
            Some(attenuation),
        )
    }
}
//
// pub struct DiffuseLight<T: Texture> {
//     pub emit: T,
// }
//
// impl<T: Texture> DiffuseLight<T> {
//     pub fn new(emit: T) -> Self {
//         Self { emit }
//     }
// }
//
// impl<T: Texture> Material for DiffuseLight<T> {
//     fn scatter(&self, _r: &Ray, _rec: &HitRecord) -> Option<(Vec3, Ray)> {
//         None
//     }
//     fn emitted(&self, rec: &HitRecord) -> Vec3 {
//         self.emit.get_color(rec)
//     }
// }
//
// pub struct Isotropic<T: Texture> {
//     pub albedo: T,
// }
//
// impl<T: Texture> Isotropic<T> {
//     pub fn new(albedo: T) -> Self {
//         Self { albedo }
//     }
// }
//
// impl<T: Texture> Material for Isotropic<T> {
//     fn scatter(&self, r: &Ray, rec: &HitRecord) -> Option<(Vec3, Ray)> {
//         Some((
//             self.albedo.get_color(rec),
//             Ray::new(rec.p, Vec3::random_in_unit_sphere(), r.time),
//         ))
//     }
// }
