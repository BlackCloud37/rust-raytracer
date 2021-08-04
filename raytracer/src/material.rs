// This file shows necessary examples of how to complete Track 4 and 5.
use crate::Ray;
use crate::Vec3;
use rand::Rng;
use std::sync::Arc;

pub trait Texture {
    fn get_color(&self, rec: &HitRecord) -> Vec3;
}
pub trait Material {
    fn scatter(&self, r: &Ray, rec: &HitRecord) -> Option<(Vec3, Ray)>;
}

pub struct ConstantTexture(pub Vec3);
pub struct DiffuseLight(pub ConstantTexture);

impl Texture for ConstantTexture {
    fn get_color(&self, rec: &HitRecord) -> Vec3 {
        self.0
    }
}
impl Texture for DiffuseLight {
    fn get_color(&self, rec: &HitRecord) -> Vec3 {
        self.0 .0
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
}

impl<T: Texture> Material for Lambertian<T> {
    fn scatter(&self, r: &Ray, rec: &HitRecord) -> Option<(Vec3, Ray)> {
        let mut scatter_direction = rec.normal + Vec3::random_unit_vector();
        if scatter_direction.is_near_zero() {
            scatter_direction = rec.normal;
        }

        let scattered = Ray::new(rec.p, scatter_direction);
        let attenutaion = self.albedo.get_color(rec);
        Some((attenutaion, scattered))
    }
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
    fn scatter(&self, r: &Ray, rec: &HitRecord) -> Option<(Vec3, Ray)> {
        let reflected = Vec3::reflect(r.dir.unit(), rec.normal);
        let scattered = Ray::new(rec.p, reflected + self.fuzz * Vec3::random_in_unit_sphere());
        if scattered.dir * rec.normal > 0. {
            let attenuation = self.albedo.get_color(rec);
            Some((attenuation, scattered))
        } else {
            None
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
    fn scatter(&self, r: &Ray, rec: &HitRecord) -> Option<(Vec3, Ray)> {
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
        let direction =
            if cannot_refract || Self::reflectance(cos_theta, refraction_ratio) > rng.gen() {
                Vec3::reflect(unit_direction, rec.normal)
            } else {
                Vec3::refract(unit_direction, rec.normal, refraction_ratio)
            };
        Some((attenuation, Ray::new(rec.p, direction)))
    }
}
pub struct HitRecord {
    pub p: Vec3,
    pub normal: Vec3,
    pub t: f64,
    pub front_face: bool,
    pub mat: Arc<dyn Material + Send + Sync>,
}

impl HitRecord {
    pub fn new(
        t: f64,
        outward_normal: Vec3,
        r: &Ray,
        mat: Arc<dyn Material + Send + Sync>,
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
        }
    }
}

pub trait Hitable {
    fn hit(&self, r: &Ray, t_min: f64, t_max: f64) -> Option<HitRecord>;
}
pub struct AABB;

/// This BVHNode should be constructed statically.
/// You should use procedural macro to generate code like this:
/// ```
/// let bvh = BVHNode::construct(
///     box BVHNode::construct(
///         box Sphere { .. }
///         box Sphere { .. }
///     ),
///     box BVHNode::construct(
///         box Sphere { .. }
///         box Sphere { .. }
///     )
/// )
/// ```
/// And you can put that `bvh` into your `HittableList`.
pub struct BVHNode<L: Hitable, R: Hitable> {
    left: Box<L>,
    right: Box<R>,
    bounding_box: AABB,
}

impl<L: Hitable, R: Hitable> BVHNode<L, R> {
    pub fn construct(_left: Box<L>, _right: Box<R>) -> Self {
        unimplemented!()
    }
}
