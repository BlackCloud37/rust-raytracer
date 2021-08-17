#![allow(dead_code)]
use crate::light::{AllLights, Light, Photon};
use crate::material::Interaction;
use crate::objects::bvh::BVHNode;
use crate::objects::hit::{HitRecord, Hitable};
use crate::vec3::degrees_to_radians;
use crate::{Ray, Vec3};
use kd_tree::KdTreeN;
use std::f64::consts::PI;
use std::sync::Arc;

// const N_PHOTONS: usize = 400000;
// const N_NEAREST: usize = 50;
const GATHER_CNT: usize = 4;
const FRAC_1_GATHER_CNT: f64 = 1. / GATHER_CNT as f64;
pub enum PMType {
    Global,
    Caustic,
}

pub struct PhotonMap {
    pm_type: PMType,
    pub map: Option<KdTreeN<Photon, typenum::U3>>,
    n_emitted: usize,
}

impl PhotonMap {
    pub fn new(pm_type: PMType) -> Self {
        Self {
            pm_type,
            map: None,
            n_emitted: 0,
        }
    }

    /// KNN Estimate
    /// return (flux, radius2)
    pub fn estimate_flux_by_count(&self, rec: &HitRecord, n_nearest: usize) -> (Vec3, f64) {
        if let Some(pm) = self.map.as_ref() {
            let p = rec.p.xyz_arr();
            let nearest = pm.nearests(&p, n_nearest);
            let mut flux = Vec3::zero();
            let mut radius2: f64 = 0.;
            for kd_tree::ItemAndDistance {
                item: photon,
                squared_distance,
            } in nearest.into_iter()
            {
                radius2 = radius2.max(squared_distance);
                flux += Vec3::elemul(rec.mat.bsdf(*photon.direction(), &rec), *photon.power());
            }
            (flux, radius2)
        } else {
            panic!("photon map not built yet")
        }
    }

    pub fn adjust_flux(&self, flux_in: Vec3, radius2: f64) -> Vec3 {
        flux_in / (PI * radius2 * self.n_emitted as f64)
    }
}

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

impl Camera {
    pub fn new(
        look_from_to: (Vec3, Vec3),
        vup: Vec3,
        vfov: f64,
        aspect_ratio: f64,
        aperture: f64,
        focus_dist: f64,
    ) -> Self {
        let (look_from, look_at) = look_from_to;
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

    pub fn get_ray(&self, s: f64, t: f64) -> Ray {
        let rd: Vec3 = self.lens_radius * Vec3::random_in_unit_disk();
        let offset: Vec3 = self.u * rd.x + self.v * rd.y;
        Ray::new(
            self.origin + offset,
            self.lower_left_corner + s * self.horizontal + t * self.vertical - self.origin - offset,
        )
    }
}

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

    pub fn gen_photon_maps(
        &self,
        n_emit_photons: usize,
        global_pm: &mut PhotonMap,
        caustic_pm: &mut PhotonMap,
    ) {
        let mut all_photons = vec![];
        let mut caustic_photons = vec![];

        // trace photon
        for _ in 0..n_emit_photons {
            let (mut ray, mut power) = self.lights.emit();
            let (mut has_specular, mut has_diffuse) = (false, false);
            while let Some(rec) = self.bvh.hit(&ray, 0.0001, f64::INFINITY) {
                // hit sth., record if diffuse and update power/ray
                let (interaction, out_ray, new_power) = rec.mat.scatter_photon(&ray, &rec, power);
                match interaction {
                    Interaction::Diffuse => {
                        all_photons.push(Photon::new(rec.p, power, ray.dir));
                        if !has_diffuse && has_specular {
                            // LS+D only
                            caustic_photons.push(Photon::new(rec.p, power, ray.dir))
                        }
                        has_diffuse = true;
                    }
                    Interaction::Absorb => {
                        break;
                    }
                    _ => {
                        has_specular = true;
                    }
                }
                if let (Some(out_ray), Some(new_power)) = (out_ray, new_power) {
                    ray = out_ray;
                    power = new_power;
                }
            }
        }

        // build kd tree
        global_pm.n_emitted += n_emit_photons;
        global_pm.map = Some(kd_tree::KdTree::build_by_ordered_float(all_photons));
        caustic_pm.n_emitted += n_emit_photons;
        caustic_pm.map = Some(kd_tree::KdTree::build_by_ordered_float(caustic_photons));
    }

    pub fn ray_color_pm(
        &self,
        r: Ray,
        depth: i32,
        global_pm: &PhotonMap,
        caustic_pm: &PhotonMap,
    ) -> Vec3 {
        let debug_render_pm = false;
        if depth <= 0 {
            return Vec3::zero();
        }
        if let Some(rec) = self.bvh.hit(&r, 0.001, f64::INFINITY) {
            if debug_render_pm {
                let (x, y, z) = rec.p.xyz();
                let within_global = global_pm
                    .map
                    .as_ref()
                    .unwrap()
                    .within_radius(&[x, y, z], 3.);
                let within_caustic = caustic_pm
                    .map
                    .as_ref()
                    .unwrap()
                    .within_radius(&[x, y, z], 3.);
                return Vec3::new(1., 0., 0.) * (within_global.len() as f64 / 50.)
                    + Vec3::new(0., 1., 0.) * (within_caustic.len() as f64 / 50.);
            }

            let emission = rec.mat.emitted(&rec);
            match rec.mat.scatter(&r, &rec) {
                (Interaction::Diffuse, Some(_scattered), Some(attenuation)) => {
                    let (caustic_flux, caustic_radius2) =
                        caustic_pm.estimate_flux_by_count(&rec, 10);
                    let caustic_flux = caustic_pm.adjust_flux(caustic_flux, caustic_radius2);

                    // let (global_flux, global_radius2) = global_pm.estimate_flux_by_count(&rec, 100);
                    // let global_flux = global_pm.adjust_flux(global_flux, global_radius2);

                    let mut global_flux = (0..GATHER_CNT)
                        .map(|_| {
                            let diffuse_ray =
                                Ray::new(rec.p, Vec3::random_in_hemisphere(&rec.normal)); //todo(uniform)
                            self.bvh.hit(&diffuse_ray, 0.0001, f64::INFINITY)
                        })
                        .map(|hit_result| match hit_result {
                            Some(another_rec) => {
                                let (f, r2) = global_pm.estimate_flux_by_count(&another_rec, 25);
                                f / r2
                            }
                            None => Vec3::zero(),
                        })
                        .sum();
                    global_flux *= FRAC_1_GATHER_CNT;
                    global_flux = global_pm.adjust_flux(global_flux, 1.);
                    global_flux = Vec3::elemul(global_flux, attenuation);

                    self.lights.sample_li(&rec, self, 10) + emission + caustic_flux + global_flux
                    // emission + caustic_flux + global_flux
                    // emission + caustic_flux
                    // emission + global_flux
                }
                (_, Some(scattered), Some(attenuation)) => {
                    emission
                        + Vec3::elemul(
                            attenuation,
                            self.ray_color_pm(scattered, depth - 1, global_pm, caustic_pm),
                        )
                }
                _ => emission,
            }
        } else {
            Vec3::zero()
        }
    }
}
