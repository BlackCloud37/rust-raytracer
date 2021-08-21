#![allow(dead_code)]
use crate::camera::Camera;
use crate::light::{AllLights, Light, Photon};
use crate::material::Interaction;
use crate::objects::bvh::BVHNode;
use crate::objects::hit::{HitRecord, Hitable};
use crate::{Ray, Vec3};
use kd_tree::KdTreeN;
use std::f64::consts::PI;
use std::sync::Arc;

const PMPT: bool = false;
const ALPHA: f64 = 0.7;
const GATHER_CNT: usize = 4;
const FRAC_1_GATHER_CNT: f64 = 1. / GATHER_CNT as f64;




pub enum PMType {
    Global,
    Caustic,
}

pub struct PhotonMap {
    pub pm_type: PMType,
    pub map: Option<KdTreeN<Photon, typenum::U3>>,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct SPPM {
    pub flux: Vec3,
    pub radius2: f64,
    pub photons: usize,
}

impl SPPM {
    pub fn update_global(&mut self, rec: &HitRecord, pm: &PhotonMap) {
        self.update(rec, pm, 100)
    }
    pub fn update_caustic(&mut self, rec: &HitRecord, pm: &PhotonMap) {
        self.update(rec, pm, 10)
    }
    pub fn update(&mut self, rec: &HitRecord, pm: &PhotonMap, photon_init: usize) {
        if self.photons == 0 {
            self.photons = photon_init;
            let (flux, r2) = pm.estimate_flux_by_count(&rec, self.photons);
            self.flux = flux;
            self.radius2 = r2;
        } else {
            let (flux, photons) = pm.estimate_flux_within_radius(rec, self.radius2.sqrt());
            let prev_photons = self.photons;
            self.photons += (ALPHA * photons as f64) as usize;
            let frac = self.photons as f64 / (prev_photons + photons) as f64;
            self.radius2 *= frac;
            self.flux = (self.flux + flux) * frac;
        }
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct SPPMPixel {
    pub global: [SPPM; GATHER_CNT],
    pub caustic: SPPM,
}

impl PhotonMap {
    pub fn new(pm_type: PMType) -> Self {
        Self { pm_type, map: None }
    }

    fn disk_factor(rec: &HitRecord, photon: &Photon) -> f64 {
        (*photon.norm() * (*photon.position() - rec.p).unit()).abs()
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
                let disk_factor = PhotonMap::disk_factor(rec, photon);
                flux += Vec3::elemul(rec.mat.bsdf(*photon.direction(), &rec), *photon.power())
                    * (1. - disk_factor);
            }
            (flux, radius2)
        } else {
            panic!("photon map not built yet")
        }
    }

    /// return (flux, photon_cnt)
    pub fn estimate_flux_within_radius(&self, rec: &HitRecord, radius: f64) -> (Vec3, usize) {
        // let frac_1_radius = 1. / radius;

        if let Some(pm) = self.map.as_ref() {
            let p = rec.p.xyz_arr();
            let nearest = pm.within_radius(&p, radius);
            let mut flux = Vec3::zero();
            let photons = nearest.len();
            for photon in nearest {
                let disk_factor = PhotonMap::disk_factor(rec, photon);
                flux += Vec3::elemul(rec.mat.bsdf(*photon.direction(), &rec), *photon.power())
                    * (1. - disk_factor);
            }
            (flux, photons)
        } else {
            panic!("photon map not built yet")
        }
    }
}

pub fn adjust_flux(flux_in: Vec3, radius2: f64, photons: usize) -> Vec3 {
    flux_in / (PI * radius2 * photons as f64)
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
            let (mut ray, mut power, _norm) = self.lights.emit();
            let (mut has_specular, mut has_diffuse) = (false, false);
            while let Some(rec) = self.bvh.hit(&ray, 0.0001, f64::INFINITY) {
                // hit sth., record if diffuse and update power/ray
                let (interaction, out_ray, new_power) = rec.mat.scatter_photon(&ray, &rec, power);
                match interaction {
                    Interaction::Diffuse => {
                        all_photons.push(Photon::new(rec.p, power, ray.dir, rec.normal));
                        if !has_diffuse && has_specular {
                            // LS+D only
                            caustic_photons.push(Photon::new(rec.p, power, ray.dir, rec.normal))
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
        global_pm.map = Some(kd_tree::KdTree::build_by_ordered_float(all_photons));
        caustic_pm.map = Some(kd_tree::KdTree::build_by_ordered_float(caustic_photons));
    }

    pub fn ray_trace_update_sppm(
        &self,
        r: Ray,
        depth: i32,
        global_pm: &PhotonMap,
        caustic_pm: &PhotonMap,
        sppm_pixel: &mut SPPMPixel,
    ) {
        if depth <= 0 {
            return;
        }
        if let Some(rec) = self.bvh.hit(&r, 0.001, f64::INFINITY) {
            match rec.mat.scatter(&r, &rec) {
                (Interaction::Diffuse, Some(_scattered), Some(_attenuation)) => {
                    sppm_pixel.caustic.update_caustic(&rec, caustic_pm);
                    if PMPT {
                        for i in 0..GATHER_CNT {
                            let global_sppm = &mut sppm_pixel.global[i];
                            let diffuse_ray =
                                Ray::new(rec.p, Vec3::random_in_hemisphere(&rec.normal));
                            if let Some(another_rec) =
                                self.bvh.hit(&diffuse_ray, 0.0001, f64::INFINITY)
                            {
                                global_sppm.update_global(&another_rec, global_pm);
                            }
                        }
                    } else {
                        sppm_pixel.global[0].update_global(&rec, global_pm);
                    }
                }
                (_, Some(scattered), Some(_attenuation)) => self.ray_trace_update_sppm(
                    scattered,
                    depth - 1,
                    global_pm,
                    caustic_pm,
                    sppm_pixel,
                ),
                _ => {}
            }
        }
    }

    pub fn ray_color(
        &self,
        r: Ray,
        depth: i32,
        sppm_pixel: &SPPMPixel,
        global_photons: usize,
        caustic_photons: usize,
    ) -> Vec3 {
        if depth <= 0 {
            return Vec3::zero();
        }
        if let Some(rec) = self.bvh.hit(&r, 0.001, f64::INFINITY) {
            let emission = rec.mat.emitted(&rec);
            match rec.mat.scatter(&r, &rec) {
                (Interaction::Diffuse, Some(_scattered), Some(attenuation)) => {
                    let caustic_flux = adjust_flux(
                        sppm_pixel.caustic.flux,
                        sppm_pixel.caustic.radius2,
                        caustic_photons,
                    );
                    let mut global_flux = Vec3::zero();
                    if PMPT {
                        for i in 0..GATHER_CNT {
                            let global_sppm = sppm_pixel.global[i];
                            global_flux +=
                                adjust_flux(global_sppm.flux, global_sppm.radius2, global_photons);
                        }
                        global_flux *= FRAC_1_GATHER_CNT;
                        global_flux = Vec3::elemul(attenuation, global_flux);
                        self.lights.sample_li(&rec, self, 1) + emission + caustic_flux + global_flux
                        // global_flux
                    } else {
                        global_flux = adjust_flux(
                            sppm_pixel.global[0].flux,
                            sppm_pixel.global[0].radius2,
                            global_photons,
                        );
                        caustic_flux + global_flux + emission
                        // emission + self.lights.sample_li(&rec, self, 1) + caustic_flux + global_flux
                        // global_flux
                    }
                }
                (_, Some(scattered), Some(attenuation)) => {
                    emission
                        + Vec3::elemul(
                            attenuation,
                            self.ray_color(
                                scattered,
                                depth - 1,
                                sppm_pixel,
                                global_photons,
                                caustic_photons,
                            ),
                        )
                }
                _ => emission,
            }
        } else {
            Vec3::zero()
        }
    }
}
