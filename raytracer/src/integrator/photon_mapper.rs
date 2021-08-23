use crate::integrator::photon_mapper::PMType::{Caustic, Global};
use crate::light::Photon;
use crate::material::Interaction;
use crate::material::Interaction::Diffuse;
use crate::objects::hit::HitRecord;
use crate::world::World;
use crate::CONFIGS;
use crate::{Ray, Vec3};
use indicatif::ProgressBar;
use kd_tree::{ItemAndDistance, KdTreeN};
use rand::Rng;
use std::f64::consts::PI;
use std::sync::mpsc::channel;
use std::sync::{Arc, Mutex};
use threadpool::ThreadPool;

const ALPHA: f64 = 0.7;
const GLOBAL_INIT_PHOTONS: usize = 100;
const CAUSTIC_INIT_PHOTONS: usize = 50;
// pub const GATHER_CNT: usize = 1;
// pub(crate) const FRAC_1_GATHER_CNT: f64 = 1. / GATHER_CNT as f64;

pub enum PMType {
    Global,
    Caustic,
}

pub struct PhotonMap {
    pub pm_type: PMType,
    pub map: KdTreeN<Photon, typenum::U3>,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct SPPM {
    pub flux: Vec3,
    pub radius2: f64,
    pub photons: usize,
    // pub count: usize,               // irradiance cache
    // pub sum_harmonic_distance: f64, // irradiance cache
}

impl SPPM {
    pub fn update_global(&mut self, rec: &HitRecord, pm: &PhotonMap) {
        self.update(rec, pm, GLOBAL_INIT_PHOTONS)
    }
    pub fn update_caustic(&mut self, rec: &HitRecord, pm: &PhotonMap) {
        self.update(rec, pm, CAUSTIC_INIT_PHOTONS)
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
    pub global: SPPM,
    pub caustic: SPPM,
}

impl PhotonMap {
    pub fn new(pm_type: PMType, map: KdTreeN<Photon, typenum::U3>) -> Self {
        Self { pm_type, map }
    }

    fn disk_factor(rec: &HitRecord, photon: &Photon) -> f64 {
        (*photon.norm() * (*photon.position() - rec.p).unit()).abs()
    }
    /// KNN Estimate
    /// return (flux, radius2)
    pub fn estimate_flux_by_count(&self, rec: &HitRecord, n_nearest: usize) -> (Vec3, f64) {
        let pm = self.map.as_ref();
        let p = rec.p.xyz_arr();
        let nearest = pm.nearests(&p, n_nearest);
        let mut flux = Vec3::zero();
        let mut radius2: f64 = 0.;
        for ItemAndDistance {
            item: photon,
            squared_distance,
        } in nearest.iter()
        {
            radius2 = radius2.max(*squared_distance);
            let disk_factor = PhotonMap::disk_factor(rec, photon);
            flux += Vec3::elemul(rec.mat.bsdf(*photon.direction(), &rec), *photon.power())
                * (1. - disk_factor);
        }
        (flux, radius2)
    }

    /// return (flux, photon_cnt)
    pub fn estimate_flux_within_radius(&self, rec: &HitRecord, radius: f64) -> (Vec3, usize) {
        let pm = self.map.as_ref();
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
    }
}

pub fn adjust_flux(flux_in: Vec3, radius2: f64, photons: usize) -> Vec3 {
    flux_in / (PI * radius2 * photons as f64)
}

pub trait Integrator: Send + Sync {
    fn sample_ray(&self, ray: Ray, x: usize, y: usize) -> Vec3;
    fn sample_emission(&self, rec: &HitRecord) -> Vec3 {
        rec.mat.emitted(&rec)
    }
}

pub struct SPPMIntegrator {
    world: Arc<World>,
    sppm_pixels: Vec<Vec<SPPMPixel>>,
    global_photons: usize,
    caustic_photons: usize,
    width: usize,
    height: usize,
    // debug_last_pm: Option<Arc<PhotonMap>>,
}

impl SPPMIntegrator {
    pub fn new(world: Arc<World>) -> Self {
        // get environment variable CI, which is true for GitHub Action
        // jobs: split image into how many parts
        // workers: maximum allowed concurrent running threads
        let (n_jobs, n_workers): (usize, usize) = CONFIGS.n_jobs_n_workers;
        let width = CONFIGS.width;
        let height = CONFIGS.height;

        // configs
        let max_iter_cnt = 50;
        let photon_per_iter = 500000;

        let sppm_pixels = Arc::new(Mutex::new(vec![
            vec![SPPMPixel::default(); height as usize];
            width
        ]));

        let bar = ProgressBar::new(n_jobs as u64 * max_iter_cnt as u64);
        // 1. sppm pass
        for _sppm_iter in 0..max_iter_cnt {
            // photon tracing pass
            let (global_pm, caustic_pm) =
                SPPMIntegrator::generate_photon_map(&world, photon_per_iter);
            let global_pm = Arc::new(global_pm);
            let caustic_pm = Arc::new(caustic_pm);

            // create a channel to send objects between threads
            let (tx, rx) = channel();
            let pool = ThreadPool::new(n_workers);

            // ray tracing pass
            for i in 0..n_jobs {
                let tx = tx.clone();
                let world_ptr = world.clone();
                let sppm_pixels_ptr = sppm_pixels.clone();
                let (global_pm_ptr, caustic_pm_ptr) = (global_pm.clone(), caustic_pm.clone());

                pool.execute(move || {
                    // here, we render some of the rows of image in one thread
                    let row_begin = height as usize * i / n_jobs;
                    let row_end = height as usize * (i + 1) / n_jobs;
                    let render_height = row_end - row_begin;
                    let mut sppm_local =
                        vec![vec![SPPMPixel::default(); render_height as usize]; width];

                    let mut rng = rand::thread_rng();
                    let (global_pm, caustic_pm) = (global_pm_ptr.as_ref(), caustic_pm_ptr.as_ref());
                    for (x, sppm_local) in sppm_local.iter_mut().enumerate().take(width) {
                        // img_y is the row in partial rendered image
                        // y is real position in final image
                        for (img_y, y) in (row_begin..row_end).enumerate() {
                            sppm_local[img_y] = sppm_pixels_ptr.lock().unwrap()[x][y];
                            let u = (x as f64 + rng.gen::<f64>()) / (width - 1) as f64;
                            let v = (y as f64 + rng.gen::<f64>()) / (height - 1) as f64;
                            let r = world_ptr.cam.get_ray(u, 1.0 - v); // y axis is reverted
                            SPPMIntegrator::update_sppm(
                                world_ptr.as_ref(),
                                r,
                                global_pm,
                                caustic_pm,
                                &mut sppm_local[img_y],
                            );
                        }
                    }
                    tx.send((row_begin..row_end, sppm_local))
                        .expect("failed to send result");
                });
            }
            for (rows, sppm) in rx.iter().take(n_jobs) {
                for (col, sppm_col) in sppm.iter().enumerate().take(width) {
                    for (idx, row) in (rows.start..rows.end).enumerate() {
                        (*sppm_pixels.lock().unwrap())[col][row] = sppm_col[idx];
                    }
                }
                bar.inc(1);
            }
        }
        bar.finish();

        let mut final_sppm_pixels = vec![vec![SPPMPixel::default(); height as usize]; width];
        let sppm_pixels = sppm_pixels.lock().unwrap();
        for (i, r) in sppm_pixels.iter().enumerate() {
            for (j, v) in r.iter().enumerate() {
                final_sppm_pixels[i][j] = *v;
            }
        }
        Self {
            world,
            sppm_pixels: final_sppm_pixels,
            global_photons: max_iter_cnt * photon_per_iter,
            caustic_photons: max_iter_cnt * photon_per_iter,
            width,
            height,
        }
    }
    fn generate_photon_map(world: &Arc<World>, n_emit_photons: usize) -> (PhotonMap, PhotonMap) {
        let mut all_photons = vec![];
        let mut caustic_photons = vec![];

        // trace photon
        for _ in 0..n_emit_photons {
            let (mut ray, mut power, _norm) = world.lights.emit();
            let (mut has_specular, mut has_diffuse) = (false, false);
            while let Some(rec) = world.hit(&ray, 0.0001, f64::INFINITY) {
                // hit sth., record if diffuse and update power/ray
                let (interaction, out_ray, new_power) = rec.mat.scatter_photon(&ray, &rec, power);
                match interaction {
                    Interaction::Diffuse => {
                        // let photon = Photon::new(rec.p, power, ray.dir, rec.normal);
                        all_photons.push(Photon::new(rec.p, power, ray.dir, rec.normal));
                        if !has_diffuse && has_specular {
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
        (
            PhotonMap::new(Global, kd_tree::KdTree::build_by_ordered_float(all_photons)),
            PhotonMap::new(
                Caustic,
                kd_tree::KdTree::build_by_ordered_float(caustic_photons),
            ),
        )
    }
    fn update_sppm(
        world: &World,
        ray: Ray,
        global_pm: &PhotonMap,
        caustic_pm: &PhotonMap,
        sppm_pixel: &mut SPPMPixel,
    ) {
        let mut curr_ray = ray;
        while let Some(rec) = world.hit(&curr_ray, 0.001, f64::INFINITY) {
            match rec.mat.scatter(&curr_ray, &rec) {
                (Diffuse, _, _) => {
                    sppm_pixel.caustic.update_caustic(&rec, caustic_pm);
                    sppm_pixel.global.update_global(&rec, global_pm);
                    break;
                }
                (_, Some(scattered), Some(_attenuation)) => {
                    curr_ray = scattered;
                }
                _ => {
                    break;
                }
            }
        }
    }
    fn estimate_caustic_radiance(&self, x: usize, y: usize) -> Vec3 {
        adjust_flux(
            self.sppm_pixels[x][y].caustic.flux,
            self.sppm_pixels[x][y].caustic.radius2,
            self.caustic_photons,
        )
    }
    fn estimate_global_radiance(&self, x: usize, y: usize) -> Vec3 {
        adjust_flux(
            self.sppm_pixels[x][y].global.flux,
            self.sppm_pixels[x][y].global.radius2,
            self.global_photons,
        )
    }

    // pub fn debug_sample_photon_map(&self, ray: Ray, pm: &PhotonMap) -> Vec3 {
    //     if let Some(rec) = self.world.hit(&ray, 0.001, f64::INFINITY) {
    //         let nearest = pm.map.within_radius(&rec.p.xyz_arr(), 3.);
    //         Vec3::new(1., 0., 0.) * (nearest.len() as f64 / 30.)
    //     } else {
    //         Vec3::zero()
    //     }
    // }
}

impl Integrator for SPPMIntegrator {
    fn sample_ray(&self, ray: Ray, x: usize, y: usize) -> Vec3 {
        if x >= self.width || y >= self.height {
            panic!("SPPM Integrator require capture size equal to sppm pixel size")
        }
        let mut throughput = Vec3::ones();
        let mut radiance = Vec3::zero(); // final radiance
        let mut curr_ray = ray;
        let mut depth = 50;
        while let Some(rec) = self.world.hit(&curr_ray, 0.001, f64::INFINITY) {
            if depth <= 0 {
                break;
            }
            depth -= 1;

            radiance += Vec3::elemul(throughput, self.sample_emission(&rec)); // Le

            let (interaction, scattered, attenuation) = rec.mat.scatter(&curr_ray, &rec);
            match (interaction, scattered, attenuation) {
                (Diffuse, Some(_scattered), Some(_attenuation)) => {
                    // throughput = Vec3::elemul(throughput, attenuation);
                    // curr_ray = scattered;
                    //
                    radiance += Vec3::elemul(throughput, self.estimate_caustic_radiance(x, y));
                    radiance += Vec3::elemul(throughput, self.estimate_global_radiance(x, y));
                    break;
                }
                (_, Some(scattered), Some(attenuation)) => {
                    // reflect or refract
                    throughput = Vec3::elemul(throughput, attenuation);
                    curr_ray = scattered;
                }
                _ => {
                    // absorb
                    break;
                }
            }
        }
        radiance
    }
}
