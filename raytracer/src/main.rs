#![allow(clippy::float_cmp)]

mod camera;
mod light;
mod material;
mod objects;
mod ray;
mod scene;
mod vec3;
mod world;

use crate::world::SPPMPixel;
use image::{ImageBuffer, Rgb, RgbImage};
use indicatif::ProgressBar;
use rand::Rng;
pub use ray::Ray;
use scene::select_scene;
use std::sync::mpsc::channel;
use std::sync::{Arc, Mutex};
use std::time::SystemTime;
use threadpool::ThreadPool;
pub use vec3::Vec3;

fn is_ci() -> bool {
    option_env!("CI").unwrap_or_default() == "true"
}

fn main() {
    // get environment variable CI, which is true for GitHub Action
    let is_ci = is_ci();

    // jobs: split image into how many parts
    // workers: maximum allowed concurrent running threads
    let (n_jobs, n_workers): (usize, usize) = if is_ci { (32, 2) } else { (64, 4) };

    println!(
        "CI: {}, using {} jobs and {} workers",
        is_ci, n_jobs, n_workers
    );

    // Image
    let aspect_ratio = 1.;
    let width = 800;
    let height = (width as f64 / aspect_ratio) as u32;

    let max_depth = 50;
    let max_iter_cnt = 100;
    let sample_per_pixel = 256;
    let photon_per_iter = 200000;

    // create a channel to send objects between threads
    let (tx, rx) = channel();
    let pool = ThreadPool::new(n_workers);

    let bar = ProgressBar::new(n_jobs as u64 * max_iter_cnt as u64);
    // use Arc to pass one instance of World to multiple threads
    let world = Arc::new(select_scene(0));
    let sppm_pixels = Arc::new(Mutex::new(vec![
        vec![SPPMPixel::default(); height as usize];
        width
    ]));

    let start_time = SystemTime::now();
    // sppm, get flux estimation
    for _sppm_iter in 0..max_iter_cnt {
        // 1. photon mapping pass
        let (global_pm, caustic_pm) = world.gen_photon_maps(photon_per_iter);
        let global_pm = Arc::new(global_pm);
        let caustic_pm = Arc::new(caustic_pm);

        // 2. ray tracing pass
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
                for x in 0..width {
                    // img_y is the row in partial rendered image
                    // y is real position in final image
                    for (img_y, y) in (row_begin..row_end).enumerate() {
                        sppm_local[x][img_y] = sppm_pixels_ptr.lock().unwrap()[x][y];
                        let u = (x as f64 + rng.gen::<f64>()) / (width - 1) as f64;
                        let v = (y as f64 + rng.gen::<f64>()) / (height - 1) as f64;
                        let r = world_ptr.cam.get_ray(u, 1.0 - v); // y axis is reverted
                        world_ptr.ray_trace_update_sppm(
                            r,
                            max_depth,
                            global_pm,
                            caustic_pm,
                            &mut sppm_local[x][img_y],
                        );
                    }
                }
                tx.send((row_begin..row_end, sppm_local))
                    .expect("failed to send result");
            });
        }
        for (rows, sppm) in rx.iter().take(n_jobs) {
            // idx is the corresponding row in partial-rendered image
            for (idx, row) in rows.enumerate() {
                for col in 0..width {
                    let row = row as u32;
                    let idx = idx as u32;
                    let mut sppm_pixels = sppm_pixels.lock().unwrap();
                    (*sppm_pixels)[col][row as usize] = sppm[col][idx as usize];
                }
            }
            bar.inc(1);
        }
    }
    bar.finish();

    let ray_tracing_start_time = SystemTime::now();
    // ray tracing only
    let bar = ProgressBar::new(n_jobs as u64);
    let (tx, rx) = channel();
    for i in 0..n_jobs {
        let tx = tx.clone();
        let world_ptr = world.clone();
        let sppm_pixels_ptr = sppm_pixels.clone();
        pool.execute(move || {
            // here, we render some of the rows of image in one thread
            let row_begin = height as usize * i / n_jobs;
            let row_end = height as usize * (i + 1) / n_jobs;
            let render_height = row_end - row_begin;
            let mut img: RgbImage = ImageBuffer::new(width as u32, render_height as u32);
            let photons = photon_per_iter * max_iter_cnt;
            let mut rng = rand::thread_rng();
            for x in 0..width {
                // img_y is the row in partial rendered image
                // y is real position in final image
                for (img_y, y) in (row_begin..row_end).enumerate() {
                    let sppm_local = sppm_pixels_ptr.lock().unwrap()[x][y];

                    // world_ptr.ray_trace_update_sppm(r, max_depth, global_pm, caustic_pm, &mut sppm_local);
                    let mut pixel_color = Vec3::zero();
                    for _s in 0..sample_per_pixel {
                        let u = (x as f64 + rng.gen::<f64>()) / (width - 1) as f64;
                        let v = (y as f64 + rng.gen::<f64>()) / (height - 1) as f64;
                        let r = world_ptr.cam.get_ray(u, 1.0 - v); // y axis is reverted
                        pixel_color +=
                            world_ptr.ray_color(r, max_depth, &sppm_local, photons, photons);
                    }
                    pixel_color /= sample_per_pixel as f64;
                    let pixel = img.get_pixel_mut(x as u32, img_y as u32);
                    *pixel = Rgb::from(pixel_color);
                }
            }
            // send row range and rendered image to main thread
            tx.send((row_begin..row_end, img))
                .expect("failed to send result");
        });
    }

    // output result image
    let mut result: RgbImage = ImageBuffer::new(width as u32, height);
    for (rows, img) in rx.iter().take(n_jobs) {
        // idx is the corresponding row in partial-rendered image
        for (idx, row) in rows.enumerate() {
            for col in 0..width {
                let row = row as u32;
                let idx = idx as u32;
                *result.get_pixel_mut(col as u32, row) = *img.get_pixel(col as u32, idx);
            }
        }
        bar.inc(1);
    }
    result.save("output/test.png").unwrap();

    bar.finish();
    println!(
        "Total: {}s\n\tSPPM: {}s\n\tRT: {}s",
        SystemTime::now()
            .duration_since(start_time)
            .unwrap()
            .as_secs(),
        ray_tracing_start_time
            .duration_since(start_time)
            .unwrap()
            .as_secs(),
        SystemTime::now()
            .duration_since(ray_tracing_start_time)
            .unwrap()
            .as_secs(),
    );
}
