#![allow(clippy::float_cmp)]

mod light;
mod material;
mod objects;
mod ray;
mod scene;
mod vec3;
mod world;

use image::{ImageBuffer, Rgb, RgbImage};
use indicatif::ProgressBar;
use rand::Rng;
pub use ray::Ray;
use scene::select_scene;
use std::sync::mpsc::channel;
use std::sync::{Arc, Mutex};
// use std::time::SystemTime;
use crate::world::PMType::{Caustic, Global};
use crate::world::{PhotonMap, SPPMPixel};
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

    let max_depth = 25;
    let max_iter_cnt = 100;

    // create a channel to send objects between threads
    let (tx, rx) = channel();
    let pool = ThreadPool::new(n_workers);

    let bar = ProgressBar::new(n_jobs as u64 * max_iter_cnt as u64);

    // use Arc to pass one instance of World to multiple threads
    let world = Arc::new(select_scene(0));
    let sppm_pixels = vec![vec![SPPMPixel::default(); height as usize]; width];
    let sppm_pixels = Arc::new(Mutex::new(sppm_pixels));

    let mut total_photons = 0;
    for iter in 0..max_iter_cnt {
        // 1. photon mapping pass
        let mut global_pm = PhotonMap::new(Global, total_photons);
        let mut caustic_pm = PhotonMap::new(Caustic, total_photons);
        world.gen_photon_maps(200000, &mut global_pm, &mut caustic_pm);
        total_photons = global_pm.n_emitted;
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
                let mut img: RgbImage = ImageBuffer::new(width as u32, render_height as u32);

                let mut sppm_local =
                    vec![vec![SPPMPixel::default(); render_height as usize]; width];

                let mut rng = rand::thread_rng();
                for x in 0..width {
                    // img_y is the row in partial rendered image
                    // y is real position in final image
                    for (img_y, y) in (row_begin..row_end).enumerate() {
                        sppm_local[x][img_y] = sppm_pixels_ptr.lock().unwrap()[x][y];
                        // sppm_local[x][img_y].global.photons += 1;

                        let u = (x as f64 + rng.gen::<f64>()) / (width - 1) as f64;
                        let v = (y as f64 + rng.gen::<f64>()) / (height - 1) as f64;
                        let r = world_ptr.cam.get_ray(u, 1.0 - v); // y axis is reverted
                        let (global_pm, caustic_pm) =
                            (global_pm_ptr.as_ref(), caustic_pm_ptr.as_ref());
                        let pixel_color = world_ptr.ray_color_pm(
                            r,
                            max_depth,
                            global_pm,
                            caustic_pm,
                            &mut sppm_local[x][img_y],
                        );
                        let pixel = img.get_pixel_mut(x as u32, img_y as u32);
                        *pixel = Rgb::from(pixel_color);
                    }
                }
                // send row range and rendered image to main thread
                tx.send((row_begin..row_end, img, sppm_local))
                    .expect("failed to send result");
            });
        }

        // 3. output result image
        let mut result: RgbImage = ImageBuffer::new(width as u32, height);

        for (rows, img, sppm) in rx.iter().take(n_jobs) {
            // idx is the corresponding row in partial-rendered image
            for (idx, row) in rows.enumerate() {
                for col in 0..width {
                    let row = row as u32;
                    let idx = idx as u32;
                    *result.get_pixel_mut(col as u32, row) = *img.get_pixel(col as u32, idx);
                    let mut sppm_pixels = sppm_pixels.lock().unwrap();
                    (*sppm_pixels)[col][row as usize] = sppm[col][idx as usize];
                }
            }
            bar.inc(1);
        }

        // print!(
        //     "{:?}\t",
        //     sppm_pixels.lock().unwrap()[500][400].global[0].radius2
        // );

        if iter % 10 == 0 || iter == max_iter_cnt - 1 {
            result.save(format!("output/iter-{}.png", iter)).unwrap();
        }
        // result.save("output/test.png").unwrap();
    }
    bar.finish();
}
