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
use std::sync::Arc;
// use std::time::SystemTime;
use crate::world::PMType::{Caustic, Global};
use crate::world::PhotonMap;
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
    let aspect_ratio = 16. / 9.;
    let width = 1024;
    let height = (width as f64 / aspect_ratio) as u32;

    let max_depth = 25;
    let samples_per_pixel = 16;
    // create a channel to send objects between threads
    let (tx, rx) = channel();
    let pool = ThreadPool::new(n_workers);

    let bar = ProgressBar::new(n_jobs as u64);

    // use Arc to pass one instance of World to multiple threads
    let world = select_scene(0);
    let mut global_pm = PhotonMap::new(Global);
    let mut caustic_pm = PhotonMap::new(Caustic);
    world.gen_photon_maps(400000, &mut global_pm, &mut caustic_pm);
    // let [global_pm, caustic_pm] = world.gen_photon_maps(400000);
    let world = Arc::new(world);
    let (global_pm, caustic_pm) = (Arc::new(global_pm), Arc::new(caustic_pm));

    for i in 0..n_jobs {
        let tx = tx.clone();
        let world_ptr = world.clone();
        let (global_pm_ptr, caustic_pm_ptr) = (global_pm.clone(), caustic_pm.clone());
        pool.execute(move || {
            // here, we render some of the rows of image in one thread
            let row_begin = height as usize * i / n_jobs;
            let row_end = height as usize * (i + 1) / n_jobs;
            let render_height = row_end - row_begin;
            let mut img: RgbImage = ImageBuffer::new(width, render_height as u32);

            let mut rng = rand::thread_rng();
            for x in 0..width {
                // img_y is the row in partial rendered image
                // y is real position in final image
                for (img_y, y) in (row_begin..row_end).enumerate() {
                    let mut pixel_color = Vec3::zero();
                    for _s in 0..samples_per_pixel {
                        let u = (x as f64 + rng.gen::<f64>()) / (width - 1) as f64;
                        let v = (y as f64 + rng.gen::<f64>()) / (height - 1) as f64;
                        let r = world_ptr.cam.get_ray(u, 1.0 - v); // y axis is reverted
                        pixel_color += world_ptr.ray_color_pm(
                            r,
                            max_depth,
                            global_pm_ptr.as_ref(),
                            caustic_pm_ptr.as_ref(),
                        );
                    }
                    pixel_color /= samples_per_pixel as f64;
                    let pixel = img.get_pixel_mut(x, img_y as u32);
                    *pixel = Rgb::from(pixel_color);
                }
            }
            // send row range and rendered image to main thread
            tx.send((row_begin..row_end, img))
                .expect("failed to send result");
        });
    }

    let mut result: RgbImage = ImageBuffer::new(width, height);

    for (rows, data) in rx.iter().take(n_jobs) {
        // idx is the corresponding row in partial-rendered image
        for (idx, row) in rows.enumerate() {
            for col in 0..width {
                let row = row as u32;
                let idx = idx as u32;
                *result.get_pixel_mut(col, row) = *data.get_pixel(col, idx);
            }
        }
        bar.inc(1);
    }

    // let timestamp = SystemTime::now();
    // let time_str = format!("{}", );
    result.save("output/test.png").unwrap();
    bar.finish();
}
