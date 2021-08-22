use crate::integrator::photon_mapper::Integrator;
use crate::vec3::degrees_to_radians;
use crate::{Ray, Vec3, CONFIGS};
use image::{ImageBuffer, Rgb, RgbImage};
use indicatif::ProgressBar;
use rand::Rng;
use std::sync::mpsc::channel;
use std::sync::Arc;
use threadpool::ThreadPool;

#[derive(Copy, Clone)]
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

    pub fn capture_image(&self, integrator: Arc<dyn Integrator>) -> RgbImage {
        // jobs: split image into how many parts
        // workers: maximum allowed concurrent running threads
        let (n_jobs, n_workers): (usize, usize) = CONFIGS.n_jobs_n_workers;
        let width = CONFIGS.width;
        let height = CONFIGS.height;

        let sample_per_pixel = 256;
        let pool = ThreadPool::new(n_workers);

        let bar = ProgressBar::new(n_jobs as u64);
        let (tx, rx) = channel();
        let cam = Arc::new(*self);
        for i in 0..n_jobs {
            let tx = tx.clone();
            let integrator = integrator.clone();
            let cam = cam.clone();
            pool.execute(move || {
                // here, we render some of the rows of image in one thread
                let row_begin = height as usize * i / n_jobs;
                let row_end = height as usize * (i + 1) / n_jobs;
                let render_height = row_end - row_begin;
                let mut img: RgbImage = ImageBuffer::new(width as u32, render_height as u32);

                let mut rng = rand::thread_rng();
                for x in 0..width {
                    // img_y is the row in partial rendered image
                    // y is real position in final image
                    for (img_y, y) in (row_begin..row_end).enumerate() {
                        let mut pixel_color = Vec3::zero();
                        for _s in 0..sample_per_pixel {
                            let u = (x as f64 + rng.gen::<f64>()) / (width - 1) as f64;
                            let v = (y as f64 + rng.gen::<f64>()) / (height - 1) as f64;
                            let r = cam.get_ray(u, 1.0 - v); // y axis is reverted
                            pixel_color += integrator.sample_ray(r, x, y);
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
        let mut result: RgbImage = ImageBuffer::new(width as u32, height as u32);
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
        bar.finish();
        result
    }
}
