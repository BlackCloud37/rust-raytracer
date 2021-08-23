#![allow(clippy::float_cmp)]
#[macro_use]
extern crate lazy_static;

mod camera;
mod integrator;
mod light;
mod material;
mod objects;
mod ray;
mod scene;
mod vec3;
mod world;

use crate::integrator::photon_mapper::SPPMIntegrator;
pub use ray::Ray;
use scene::select_scene;
use std::sync::Arc;
use std::time::SystemTime;
pub use vec3::Vec3;

fn is_ci() -> bool {
    option_env!("CI").unwrap_or_default() == "true"
}

#[derive(Default)]
pub struct GlobalConfig {
    n_jobs_n_workers: (usize, usize),
    aspect_ratio: f64,
    width: usize,
    height: usize,
}

const WIDTH: usize = 800;
const ASPECT_RATIO: f64 = 1.;
lazy_static! {
    // todo!(integrator configs)
    pub static ref CONFIGS: GlobalConfig = GlobalConfig {
        // get environment variable CI, which is true for GitHub Action
        // jobs: split image into how many parts
        // workers: maximum allowed concurrent running threads
        n_jobs_n_workers: if is_ci() { (32, 2) } else { (64, 8) },
        aspect_ratio: ASPECT_RATIO,
        width: WIDTH,
        height: (WIDTH as f64 / ASPECT_RATIO) as usize,
    };
}

fn main() {
    let world = Arc::new(select_scene(0));
    let start_time = SystemTime::now();
    let integrator = Arc::new(SPPMIntegrator::new(world.clone()));
    let ray_tracing_start_time = SystemTime::now();
    let result = world.cam.capture_image(integrator);
    result.save("output/test.png").unwrap();

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
