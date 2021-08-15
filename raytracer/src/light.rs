use crate::vec3::polar_direction;
use crate::{Ray, Vec3};
use kd_tree::KdPoint;
use rand::distributions::Distribution;
use rand::distributions::WeightedIndex;
use rand::thread_rng;
use std::f64::consts::PI;
use std::sync::Arc;

pub struct Photon {
    position: Vec3,
    phi: u8,
    theta: u8,
    power: Vec3,
}

impl Photon {
    pub fn new(position: Vec3, power: Vec3, direction: Vec3) -> Self {
        let phi = (255. * (f64::atan2(direction.y, direction.x) + PI) / (2. * PI)).floor() as u8;
        let theta = (255. * f64::acos(direction.x) / PI).floor() as u8;
        Self {
            position,
            phi,
            theta,
            power,
        }
    }
    pub fn position(&self) -> Vec3 {
        self.position
    }
    pub fn power(&self) -> Vec3 {
        self.power
    }
    pub fn direction(&self) -> Vec3 {
        polar_direction(self.theta, self.phi)
    }
}

impl Default for Photon {
    fn default() -> Self {
        Photon::new(Vec3::zero(), Vec3::zero(), Vec3::zero())
    }
}

impl KdPoint for Photon {
    type Scalar = f64;
    type Dim = typenum::U3;
    fn at(&self, i: usize) -> Self::Scalar {
        self.position[i]
    }
}

pub trait Light: Sync + Send {
    fn emit(&self) -> (Ray, Vec3); // ray, power
    fn power(&self) -> Vec3; // total power
    fn sample_li(&self) -> Vec3;
}

pub struct SphereLight {
    pub position: Vec3,
    pub flux: Vec3,
    pub scale: f64,
}

impl Light for SphereLight {
    fn emit(&self) -> (Ray, Vec3) {
        (
            Ray::new(self.position, Vec3::random_in_unit_sphere(), 0.),
            self.flux * self.scale,
        )
    }
    fn power(&self) -> Vec3 {
        self.scale * self.flux
    }
    fn sample_li(&self) -> Vec3 {
        todo!()
    }
}

pub struct AllLights {
    // lights: RangeMap<f64, Arc<dyn Light>>,
    lights: Vec<Arc<dyn Light>>,
    lights_prob: Vec<f64>,
    // light_dist: WeightedIndex<f64>,
}

impl AllLights {
    pub fn new(lights: Vec<Arc<dyn Light>>) -> Self {
        let light_powers = lights
            .iter()
            .map(|l| l.power().length())
            .collect::<Vec<f64>>();
        let tot_power: f64 = light_powers.iter().sum();
        let lights_prob = light_powers
            .iter()
            .map(|p| p / tot_power)
            .collect::<Vec<f64>>();
        Self {
            lights,
            lights_prob,
        }
    }
}

impl Light for AllLights {
    fn emit(&self) -> (Ray, Vec3) {
        let mut rng = thread_rng();
        let light_dist = WeightedIndex::new(&self.lights_prob).unwrap();
        let idx: usize = light_dist.sample(&mut rng);
        self.lights[idx].emit()
    }
    fn power(&self) -> Vec3 {
        self.lights.iter().map(|l| l.power()).sum()
    }
    fn sample_li(&self) -> Vec3 {
        todo!()
    }
}
