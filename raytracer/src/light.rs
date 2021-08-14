use crate::vec3::polar_direction;
use crate::{Ray, Vec3};
use kd_tree::KdPoint;
use std::f64::consts::PI;

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
    fn power(&self) -> Vec3;
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
}
