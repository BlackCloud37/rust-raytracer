use crate::Vec3;

pub struct Ray {
    pub orig: Vec3,
    pub dir: Vec3,
    pub time: f64,
}

impl Ray {
    pub fn new(orig: Vec3, dir: Vec3, time: f64) -> Self {
        Self { orig, dir, time }
    }
    pub fn at(&self, t: f64) -> Vec3 {
        self.orig + t * self.dir
    }
}
