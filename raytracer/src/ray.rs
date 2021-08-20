use crate::Vec3;

pub struct Ray {
    pub orig: Vec3,
    pub dir: Vec3,
}

impl Ray {
    pub fn new(orig: Vec3, dir: Vec3) -> Self {
        Self { orig, dir }
    }
    pub fn at(&self, t: f64) -> Vec3 {
        self.orig + t * self.dir
    }
}
