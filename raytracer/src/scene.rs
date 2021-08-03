use crate::material::Hitable;
use crate::material::{ConstantTexture, DiffuseLight};
use crate::Ray;
use crate::Vec3;
use crate::World;
use raytracer_codegen::make_spheres_impl;

// Call the procedural macro, which will become `make_spheres` function.
make_spheres_impl! {}

pub struct Sphere {
    pub center: Vec3,
    pub radius: f64,
    pub material: DiffuseLight,
}

impl Hitable for Sphere {
    fn hit(&self, r: &Ray) -> bool {
        let oc = r.orig - self.center;
        let a = r.dir.squared_length();
        let b = 2. * oc * r.dir;
        let c = oc.squared_length() - self.radius * self.radius;
        let discriminant = b * b - 4. * a * c;
        discriminant > 0.
    }
}

pub fn example_scene() -> World {
    let mut spheres: Vec<Box<Sphere>> = make_spheres(); // Now `spheres` stores two spheres.
    let mut hittable_list = vec![];
    // You can now add spheres to your own world
    hittable_list.append(&mut spheres);

    hittable_list.clear();
    World { height: 512 }
}
