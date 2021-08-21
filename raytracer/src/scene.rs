#![allow(unused_imports)]
use crate::camera::Camera;
use crate::light::{SphereDiffuseLight, XZRectLight};
use crate::material::{ConstantTexture, Dielectric, Lambertian, Material, Metal};
use crate::objects::hit::Hitable;
use crate::objects::rectangle::{XYRectangle, XZRectangle, YZRectangle};
use crate::objects::sphere::Sphere;
use crate::world::World;
use crate::Vec3;
use std::sync::Arc;

fn cornell_box_scene() -> World {
    let red: Arc<dyn Material> = Arc::new(Lambertian::new(ConstantTexture(Vec3::new(
        0.75, 0.25, 0.25,
    ))));
    let white: Arc<dyn Material> = Arc::new(Lambertian::new(ConstantTexture(Vec3::new(
        0.75, 0.75, 0.75,
    ))));
    let blue: Arc<dyn Material> = Arc::new(Lambertian::new(ConstantTexture(Vec3::new(
        0.25, 0.25, 0.75,
    ))));
    // let light = SphereDiffuseLight::new(
    //     Vec3::new(275., 550., 275.),
    //     50.,
    //     Vec3::new(1., 1., 1.),
    //     15000000.,
    // );
    let light = XZRectLight::new(
        (213., 227.),
        (343., 332.),
        554.,
        Vec3::new(1., 1., 1.),
        2000000.,
    );
    let hitable_list: Vec<Arc<dyn Hitable>> = vec![
        Arc::new(YZRectangle {
            yz0: (0.0, 0.0),
            yz1: (555.0, 555.0),
            x: 555.,
            material: Arc::clone(&red),
        }),
        Arc::new(YZRectangle {
            yz0: (0., 0.),
            yz1: (555., 555.),
            x: 0.,
            material: Arc::clone(&blue),
        }),
        // Arc::new(XZRectangle {
        //     xz0: (113., 127.),
        //     xz1: (443., 432.),
        //     y: 554.,
        //     material: Arc::clone(&light),
        // }),
        Arc::new(XZRectangle {
            xz0: (0., 0.),
            xz1: (555., 555.),
            y: 0.,
            material: Arc::clone(&white),
        }),
        Arc::new(XZRectangle {
            xz0: (0., 0.),
            xz1: (555., 555.),
            y: 555.,
            material: Arc::clone(&white),
        }),
        Arc::new(XYRectangle {
            xy0: (0., 0.),
            xy1: (555., 555.),
            z: 555.,
            material: Arc::clone(&white),
        }),
        Arc::new(Sphere {
            center: Vec3::new(140., 100., 240.),
            radius: 100.,
            material: Arc::new(Dielectric::new(1.5, ConstantTexture(Vec3::ones() * 0.999))),
        }),
        Arc::new(Sphere {
            center: Vec3::new(400., 100., 360.),
            radius: 100.,
            material: Arc::new(Metal::new(ConstantTexture(Vec3::ones() * 0.999), 0.)),
        }),
        Arc::new(light.clone()),
    ];

    World::new(
        hitable_list,
        Camera::new(
            (Vec3::new(278., 278., -800.), Vec3::new(278., 278., 278.)),
            Vec3::new(0., 1., 0.),
            50.,
            1.,
            0.0,
            10.0,
        ),
        vec![Arc::new(light)],
    )
}

pub fn select_scene(_index: usize) -> World {
    cornell_box_scene()
}
