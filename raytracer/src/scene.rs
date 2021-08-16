use crate::light::SphereDiffuseLight;
use crate::material::{ConstantTexture, Dielectric, Lambertian, Material, Metal};
use crate::objects::hit::Hitable;
use crate::objects::rectangle::{XYRectangle, XZRectangle, YZRectangle};
use crate::objects::sphere::Sphere;
use crate::world::{Camera, World};
use crate::Vec3;
use std::sync::Arc;

fn cornell_box_scene() -> World {
    let red: Arc<dyn Material> = Arc::new(Lambertian::new(ConstantTexture(Vec3::new(
        0.65, 0.05, 0.05,
    ))));
    let white: Arc<dyn Material> = Arc::new(Lambertian::new(ConstantTexture(Vec3::new(
        0.73, 0.73, 0.73,
    ))));
    let green: Arc<dyn Material> = Arc::new(Lambertian::new(ConstantTexture(Vec3::new(
        0.12, 0.45, 0.15,
    ))));
    let light = SphereDiffuseLight::new(
        Vec3::new(275., 550., 275.),
        50.,
        Vec3::new(4., 4., 4.),
        20000000.,
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
            material: Arc::clone(&green),
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
            center: Vec3::new(200., 120., 300.),
            radius: 120.,
            material: Arc::new(Dielectric::new(1.5, ConstantTexture(Vec3::ones()))),
        }),
        Arc::new(Sphere {
            center: Vec3::new(90., 90., 150.),
            radius: 90.,
            material: Arc::clone(&white),
        }),
        Arc::new(Sphere {
            center: Vec3::new(450., 60., 200.),
            radius: 60.,
            material: Arc::new(Metal::new(ConstantTexture(Vec3::ones()), 0.)),
        }),
        Arc::new(light.clone()),
    ];

    World::new(
        hitable_list,
        Camera::new(
            (Vec3::new(278., 278., -800.), Vec3::new(278., 278., 278.)),
            Vec3::new(0., 1., 0.),
            40.,
            16. / 9.,
            0.0,
            10.0,
        ),
        vec![Arc::new(light)],
    )
}

pub fn select_scene(_index: usize) -> World {
    cornell_box_scene()
}
