use crate::light::{AllLights, Light, Photon, SphereLight};
use crate::material::{
    ConstantTexture, Dielectric, DiffuseLight, Interaction, Lambertian, Material, Metal,
};
use crate::objects::bvh::BVHNode;
// use crate::objects::cube::Cube;
use crate::objects::hit::{HitRecord, Hitable};
// use crate::objects::medium::ConstantMedium;
use crate::objects::rectangle::{XYRectangle, XZRectangle, YZRectangle};
use crate::objects::sphere::Sphere;
// use crate::objects::transform::Transform;
use crate::{Ray, Vec3};
use kd_tree::KdTreeN;
use rand::Rng;
use std::f64::consts::PI;
use std::sync::Arc;

const N_PHOTONS: usize = 100000;
// const N_NEAREST: usize = 50;
pub struct Camera {
    pub origin: Vec3,
    pub lower_left_corner: Vec3,
    pub horizontal: Vec3,
    pub vertical: Vec3,
    pub u: Vec3,
    pub v: Vec3,
    pub w: Vec3,
    pub lens_radius: f64,
    pub time0: f64,
    pub time1: f64,
}

pub fn degrees_to_radians(degrees: f64) -> f64 {
    degrees * PI / 180.
}

impl Camera {
    pub fn new(
        look_from_to: (Vec3, Vec3),
        vup: Vec3,
        vfov: f64,
        aspect_ratio: f64,
        aperture: f64,
        focus_dist: f64,
        time_range: (f64, f64),
    ) -> Self {
        let (look_from, look_at) = look_from_to;
        let (time0, time1) = time_range;
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
            time0,
            time1,
        }
    }

    pub fn get_ray(&self, s: f64, t: f64) -> Ray {
        let rd: Vec3 = self.lens_radius * Vec3::random_in_unit_disk();
        let offset: Vec3 = self.u * rd.x + self.v * rd.y;
        let mut rng = rand::thread_rng();
        Ray::new(
            self.origin + offset,
            self.lower_left_corner + s * self.horizontal + t * self.vertical - self.origin - offset,
            rng.gen_range(self.time0..self.time1),
        )
    }
}

pub struct World {
    // pub hitable_list: Vec<Arc<dyn Hitable>>,
    pub bvh: BVHNode,
    pub cam: Camera,
    pub lights: AllLights,
    pub global_pm: KdTreeN<Photon, typenum::U3>,
    pub caustic_pm: KdTreeN<Photon, typenum::U3>,
    // pub volume_pm
}

impl World {
    pub fn new(
        hitable_list: Vec<Arc<dyn Hitable>>,
        cam: Camera,
        lights: Vec<Arc<dyn Light>>,
    ) -> Self {
        Self {
            bvh: BVHNode::new(hitable_list, 0.0, 1.0),
            cam,
            lights: AllLights::new(lights),
            global_pm: KdTreeN::default(),
            caustic_pm: KdTreeN::default(),
        }
    }

    pub fn estimate_flux(
        pm: &KdTreeN<Photon, typenum::U3>,
        rec: &HitRecord,
        n_nearest: usize,
    ) -> Vec3 {
        let (x, y, z) = rec.p.xyz();
        let nearest = pm.nearests(&[x, y, z], n_nearest);
        let mut flux = Vec3::zero();
        let mut radius2: f64 = 0.;
        for kd_tree::ItemAndDistance {
            item: photon,
            squared_distance,
        } in nearest.into_iter()
        {
            radius2 = radius2.max(squared_distance);
            if let (_, _, Some(f)) = rec
                .mat
                .scatter(&Ray::new(photon.position(), photon.direction(), 0.), &rec)
            {
                flux += Vec3::elemul(f, photon.power());
            }
        }
        flux / (PI * radius2 * N_PHOTONS as f64)
    }

    pub fn map_photons(&mut self) {
        let mut all_photons = vec![];
        let mut caustic_photons = vec![];

        // trace photon
        for _ in 0..N_PHOTONS {
            let (mut ray, mut power) = self.lights.emit();
            let (mut has_specular, mut has_diffuse) = (false, false);
            while let Some(rec) = self.bvh.hit(&ray, 0.0001, f64::INFINITY) {
                // hit sth., record if diffuse and update power/ray
                let (interaction, out_ray, new_power) = rec.mat.scatter_photon(&ray, &rec, power);
                match interaction {
                    Interaction::Diffuse => {
                        all_photons.push(Photon::new(rec.p, power, ray.dir));
                        if !has_diffuse && has_specular {
                            // LS+D only
                            caustic_photons.push(Photon::new(rec.p, power, ray.dir))
                        }
                        has_diffuse = true;
                    }
                    Interaction::Absorb => {
                        break;
                    }
                    _ => {
                        has_specular = true;
                    }
                }
                if let (Some(out_ray), Some(new_power)) = (out_ray, new_power) {
                    ray = out_ray;
                    power = new_power;
                }
            }
        }

        // build kd tree
        self.global_pm = kd_tree::KdTree::build_by_ordered_float(all_photons);
        self.caustic_pm = kd_tree::KdTree::build_by_ordered_float(caustic_photons);
    }

    pub fn ray_color_pm(&self, r: Ray, depth: i32) -> Vec3 {
        let debug_render_pm = false;
        if depth <= 0 {
            return Vec3::zero();
        }
        if let Some(rec) = self.bvh.hit(&r, 0.001, f64::INFINITY) {
            if debug_render_pm {
                let (x, y, z) = rec.p.xyz();
                let within_global = self.global_pm.within_radius(&[x, y, z], 3.);
                let within_caustic = self.caustic_pm.within_radius(&[x, y, z], 3.);
                return Vec3::new(1., 0., 0.) * (within_global.len() as f64 / 100.)
                    + Vec3::new(0., 1., 0.) * (within_caustic.len() as f64 / 100.);
            }

            // let emission = Vec3::zero();
            let emission = rec.mat.emitted(&rec); // todo!(emission from light source)
            match rec.mat.scatter(&r, &rec) {
                (Interaction::Diffuse, Some(scattered), Some(attenuation)) => {
                    let caustic_flux = World::estimate_flux(&self.caustic_pm, &rec, 10);
                    // global
                    // let diffuse_ray = Ray::new(rec.p, Vec3::random_in_hemisphere(&rec.normal), 0.);
                    // let global_flux = World::estimate_flux(&self.global_pm, &rec, 100);
                    let mut global_flux = Vec3::zero();
                    if let Some(another_rec) = self.bvh.hit(&scattered, 0.0001, f64::INFINITY) {
                        global_flux = World::estimate_flux(&self.global_pm, &another_rec, 100);
                    }
                    global_flux = Vec3::elemul(global_flux, attenuation);
                    // emission + Vec3::elemul(attenuation, self.ray_color_pm(scattered, depth - 1))
                    self.lights.sample_li(&rec, self) + emission + caustic_flux + global_flux
                }
                (_, Some(scattered), Some(attenuation)) => {
                    emission + Vec3::elemul(attenuation, self.ray_color_pm(scattered, depth - 1))
                }
                _ => emission,
            }
        } else {
            Vec3::zero()
        }
    }
}

fn random_scene() -> World {
    // use image::io::Reader as ImageReader;
    // let res = ImageReader::open("texture/earthmap.jpg");
    // if res.is_err() {
    //     panic!("Err reading texture")
    // }
    // let decode_res = res.unwrap().decode();
    // if decode_res.is_err() {
    //     panic!("Err reading texture")
    // }
    // let img = decode_res.unwrap();
    // let mut hitable_list: Vec<Arc<dyn Hitable>> = vec![
    //     Arc::new(Sphere {
    //         center: Vec3::new(0., -1000., 0.),
    //         radius: 1000.,
    //         material: Arc::new(Lambertian::new(CheckerTexture(
    //             ConstantTexture(Vec3::new(0.2, 0.3, 0.1)),
    //             ConstantTexture(Vec3::new(0.9, 0.9, 0.9)),
    //         ))),
    //     }),
    //     Arc::new(Sphere {
    //         center: Vec3::new(0., 1., 0.),
    //         radius: 1.,
    //         material: Arc::new(Lambertian::new(ImageTexture(img))),
    //     }),
    //     Arc::new(Sphere {
    //         center: Vec3::new(-4., 1., 0.),
    //         radius: 1.,
    //         material: Arc::new(DiffuseLight::new(ConstantTexture(Vec3::new(4., 4., 4.)))),
    //     }),
    //     Arc::new(Sphere {
    //         center: Vec3::new(4., 1., 0.),
    //         radius: 1.,
    //         material: Arc::new(Metal::new(ConstantTexture(Vec3::new(0.7, 0.6, 0.5)), 0.)),
    //     }),
    // ];
    // let mut rng = rand::thread_rng();
    // for a in -11..12 {
    //     for b in -11..12 {
    //         let choose_mat: f64 = rng.gen();
    //         let center = Vec3::new(
    //             f64::from(a) + 0.9_f64 * rng.gen::<f64>(),
    //             0.2,
    //             f64::from(b) + 0.9_f64 * rng.gen::<f64>(),
    //         );
    //         if (center - Vec3::new(4., 0.2, 0.)).length() > 0.9 {
    //             if choose_mat < 0.8 {
    //                 // diffuse
    //                 let albedo = Vec3::random_in_range(0., 1.);
    //                 hitable_list.push(Arc::new(Sphere {
    //                     center,
    //                     radius: 0.2,
    //                     material: Arc::new(Lambertian::new(ConstantTexture(albedo))),
    //                 }));
    //             } else if choose_mat < 0.95 {
    //                 // metal
    //                 let albedo = Vec3::random_in_range(0.5, 1.);
    //                 let fuzz: f64 = rng.gen_range(0.0..0.5);
    //                 hitable_list.push(Arc::new(Sphere {
    //                     center,
    //                     radius: 0.2,
    //                     material: Arc::new(Metal::new(ConstantTexture(albedo), fuzz)),
    //                 }))
    //             } else {
    //                 // glass
    //                 hitable_list.push(Arc::new(Sphere {
    //                     center,
    //                     radius: 0.2,
    //                     material: Arc::new(Dielectric::new(1.5)),
    //                 }))
    //             }
    //         }
    //     }
    // }
    World::new(
        vec![],
        Camera::new(
            (Vec3::new(13., 2., 3.), Vec3::new(0., 0., 0.)),
            Vec3::new(0., 1., 0.),
            20.,
            16. / 9.,
            0.0,
            10.0,
            (0.0, 1.0),
        ),
        vec![],
    )
}

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
    let light: Arc<dyn Material> =
        Arc::new(DiffuseLight::new(ConstantTexture(Vec3::new(1., 1., 1.))));
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
        Arc::new(XZRectangle {
            xz0: (113., 127.),
            xz1: (443., 432.),
            y: 554.,
            material: Arc::clone(&light),
        }),
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
            material: Arc::new(Dielectric::new(1.5)),
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
            (0.0, 1.0),
        ),
        vec![
            Arc::new(SphereLight {
                position: Vec3::new(275., 553., 275.),
                flux: Vec3::new(0.5, 0.5, 0.5),
                scale: 5000000.,
            }),
            // Arc::new(SphereLight {
            //     position: Vec3::new(450., 500., 275.),
            //     flux: Vec3::new(1., 1., 1.),
            //     scale: 500000.,
            // }),
        ],
    )
}

pub fn select_scene(index: usize) -> World {
    match index {
        0 => cornell_box_scene(),
        _ => random_scene(),
    }
}
