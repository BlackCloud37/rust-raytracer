use crate::material::Material;
use crate::objects::aabb::AABB;
use crate::objects::bvh::BVHNode;
use crate::objects::hit::{HitRecord, Hitable};
use crate::{Ray, Vec3};
use std::sync::Arc;

pub struct Triangle {
    pub a: usize,
    pub b: usize,
    pub c: usize,
    pub positions: Arc<Vec<Vec3>>,
    pub normals: Arc<Vec<Vec3>>,
    pub texcoords: Arc<Vec<Vec3>>,
    pub bounding_box: AABB,
    pub material: Arc<dyn Material>,
}

impl Triangle {
    pub fn new(
        a: usize,
        b: usize,
        c: usize,
        positions: Arc<Vec<Vec3>>,
        normals: Arc<Vec<Vec3>>,
        texcoords: Arc<Vec<Vec3>>,
        material: Arc<dyn Material>,
    ) -> Triangle {
        let pa = &positions[a];
        let pb = &positions[b];
        let pc = &positions[c];

        let max = Vec3::new(
            pa.x.max(pb.x).max(pc.x) + 0.1,
            pa.y.max(pb.y).max(pc.y) + 0.1,
            pa.z.max(pb.z).max(pc.z) + 0.1,
        );
        let min = Vec3::new(
            pa.x.min(pb.x).min(pc.x) - 0.1,
            pa.y.min(pb.y).min(pc.y) - 0.1,
            pa.z.min(pb.z).min(pc.z) - 0.1,
        );
        Triangle {
            a,
            b,
            c,
            positions,
            normals,
            texcoords,
            bounding_box: AABB::new(min, max),
            material,
        }
    }
}

impl Hitable for Triangle {
    fn hit(&self, ray: &Ray, t_min: f64, t_max: f64) -> Option<HitRecord> {
        // reference: https://github.com/Twinklebear/tray_rust/blob/master/src/geometry/mesh.rs
        let pa = &self.positions[self.a];
        let pb = &self.positions[self.b];
        let pc = &self.positions[self.c];
        let na = &self.normals[self.a];
        let nb = &self.normals[self.b];
        let nc = &self.normals[self.c];
        // let ta = &self.texcoords[self.a];
        // let tb = &self.texcoords[self.b];
        // let tc = &self.texcoords[self.c];

        let edge = [*pb - *pa, *pc - *pa];
        let mut s = [Vec3::zero(); 2];
        s[0] = Vec3::cross(ray.dir, edge[1]);

        let div = match s[0] * edge[0] {
            // 0.0 => degenerate triangle, can't hit
            d if d == 0.0 => return None,
            d => 1.0 / d,
        };

        let d = ray.orig - *pa;
        let mut bary = [0.0; 3];
        bary[1] = d * s[0] * div;
        // Check that the first barycentric coordinate is in the triangle bounds
        if bary[1] < 0.0 || bary[1] > 1.0 {
            return None;
        }

        s[1] = Vec3::cross(d, edge[0]);
        bary[2] = ray.dir * s[1] * div;
        // Check the second barycentric coordinate is in the triangle bounds
        if bary[2] < 0.0 || bary[1] + bary[2] > 1.0 {
            return None;
        }

        // We've hit the triangle with the ray, now check the hit location is in the ray range
        let t = edge[1] * s[1] * div;
        if t < t_min || t > t_max {
            return None;
        }

        bary[0] = 1.0 - bary[1] - bary[2];
        // let p = r.at(t);

        // Now compute normal at this location on the triangle
        let n = (bary[0] * *na + bary[1] * *nb + bary[2] * *nc).unit();

        // todo!(texcoord)
        // // Compute parameterization of surface and various derivatives for texturing
        // // Triangles are parameterized by the obj texcoords at the vertices
        // let texcoord = bary[0] * *ta + bary[1] * *tb + bary[2] * *tc;
        //
        // // Triangle points can be found by p_i = p_0 + u_i dp/du + v_i dp/dv
        // // we use this property to find the derivatives dp/du and dp/dv
        // let du = [ta.x - tc.x, tb.x - tc.x];
        // let dv = [ta.y - tc.y, tb.y - tc.y];
        // let det = du[0] * dv[1] - dv[0] * du[1];
        // //If the texcoords are degenerate pick arbitrary coordinate system
        // let (dp_du, dp_dv) =
        //     if det == 0.0 {
        //         (0.0, 0.0)
        //         // linalg::coordinate_system(&linalg::cross(&e[1], &e[0]).normalized())
        //     }
        //     else {
        //         let det = 1.0 / det;
        //         let dp = [*pa - *pc, *pb - *pc];
        //         let dp_du = (dv[1] * dp[0] - dv[0] * dp[1]) * det;
        //         let dp_dv = (-du[1] * dp[0] + du[0] * dp[1]) * det;
        //         (dp_du, dp_dv)
        //     };

        Some(HitRecord::new(
            t,
            n,
            &ray,
            self.material.clone(),
            (0.0, 0.0),
        ))
    }

    fn bounding_box(&self) -> Option<AABB> {
        Some(self.bounding_box)
    }
}

pub struct Mesh {
    bvh: BVHNode,
}

impl Mesh {
    pub fn load_obj(obj_file: String, material: Arc<dyn Material>) -> Self {
        let (models, _) = tobj::load_obj(
            &obj_file,
            &tobj::LoadOptions {
                single_index: true,
                triangulate: true,
                ..Default::default()
            },
        )
        .expect("Failed to load OBJ file.");
        let mesh = &models[0].mesh;
        let positions = Arc::new(
            mesh.positions
                .chunks(3)
                .map(|i| Vec3::new(i[0] as f64, i[1] as f64, i[2] as f64))
                .collect::<Vec<Vec3>>(),
        );
        let normals = Arc::new(
            mesh.normals
                .chunks(3)
                .map(|i| Vec3::new(i[0] as f64, i[1] as f64, i[2] as f64))
                .collect::<Vec<Vec3>>(),
        );
        let texcoords = Arc::new(
            mesh.texcoords
                .chunks(2)
                .map(|i| Vec3::new(i[0] as f64, i[1] as f64, 0.0))
                .collect::<Vec<Vec3>>(),
        );
        let triangles: Vec<Arc<dyn Hitable>> = mesh
            .indices
            .chunks(3)
            .map(|i| {
                let t: Arc<dyn Hitable> = Arc::new(Triangle::new(
                    i[0] as usize,
                    i[1] as usize,
                    i[2] as usize,
                    positions.clone(),
                    normals.clone(),
                    texcoords.clone(),
                    material.clone(),
                ));
                t
            })
            .collect();
        Self {
            bvh: BVHNode::new(triangles),
            // material,
        }
    }
}

impl Hitable for Mesh {
    fn hit(&self, r: &Ray, t_min: f64, t_max: f64) -> Option<HitRecord> {
        self.bvh.hit(r, t_min, t_max)
    }
    fn bounding_box(&self) -> Option<AABB> {
        self.bvh.bounding_box()
    }
}
