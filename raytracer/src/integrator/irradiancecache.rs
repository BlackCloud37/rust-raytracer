// #![allow(dead_code)]
// use crate::objects::aabb::AABB;
// use crate::objects::hit::HitRecord;
// use crate::world::{adjust_flux, SPPMPixel, FRAC_1_GATHER_CNT, GATHER_CNT, PMPT};
// use crate::Vec3;
// use std::f64::consts::PI;
// use std::sync::Arc;
//
// #[derive(Default, Clone)]
// struct OctreeNode {
//     is_leaf: bool,
//     children: [Option<Arc<Self>>; 8],
//     bounding_box: AABB,
//     units: Vec<Arc<IrradianceCacheUnit>>,
// }
//
// impl OctreeNode {
//     pub fn new(bounding_box: AABB, is_leaf: bool) -> Self {
//         Self {
//             is_leaf,
//             children: Default::default(),
//             bounding_box,
//             units: Vec::new(),
//         }
//     }
//
//     pub fn build(depth: isize, bounding_box: AABB, units: Vec<Arc<IrradianceCacheUnit>>) -> Self {
//         if depth <= 0 || units.len() <= 100 {
//             let mut cur = Self::new(bounding_box, true);
//             for unit in units.iter() {
//                 if unit.bounding_box().is_intersect_box(&bounding_box) {
//                     cur.units.push(unit.clone());
//                 }
//             }
//             return cur;
//         }
//
//         let mut cur = Self::new(bounding_box, false);
//         for (index, sub_box) in bounding_box.oct_sub_box().iter().enumerate() {
//             let mut sub_units = Vec::new();
//             for unit in units.iter() {
//                 if unit.bounding_box().is_intersect_box(sub_box) {
//                     sub_units.push(unit.clone());
//                 }
//             }
//             cur.children[index] = Some(Arc::new(Self::build(depth - 1, *sub_box, sub_units)));
//         }
//         cur
//     }
//
//     pub fn query_units(&self, position: &Vec3) -> Vec<Arc<IrradianceCacheUnit>> {
//         if !self.bounding_box.is_intersect_point(position) {
//             return Vec::new();
//         }
//
//         if self.is_leaf {
//             let mut res = Vec::new();
//             for unit in self.units.iter() {
//                 if unit.bounding_box().is_intersect_point(position) {
//                     res.push(unit.clone());
//                 }
//             }
//             return res;
//         }
//
//         for child in self.children.iter().flatten() {
//             if child.bounding_box.is_intersect_point(position) {
//                 return child.query_units(position);
//             }
//         }
//         Vec::new()
//     }
// }
//
// #[derive(Copy, Clone)]
// pub struct IrradianceCacheUnit {
//     position: Vec3,
//     normal: Vec3,
//     irradiance: Vec3,
//     harmonic_distance: f64,
// }
//
// impl IrradianceCacheUnit {
//     pub fn bounding_box(&self) -> AABB {
//         let (x, y, z) = self.position.xyz();
//         AABB::new(
//             Vec3::new(
//                 x - self.harmonic_distance,
//                 y - self.harmonic_distance,
//                 z - self.harmonic_distance,
//             ),
//             Vec3::new(
//                 x + self.harmonic_distance,
//                 y + self.harmonic_distance,
//                 z + self.harmonic_distance,
//             ),
//         )
//     }
// }
//
// pub struct IrradianceCache {
//     frac_1_valid_dis: f64, // 1 / a
//     octree: OctreeNode,
//     units: Vec<Arc<IrradianceCacheUnit>>,
// }
//
// impl IrradianceCache {
//     pub fn new(valid_dis: f64) -> Self {
//         if !PMPT {
//             panic!("only pmpt uses irradiance cache")
//         }
//         Self {
//             frac_1_valid_dis: 1. / valid_dis,
//             octree: Default::default(),
//             units: Vec::new(),
//         }
//     }
//
//     pub fn estimate_irradiance(&self, rec: &HitRecord) -> Option<Vec3> {
//         let mut sum_weighted_irradiance = Vec3::zero();
//         let mut sum_weight = 0.;
//
//         let possible_units = self.octree.query_units(&rec.p);
//         for unit in possible_units.iter() {
//             let p1 = (rec.p - unit.position).length() / unit.harmonic_distance;
//             let p2 = (1. - rec.normal * unit.normal).sqrt();
//             let wi = 1. / (p1 + p2);
//             if wi > self.frac_1_valid_dis {
//                 // valid cache
//                 sum_weighted_irradiance += wi * unit.irradiance;
//                 sum_weight += wi;
//             }
//         }
//         if sum_weight > 0. {
//             return Some(sum_weighted_irradiance / sum_weight);
//         }
//         None
//     }
//
//     pub fn add_cache(&mut self, rec: &HitRecord, sppm_pixel: &SPPMPixel, global_photons: usize) {
//         let mut irradiance = Vec3::zero();
//         let mut sum_harmonic = 0.;
//         let mut count = 0;
//         for i in 0..GATHER_CNT {
//             let global_sppm = sppm_pixel.global;
//             irradiance += adjust_flux(global_sppm.flux, global_sppm.radius2, global_photons);
//             sum_harmonic += global_sppm.sum_harmonic_distance;
//             count += global_sppm.count;
//         }
//         if count == 0 || sum_harmonic == 0. {
//             return;
//         }
//         irradiance *= FRAC_1_GATHER_CNT * PI / count as f64;
//         self.units.push(Arc::new(IrradianceCacheUnit {
//             position: rec.p,
//             normal: rec.normal,
//             irradiance,
//             harmonic_distance: count as f64 / sum_harmonic,
//         }))
//     }
//
//     pub fn build_tree(&mut self) {
//         let mut bounding_box = self.units[0].bounding_box();
//         for unit in self.units.iter() {
//             bounding_box = AABB::surrounding_box(&bounding_box, &unit.bounding_box());
//         }
//         self.octree = OctreeNode::build(6, bounding_box, self.units.to_vec());
//     }
//
//     pub fn debug_has_cache_point(&self, x: &Vec3) -> bool {
//         for unit in self.units.iter() {
//             if (unit.position - *x).length() < 5. {
//                 return true;
//             }
//         }
//         false
//     }
// }

// // build irradiance cache
// let mut irradiance_cache = IrradianceCache::new(0.2);
// let mut rng = rand::thread_rng();
// for x in 0..WIDTH {
//     for y in 0..height {
//         let u = (x as f64 + rng.gen::<f64>()) / (WIDTH - 1) as f64;
//         let v = (y as f64 + rng.gen::<f64>()) / (height - 1) as f64;
//         let r = world.cam.get_ray(u, 1.0 - v); // y axis is reverted
//         let sppm_local = sppm_pixels.lock().unwrap()[x as usize][y as usize];
//         world.build_irradiance_cache(
//             &mut irradiance_cache,
//             &r,
//             max_depth,
//             &sppm_local,
//             photons,
//         );
//     }
// }
// irradiance_cache.build_tree();