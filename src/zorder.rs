//! Z-order curve construction.
//!
//! The [Z-order curve][wiki] is a space-filling curve useful for imposing a total order on
//! N-dimensional data while preserving the locality of the points.
//!
//! [wiki]: https://en.wikipedia.org/wiki/Z-order_curve

use glam::Vec3A;

const SHIFT32_MASK: u64 = 0x1F00000000FFFF;
const SHIFT16_MASK: u64 = 0x1F0000FF0000FF;
const SHIFT8_MASK: u64 = 0x100F00F00F00F00F;
const SHIFT4_MASK: u64 = 0x10C30C30C30C30C3;
const SHIFT2_MASK: u64 = 0x1249249249249249;

const U21_MAX: u32 = (1_u32 << 21) - 1;

#[cfg(feature = "portable_simd")]
#[inline(never)]
pub fn zorder(n: [u32; 3]) -> u64 {
    use std::simd::u64x4;

    const SHIFT32_MASK_X4: u64x4 = u64x4::from_array([SHIFT32_MASK; 4]);
    const SHIFT16_MASK_X4: u64x4 = u64x4::from_array([SHIFT16_MASK; 4]);
    const SHIFT8_MASK_X4: u64x4 = u64x4::from_array([SHIFT8_MASK; 4]);
    const SHIFT4_MASK_X4: u64x4 = u64x4::from_array([SHIFT4_MASK; 4]);
    const SHIFT2_MASK_X4: u64x4 = u64x4::from_array([SHIFT2_MASK; 4]);

    let mut v = u64x4::from_array([n[0].into(), n[1].into(), n[2].into(), 0]);

    let shl32 = v << u64x4::splat(32);
    let xor32 = v ^ shl32;
    v = xor32 & SHIFT32_MASK_X4;

    let shl16 = v << u64x4::splat(16);
    let xor16 = v ^ shl16;
    v = xor16 & SHIFT16_MASK_X4;

    let shl8 = v << u64x4::splat(8);
    let xor8 = v ^ shl8;
    v = xor8 & SHIFT8_MASK_X4;

    let shl4 = v << u64x4::splat(4);
    let xor4 = v ^ shl4;
    v = xor4 & SHIFT4_MASK_X4;

    let shl2 = v << u64x4::splat(2);
    let xor2 = v ^ shl2;
    v = xor2 & SHIFT2_MASK_X4;

    v[2] << 2 | v[1] << 1 | v[0]
}

fn dilate(n: u32) -> u64 {
    let mut n = u64::from(n & 0x1FFFFF);
    n = (n ^ (n << 32)) & SHIFT32_MASK;
    n = (n ^ (n << 16)) & SHIFT16_MASK;
    n = (n ^ (n << 8)) & SHIFT8_MASK;
    n = (n ^ (n << 4)) & SHIFT4_MASK;
    (n ^ (n << 2)) & SHIFT2_MASK
}

// Generates the z-order key for three 21-bit numbers.
#[cfg(not(feature = "portable_simd"))]
#[inline]
fn zorder([x, y, z]: [u32; 3]) -> u64 {
    dilate(z) << 2 | dilate(y) << 1 | dilate(x)
}

/// Sorts a slice of points by Z-order index.
pub fn sort_z_order(points: &mut [Vec3A]) {
    let min = points
        .iter()
        .fold(Vec3A::splat(f32::INFINITY), |acc, &v| acc.min(v))
        .min_element();
    let max = points
        .iter()
        .fold(Vec3A::splat(f32::NEG_INFINITY), |acc, &v| acc.max(v))
        .max_element();

    // Perform the scaling factor computation in double precision, as `range` can exceed `f32::MAX`.
    let range = max as f64 - min as f64;
    let scale = range.recip() as f32;
    let bias = scale * -min;

    points.sort_by_cached_key(|&v: &Vec3A| -> u64 {
        // Normalize the vector so that each element is on the range [0, 1].
        let normalized = scale * v + bias;

        // Rescale to unsigned 21-bit range.
        let rescaled_f = U21_MAX as f32 * normalized;

        // Cast to unsigned int and compute Z-order.
        zorder(rescaled_f.to_array().map(|f| f as u32))
    });
}
