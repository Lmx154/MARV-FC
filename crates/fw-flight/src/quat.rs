//! Vector and quaternion helpers. Quaternions are Hamilton, body to NED, stored [w, x, y, z]. Square roots come from `libm`.

use core::f32::consts::PI;

/// Hamilton product a (x) b, [w, x, y, z].
pub(crate) fn mul(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
    let [aw, ax, ay, az] = a;
    let [bw, bx, by, bz] = b;
    [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]
}

/// Conjugate [w, -x, -y, -z]: the inverse rotation of a unit quaternion.
pub(crate) fn conj(q: [f32; 4]) -> [f32; 4] {
    let [w, x, y, z] = q;
    [w, -x, -y, -z]
}

/// q scaled to unit length; the identity [1, 0, 0, 0] if its norm is not above 0.
pub(crate) fn normalise(q: [f32; 4]) -> [f32; 4] {
    let n = libm::sqrtf(q.iter().map(|v| v * v).sum());
    if n > 0.0 {
        q.map(|v| v / n)
    } else {
        [1.0, 0.0, 0.0, 0.0]
    }
}

/// Cross product a x b.
pub(crate) fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    let [ax, ay, az] = a;
    let [bx, by, bz] = b;
    [ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx]
}

/// NED down seen in the body frame (FRD, unit vector) for a body-to-NED q:
/// third row of R(q).
pub(crate) fn down_in_body(q: [f32; 4]) -> [f32; 3] {
    let [w, x, y, z] = q;
    [
        2.0 * (x * z - w * y),
        2.0 * (y * z + w * x),
        w * w - x * x - y * y + z * z,
    ]
}

/// One step of a first-order low-pass (PT1): y' = y + a (x - y) with a = dt / (dt + 1 / (2 pi f_c)).
/// `dt_s` in s (0 keeps y), `cutoff_hz` in Hz.
pub(crate) fn pt1(y: f32, x: f32, dt_s: f32, cutoff_hz: f32) -> f32 {
    let rc = 1.0 / (2.0 * PI * cutoff_hz);
    y + dt_s / (dt_s + rc) * (x - y)
}
