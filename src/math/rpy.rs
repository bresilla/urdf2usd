//! URDF roll/pitch/yaw -> quaternion. Matches Python's
//! `_impl/utils.float3_to_quatf`: intrinsic X-Y-Z Tait-Bryan (r, p, y) with
//! `q = Rz(y) * Ry(p) * Rx(r)`.
//!
//! Quaternion convention throughout: `(w, x, y, z)` — USD's `Quatf` is
//! `(real, imag.x, imag.y, imag.z)`.

pub fn rpy_to_quat(roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
    let (sr, cr) = (roll * 0.5).sin_cos();
    let (sp, cp) = (pitch * 0.5).sin_cos();
    let (sy, cy) = (yaw * 0.5).sin_cos();

    let w = cr * cp * cy + sr * sp * sy;
    let x = sr * cp * cy - cr * sp * sy;
    let y = cr * sp * cy + sr * cp * sy;
    let z = cr * cp * sy - sr * sp * cy;

    [w, x, y, z]
}

/// Hamilton product `a · b`, both `(w, x, y, z)`.
pub fn quat_mul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let (aw, ax, ay, az) = (a[0], a[1], a[2], a[3]);
    let (bw, bx, by, bz) = (b[0], b[1], b[2], b[3]);
    [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]
}

/// Conjugate (i.e. inverse, for unit quaternions): `(w, -x, -y, -z)`.
pub fn quat_conjugate(q: [f64; 4]) -> [f64; 4] {
    [q[0], -q[1], -q[2], -q[3]]
}

/// Rotate a 3-vector by a unit quaternion. `v' = q · v · q⁻¹` expanded.
pub fn quat_rotate_vec3(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let t = [
        2.0 * (y * v[2] - z * v[1]),
        2.0 * (z * v[0] - x * v[2]),
        2.0 * (x * v[1] - y * v[0]),
    ];
    [
        v[0] + w * t[0] + (y * t[2] - z * t[1]),
        v[1] + w * t[1] + (z * t[0] - x * t[2]),
        v[2] + w * t[2] + (x * t[1] - y * t[0]),
    ]
}

/// Shortest rotation taking the canonical `+X` unit vector to `axis`
/// (assumed non-zero; caller should normalize).
///
/// Used to absorb a URDF joint axis that isn't a canonical direction into
/// the joint's `localRot` so we can emit `physics:axis = "X"` uniformly.
pub fn quat_from_x_to(axis: [f64; 3]) -> [f64; 4] {
    let n = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
    if n < 1e-12 {
        return [1.0, 0.0, 0.0, 0.0];
    }
    let a = [axis[0] / n, axis[1] / n, axis[2] / n];
    let dot = a[0]; // +X · a
    if dot > 1.0 - 1e-9 {
        return [1.0, 0.0, 0.0, 0.0];
    }
    if dot < -1.0 + 1e-9 {
        // 180°: rotate around any axis perpendicular to +X. Pick +Y.
        return [0.0, 0.0, 1.0, 0.0];
    }
    // Cross (+X, a) = (0, -a.z, a.y).
    let cx = 0.0;
    let cy = -a[2];
    let cz = a[1];
    let s = ((1.0 + dot) * 2.0).sqrt();
    let inv = 1.0 / s;
    [s * 0.5, cx * inv, cy * inv, cz * inv]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity() {
        let q = rpy_to_quat(0.0, 0.0, 0.0);
        assert!((q[0] - 1.0).abs() < 1e-12);
        assert!(q[1].abs() < 1e-12);
        assert!(q[2].abs() < 1e-12);
        assert!(q[3].abs() < 1e-12);
    }

    #[test]
    fn unit_norm() {
        let q = rpy_to_quat(0.3, -0.7, 1.1);
        let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        assert!((n - 1.0).abs() < 1e-12);
    }

    fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
        let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
        let t = [
            2.0 * (y * v[2] - z * v[1]),
            2.0 * (z * v[0] - x * v[2]),
            2.0 * (x * v[1] - y * v[0]),
        ];
        [
            v[0] + w * t[0] + (y * t[2] - z * t[1]),
            v[1] + w * t[1] + (z * t[0] - x * t[2]),
            v[2] + w * t[2] + (x * t[1] - y * t[0]),
        ]
    }

    #[test]
    fn x_to_axis_identity() {
        let q = quat_from_x_to([1.0, 0.0, 0.0]);
        assert_eq!(q, [1.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn x_to_axis_maps_x_to_target() {
        let targets: [[f64; 3]; 4] = [
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.7, 0.0, 0.7],
            [0.3, 0.5, -0.2],
        ];
        for target in targets {
            let n = (target[0].powi(2) + target[1].powi(2) + target[2].powi(2)).sqrt();
            let t = [target[0] / n, target[1] / n, target[2] / n];
            let q = quat_from_x_to(t);
            let rotated = rotate(q, [1.0, 0.0, 0.0]);
            assert!((rotated[0] - t[0]).abs() < 1e-9);
            assert!((rotated[1] - t[1]).abs() < 1e-9);
            assert!((rotated[2] - t[2]).abs() < 1e-9);
        }
    }

    #[test]
    fn x_to_axis_flip() {
        let q = quat_from_x_to([-1.0, 0.0, 0.0]);
        let rotated = rotate(q, [1.0, 0.0, 0.0]);
        assert!((rotated[0] + 1.0).abs() < 1e-9);
    }

    #[test]
    fn quat_mul_identity() {
        let a = [0.7071, 0.0, 0.7071, 0.0];
        let id = [1.0, 0.0, 0.0, 0.0];
        assert_eq!(quat_mul(a, id), a);
        assert_eq!(quat_mul(id, a), a);
    }
}
