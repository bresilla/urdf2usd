//! 3×3 inertia tensor -> principal moments + principal-axes rotation.
//!
//! Mirrors Python's `_impl/link.py` inertia handling: `np.linalg.eigh` +
//! canonicalization for degenerate eigenvalues. We use
//! `nalgebra::SymmetricEigen`, sort eigenvalues ascending for determinism,
//! force the eigenvector matrix to a proper rotation (det = +1), and when
//! eigenvalues are degenerate (two equal, or all equal) we replace the
//! arbitrary basis nalgebra chose in the degenerate subspace with a
//! canonical one derived from the nearest world-axis. That matches Python's
//! `_fix_degenerate_plane` so the output is deterministic across runs /
//! platforms even for bodies with rotational symmetry (e.g. cylinders).
//!
//! Return convention:
//! - `diag = (Ixx, Iyy, Izz)` principal moments (eigenvalues) ascending.
//! - `quat = (w, x, y, z)` rotation from body frame to principal-axes frame.

use nalgebra::{Matrix3, SymmetricEigen, Vector3};

pub struct PrincipalInertia {
    pub diagonal: [f64; 3],
    pub quat: [f64; 4],
}

/// Two eigenvalues are treated as equal when their absolute difference is
/// below this threshold. Matches Python's 1e-9 tolerance.
const DEGENERACY_EPS: f64 = 1e-9;

pub fn diagonalize(inertia: &urdf_rs::Inertia) -> PrincipalInertia {
    let m = Matrix3::new(
        inertia.ixx, inertia.ixy, inertia.ixz,
        inertia.ixy, inertia.iyy, inertia.iyz,
        inertia.ixz, inertia.iyz, inertia.izz,
    );

    let eig = SymmetricEigen::new(m);
    let mut eigvals = [eig.eigenvalues[0], eig.eigenvalues[1], eig.eigenvalues[2]];
    let mut cols = [
        eig.eigenvectors.column(0).into_owned(),
        eig.eigenvectors.column(1).into_owned(),
        eig.eigenvectors.column(2).into_owned(),
    ];

    // Sort ascending so the output ordering is deterministic and matches
    // numpy's `eigh` convention.
    let mut order = [0usize, 1, 2];
    order.sort_by(|&a, &b| eigvals[a].partial_cmp(&eigvals[b]).unwrap_or(std::cmp::Ordering::Equal));
    let sorted_vals = [eigvals[order[0]], eigvals[order[1]], eigvals[order[2]]];
    let sorted_cols = [cols[order[0]].clone(), cols[order[1]].clone(), cols[order[2]].clone()];
    eigvals = sorted_vals;
    cols = sorted_cols;

    // Canonicalize degeneracies. Without this, nalgebra's eigenvectors for
    // repeated eigenvalues are underdetermined — any orthonormal basis of
    // the degenerate subspace is a valid answer, so round-tripping or
    // comparing outputs across runs fails.
    let all_equal = (eigvals[2] - eigvals[0]).abs() < DEGENERACY_EPS;
    let pair_01_equal = (eigvals[1] - eigvals[0]).abs() < DEGENERACY_EPS;
    let pair_12_equal = (eigvals[2] - eigvals[1]).abs() < DEGENERACY_EPS;

    if all_equal {
        // Isotropic — any orthonormal frame diagonalizes the tensor. Pick
        // the identity; the body's "principal axes" are its own axes.
        cols[0] = Vector3::new(1.0, 0.0, 0.0);
        cols[1] = Vector3::new(0.0, 1.0, 0.0);
        cols[2] = Vector3::new(0.0, 0.0, 1.0);
    } else if pair_01_equal {
        // λ0 == λ1 < λ2. Keep the unique axis (col 2), rebuild cols 0 & 1.
        canonicalize_pair(&mut cols, 0, 1, 2);
    } else if pair_12_equal {
        // λ0 < λ1 == λ2. Keep the unique axis (col 0), rebuild cols 1 & 2.
        canonicalize_pair(&mut cols, 1, 2, 0);
    }

    // Force det = +1 (right-handed). Without this the resulting quaternion
    // can flip a reflection, which USD's Quatf isn't meant to represent.
    let det = cols[0].cross(&cols[1]).dot(&cols[2]);
    if det < 0.0 {
        cols[2] = -cols[2].clone();
    }

    let r = Matrix3::from_columns(&cols);
    let q = mat3_to_quat(&r);

    PrincipalInertia {
        diagonal: eigvals,
        quat: q,
    }
}

/// Rebuild the two columns at `a_idx` and `b_idx` using a deterministic
/// orthonormal basis perpendicular to `cols[unique_idx]`. We pick whichever
/// world axis is least aligned with the unique eigenvector as the seed for
/// `cols[a_idx]`, then take the cross product for `cols[b_idx]`.
fn canonicalize_pair(cols: &mut [Vector3<f64>; 3], a_idx: usize, b_idx: usize, unique_idx: usize) {
    let unique = cols[unique_idx].normalize();
    let candidates = [
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
    ];
    // Pick the world axis with the smallest |dot product| to the unique
    // eigenvector — most perpendicular, least prone to numerical trouble.
    let seed_idx = (0..3)
        .min_by(|&i, &j| {
            candidates[i]
                .dot(&unique)
                .abs()
                .partial_cmp(&candidates[j].dot(&unique).abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .unwrap_or(0);
    let seed = candidates[seed_idx];

    // Project seed onto the plane perpendicular to `unique` and normalize.
    let a = (seed - unique * seed.dot(&unique)).normalize();
    let b = unique.cross(&a);

    cols[a_idx] = a;
    cols[b_idx] = b;
    cols[unique_idx] = unique;
}

fn mat3_to_quat(m: &Matrix3<f64>) -> [f64; 4] {
    let trace = m.m11 + m.m22 + m.m33;
    if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        let w = 0.25 * s;
        let x = (m.m32 - m.m23) / s;
        let y = (m.m13 - m.m31) / s;
        let z = (m.m21 - m.m12) / s;
        [w, x, y, z]
    } else if m.m11 > m.m22 && m.m11 > m.m33 {
        let s = (1.0 + m.m11 - m.m22 - m.m33).sqrt() * 2.0;
        let w = (m.m32 - m.m23) / s;
        let x = 0.25 * s;
        let y = (m.m12 + m.m21) / s;
        let z = (m.m13 + m.m31) / s;
        [w, x, y, z]
    } else if m.m22 > m.m33 {
        let s = (1.0 + m.m22 - m.m11 - m.m33).sqrt() * 2.0;
        let w = (m.m13 - m.m31) / s;
        let x = (m.m12 + m.m21) / s;
        let y = 0.25 * s;
        let z = (m.m23 + m.m32) / s;
        [w, x, y, z]
    } else {
        let s = (1.0 + m.m33 - m.m11 - m.m22).sqrt() * 2.0;
        let w = (m.m21 - m.m12) / s;
        let x = (m.m13 + m.m31) / s;
        let y = (m.m23 + m.m32) / s;
        let z = 0.25 * s;
        [w, x, y, z]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn inertia(ixx: f64, iyy: f64, izz: f64, ixy: f64, ixz: f64, iyz: f64) -> urdf_rs::Inertia {
        urdf_rs::Inertia { ixx, iyy, izz, ixy, ixz, iyz }
    }

    #[test]
    fn diagonal_matrix_is_identity_rotation() {
        let r = diagonalize(&inertia(1.0, 2.0, 3.0, 0.0, 0.0, 0.0));
        assert!((r.diagonal[0] - 1.0).abs() < 1e-10);
        assert!((r.diagonal[1] - 2.0).abs() < 1e-10);
        assert!((r.diagonal[2] - 3.0).abs() < 1e-10);
        let n: f64 = r.quat.iter().map(|q| q * q).sum();
        assert!((n - 1.0).abs() < 1e-10);
    }

    #[test]
    fn isotropic_is_deterministic() {
        let r = diagonalize(&inertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0));
        assert_eq!(r.quat, [1.0, 0.0, 0.0, 0.0]);
        assert!((r.diagonal[0] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn degenerate_z_symmetric_is_deterministic() {
        // ixx == iyy, izz different — rotationally symmetric about Z.
        let a = diagonalize(&inertia(2.0, 2.0, 5.0, 0.0, 0.0, 0.0));
        let b = diagonalize(&inertia(2.0, 2.0, 5.0, 0.0, 0.0, 0.0));
        assert_eq!(a.quat, b.quat);
        // Principal moments unchanged.
        assert!((a.diagonal[0] - 2.0).abs() < 1e-12);
        assert!((a.diagonal[1] - 2.0).abs() < 1e-12);
        assert!((a.diagonal[2] - 5.0).abs() < 1e-12);
    }
}
