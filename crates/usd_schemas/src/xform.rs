//! Xform authoring helpers.
//!
//! We emit a TRS op stack: `xformOp:translate`, `xformOp:orient`,
//! (`xformOp:scale` when provided), listed on `xformOpOrder`.

use openusd::sdf::{Path, Value};

use anyhow::Result;
use crate::math::rpy_to_quat;

use super::Stage;

pub struct Pose {
    pub xyz: [f64; 3],
    pub rpy: [f64; 3],
}

impl Pose {
    pub fn identity() -> Self {
        Self {
            xyz: [0.0; 3],
            rpy: [0.0; 3],
        }
    }

    pub fn new(xyz: [f64; 3], rpy: [f64; 3]) -> Self {
        Self { xyz, rpy }
    }
}

/// Write TRS ops on `prim` from a URDF origin. Omits any op that is identity.
pub fn set_pose(stage: &mut Stage, prim: &Path, pose: &Pose) -> Result<()> {
    set_trs(stage, prim, pose, None)
}

/// Write TRS ops with an optional non-uniform scale (e.g. URDF `<box size="...">`
/// baked into the xform).
pub fn set_trs(
    stage: &mut Stage,
    prim: &Path,
    pose: &Pose,
    scale: Option<[f64; 3]>,
) -> Result<()> {
    let mut order: Vec<String> = Vec::new();

    let translate_identity = pose.xyz == [0.0, 0.0, 0.0];
    let rotate_identity = pose.rpy == [0.0, 0.0, 0.0];
    let scale_identity = scale.is_none_or(|s| s == [1.0, 1.0, 1.0]);

    if !translate_identity {
        stage.define_attribute(
            prim,
            "xformOp:translate",
            "double3",
            Value::Vec3d(pose.xyz),
            false,
        )?;
        order.push("xformOp:translate".into());
    }

    if !rotate_identity {
        let q = rpy_to_quat(pose.rpy[0], pose.rpy[1], pose.rpy[2]);
        // USD Quatf is stored as [real, imag.x, imag.y, imag.z] = [w, x, y, z]
        let q_f: [f32; 4] = [q[0] as f32, q[1] as f32, q[2] as f32, q[3] as f32];
        stage.define_attribute(prim, "xformOp:orient", "quatf", Value::Quatf(q_f), false)?;
        order.push("xformOp:orient".into());
    }

    if !scale_identity {
        let s = scale.unwrap();
        stage.define_attribute(prim, "xformOp:scale", "double3", Value::Vec3d(s), false)?;
        order.push("xformOp:scale".into());
    }

    if !order.is_empty() {
        stage.define_attribute(
            prim,
            "xformOpOrder",
            "token[]",
            Value::TokenVec(order),
            true,
        )?;
    }
    Ok(())
}

/// Define an `Xform` child of `parent` and apply a pose. Returns the prim path.
pub fn define_xform(
    stage: &mut Stage,
    parent: &Path,
    name: &str,
    pose: &Pose,
) -> Result<Path> {
    let p = stage.define_prim(parent, name, super::tokens::T_XFORM)?;
    set_pose(stage, &p, pose)?;
    Ok(p)
}

