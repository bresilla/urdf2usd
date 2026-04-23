//! URDF joints -> UsdPhysics joints. Authored into the physics layer.
//!
//! All joints live under `/<robot>/Joints/<joint_name>`. Each joint's
//! parent-side frame carries the URDF origin (+ any axis correction);
//! the child side carries the axis correction only (child's own xform
//! already accounts for the joint origin — see `convert::links`). A
//! world-frame fixed joint anchors the root non-ghost link.
//!
//! All revolute/prismatic joints emit `physics:axis = "X"`. URDF lets the
//! joint axis be any 3-vector; we absorb the rotation that takes `+X` to
//! the URDF axis into both `localRot0` / `localRot1`, so downstream
//! solvers that assume `axis=X` see exactly the intended direction.

use openusd::sdf::{Path, Value};

use crate::math::rpy::{quat_from_x_to, quat_mul, rpy_to_quat};
use usd_schemas::physics::{
    JointFrame, JointLimit, MassProps, apply_articulation_root, apply_limit, apply_mass,
    apply_mimic, apply_rigid_body, define_fixed_joint, define_generic_joint,
    define_prismatic_joint, define_revolute_joint,
};
use usd_schemas::tokens::T_SCOPE;
use crate::{Error, Result};

use super::Ctx;

pub fn convert_joints(ctx: &mut Ctx<'_>) -> Result<()> {
    if let Some(first) = ctx.hierarchy.first_non_ghost_link() {
        if let Some(prim) = ctx.link_prims.get(first).cloned() {
            apply_articulation_root(&mut ctx.stage_physics, &prim)?;
        }
    }

    let robot_links: Vec<urdf_rs::Link> = ctx.robot.links.clone();
    for link in &robot_links {
        if ctx.hierarchy.should_skip_rigid_body(&link.name) {
            continue;
        }
        let Some(prim) = ctx.link_prims.get(&link.name).cloned() else {
            continue;
        };
        apply_rigid_body(&mut ctx.stage_physics, &prim)?;

        // Compose the inertial-origin rotation with the inertia-tensor's
        // principal-axes rotation. Without this, a link whose inertial frame
        // is rotated relative to the link frame misreports its principal
        // axes (Python: `link.py:128-133`).
        let inertia = crate::math::inertia::diagonalize(&link.inertial.inertia);
        let inertial_rot = rpy_to_quat(
            link.inertial.origin.rpy[0],
            link.inertial.origin.rpy[1],
            link.inertial.origin.rpy[2],
        );
        let composed = quat_mul(inertial_rot, inertia.quat);
        let props = MassProps {
            mass: link.inertial.mass.value,
            center_of_mass: link.inertial.origin.xyz.0,
            diagonal_inertia: [
                inertia.diagonal[0] as f32,
                inertia.diagonal[1] as f32,
                inertia.diagonal[2] as f32,
            ],
            principal_axes: [
                composed[0] as f32,
                composed[1] as f32,
                composed[2] as f32,
                composed[3] as f32,
            ],
        };
        apply_mass(&mut ctx.stage_physics, &prim, &props)?;
    }

    let root_anchor = ctx
        .hierarchy
        .first_non_ghost_link()
        .and_then(|n| ctx.link_prims.get(n).cloned());

    if ctx.robot.joints.is_empty() && root_anchor.is_none() {
        return Ok(());
    }

    let joints_scope = ctx
        .stage_physics
        .define_prim(&ctx.robot_prim.clone(), "Joints", T_SCOPE)?;

    if let Some(root_prim) = root_anchor {
        let frame = JointFrame {
            body0: None,
            body1: Some(root_prim),
            local_pos0: [0.0, 0.0, 0.0],
            local_rot0: [1.0, 0.0, 0.0, 0.0],
            local_pos1: [0.0, 0.0, 0.0],
            local_rot1: [1.0, 0.0, 0.0, 0.0],
        };
        let root_name = ctx.names.claim(joints_scope.as_str(), "RootJoint");
        define_fixed_joint(&mut ctx.stage_physics, &joints_scope, &root_name, &frame)?;
    }

    let joints: Vec<urdf_rs::Joint> = ctx.robot.joints.clone();

    for joint in &joints {
        if let Some(path) = convert_joint(ctx, &joints_scope, joint)? {
            author_urdf_passthrough(&mut ctx.stage_physics, &path, joint)?;
            ctx.joint_prims.insert(joint.name.clone(), path);
        }
    }

    for joint in &joints {
        let Some(mimic) = &joint.mimic else { continue };
        let Some(self_path) = ctx.joint_prims.get(&joint.name).cloned() else {
            continue;
        };
        let Some(target_path) = ctx.joint_prims.get(&mimic.joint).cloned() else {
            eprintln!(
                "warning: mimic joint `{}` references unknown joint `{}`; skipping",
                joint.name, mimic.joint
            );
            continue;
        };
        apply_mimic(
            &mut ctx.stage_physics,
            &self_path,
            &target_path,
            mimic.multiplier.unwrap_or(1.0),
            mimic.offset.unwrap_or(0.0),
        )?;
    }

    Ok(())
}

fn convert_joint(
    ctx: &mut Ctx<'_>,
    parent: &Path,
    joint: &urdf_rs::Joint,
) -> Result<Option<Path>> {
    if ctx.hierarchy.should_skip_rigid_body(&joint.parent.link)
        || ctx.hierarchy.should_skip_rigid_body(&joint.child.link)
    {
        return Ok(None);
    }
    let Some(parent_body) = ctx.link_prims.get(joint.parent.link.as_str()).cloned() else {
        return Err(Error::msg(format!(
            "joint `{}` has unknown parent link `{}`",
            joint.name, joint.parent.link
        )));
    };
    let Some(child_body) = ctx.link_prims.get(joint.child.link.as_str()).cloned() else {
        return Err(Error::msg(format!(
            "joint `{}` has unknown child link `{}`",
            joint.name, joint.child.link
        )));
    };

    let origin_q = rpy_to_quat(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2]);
    // Compute the rotation that takes +X to the URDF axis; absorb into both
    // localRots so we can emit `physics:axis = "X"` uniformly even for
    // non-canonical URDF axes.
    let axis_rot_q = quat_from_x_to(joint.axis.xyz.0);
    let local_rot0_q = quat_mul(origin_q, axis_rot_q);
    let local_rot1_q = axis_rot_q;

    let frame = JointFrame {
        body0: Some(parent_body),
        body1: Some(child_body),
        local_pos0: [
            joint.origin.xyz[0] as f32,
            joint.origin.xyz[1] as f32,
            joint.origin.xyz[2] as f32,
        ],
        local_rot0: [
            local_rot0_q[0] as f32,
            local_rot0_q[1] as f32,
            local_rot0_q[2] as f32,
            local_rot0_q[3] as f32,
        ],
        local_pos1: [0.0, 0.0, 0.0],
        local_rot1: [
            local_rot1_q[0] as f32,
            local_rot1_q[1] as f32,
            local_rot1_q[2] as f32,
            local_rot1_q[3] as f32,
        ],
    };

    let safe = ctx.names.claim(parent.as_str(), &joint.name);
    // Fixed joints don't use an axis; revolute/prismatic/planar always emit
    // "X" and rely on the axis-correction rotation embedded in localRots.
    let axis = "X";

    let path = match joint.joint_type {
        urdf_rs::JointType::Fixed => {
            // Fixed joints should ignore the URDF axis; emit the origin only.
            let fixed_frame = JointFrame {
                body0: frame.body0.clone(),
                body1: frame.body1.clone(),
                local_pos0: frame.local_pos0,
                local_rot0: [
                    origin_q[0] as f32,
                    origin_q[1] as f32,
                    origin_q[2] as f32,
                    origin_q[3] as f32,
                ],
                local_pos1: frame.local_pos1,
                local_rot1: [1.0, 0.0, 0.0, 0.0],
            };
            define_fixed_joint(&mut ctx.stage_physics, parent, &safe, &fixed_frame)?
        }
        urdf_rs::JointType::Revolute => {
            let lim = Some(JointLimit {
                lower: joint.limit.lower,
                upper: joint.limit.upper,
            });
            define_revolute_joint(&mut ctx.stage_physics, parent, &safe, &frame, axis, lim)?
        }
        urdf_rs::JointType::Continuous => {
            define_revolute_joint(&mut ctx.stage_physics, parent, &safe, &frame, axis, None)?
        }
        urdf_rs::JointType::Prismatic => {
            let lim = Some(JointLimit {
                lower: joint.limit.lower,
                upper: joint.limit.upper,
            });
            define_prismatic_joint(&mut ctx.stage_physics, parent, &safe, &frame, axis, lim)?
        }
        urdf_rs::JointType::Floating => return Ok(None),
        urdf_rs::JointType::Planar => {
            author_planar(&mut ctx.stage_physics, parent, &safe, &frame)?
        }
        urdf_rs::JointType::Spherical => {
            define_generic_joint(&mut ctx.stage_physics, parent, &safe, &frame)?
        }
    };
    Ok(Some(path))
}

/// Planar joint: after axis correction the URDF axis is the joint's +X.
/// Free DOFs are translation in the YZ plane and rotation around X. We lock
/// the remaining three DOFs with `PhysicsLimitAPI` (low > high encodes lock).
fn author_planar(
    stage: &mut usd_schemas::Stage,
    parent: &Path,
    name: &str,
    frame: &JointFrame,
) -> Result<Path> {
    let p = define_generic_joint(stage, parent, name, frame)?;
    apply_limit(stage, &p, "transX", f64::INFINITY, f64::NEG_INFINITY)?;
    apply_limit(stage, &p, "rotY", f64::INFINITY, f64::NEG_INFINITY)?;
    apply_limit(stage, &p, "rotZ", f64::INFINITY, f64::NEG_INFINITY)?;
    Ok(p)
}

/// Author URDF joint fields that UsdPhysics has no schema for as
/// `urdf:*` custom float attributes (matching Python `link.py:376-425`).
///
/// `<limit effort/velocity>` always exist (serde defaults to 0). Others are
/// only authored when the optional URDF subtree is present.
fn author_urdf_passthrough(
    stage: &mut usd_schemas::Stage,
    joint_prim: &Path,
    joint: &urdf_rs::Joint,
) -> Result<()> {
    // Preserve the original URDF joint type so `usd_to_urdf` can recover
    // revolute-vs-continuous and planar-vs-spherical — both collapse to
    // the same UsdPhysics type on the forward path.
    let kind = match joint.joint_type {
        urdf_rs::JointType::Fixed => "fixed",
        urdf_rs::JointType::Revolute => "revolute",
        urdf_rs::JointType::Continuous => "continuous",
        urdf_rs::JointType::Prismatic => "prismatic",
        urdf_rs::JointType::Floating => "floating",
        urdf_rs::JointType::Planar => "planar",
        urdf_rs::JointType::Spherical => "spherical",
    };
    stage.define_custom_attribute(
        joint_prim,
        "urdf:jointType",
        "token",
        Value::Token(kind.into()),
    )?;
    stage.define_custom_attribute(
        joint_prim,
        "urdf:limit:effort",
        "float",
        Value::Float(joint.limit.effort as f32),
    )?;
    stage.define_custom_attribute(
        joint_prim,
        "urdf:limit:velocity",
        "float",
        Value::Float(joint.limit.velocity as f32),
    )?;
    if let Some(dynamics) = &joint.dynamics {
        stage.define_custom_attribute(
            joint_prim,
            "urdf:dynamics:damping",
            "float",
            Value::Float(dynamics.damping as f32),
        )?;
        stage.define_custom_attribute(
            joint_prim,
            "urdf:dynamics:friction",
            "float",
            Value::Float(dynamics.friction as f32),
        )?;
    }
    if let Some(cal) = &joint.calibration {
        if let Some(rising) = cal.rising {
            stage.define_custom_attribute(
                joint_prim,
                "urdf:calibration:rising",
                "float",
                Value::Float(rising as f32),
            )?;
        }
        if let Some(falling) = cal.falling {
            stage.define_custom_attribute(
                joint_prim,
                "urdf:calibration:falling",
                "float",
                Value::Float(falling as f32),
            )?;
        }
    }
    if let Some(sc) = &joint.safety_controller {
        stage.define_custom_attribute(
            joint_prim,
            "urdf:safety_controller:k_velocity",
            "float",
            Value::Float(sc.k_velocity as f32),
        )?;
        stage.define_custom_attribute(
            joint_prim,
            "urdf:safety_controller:k_position",
            "float",
            Value::Float(sc.k_position as f32),
        )?;
        stage.define_custom_attribute(
            joint_prim,
            "urdf:safety_controller:soft_lower_limit",
            "float",
            Value::Float(sc.soft_lower_limit as f32),
        )?;
        stage.define_custom_attribute(
            joint_prim,
            "urdf:safety_controller:soft_upper_limit",
            "float",
            Value::Float(sc.soft_upper_limit as f32),
        )?;
    }
    Ok(())
}
