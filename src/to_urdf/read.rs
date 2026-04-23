//! USD prim / attribute read helpers for the back-converter.
//!
//! Every function here takes a composed [`Stage`] so opinions from every
//! sublayer (Geometry / Physics / Materials) are already merged.

use std::collections::HashMap;

use openusd::Stage;
use openusd::sdf::{self, FieldKey, ListOp, Path, Value};

use crate::{Error, Result};

pub fn children(stage: &Stage, prim: &Path) -> Result<Vec<String>> {
    stage
        .prim_children(prim.clone())
        .map_err(|e| Error::msg(format!("USD children read: {e}")))
}

pub fn child_path(parent: &Path, name: &str) -> Path {
    let full = if parent.as_str() == "/" {
        format!("/{name}")
    } else {
        format!("{}/{}", parent.as_str(), name)
    };
    sdf::path(&full).expect("sanitized prim name produces a valid path")
}

pub fn child_named(stage: &Stage, prim: &Path, name: &str) -> Result<Option<Path>> {
    for c in children(stage, prim)? {
        if c == name {
            return Ok(Some(child_path(prim, &c)));
        }
    }
    Ok(None)
}

pub fn display_name(stage: &Stage, prim: &Path) -> Option<String> {
    stage
        .field(prim.clone(), FieldKey::DisplayName)
        .ok()
        .flatten()
        .and_then(|v: Value| match v {
            Value::String(s) => Some(s),
            Value::Token(s) => Some(s),
            _ => None,
        })
}

pub fn attr_value(stage: &Stage, prim: &Path, attr: &str) -> Result<Option<Value>> {
    let attr_path = prim
        .append_property(attr)
        .map_err(|e| Error::msg(format!("property path: {e}")))?;
    stage
        .field(attr_path, FieldKey::Default)
        .map_err(|e| Error::msg(format!("USD field read: {e}")))
}

pub fn attr_f32(stage: &Stage, prim: &Path, attr: &str) -> Option<f32> {
    attr_value(stage, prim, attr).ok().flatten().and_then(|v| match v {
        Value::Float(f) => Some(f),
        Value::Double(d) => Some(d as f32),
        _ => None,
    })
}

pub fn attr_f64(stage: &Stage, prim: &Path, attr: &str) -> Option<f64> {
    attr_value(stage, prim, attr).ok().flatten().and_then(|v| match v {
        Value::Double(d) => Some(d),
        Value::Float(f) => Some(f as f64),
        _ => None,
    })
}

pub fn attr_vec3d(stage: &Stage, prim: &Path, attr: &str) -> Option<[f64; 3]> {
    attr_value(stage, prim, attr).ok().flatten().and_then(|v| match v {
        Value::Vec3d(a) => Some(a),
        Value::Vec3f(a) => Some([a[0] as f64, a[1] as f64, a[2] as f64]),
        _ => None,
    })
}

pub fn attr_quat(stage: &Stage, prim: &Path, attr: &str) -> Option<[f64; 4]> {
    attr_value(stage, prim, attr).ok().flatten().and_then(|v| match v {
        Value::Quatf(q) => Some([q[0] as f64, q[1] as f64, q[2] as f64, q[3] as f64]),
        Value::Quatd(q) => Some(q),
        _ => None,
    })
}

pub fn attr_token(stage: &Stage, prim: &Path, attr: &str) -> Option<String> {
    attr_value(stage, prim, attr).ok().flatten().and_then(|v| match v {
        Value::Token(s) => Some(s),
        Value::String(s) => Some(s),
        _ => None,
    })
}

pub fn attr_string(stage: &Stage, prim: &Path, attr: &str) -> Option<String> {
    attr_token(stage, prim, attr)
}

/// Read the *first* relationship target from a rel attribute. We only
/// ever author single-target rels (material:binding, body0/1), so this is
/// enough for the back-converter.
pub fn rel_target_first(stage: &Stage, prim: &Path, name: &str) -> Option<Path> {
    let rel_path = prim.append_property(name).ok()?;
    let v: Value = stage
        .field(rel_path, FieldKey::TargetPaths)
        .ok()
        .flatten()?;
    match v {
        Value::PathListOp(op) => take_first_listop_path(&op),
        Value::PathVec(v) => v.into_iter().next(),
        _ => None,
    }
}

fn take_first_listop_path(op: &ListOp<Path>) -> Option<Path> {
    if let Some(p) = op.explicit_items.first() {
        return Some(p.clone());
    }
    if let Some(p) = op.prepended_items.first() {
        return Some(p.clone());
    }
    if let Some(p) = op.appended_items.first() {
        return Some(p.clone());
    }
    op.added_items.first().cloned()
}

/// Applied apiSchemas list, merged from every layer.
#[allow(dead_code)]
pub fn api_schemas(stage: &Stage, prim: &Path) -> Vec<String> {
    let raw: Option<Value> = stage.field(prim.clone(), "apiSchemas").ok().flatten();
    match raw {
        Some(Value::TokenListOp(op)) => {
            let mut out: Vec<String> = Vec::new();
            for s in op
                .explicit_items
                .iter()
                .chain(op.prepended_items.iter())
                .chain(op.appended_items.iter())
                .chain(op.added_items.iter())
            {
                if !out.iter().any(|x| x == s) {
                    out.push(s.clone());
                }
            }
            out
        }
        Some(Value::TokenVec(v)) => v,
        _ => Vec::new(),
    }
}

/// Convert the `physics:axis` token (`"X"`, `"Y"`, `"Z"`) into a urdf-rs
/// axis vector. Defaults to `[1, 0, 0]` if missing.
///
/// No longer used internally — we recover axes losslessly from the
/// joint's `localRot1` quaternion. Kept for callers that want to read a
/// joint authored by another pipeline where `localRot1` is identity.
#[allow(dead_code)]
pub fn axis_token_to_vec3(token: Option<&str>) -> [f64; 3] {
    match token.unwrap_or("X") {
        "Y" => [0.0, 1.0, 0.0],
        "Z" => [0.0, 0.0, 1.0],
        _ => [1.0, 0.0, 0.0],
    }
}

/// Turn a quaternion (w,x,y,z) back into roll/pitch/yaw radians (intrinsic
/// XYZ; inverse of `math::rpy::rpy_to_quat`).
pub fn quat_to_rpy(q: [f64; 4]) -> [f64; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);

    // Roll (X)
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = sinr_cosp.atan2(cosr_cosp);

    // Pitch (Y)
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        std::f64::consts::FRAC_PI_2.copysign(sinp)
    } else {
        sinp.asin()
    };

    // Yaw (Z)
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny_cosp.atan2(cosy_cosp);

    [roll, pitch, yaw]
}

pub fn mesh_library(
    stage: &Stage,
    robot_prim: &Path,
) -> Result<HashMap<String, urdf_rs::Geometry>> {
    let mut map = HashMap::new();
    let scope = match child_named(stage, robot_prim, "MeshLibrary")? {
        Some(p) => p,
        None => return Ok(map),
    };
    for name in children(stage, &scope)? {
        let lib_path = child_path(&scope, &name);
        let urdf_filename = attr_string(stage, &lib_path, "urdf:sourceFilename");
        // Default: reconstruct a filename from the library prim name
        // (e.g. `box_stl` -> `box.stl`). This is lossy but keeps the
        // reference usable.
        let filename = urdf_filename.unwrap_or_else(|| recover_filename(&name));
        map.insert(
            lib_path.as_str().to_string(),
            urdf_rs::Geometry::Mesh {
                filename,
                scale: None,
            },
        );
    }
    Ok(map)
}

fn recover_filename(safe_name: &str) -> String {
    // Common case: the last underscore-separated segment is the extension
    // (`box_stl` -> `box.stl`). Doesn't recover subpaths — callers that
    // need the original URDF path should preserve it via the
    // `urdf:sourceFilename` attribute authored at forward time.
    if let Some(idx) = safe_name.rfind('_') {
        let (stem, ext) = safe_name.split_at(idx);
        let ext = &ext[1..];
        if matches!(ext, "stl" | "obj" | "dae") {
            return format!("{stem}.{ext}");
        }
    }
    safe_name.to_string()
}

pub fn reconstruct_material(
    stage: &Stage,
    mat_path: &Path,
    name: &str,
) -> Result<Option<urdf_rs::Material>> {
    // PreviewMaterial interface: `inputs:diffuseColor` lives on the
    // Material prim itself (not the shader). We read it back from there.
    let diffuse = attr_value(stage, mat_path, "inputs:diffuseColor")
        .ok()
        .flatten()
        .and_then(|v| match v {
            Value::Vec3f(a) => Some([a[0] as f64, a[1] as f64, a[2] as f64]),
            Value::Vec3d(a) => Some(a),
            _ => None,
        });
    let opacity = attr_f32(stage, mat_path, "inputs:opacity").unwrap_or(1.0);

    let rgba = diffuse.map(|c| [c[0], c[1], c[2], opacity as f64]);
    let color = rgba.map(|v| urdf_rs::Color {
        rgba: urdf_rs::Vec4(v),
    });
    Ok(Some(urdf_rs::Material {
        name: name.to_string(),
        color,
        texture: None,
    }))
}

pub fn reconstruct_joint(
    stage: &Stage,
    joint_path: &Path,
    name: &str,
) -> Result<Option<urdf_rs::Joint>> {
    // Prefer the URDF-original type we preserved on the forward path; it
    // disambiguates revolute/continuous + planar/spherical which share
    // UsdPhysics types. Fall back to TypeName-based inference for USD
    // that wasn't authored by us.
    let urdf_hint = attr_token(stage, joint_path, "urdf:jointType");
    let joint_type = if let Some(hint) = urdf_hint.as_deref() {
        match hint {
            "fixed" => urdf_rs::JointType::Fixed,
            "revolute" => urdf_rs::JointType::Revolute,
            "continuous" => urdf_rs::JointType::Continuous,
            "prismatic" => urdf_rs::JointType::Prismatic,
            "floating" => urdf_rs::JointType::Floating,
            "planar" => urdf_rs::JointType::Planar,
            "spherical" => urdf_rs::JointType::Spherical,
            _ => urdf_rs::JointType::Fixed,
        }
    } else {
        let ty: Option<String> = stage
            .field(joint_path.clone(), FieldKey::TypeName)
            .map_err(|e| Error::msg(format!("USD field: {e}")))?
            .and_then(|v: Value| match v {
                Value::Token(t) => Some(t),
                _ => None,
            });
        match ty.as_deref() {
            Some("PhysicsFixedJoint") => urdf_rs::JointType::Fixed,
            Some("PhysicsRevoluteJoint") => urdf_rs::JointType::Revolute,
            Some("PhysicsPrismaticJoint") => urdf_rs::JointType::Prismatic,
            Some("PhysicsJoint") => urdf_rs::JointType::Planar,
            _ => urdf_rs::JointType::Fixed,
        }
    };

    let body0 = rel_target_first(stage, joint_path, "physics:body0");
    let body1 = rel_target_first(stage, joint_path, "physics:body1");

    let Some(body1) = body1 else {
        return Ok(None);
    };

    let parent_name = body0
        .as_ref()
        .and_then(|p| leaf(p))
        .unwrap_or_else(|| "world".to_string());
    let child_name = leaf(&body1).unwrap_or_else(|| "unknown".to_string());

    let local_pos0 = attr_vec3d(stage, joint_path, "physics:localPos0").unwrap_or([0.0, 0.0, 0.0]);
    let local_rot0 = attr_quat(stage, joint_path, "physics:localRot0").unwrap_or([1.0, 0.0, 0.0, 0.0]);
    // `localRot1` is exactly the axis-correction quaternion we folded in
    // on the forward path (rotation that takes `+X` to the URDF axis).
    // Recovering the original URDF axis and joint origin requires peeling
    // that factor back off:
    //
    //   localRot0 = origin_rot · axis_rot  ⇒  origin_rot = localRot0 · axis_rot⁻¹
    //   URDF axis = axis_rot · (+X)
    //
    // Without this inversion the 90° (or whatever) alignment rotation
    // bleeds into `origin.rpy`, tilting the child link in every
    // downstream viewer — e.g. the "wheel 80° wrong" symptom.
    let local_rot1 = attr_quat(stage, joint_path, "physics:localRot1").unwrap_or([1.0, 0.0, 0.0, 0.0]);
    let axis_rot_inv = crate::math::rpy::quat_conjugate(local_rot1);
    let origin_rot = crate::math::rpy::quat_mul(local_rot0, axis_rot_inv);
    let rpy = quat_to_rpy(origin_rot);

    // Axis: rotate the canonical `+X` by the recovered axis rotation to
    // get back to whatever the URDF originally authored (canonical or
    // oblique). For fixed joints `localRot1` is identity so this yields
    // `(1,0,0)` — URDF fixed joints ignore axis anyway.
    let axis_vec = crate::math::rpy::quat_rotate_vec3(local_rot1, [1.0, 0.0, 0.0]);
    let axis = axis_vec;

    let lower = attr_f32(stage, joint_path, "physics:lowerLimit").unwrap_or(0.0) as f64;
    let upper = attr_f32(stage, joint_path, "physics:upperLimit").unwrap_or(0.0) as f64;
    let effort = attr_f32(stage, joint_path, "urdf:limit:effort").unwrap_or(0.0) as f64;
    let velocity = attr_f32(stage, joint_path, "urdf:limit:velocity").unwrap_or(0.0) as f64;

    // Revolute limits were authored in degrees; switch back to radians.
    let (lower, upper) = if matches!(joint_type, urdf_rs::JointType::Revolute) {
        (lower.to_radians(), upper.to_radians())
    } else {
        (lower, upper)
    };

    Ok(Some(urdf_rs::Joint {
        name: name.to_string(),
        joint_type,
        origin: urdf_rs::Pose {
            xyz: urdf_rs::Vec3(local_pos0),
            rpy: urdf_rs::Vec3(rpy),
        },
        parent: urdf_rs::LinkName { link: parent_name },
        child: urdf_rs::LinkName { link: child_name },
        axis: urdf_rs::Axis { xyz: urdf_rs::Vec3(axis) },
        limit: urdf_rs::JointLimit {
            lower,
            upper,
            effort,
            velocity,
        },
        dynamics: None,
        mimic: None,
        calibration: None,
        safety_controller: None,
    }))
}

pub fn reconstruct_link(
    stage: &Stage,
    name: &str,
    link_path: &Path,
    mesh_library: &HashMap<String, urdf_rs::Geometry>,
    materials_by_path: &HashMap<String, urdf_rs::Material>,
) -> Result<urdf_rs::Link> {
    let mut visuals = Vec::new();
    let mut collisions = Vec::new();

    for child_name in children(stage, link_path)? {
        let child_path = child_path(link_path, &child_name);
        let ty: Option<String> = stage
            .field(&child_path, FieldKey::TypeName)
            .ok()
            .flatten()
            .and_then(|v: Value| match v {
                Value::Token(t) => Some(t),
                _ => None,
            });
        if ty.as_deref() != Some("Xform") {
            continue;
        }
        let purpose = attr_token(stage, &child_path, "purpose");
        let is_collision = purpose.as_deref() == Some("guide")
            || child_name.starts_with("collision");
        let is_visual = !is_collision
            && (child_name.starts_with("visual") || child_name.starts_with("visual_"));

        if !(is_visual || is_collision) {
            continue;
        }

        let origin = read_pose(stage, &child_path);
        let geom_prim = child_path_from(stage, &child_path, "geom")?;
        let (geometry, material_opt) = match geom_prim {
            Some(gp) => reconstruct_geometry(stage, &gp, mesh_library, materials_by_path)?,
            None => (default_box(), None),
        };

        if is_collision {
            collisions.push(urdf_rs::Collision {
                name: Some(child_name),
                origin,
                geometry,
            });
        } else {
            visuals.push(urdf_rs::Visual {
                name: Some(child_name),
                origin,
                geometry,
                material: material_opt,
            });
        }
    }

    // Read inertial from Physics layer (overs on the link prim).
    let inertial = read_inertial(stage, link_path);

    Ok(urdf_rs::Link {
        name: name.to_string(),
        inertial,
        visual: visuals,
        collision: collisions,
    })
}

fn child_path_from(stage: &Stage, parent: &Path, name: &str) -> Result<Option<Path>> {
    for c in children(stage, parent)? {
        if c == name {
            return Ok(Some(child_path(parent, &c)));
        }
    }
    Ok(None)
}

fn leaf(path: &Path) -> Option<String> {
    path.as_str().rsplit('/').next().map(|s| s.to_string())
}

fn read_pose(stage: &Stage, prim: &Path) -> urdf_rs::Pose {
    let xyz = attr_vec3d(stage, prim, "xformOp:translate").unwrap_or([0.0, 0.0, 0.0]);
    let rpy = attr_quat(stage, prim, "xformOp:orient")
        .map(quat_to_rpy)
        .unwrap_or([0.0, 0.0, 0.0]);
    urdf_rs::Pose {
        xyz: urdf_rs::Vec3(xyz),
        rpy: urdf_rs::Vec3(rpy),
    }
}

fn default_box() -> urdf_rs::Geometry {
    urdf_rs::Geometry::Box {
        size: urdf_rs::Vec3([0.0, 0.0, 0.0]),
    }
}

fn reconstruct_geometry(
    stage: &Stage,
    geom_prim: &Path,
    mesh_library: &HashMap<String, urdf_rs::Geometry>,
    materials_by_path: &HashMap<String, urdf_rs::Material>,
) -> Result<(urdf_rs::Geometry, Option<urdf_rs::Material>)> {
    let ty: Option<String> = stage
        .field(geom_prim.clone(), FieldKey::TypeName)
        .ok()
        .flatten()
        .and_then(|v: Value| match v {
            Value::Token(t) => Some(t),
            _ => None,
        });

    let material = rel_target_first(stage, geom_prim, "material:binding")
        .and_then(|mat_path| materials_by_path.get(mat_path.as_str()).cloned());

    let geom = match ty.as_deref() {
        Some("Cube") => {
            let scale = attr_vec3d(stage, geom_prim, "xformOp:scale").unwrap_or([1.0, 1.0, 1.0]);
            let size = attr_f64(stage, geom_prim, "size").unwrap_or(1.0);
            urdf_rs::Geometry::Box {
                size: urdf_rs::Vec3([size * scale[0], size * scale[1], size * scale[2]]),
            }
        }
        Some("Sphere") => urdf_rs::Geometry::Sphere {
            radius: attr_f64(stage, geom_prim, "radius").unwrap_or(1.0),
        },
        Some("Cylinder") => urdf_rs::Geometry::Cylinder {
            radius: attr_f64(stage, geom_prim, "radius").unwrap_or(1.0),
            length: attr_f64(stage, geom_prim, "height").unwrap_or(1.0),
        },
        Some("Capsule") => urdf_rs::Geometry::Capsule {
            radius: attr_f64(stage, geom_prim, "radius").unwrap_or(1.0),
            length: attr_f64(stage, geom_prim, "height").unwrap_or(1.0),
        },
        Some("Mesh") | Some("Xform") => {
            // We author mesh use sites as `def Mesh (references = </lib>)`
            // (or `def Xform` for multi-shape libraries). The reference
            // target path tells us which library entry to look up for
            // the original URDF filename.
            let lib_target = read_references_first(stage, geom_prim);
            let mesh = lib_target
                .and_then(|p| mesh_library.get(p.as_str()).cloned())
                .unwrap_or_else(|| urdf_rs::Geometry::Mesh {
                    filename: String::new(),
                    scale: None,
                });
            match mesh {
                urdf_rs::Geometry::Mesh { filename, .. } => {
                    let scale = attr_vec3d(stage, geom_prim, "xformOp:scale")
                        .map(urdf_rs::Vec3);
                    urdf_rs::Geometry::Mesh { filename, scale }
                }
                other => other,
            }
        }
        _ => default_box(),
    };
    Ok((geom, material))
}

fn read_references_first(stage: &Stage, prim: &Path) -> Option<Path> {
    let v: Value = stage
        .field(prim.clone(), FieldKey::References)
        .ok()
        .flatten()?;
    match v {
        Value::ReferenceListOp(op) => {
            for r in op
                .explicit_items
                .iter()
                .chain(op.prepended_items.iter())
                .chain(op.appended_items.iter())
                .chain(op.added_items.iter())
            {
                if !r.prim_path.is_empty() {
                    return Some(r.prim_path.clone());
                }
            }
            None
        }
        _ => None,
    }
}

fn read_inertial(stage: &Stage, link_path: &Path) -> urdf_rs::Inertial {
    let mass = attr_f32(stage, link_path, "physics:mass").unwrap_or(0.0) as f64;
    let com = attr_vec3d(stage, link_path, "physics:centerOfMass").unwrap_or([0.0, 0.0, 0.0]);
    let diag = attr_value(stage, link_path, "physics:diagonalInertia")
        .ok()
        .flatten()
        .and_then(|v| match v {
            Value::Vec3f(a) => Some([a[0] as f64, a[1] as f64, a[2] as f64]),
            Value::Vec3d(a) => Some(a),
            _ => None,
        })
        .unwrap_or([0.0, 0.0, 0.0]);
    let rpy = attr_quat(stage, link_path, "physics:principalAxes")
        .map(quat_to_rpy)
        .unwrap_or([0.0, 0.0, 0.0]);

    // Principal frame — off-diagonal components are zero here. Users who
    // started from an arbitrary URDF inertia will get a diagonalised
    // tensor back; the orientation is captured in origin.rpy so the
    // physical moment-of-inertia stays correct.
    urdf_rs::Inertial {
        origin: urdf_rs::Pose {
            xyz: urdf_rs::Vec3(com),
            rpy: urdf_rs::Vec3(rpy),
        },
        mass: urdf_rs::Mass { value: mass },
        inertia: urdf_rs::Inertia {
            ixx: diag[0],
            iyy: diag[1],
            izz: diag[2],
            ixy: 0.0,
            ixz: 0.0,
            iyz: 0.0,
        },
    }
}
