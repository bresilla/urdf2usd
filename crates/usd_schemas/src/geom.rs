//! UsdGeom authoring: Cube, Sphere, Cylinder, Capsule, Mesh.

use openusd::sdf::{Path, Value};

use anyhow::Result;

use super::Stage;
use super::tokens::*;

/// UsdGeom.Cube with explicit size=1 + an xformOp:scale to hit the URDF box
/// size. Parent is expected to already carry the URDF visual/collision pose.
pub fn define_box(
    stage: &mut Stage,
    parent: &Path,
    name: &str,
    size: [f64; 3],
) -> Result<Path> {
    let p = stage.define_prim(parent, name, T_CUBE)?;
    stage.define_attribute(&p, "size", "double", Value::Double(1.0), false)?;
    super::xform::set_trs(
        stage,
        &p,
        &super::xform::Pose::identity(),
        Some(size),
    )?;
    Ok(p)
}

pub fn define_sphere(
    stage: &mut Stage,
    parent: &Path,
    name: &str,
    radius: f64,
) -> Result<Path> {
    let p = stage.define_prim(parent, name, T_SPHERE)?;
    stage.define_attribute(&p, "radius", "double", Value::Double(radius), false)?;
    Ok(p)
}

pub fn define_cylinder(
    stage: &mut Stage,
    parent: &Path,
    name: &str,
    radius: f64,
    length: f64,
) -> Result<Path> {
    let p = stage.define_prim(parent, name, T_CYLINDER)?;
    stage.define_attribute(&p, "radius", "double", Value::Double(radius), false)?;
    stage.define_attribute(&p, "height", "double", Value::Double(length), false)?;
    stage.define_attribute(&p, "axis", "token", Value::Token("Z".into()), true)?;
    Ok(p)
}

pub fn define_capsule(
    stage: &mut Stage,
    parent: &Path,
    name: &str,
    radius: f64,
    length: f64,
) -> Result<Path> {
    let p = stage.define_prim(parent, name, T_CAPSULE)?;
    stage.define_attribute(&p, "radius", "double", Value::Double(radius), false)?;
    stage.define_attribute(&p, "height", "double", Value::Double(length), false)?;
    stage.define_attribute(&p, "axis", "token", Value::Token("Z".into()), true)?;
    Ok(p)
}

pub struct MeshData {
    pub points: Vec<[f32; 3]>,
    pub face_vertex_counts: Vec<i32>,
    pub face_vertex_indices: Vec<i32>,
    pub normals: Option<Vec<[f32; 3]>>,
    /// Per-vertex texture coords (optional). If present its length must equal
    /// `points.len()`.
    pub uvs: Option<Vec<[f32; 2]>>,
}

pub fn define_mesh(
    stage: &mut Stage,
    parent: &Path,
    name: &str,
    mesh: &MeshData,
) -> Result<Path> {
    let p = stage.define_prim(parent, name, T_MESH)?;

    stage.define_attribute(
        &p,
        "points",
        "point3f[]",
        Value::Vec3fVec(mesh.points.clone()),
        false,
    )?;
    stage.define_attribute(
        &p,
        "faceVertexCounts",
        "int[]",
        Value::IntVec(mesh.face_vertex_counts.clone()),
        false,
    )?;
    stage.define_attribute(
        &p,
        "faceVertexIndices",
        "int[]",
        Value::IntVec(mesh.face_vertex_indices.clone()),
        false,
    )?;
    if let Some(normals) = &mesh.normals {
        stage.define_attribute(
            &p,
            "normals",
            "normal3f[]",
            Value::Vec3fVec(normals.clone()),
            false,
        )?;
    }
    if let Some(uvs) = &mesh.uvs {
        stage.define_attribute(
            &p,
            "primvars:st",
            "texCoord2f[]",
            Value::Vec2fVec(uvs.clone()),
            false,
        )?;
    }
    Ok(p)
}

/// Set `purpose = "guide"` on a prim (used to tag collision geometry so it
/// doesn't render in the default view).
pub fn set_purpose_guide(stage: &mut Stage, prim: &Path) -> Result<()> {
    stage.define_attribute(prim, "purpose", "token", Value::Token("guide".into()), true)
}

/// Author a `GeomSubset` child of `parent_mesh` that selects a set of face
/// indices for per-subset material binding (family `"materialBind"`).
pub fn define_geom_subset_face(
    stage: &mut Stage,
    parent_mesh: &Path,
    name: &str,
    face_indices: &[i32],
) -> Result<Path> {
    let p = stage.define_prim(parent_mesh, name, T_GEOM_SUBSET)?;
    stage.define_attribute(
        &p,
        "familyName",
        "token",
        Value::Token("materialBind".into()),
        true,
    )?;
    stage.define_attribute(
        &p,
        "elementType",
        "token",
        Value::Token("face".into()),
        true,
    )?;
    stage.define_attribute(
        &p,
        "indices",
        "int[]",
        Value::IntVec(face_indices.to_vec()),
        false,
    )?;
    Ok(p)
}
