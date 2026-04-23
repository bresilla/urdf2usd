//! USD → URDF back-converter. Reads a USD asset authored by [`to_usd`]
//! (or any USD asset that conforms to the same conventions) and rebuilds
//! a [`urdf_rs::Robot`] so downstream tools that only speak URDF (ROS,
//! rapier3d-urdf, bevy_urdf) can consume the output.
//!
//! The reverse direction is *deliberately* symmetric with `to_usd`:
//! - Links come from Xform prims under the default prim (recursive).
//! - Joints come from `/<robot>/Joints/<name>` typed as
//!   `PhysicsFixedJoint` / `PhysicsRevoluteJoint` / `PhysicsPrismaticJoint`
//!   / `PhysicsJoint` — body0/body1 rels identify parent/child by prim
//!   name; localPos0 + localRot0 supply the URDF origin.
//! - Visual and collision primitives are reconstructed from the `Xform`
//!   wrapper we author around each shape — type + size/radius/height/
//!   scale attributes fold back into URDF `<geometry>`.
//! - Meshes resolve through the `MeshLibrary` scope; the original URDF
//!   filename is preserved on the library Mesh prim as
//!   `urdf:sourceFilename` so round-trips are lossless.
//! - Materials are looked up by their prim name under `/<robot>/Materials`;
//!   the diffuse color is read off the `Material.inputs:diffuseColor`
//!   interface.

mod read;

use std::collections::HashMap;
use std::path::Path as StdPath;

use openusd::Stage;
use openusd::sdf::{self, Path, Value};

use crate::{Error, Result};

pub fn run(input_file: &StdPath) -> Result<urdf_rs::Robot> {
    let path_str = input_file
        .to_str()
        .ok_or_else(|| Error::msg(format!("non-UTF-8 path: {}", input_file.display())))?;
    let stage = Stage::open(path_str).map_err(|e| Error::msg(format!("USD open: {e}")))?;

    let default = stage
        .default_prim()
        .ok_or_else(|| Error::msg("USD has no defaultPrim — unable to find robot root"))?;
    let robot_prim = sdf::path(&format!("/{default}")).map_err(|e| Error::msg(format!("{e}")))?;

    let robot_name = read::display_name(&stage, &robot_prim).unwrap_or(default);

    let mut materials_by_path: HashMap<String, urdf_rs::Material> = HashMap::new();
    read_materials(&stage, &robot_prim, &mut materials_by_path)?;

    let mut links = Vec::new();
    let joints = read_joints(&stage, &robot_prim)?;
    let link_paths = read_link_paths(&stage, &robot_prim)?;

    // MeshLibrary lookup: library prim path -> recovered URDF mesh filename.
    let mesh_library = read::mesh_library(&stage, &robot_prim)?;

    for (link_name, link_path) in &link_paths {
        let link = read::reconstruct_link(
            &stage,
            link_name,
            link_path,
            &mesh_library,
            &materials_by_path,
        )?;
        links.push(link);
    }

    let materials: Vec<urdf_rs::Material> = materials_by_path.into_values().collect();

    Ok(urdf_rs::Robot {
        name: robot_name,
        links,
        joints,
        materials,
    })
}

fn read_materials(
    stage: &Stage,
    robot_prim: &Path,
    out: &mut HashMap<String, urdf_rs::Material>,
) -> Result<()> {
    let materials_scope = match read::child_named(stage, robot_prim, "Materials")? {
        Some(p) => p,
        None => return Ok(()),
    };
    for name in read::children(stage, &materials_scope)? {
        let mat_path = read::child_path(&materials_scope, &name);
        if let Some(mat) = read::reconstruct_material(stage, &mat_path, &name)? {
            out.insert(mat_path.as_str().to_string(), mat);
        }
    }
    Ok(())
}

fn read_joints(stage: &Stage, robot_prim: &Path) -> Result<Vec<urdf_rs::Joint>> {
    let Some(joints_scope) = read::child_named(stage, robot_prim, "Joints")? else {
        return Ok(Vec::new());
    };
    let mut out = Vec::new();
    for name in read::children(stage, &joints_scope)? {
        let path = read::child_path(&joints_scope, &name);
        if name == "RootJoint" {
            // World→root anchor — re-created automatically when urdf-rs
            // loads the robot; no URDF element corresponds.
            continue;
        }
        if let Some(j) = read::reconstruct_joint(stage, &path, &name)? {
            out.push(j);
        }
    }
    Ok(out)
}

fn read_link_paths(stage: &Stage, robot_prim: &Path) -> Result<Vec<(String, Path)>> {
    let mut out = Vec::new();
    // Links are every Xform-typed descendant that isn't one of our reserved
    // scopes (Joints / Materials / MeshLibrary) or a visual/collision
    // wrapper under a link.
    collect_links_recursive(stage, robot_prim, &mut out)?;
    Ok(out)
}

fn collect_links_recursive(
    stage: &Stage,
    prim: &Path,
    out: &mut Vec<(String, Path)>,
) -> Result<()> {
    for name in read::children(stage, prim)? {
        if matches!(name.as_str(), "Joints" | "Materials" | "MeshLibrary") {
            continue;
        }
        let child = read::child_path(prim, &name);
        let ty: Option<String> = stage
            .field(&child, sdf::FieldKey::TypeName)
            .map_err(|e| Error::msg(format!("USD field read: {e}")))?
            .and_then(|v: Value| match v {
                Value::Token(t) => Some(t),
                _ => None,
            });
        // Under a link's Xform, visual_N and collision_N are also Xforms.
        // We skip them — they're handled as part of the parent link.
        if name.starts_with("visual") || name.starts_with("collision") {
            continue;
        }
        if ty.as_deref() == Some("Xform") {
            out.push((name.clone(), child.clone()));
            collect_links_recursive(stage, &child, out)?;
        }
    }
    Ok(())
}
