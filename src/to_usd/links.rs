//! Walk the URDF link tree and emit Xforms + visual + collision geometry.

use std::collections::HashSet;

use openusd::sdf::Path;

use usd_schemas::{geom, xform};
use crate::{Error, Result};

use super::{Ctx, materials, meshes};

pub fn convert_link_subtree(
    ctx: &mut Ctx<'_>,
    link_name: &str,
    parent_prim: &Path,
) -> Result<()> {
    let mut visited: HashSet<String> = HashSet::new();
    convert_link_rec(ctx, link_name, parent_prim, &xform::Pose::identity(), &mut visited)
}

fn convert_link_rec(
    ctx: &mut Ctx<'_>,
    link_name: &str,
    parent_prim: &Path,
    pose: &xform::Pose,
    visited: &mut HashSet<String>,
) -> Result<()> {
    if !visited.insert(link_name.to_string()) {
        return Err(Error::msg(format!(
            "closed-loop articulation detected at link `{link_name}`"
        )));
    }

    let link = ctx
        .hierarchy
        .link_by_name
        .get(link_name)
        .copied()
        .ok_or_else(|| Error::msg(format!("unknown link `{link_name}`")))?;

    let scope_key = parent_prim.as_str().to_string();
    let safe = ctx.names.claim(&scope_key, link_name);
    let link_prim = xform::define_xform(&mut ctx.stage, parent_prim, &safe, pose)?;
    ctx.link_prims.insert(link_name.to_string(), link_prim.clone());

    for (i, vis) in link.visual.iter().enumerate() {
        let vname = vis.name.clone().unwrap_or_else(|| format!("visual_{i}"));
        convert_visual(ctx, &link_prim, &vname, vis)?;
    }
    for (i, col) in link.collision.iter().enumerate() {
        let cname = col.name.clone().unwrap_or_else(|| format!("collision_{i}"));
        convert_collision(ctx, &link_prim, &cname, col)?;
    }

    // Recurse into children. Each child's xform carries the joint origin;
    // we clone the joint list to drop the immutable borrow on ctx.hierarchy
    // before recursing into ctx mutably.
    let child_joints: Vec<urdf_rs::Joint> = ctx
        .hierarchy
        .children_of(link_name)
        .iter()
        .map(|j| (*j).clone())
        .collect();
    for joint in child_joints {
        let child_pose = xform::Pose::new(joint.origin.xyz.0, joint.origin.rpy.0);
        convert_link_rec(
            ctx,
            joint.child.link.as_str(),
            &link_prim,
            &child_pose,
            visited,
        )?;
    }
    Ok(())
}

fn convert_visual(
    ctx: &mut Ctx<'_>,
    link_prim: &Path,
    name: &str,
    vis: &urdf_rs::Visual,
) -> Result<()> {
    let scope_key = link_prim.as_str().to_string();
    let safe = ctx.names.claim(&scope_key, name);

    let wrap = xform::define_xform(&mut ctx.stage, link_prim, &safe, &xform::Pose::new(vis.origin.xyz.0, vis.origin.rpy.0))?;

    let (geom_prim, mesh_material) = convert_geometry(ctx, &wrap, "geom", &vis.geometry)?;

    let urdf_material = if let Some(mat) = &vis.material {
        materials::ensure_material(ctx, mat)?
    } else {
        None
    };

    // URDF `<material>` overrides any material baked into the mesh file.
    let bind = urdf_material.or(mesh_material);
    if let Some(m) = bind {
        usd_schemas::shade::bind_material(&mut ctx.stage_materials, &geom_prim, &m)?;
    }

    Ok(())
}

fn convert_collision(
    ctx: &mut Ctx<'_>,
    link_prim: &Path,
    name: &str,
    col: &urdf_rs::Collision,
) -> Result<()> {
    let scope_key = link_prim.as_str().to_string();
    let safe = ctx.names.claim(&scope_key, name);

    let wrap = xform::define_xform(
        &mut ctx.stage,
        link_prim,
        &safe,
        &xform::Pose::new(col.origin.xyz.0, col.origin.rpy.0),
    )?;
    geom::set_purpose_guide(&mut ctx.stage, &wrap)?;

    let (geom_prim, _) = convert_geometry(ctx, &wrap, "geom", &col.geometry)?;

    match &col.geometry {
        urdf_rs::Geometry::Mesh { .. } => {
            usd_schemas::physics::apply_mesh_collision_convex_hull(&mut ctx.stage, &geom_prim)?;
        }
        _ => {
            usd_schemas::physics::apply_collision(&mut ctx.stage, &geom_prim)?;
        }
    }
    Ok(())
}

fn convert_geometry(
    ctx: &mut Ctx<'_>,
    parent: &Path,
    name: &str,
    g: &urdf_rs::Geometry,
) -> Result<(Path, Option<Path>)> {
    Ok(match g {
        urdf_rs::Geometry::Box { size } => {
            (geom::define_box(&mut ctx.stage, parent, name, size.0)?, None)
        }
        urdf_rs::Geometry::Sphere { radius } => {
            (geom::define_sphere(&mut ctx.stage, parent, name, *radius)?, None)
        }
        urdf_rs::Geometry::Cylinder { radius, length } => {
            (
                geom::define_cylinder(&mut ctx.stage, parent, name, *radius, *length)?,
                None,
            )
        }
        urdf_rs::Geometry::Capsule { radius, length } => {
            (
                geom::define_capsule(&mut ctx.stage, parent, name, *radius, *length)?,
                None,
            )
        }
        urdf_rs::Geometry::Mesh { filename, scale } => {
            let m = meshes::define_mesh_from_urdf(
                ctx,
                parent,
                name,
                filename,
                scale.map(|s| s.0),
            )?;
            (m.prim, m.material)
        }
    })
}
