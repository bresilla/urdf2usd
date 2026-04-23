//! URDF -> USD conversion orchestrator. Mirrors the Python
//! `_impl/convert.py` entry point in shape: parse -> walk -> write.

pub mod dae;
pub mod joints;
pub mod links;
pub mod materials;
pub mod meshes;
pub mod names;
pub mod scene;
pub mod textures;

use std::path::{Path, PathBuf};

use openusd::sdf;

use crate::hierarchy::LinkHierarchy;
use crate::opts::Options;
use crate::resolve::PackageResolver;
use usd_schemas::Stage;
use crate::{Error, Result};

/// Shared state threaded through the conversion. Three `Stage`s so we can
/// split the asset into Geometry / Physics / Materials sublayers; all three
/// address the same prim paths but contribute different opinions.
///
/// `stage` is the geometry (primary) layer — Xforms, meshes, primitives.
/// `stage_physics` carries PhysicsScene / joints / Rigid-body & Mass APIs as
/// overlays on the link prims. `stage_materials` carries the Materials scope
/// and adds `material:binding` as overlays on the geom-layer visuals.
pub struct Ctx<'a> {
    pub stage: Stage,
    pub stage_physics: Stage,
    pub stage_materials: Stage,
    pub robot: &'a urdf_rs::Robot,
    pub hierarchy: &'a LinkHierarchy<'a>,
    pub resolver: &'a PackageResolver,
    pub opts: &'a Options,
    pub names: names::NameCache,
    pub robot_prim: sdf::Path,
    pub link_prims: std::collections::HashMap<String, sdf::Path>,
    pub joint_prims: std::collections::HashMap<String, sdf::Path>,
    pub materials_scope: Option<sdf::Path>,
    pub material_prims: std::collections::HashMap<String, sdf::Path>,
    pub output_dir: std::path::PathBuf,
    pub textures_scope: Option<sdf::Path>,
    pub texture_files: std::collections::HashMap<std::path::PathBuf, String>,
    pub undefined: &'a crate::undefined::UndefinedMap,
    pub mesh_library_scope: Option<sdf::Path>,
    pub mesh_library_entries:
        std::collections::HashMap<std::path::PathBuf, meshes::MeshLibEntry>,
}

impl<'a> Ctx<'a> {
    /// Lazily create (or return) the `/<robot>/Materials` Scope prim in the
    /// materials layer. Shared between `convert::materials` and
    /// `convert::meshes` since either can be first to need it.
    pub fn ensure_materials_scope(&mut self) -> Result<sdf::Path> {
        if let Some(p) = self.materials_scope.clone() {
            return Ok(p);
        }
        let robot_prim = self.robot_prim.clone();
        let p = self
            .stage_materials
            .define_prim(&robot_prim, "Materials", usd_schemas::tokens::T_SCOPE)?;
        self.materials_scope = Some(p.clone());
        Ok(p)
    }
}

pub fn run<'a>(
    robot: &'a urdf_rs::Robot,
    hierarchy: &'a LinkHierarchy<'a>,
    resolver: &'a PackageResolver,
    output_dir: &Path,
    opts: &'a Options,
    undefined: &'a crate::undefined::UndefinedMap,
) -> Result<PathBuf> {
    let mut names = names::NameCache::new();
    let robot_name = names.claim_global(&robot.name);

    let mut stage = Stage::new(&robot_name)?;
    let mut stage_physics = Stage::new_sublayer();
    let stage_materials = Stage::new_sublayer();
    let robot_prim = sdf::path(&format!("/{robot_name}")).map_err(Error::Usd)?;

    // Author identifier for every layer — lets downstream tools tell where
    // the asset came from.
    let authoring = authoring_metadata();
    stage.set_layer_metadata(
        "customLayerData",
        authoring_custom_data(&authoring),
    );
    stage_physics.set_layer_metadata(
        "customLayerData",
        authoring_custom_data(&authoring),
    );
    // Physics layer needs its own kilogramsPerUnit (Python sets it on both
    // the asset stage and the physics stage — `convert.py:121, 142`).
    stage_physics.set_layer_metadata(
        "kilogramsPerUnit",
        openusd::sdf::Value::Double(1.0),
    );

    if !opts.comment.is_empty() {
        stage.set_layer_metadata(
            "comment",
            openusd::sdf::Value::String(opts.comment.clone()),
        );
    }

    let mut ctx = Ctx {
        stage,
        stage_physics,
        stage_materials,
        robot,
        hierarchy,
        resolver,
        opts,
        names,
        robot_prim: robot_prim.clone(),
        link_prims: Default::default(),
        joint_prims: Default::default(),
        materials_scope: None,
        material_prims: Default::default(),
        output_dir: output_dir.to_path_buf(),
        textures_scope: None,
        texture_files: Default::default(),
        undefined,
        mesh_library_scope: None,
        mesh_library_entries: Default::default(),
    };

    materials::preregister_global_materials(&mut ctx)?;

    for root in hierarchy.roots.clone() {
        links::convert_link_subtree(&mut ctx, root, &robot_prim)?;
    }

    joints::convert_joints(&mut ctx)?;

    if opts.scene {
        scene::define_scene(&mut ctx)?;
    }

    apply_undefined_passthrough(&mut ctx)?;

    write_output(ctx, &robot_name, output_dir, opts)
}

fn apply_undefined_passthrough(ctx: &mut Ctx<'_>) -> Result<()> {
    use openusd::sdf::Value;

    let robot_prim = ctx.robot_prim.clone();
    for entry in &ctx.undefined.robot_level {
        ctx.stage.define_custom_attribute(
            &robot_prim,
            &entry.attr_name,
            "string",
            Value::String(entry.value.clone()),
        )?;
    }

    let link_entries: Vec<(String, Vec<crate::undefined::UndefinedEntry>)> = ctx
        .undefined
        .by_link
        .iter()
        .map(|(k, v)| (k.clone(), v.clone()))
        .collect();
    for (link_name, entries) in link_entries {
        let Some(prim) = ctx.link_prims.get(&link_name).cloned() else {
            continue;
        };
        for e in entries {
            ctx.stage.define_custom_attribute(
                &prim,
                &e.attr_name,
                "string",
                Value::String(e.value),
            )?;
        }
    }

    let joint_entries: Vec<(String, Vec<crate::undefined::UndefinedEntry>)> = ctx
        .undefined
        .by_joint
        .iter()
        .map(|(k, v)| (k.clone(), v.clone()))
        .collect();
    for (joint_name, entries) in joint_entries {
        let Some(prim) = ctx.joint_prims.get(&joint_name).cloned() else {
            continue;
        };
        for e in entries {
            ctx.stage_physics.define_custom_attribute(
                &prim,
                &e.attr_name,
                "string",
                Value::String(e.value),
            )?;
        }
    }
    Ok(())
}

fn write_output(ctx: Ctx<'_>, robot_name: &str, output_dir: &Path, opts: &Options) -> Result<PathBuf> {
    if opts.layer_structure {
        write_layered(ctx, robot_name, output_dir)
    } else {
        write_flat(ctx, robot_name, output_dir)
    }
}

/// Flat output: merge physics + materials opinions into the geom stage and
/// write a single `.usda` next to the robot name. Used by `--no-layer-structure`.
fn write_flat(ctx: Ctx<'_>, robot_name: &str, output_dir: &Path) -> Result<PathBuf> {
    let Ctx {
        mut stage,
        stage_physics,
        stage_materials,
        ..
    } = ctx;
    stage.merge_from(stage_physics)?;
    stage.merge_from(stage_materials)?;
    let out_path = output_dir.join(format!("{robot_name}.usda"));
    stage.write_usda(&out_path)?;
    Ok(out_path)
}

/// Layered output: Geometry.usda + Physics.usda + Materials.usda + a top
/// `<robot>.usda` that subLayers them together.
fn write_layered(ctx: Ctx<'_>, robot_name: &str, output_dir: &Path) -> Result<PathBuf> {
    let comment = ctx.opts.comment.clone();
    let Ctx {
        stage,
        stage_physics,
        stage_materials,
        ..
    } = ctx;

    let geom_path = output_dir.join("Geometry.usda");
    let physics_path = output_dir.join("Physics.usda");
    let materials_path = output_dir.join("Materials.usda");
    stage.write_usda(&geom_path)?;
    stage_physics.write_usda(&physics_path)?;
    stage_materials.write_usda(&materials_path)?;

    let authoring = authoring_metadata();
    let top_path = output_dir.join(format!("{robot_name}.usda"));
    let comment_line = if comment.is_empty() {
        String::new()
    } else {
        format!("    comment = {}\n", quote_usda_string(&comment))
    };
    let top_content = format!(
        r#"#usda 1.0
(
{comment_line}    defaultPrim = "{robot_name}"
    upAxis = "Z"
    metersPerUnit = 1
    kilogramsPerUnit = 1
    customLayerData = {{
        string creator = "{authoring}"
    }}
    subLayers = [
        @./Geometry.usda@,
        @./Physics.usda@,
        @./Materials.usda@
    ]
)
"#
    );
    std::fs::write(&top_path, top_content)?;
    Ok(top_path)
}

fn authoring_metadata() -> String {
    format!("urdf2usd {}", env!("CARGO_PKG_VERSION"))
}

fn authoring_custom_data(authoring: &str) -> openusd::sdf::Value {
    use std::collections::HashMap;
    let mut dict: HashMap<String, openusd::sdf::Value> = HashMap::new();
    dict.insert(
        "creator".to_string(),
        openusd::sdf::Value::String(authoring.to_string()),
    );
    openusd::sdf::Value::Dictionary(dict)
}

/// Escape a user-supplied string for USDA text output (double-quoted).
fn quote_usda_string(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str(r#"\""#),
            '\\' => out.push_str(r"\\"),
            '\n' => out.push_str(r"\n"),
            other => out.push(other),
        }
    }
    out.push('"');
    out
}
