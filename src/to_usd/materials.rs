//! URDF `<material>` -> USD `Material` + `UsdPreviewSurface`.
//!
//! Authored into the materials sublayer. Covers URDF `<color>` + `<texture>`.
//! Textures are copied into `<output_dir>/Textures/` with dedup + safe
//! renaming.

use openusd::sdf::Path;

use crate::Result;
use usd_schemas::shade::{MaterialSpec, define_preview_material};

use super::{Ctx, textures};

pub fn preregister_global_materials(ctx: &mut Ctx<'_>) -> Result<()> {
    if ctx.robot.materials.is_empty() {
        return Ok(());
    }
    ctx.ensure_materials_scope()?;
    let robot_materials: Vec<urdf_rs::Material> = ctx.robot.materials.clone();
    for mat in &robot_materials {
        if mat.name.is_empty() {
            continue;
        }
        ensure_material(ctx, mat)?;
    }
    Ok(())
}

pub fn ensure_material(ctx: &mut Ctx<'_>, mat: &urdf_rs::Material) -> Result<Option<Path>> {
    if mat.name.is_empty() && mat.color.is_none() && mat.texture.is_none() {
        return Ok(None);
    }

    let key = if mat.name.is_empty() {
        if let Some(c) = &mat.color {
            format!("_anon_{:08x}", color_hash(c.rgba.0))
        } else {
            "_anon".to_string()
        }
    } else {
        mat.name.clone()
    };

    if let Some(existing) = ctx.material_prims.get(&key) {
        return Ok(Some(existing.clone()));
    }

    let resolved = if mat.color.is_none() && mat.texture.is_none() && !mat.name.is_empty() {
        ctx.robot
            .materials
            .iter()
            .find(|m| m.name == mat.name)
            .cloned()
            .unwrap_or_else(|| mat.clone())
    } else {
        mat.clone()
    };

    let scope = ctx.ensure_materials_scope()?;
    let safe = ctx.names.claim(scope.as_str(), &key);

    let (rgb, a) = if let Some(c) = &resolved.color {
        let [r, g, b, a] = c.rgba.0;
        ([r as f32, g as f32, b as f32], a as f32)
    } else {
        ([0.8, 0.8, 0.8], 1.0)
    };

    let diffuse_tex_path = if let Some(tex) = &resolved.texture {
        match ctx.resolver.resolve(&tex.filename) {
            Some(p) if p.is_file() => match textures::ensure_texture(ctx, &p) {
                Ok(rel) => Some(rel),
                Err(e) => {
                    eprintln!(
                        "warning: failed to stage texture `{}`: {e}",
                        tex.filename
                    );
                    None
                }
            },
            _ => {
                eprintln!("warning: texture not available: `{}`", tex.filename);
                None
            }
        }
    } else {
        None
    };

    let spec = MaterialSpec {
        diffuse_srgb: rgb,
        opacity: a,
        diffuse_texture: diffuse_tex_path.as_deref(),
        ..MaterialSpec::default()
    };
    let prim = define_preview_material(&mut ctx.stage_materials, &scope, &safe, &spec)?;
    ctx.material_prims.insert(key, prim.clone());
    Ok(Some(prim))
}

fn color_hash(rgba: [f64; 4]) -> u32 {
    let mut h = 2166136261u32;
    for v in rgba {
        let bytes = (v as f32).to_le_bytes();
        for b in bytes {
            h ^= b as u32;
            h = h.wrapping_mul(16777619);
        }
    }
    h
}
