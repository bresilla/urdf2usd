//! Texture copy + dedup.
//!
//! Each resolved source texture path is copied into `<output_dir>/Textures/`
//! with a unique filename (the original basename if available, with a
//! counter suffix when two sources collide). The returned string is the
//! relative USD-friendly path to embed in `UsdUVTexture.inputs:file`
//! (`@./Textures/<file>@`).

use std::path::{Path, PathBuf};

use crate::{Error, Result};

use super::Ctx;

pub fn ensure_texture(ctx: &mut Ctx<'_>, src: &Path) -> Result<String> {
    if let Some(rel) = ctx.texture_files.get(src) {
        return Ok(rel.clone());
    }

    let textures_dir = ctx.output_dir.join("Textures");
    std::fs::create_dir_all(&textures_dir)?;

    let original = src
        .file_name()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| "texture".to_string());

    let dest_name = unique_name(&textures_dir, &original);
    let dest = textures_dir.join(&dest_name);
    std::fs::copy(src, &dest).map_err(|e| {
        Error::msg(format!(
            "failed to copy texture `{}` -> `{}`: {e}",
            src.display(),
            dest.display()
        ))
    })?;

    let rel = format!("./Textures/{dest_name}");
    ctx.texture_files.insert(src.to_path_buf(), rel.clone());
    Ok(rel)
}

fn unique_name(dir: &Path, original: &str) -> String {
    if !dir.join(original).exists() {
        return original.to_string();
    }
    let pb = PathBuf::from(original);
    let stem = pb
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| "texture".to_string());
    let ext = pb
        .extension()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_default();
    let mut i = 1u32;
    loop {
        let candidate = if ext.is_empty() {
            format!("{stem}_{i}")
        } else {
            format!("{stem}_{i}.{ext}")
        };
        if !dir.join(&candidate).exists() {
            return candidate;
        }
        i += 1;
    }
}
