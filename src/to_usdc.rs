//! Post-process a layered `.usda` directory by converting every
//! file to binary `.usdc` and rewriting the top layer's `subLayers`
//! asset paths so the cross-layer references still resolve.
//!
//! Usage shape:
//!   1. `urdf_to_usd` writes `<dir>/<robot>.usda` plus
//!      `Geometry.usda` / `Materials.usda` / `Physics.usda` sublayers
//!      and any `Textures/*` images.
//!   2. Call [`convert_layered_dir_to_usdc`] on `<dir>`.
//!   3. The directory now contains `.usdc` files (binary, smaller,
//!      faster to parse) instead of `.usda`. Textures stay as-is.
//!
//! `pack_dir_to_usdz` packs whatever `.usd*` files it finds, so this
//! step plays nicely with USDZ packaging on top.

use std::path::{Path, PathBuf};

use openusd::sdf::AbstractData;
use openusd::usda::TextReader;
use openusd::usdc::CrateWriter;

use crate::{Error, Result};

const SUBLAYER_BASENAMES: &[&str] = &["Geometry", "Materials", "Physics"];

/// Walk `dir`, convert every `.usda` to `.usdc`, and patch the top
/// layer's `subLayers` to reference the new `.usdc` filenames so the
/// composed stage still resolves. Mutates `dir` in place: removes
/// the original `.usda` files. Files at any extension other than
/// `.usda` (textures, sublayer references in `Textures/`, etc.)
/// are left untouched.
pub fn convert_layered_dir_to_usdc(dir: &Path) -> Result<()> {
    if !dir.is_dir() {
        return Err(Error::msg(format!(
            "USDC convert: not a directory: {}",
            dir.display()
        )));
    }

    // Collect every `*.usda` directly under `dir` first (we don't
    // recurse — sublayers live at the top level alongside the root).
    let mut usda_files: Vec<PathBuf> = Vec::new();
    for entry in std::fs::read_dir(dir)? {
        let entry = entry?;
        let path = entry.path();
        if !entry.file_type()?.is_file() {
            continue;
        }
        if path.extension().and_then(|e| e.to_str()) == Some("usda") {
            usda_files.push(path);
        }
    }
    if usda_files.is_empty() {
        return Ok(());
    }

    // Step 1: rewrite the top layer's text to point at `*.usdc`
    // sublayers BEFORE we parse it. The `TextReader` parser captures
    // asset paths verbatim into the spec map; swapping the bytes
    // up-front means the parsed `subLayers` list already carries
    // `.usdc` extensions.
    let top_path = pick_top_layer(&usda_files, dir);
    if let Some(top) = top_path.as_ref() {
        let original = std::fs::read_to_string(top)?;
        let mut patched = original.clone();
        for base in SUBLAYER_BASENAMES {
            // Match exactly the asset-path form `@./<Base>.usda@`
            // that our writer emits — narrowest substitution we can
            // do without a real .usda parser. Belt-and-braces also
            // handle `@<Base>.usda@` (no `./`).
            patched = patched.replace(
                &format!("@./{base}.usda@"),
                &format!("@./{base}.usdc@"),
            );
            patched = patched.replace(
                &format!("@{base}.usda@"),
                &format!("@{base}.usdc@"),
            );
        }
        if patched != original {
            std::fs::write(top, &patched)?;
        }
    }

    // Step 2: convert each `.usda` to `.usdc` (binary). `TextReader`
    // parses the whole layer; `CrateWriter` serialises an
    // `AbstractData` to USDC — `TextReader` implements that trait.
    for usda in &usda_files {
        let reader = TextReader::read(usda).map_err(Error::Usd)?;
        let usdc_path = usda.with_extension("usdc");
        let abs: &dyn AbstractData = &reader;
        CrateWriter::write_to_file(abs, &usdc_path).map_err(Error::Usd)?;
        std::fs::remove_file(usda)?;
    }

    Ok(())
}

/// Identify the "top" layer file — the one whose stem matches the
/// directory's basename, or, failing that, the lone `.usda` that
/// isn't a known sublayer name. Returns `None` when there's no
/// obvious top layer (e.g. flat single-file output already named
/// after the robot — every layer is "top").
fn pick_top_layer(usda_files: &[PathBuf], dir: &Path) -> Option<PathBuf> {
    let dir_stem = dir.file_name().and_then(|s| s.to_str()).map(|s| s.to_string());
    if let Some(stem) = dir_stem.as_deref() {
        for f in usda_files {
            if f.file_stem().and_then(|s| s.to_str()) == Some(stem) {
                return Some(f.clone());
            }
        }
    }
    let mut non_sublayer: Vec<&PathBuf> = usda_files
        .iter()
        .filter(|f| {
            let name = f.file_stem().and_then(|s| s.to_str()).unwrap_or("");
            !SUBLAYER_BASENAMES.contains(&name)
        })
        .collect();
    if non_sublayer.len() == 1 {
        return Some(non_sublayer.remove(0).clone());
    }
    None
}
