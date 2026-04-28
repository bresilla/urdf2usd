//! USDZ packaging — bundle a layered USD asset (top file +
//! `Geometry.usda` / `Materials.usda` / `Physics.usda` sublayers +
//! `Textures/` directory) into a single `.usdz` archive.
//!
//! USDZ is a `STORE`-only ZIP with 64-byte data alignment;
//! `openusd::usdz::ArchiveWriter` does the heavy lifting. The first
//! entry is treated as the package's root layer by every USD reader,
//! so we feed `<robot>.usda` (or `.usdc` if we ever switch the
//! writer) first, then sublayers, then textures.
//!
//! `pack_dir_to_usdz` is the entry point: point it at the directory
//! `urdf_to_usd` produced and the desired `.usdz` path. It walks the
//! directory, packs every file, and writes the archive — no
//! intermediate copies, no compression.
//!
//! After packing, the caller may delete the source directory to
//! leave only the `.usdz`. We don't do that here so the helper
//! stays pure.

use std::path::{Path, PathBuf};

use openusd::usdz::ArchiveWriter;

use crate::{Error, Result};

/// Pack every regular file under `layered_dir` into a USDZ archive
/// at `output`. The top-level layer (the `.usda` whose stem matches
/// `layered_dir`'s basename, or otherwise the lone non-sublayer
/// `.usda` in the dir) is written first so USD readers treat it as
/// the archive's root. Sublayers and textures keep their relative
/// paths inside the archive so internal `subLayers` / texture-asset
/// references resolve at read time.
pub fn pack_dir_to_usdz(layered_dir: &Path, output: &Path) -> Result<PathBuf> {
    if !layered_dir.is_dir() {
        return Err(Error::msg(format!(
            "USDZ pack: not a directory: {}",
            layered_dir.display()
        )));
    }

    let entries = collect_files(layered_dir)?;
    if entries.is_empty() {
        return Err(Error::msg(format!(
            "USDZ pack: no files under {}",
            layered_dir.display()
        )));
    }

    let top_idx = pick_top_layer(&entries, layered_dir)
        .ok_or_else(|| Error::msg("USDZ pack: no `.usda` / `.usdc` / `.usd` top layer found"))?;

    if let Some(parent) = output.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let mut archive = ArchiveWriter::create(output).map_err(Error::Usd)?;

    // Top layer first — USD readers use the archive's first entry as
    // the root.
    let (rel_top, abs_top) = &entries[top_idx];
    let bytes = std::fs::read(abs_top)?;
    archive
        .add_layer(rel_top, &bytes)
        .map_err(Error::Usd)?;

    for (i, (rel, abs)) in entries.iter().enumerate() {
        if i == top_idx {
            continue;
        }
        let bytes = std::fs::read(abs)?;
        archive.add_layer(rel, &bytes).map_err(Error::Usd)?;
    }

    archive.finish().map_err(Error::Usd)?;
    Ok(output.to_path_buf())
}

/// Walk `root` and return every regular file as `(relative-forward-
/// slash path, absolute path)`. Sorted lexicographically so archive
/// layout is deterministic across platforms.
fn collect_files(root: &Path) -> Result<Vec<(String, PathBuf)>> {
    let mut out = Vec::new();
    walk(root, root, &mut out)?;
    out.sort_by(|a, b| a.0.cmp(&b.0));
    Ok(out)
}

fn walk(root: &Path, dir: &Path, out: &mut Vec<(String, PathBuf)>) -> Result<()> {
    for entry in std::fs::read_dir(dir)? {
        let entry = entry?;
        let path = entry.path();
        let ft = entry.file_type()?;
        if ft.is_dir() {
            walk(root, &path, out)?;
        } else if ft.is_file() {
            let rel = path
                .strip_prefix(root)
                .map_err(|e| Error::msg(format!("strip_prefix: {e}")))?;
            // USDZ entry names are forward-slash relative.
            let mut rel_str = String::new();
            for (i, comp) in rel.components().enumerate() {
                if i > 0 {
                    rel_str.push('/');
                }
                let s = comp
                    .as_os_str()
                    .to_str()
                    .ok_or_else(|| Error::msg("non-UTF-8 path component"))?;
                rel_str.push_str(s);
            }
            out.push((rel_str, path));
        }
    }
    Ok(())
}

/// Choose the entry that should be the archive's first (root) layer.
/// Preference order:
///   1. A `.usd*` file whose stem matches `dir.file_name()`
///      (`pkg_dir/foo.usda` for `pkg_dir/`).
///   2. Any `.usd*` not named `Geometry`/`Materials`/`Physics`.
///   3. The first `.usd*` in alphabetical order.
fn pick_top_layer(entries: &[(String, PathBuf)], dir: &Path) -> Option<usize> {
    let dir_stem = dir.file_name().and_then(|s| s.to_str()).map(|s| s.to_string());
    let is_usd = |name: &str| {
        name.ends_with(".usda") || name.ends_with(".usdc") || name.ends_with(".usd")
    };
    let stem_match = |name: &str| -> bool {
        let Some(stem) = dir_stem.as_deref() else {
            return false;
        };
        // Match top-level only: archive entries have forward slashes,
        // so "foo.usda" but not "subdir/foo.usda".
        if name.contains('/') {
            return false;
        }
        let candidate = name.rsplit('.').nth(1).unwrap_or("");
        candidate == stem
    };
    let is_sublayer_name = |name: &str| {
        let leaf = name.rsplit('/').next().unwrap_or(name);
        // Extension-agnostic match: `Geometry.usda` and
        // `Geometry.usdc` are both sublayers (the loose-layered
        // output and the post-`--usdc`-converted output respectively).
        let stem = leaf.rsplit_once('.').map(|(s, _)| s).unwrap_or(leaf);
        matches!(stem, "Geometry" | "Materials" | "Physics")
    };

    // 1.
    for (i, (rel, _)) in entries.iter().enumerate() {
        if is_usd(rel) && stem_match(rel) {
            return Some(i);
        }
    }
    // 2.
    for (i, (rel, _)) in entries.iter().enumerate() {
        if is_usd(rel) && !is_sublayer_name(rel) && !rel.contains('/') {
            return Some(i);
        }
    }
    // 3.
    for (i, (rel, _)) in entries.iter().enumerate() {
        if is_usd(rel) {
            return Some(i);
        }
    }
    None
}
