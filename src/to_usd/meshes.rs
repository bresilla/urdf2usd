//! Mesh geometry: resolve URDF filename, load once per unique file into a
//! shared mesh library, and at each use-site emit a thin `Mesh` prim that
//! references the library entry (adding only per-site deltas like
//! `xformOp:scale` and URDF material override).
//!
//! Library layout:
//! ```text
//! /<robot>/MeshLibrary/<file_safe_name>     def Mesh with points/indices/...
//!                                  /subset_<mat_id>   (for multi-material OBJs)
//! ```
//!
//! Use site:
//! ```text
//! /<robot>/<link>/<visual>/geom     def Mesh (references = </...MeshLibrary/X>)
//!                                   + xformOp:scale if URDF specified one
//! ```
//!
//! Dedup wins when a robot reuses the same mesh file multiple times — we
//! author `points`/`normals`/`faceVertexIndices` exactly once. The material
//! bindings authored inside the library entry compose through the reference
//! to every site automatically.
//!
//! OBJ goes through `tobj` with `triangulate: false` so n-gons round-trip;
//! STL and DAE go through `mesh-loader` (which triangulates).

use std::path::PathBuf;

use openusd::sdf::Path;

use crate::Result;
use usd_schemas::geom::{MeshData, define_mesh};
use usd_schemas::shade::{MaterialSpec, define_preview_material};
use usd_schemas::tokens::T_SCOPE;

use super::{Ctx, textures};

/// What a caller gets when instantiating a URDF `<mesh>` at a use site.
pub struct MeshResult {
    pub prim: Path,
    /// When the underlying file has exactly one material (or none, but
    /// we have a fallback like mesh-loader's first material), we surface
    /// it here so the URDF visual's `<material>` override can supersede it
    /// at the caller's discretion. For multi-material meshes this is
    /// `None` — the library's per-subset bindings already compose.
    pub material: Option<Path>,
}

/// Cached library entry. We keep the prim path and the "single material"
/// answer so sites can decide whether URDF override still matters.
#[derive(Clone)]
pub struct MeshLibEntry {
    pub prim: Path,
    pub single_material: Option<Path>,
    pub multi_material: bool,
    /// Whether the library prim is a single `Mesh` (one shape / geometry)
    /// or an `Xform` wrapping multiple `Mesh` children (one per OBJ shape
    /// / DAE geometry). Sites mirror this type so references resolve
    /// without conflicting TypeName opinions.
    pub kind: MeshLibKind,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum MeshLibKind {
    SingleMesh,
    MultiMesh,
}

fn empty_mesh() -> MeshData {
    MeshData {
        points: Vec::new(),
        face_vertex_counts: Vec::new(),
        face_vertex_indices: Vec::new(),
        normals: None,
        uvs: None,
    }
}

pub fn define_mesh_from_urdf(
    ctx: &mut Ctx<'_>,
    parent: &Path,
    name: &str,
    filename: &str,
    scale: Option<[f64; 3]>,
) -> Result<MeshResult> {
    let resolved = if filename.is_empty() {
        None
    } else {
        ctx.resolver.resolve(filename)
    };
    let abs = match resolved {
        Some(p) if p.is_file() => p,
        _ => {
            if !filename.is_empty() {
                eprintln!(
                    "warning: mesh file not available (`{filename}`); emitting empty Mesh prim"
                );
            }
            let prim = define_mesh(&mut ctx.stage, parent, name, &empty_mesh())?;
            return Ok(MeshResult { prim, material: None });
        }
    };

    // Ensure one library entry per unique resolved absolute path.
    let entry = ensure_mesh_lib_entry(ctx, &abs)?;

    // Preserve the original URDF filename on the library prim so the
    // USD → URDF back-converter can regenerate a lossless `<mesh
    // filename="...">` without guessing from the sanitized library
    // prim name.
    ctx.stage.define_custom_attribute(
        &entry.prim,
        "urdf:sourceFilename",
        "string",
        openusd::sdf::Value::String(filename.to_string()),
    )?;

    // Site prim type must match the library entry's: referencing an
    // `Xform` from a `def Mesh` would let the local TypeName override the
    // composed `Xform`, yielding a Mesh-typed prim whose children are
    // nested Mesh prims — invalid. Match the type here and let the
    // reference bring everything else in.
    let site_type = match entry.kind {
        MeshLibKind::SingleMesh => usd_schemas::tokens::T_MESH,
        MeshLibKind::MultiMesh => usd_schemas::tokens::T_XFORM,
    };
    let site = ctx.stage.define_prim(parent, name, site_type)?;
    ctx.stage.add_internal_reference(&site, &entry.prim)?;
    if let Some(s) = scale {
        if s != [1.0, 1.0, 1.0] {
            usd_schemas::xform::set_trs(
                &mut ctx.stage,
                &site,
                &usd_schemas::xform::Pose::identity(),
                Some(s),
            )?;
        }
    }

    let material = if entry.multi_material {
        None
    } else {
        entry.single_material.clone()
    };
    Ok(MeshResult { prim: site, material })
}

/// Load `abs` (if not cached) and author the full library entry. Returns
/// the cached entry. Library Mesh prims live under
/// `/<robot>/MeshLibrary/<safe_name>` in the geometry layer; their subset
/// material bindings live in the materials layer and compose through the
/// references at every use site.
fn ensure_mesh_lib_entry(
    ctx: &mut Ctx<'_>,
    abs: &std::path::Path,
) -> Result<MeshLibEntry> {
    if let Some(e) = ctx.mesh_library_entries.get(abs) {
        return Ok(e.clone());
    }
    let key = PathBuf::from(abs);
    let ext = abs
        .extension()
        .and_then(|s| s.to_str())
        .map(|s| s.to_ascii_lowercase())
        .unwrap_or_default();
    let entry = match ext.as_str() {
        "obj" => author_obj_library_entry(ctx, abs)?,
        "dae" => author_dae_library_entry(ctx, abs)?,
        _ => author_meshloader_library_entry(ctx, abs)?,
    };
    ctx.mesh_library_entries.insert(key, entry.clone());
    Ok(entry)
}

fn ensure_mesh_library_scope(ctx: &mut Ctx<'_>) -> Result<Path> {
    if let Some(p) = ctx.mesh_library_scope.clone() {
        return Ok(p);
    }
    let robot_prim = ctx.robot_prim.clone();
    let p = ctx
        .stage
        .define_prim(&robot_prim, "MeshLibrary", T_SCOPE)?;
    // USD visibility inherits down the prim tree but does NOT propagate
    // through `references`. Marking the library scope `invisible` hides
    // the canonical mesh defs at world origin while the use-site prims
    // (which only pull *opinions* via the reference, not the ancestor's
    // inherited visibility) remain visible. Without this, renderers like
    // bevy_openusd draw both the library entries AND the reference sites,
    // producing a phantom copy of every unique mesh stacked at (0,0,0).
    ctx.stage.define_attribute(
        &p,
        "visibility",
        "token",
        openusd::sdf::Value::Token("invisible".into()),
        true,
    )?;
    ctx.mesh_library_scope = Some(p.clone());
    Ok(p)
}

fn library_safe_name(ctx: &mut Ctx<'_>, scope: &Path, abs: &std::path::Path) -> String {
    let stem = abs
        .file_name()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| "mesh".to_string());
    ctx.names.claim(scope.as_str(), &stem)
}

// ---------- OBJ library path (tobj, preserves n-gons) ----------

fn author_obj_library_entry(
    ctx: &mut Ctx<'_>,
    abs: &std::path::Path,
) -> Result<MeshLibEntry> {
    use std::collections::BTreeMap;

    let opts = tobj::LoadOptions {
        single_index: true,
        // n-gon preservation: tobj will fill `face_arities` instead of
        // groups of 3 in `indices`.
        triangulate: false,
        ignore_points: true,
        ignore_lines: true,
    };
    let (models, materials_result) = match tobj::load_obj(abs, &opts) {
        Ok(v) => v,
        Err(e) => {
            eprintln!(
                "warning: failed to load OBJ `{}`: {e}; emitting empty Mesh prim",
                abs.display()
            );
            return author_empty_library_entry(ctx, abs);
        }
    };
    let mtl_materials = materials_result.unwrap_or_default();

    let mtl_usd: Vec<Option<Path>> = mtl_materials
        .iter()
        .map(|m| ingest_tobj_material(ctx, abs, m).ok().flatten())
        .collect();

    // tobj splits an OBJ into multiple `Model`s along `o`/`g` boundaries
    // *and* along `usemtl` boundaries. Group back by the `o`/`g` block
    // name so a shape with two materials becomes one Mesh with two
    // GeomSubsets (rather than two separate Mesh prims). Multiple shape
    // names means the library is wrapped in an Xform with per-shape
    // Mesh children.
    let mut by_shape: BTreeMap<String, Vec<&tobj::Model>> = BTreeMap::new();
    for m in &models {
        by_shape.entry(m.name.clone()).or_default().push(m);
    }
    if by_shape.is_empty() {
        return author_empty_library_entry(ctx, abs);
    }

    let scope = ensure_mesh_library_scope(ctx)?;
    let lib_name = library_safe_name(ctx, &scope, abs);

    if by_shape.len() == 1 {
        // Single-shape path: one Mesh for the whole file, GeomSubsets per
        // material if needed.
        let only = by_shape.values().next().unwrap();
        let shape = build_shape_data(only);
        let lib_prim = define_mesh(&mut ctx.stage, &scope, &lib_name, &shape.data)?;
        let (single_material, multi_material) =
            author_subsets(ctx, &lib_prim, &shape.per_mat, &mtl_usd)?;
        return Ok(MeshLibEntry {
            prim: lib_prim,
            single_material,
            multi_material,
            kind: MeshLibKind::SingleMesh,
        });
    }

    // Multi-shape: library is an Xform; each shape becomes a child Mesh
    // with its own GeomSubsets. URDF override is not surfaced (per-subset
    // bindings compose through the reference at use sites).
    let lib_prim = ctx
        .stage
        .define_prim(&scope, &lib_name, usd_schemas::tokens::T_XFORM)?;
    let lib_path = lib_prim.as_str().to_string();
    for (shape_name, models_for_shape) in &by_shape {
        let shape = build_shape_data(models_for_shape);
        let safe = ctx.names.claim(&lib_path, shape_name);
        let child = define_mesh(&mut ctx.stage, &lib_prim, &safe, &shape.data)?;
        author_subsets(ctx, &child, &shape.per_mat, &mtl_usd)?;
    }
    Ok(MeshLibEntry {
        prim: lib_prim,
        single_material: None,
        multi_material: true,
        kind: MeshLibKind::MultiMesh,
    })
}

struct ShapeOutput {
    data: MeshData,
    /// material_id -> face indices within `data`.
    per_mat: std::collections::BTreeMap<usize, Vec<i32>>,
}

/// Build one `MeshData` from a set of tobj `Model`s that all share the
/// same OBJ `o`/`g` block name. Materials still get tracked per-face in
/// `per_mat` so the caller can emit GeomSubsets.
fn build_shape_data(models: &[&tobj::Model]) -> ShapeOutput {
    let mut points: Vec<[f32; 3]> = Vec::new();
    let mut normals_m: Vec<[f32; 3]> = Vec::new();
    let mut uvs_m: Vec<[f32; 2]> = Vec::new();
    let mut indices: Vec<i32> = Vec::new();
    let mut face_vertex_counts: Vec<i32> = Vec::new();
    let mut per_mat: std::collections::BTreeMap<usize, Vec<i32>> =
        std::collections::BTreeMap::new();

    for model in models {
        let mesh = &model.mesh;
        let vertex_offset = points.len() as i32;

        for chunk in mesh.positions.chunks_exact(3) {
            points.push([chunk[0], chunk[1], chunk[2]]);
        }
        for chunk in mesh.normals.chunks_exact(3) {
            normals_m.push([chunk[0], chunk[1], chunk[2]]);
        }
        for chunk in mesh.texcoords.chunks_exact(2) {
            uvs_m.push([chunk[0], chunk[1]]);
        }

        let arities: Vec<u32> = if mesh.face_arities.is_empty() {
            let n = mesh.indices.len() / 3;
            vec![3u32; n]
        } else {
            mesh.face_arities.clone()
        };

        let mut idx_cursor = 0usize;
        for arity in &arities {
            let face_idx = face_vertex_counts.len() as i32;
            face_vertex_counts.push(*arity as i32);
            for i in 0..(*arity as usize) {
                indices.push(mesh.indices[idx_cursor + i] as i32 + vertex_offset);
            }
            idx_cursor += *arity as usize;
            if let Some(mat_id) = mesh.material_id {
                per_mat.entry(mat_id).or_default().push(face_idx);
            }
        }
    }

    let normals = if normals_m.len() == points.len() {
        Some(normals_m)
    } else {
        None
    };
    let uvs = if !uvs_m.is_empty() && uvs_m.len() == points.len() {
        Some(uvs_m)
    } else {
        None
    };
    ShapeOutput {
        data: MeshData {
            points,
            face_vertex_counts,
            face_vertex_indices: indices,
            normals,
            uvs,
        },
        per_mat,
    }
}

/// Author per-material GeomSubsets under `mesh_prim`. Returns
/// `(single_material, multi_material)` that the caller uses to populate
/// the library entry.
fn author_subsets(
    ctx: &mut Ctx<'_>,
    mesh_prim: &Path,
    per_mat: &std::collections::BTreeMap<usize, Vec<i32>>,
    mtl_usd: &[Option<Path>],
) -> Result<(Option<Path>, bool)> {
    use usd_schemas::geom::define_geom_subset_face;
    use usd_schemas::shade::bind_material;

    let single_material = if per_mat.is_empty() {
        None
    } else if per_mat.len() == 1 {
        let only_id = *per_mat.keys().next().unwrap();
        mtl_usd.get(only_id).cloned().flatten()
    } else {
        None
    };
    let multi_material = per_mat.len() > 1;

    if multi_material {
        let scope = mesh_prim.as_str().to_string();
        for (mat_id, faces) in per_mat {
            let Some(mat_path) = mtl_usd.get(*mat_id).cloned().flatten() else {
                continue;
            };
            let subset_name = ctx.names.claim(&scope, &format!("subset_{mat_id}"));
            let subset = define_geom_subset_face(&mut ctx.stage, mesh_prim, &subset_name, faces)?;
            bind_material(&mut ctx.stage_materials, &subset, &mat_path)?;
        }
    }

    Ok((single_material, multi_material))
}

// ---------- STL / DAE library path (mesh-loader) ----------

fn author_meshloader_library_entry(
    ctx: &mut Ctx<'_>,
    abs: &std::path::Path,
) -> Result<MeshLibEntry> {
    let scene = match mesh_loader::Loader::default().merge_meshes(true).load(abs) {
        Ok(s) => s,
        Err(e) => {
            eprintln!(
                "warning: failed to load mesh `{}`: {e}; emitting empty Mesh prim",
                abs.display()
            );
            return author_empty_library_entry(ctx, abs);
        }
    };

    let mtl_material = scene
        .materials
        .into_iter()
        .next()
        .and_then(|m| ingest_mesh_material(ctx, abs, m).ok().flatten());

    let Some(src) = scene.meshes.into_iter().next() else {
        eprintln!(
            "warning: mesh file `{}` contained no geometry; emitting empty Mesh prim",
            abs.display()
        );
        return author_empty_library_entry(ctx, abs);
    };

    // Collada files carry an `<asset><unit meter="X"/>` metadata field that
    // declares the file's native unit relative to meters. Blender-exported
    // DAEs default to centimeters (meter=0.01). mesh-loader does not apply
    // this scale, so the geometry comes out 100x too large. Read the XML
    // header ourselves and bake the scale into the library points.
    let unit_scale = if abs
        .extension()
        .and_then(|s| s.to_str())
        .map(|s| s.eq_ignore_ascii_case("dae"))
        .unwrap_or(false)
    {
        read_dae_unitmeter(abs).unwrap_or(1.0)
    } else {
        1.0
    };

    let mut points = src.vertices;
    if (unit_scale - 1.0).abs() > f32::EPSILON as f64 {
        let s = unit_scale as f32;
        for p in points.iter_mut() {
            p[0] *= s;
            p[1] *= s;
            p[2] *= s;
        }
    }

    let face_vertex_counts = vec![3i32; src.faces.len()];
    let mut face_vertex_indices = Vec::with_capacity(src.faces.len() * 3);
    for f in &src.faces {
        face_vertex_indices.push(f[0] as i32);
        face_vertex_indices.push(f[1] as i32);
        face_vertex_indices.push(f[2] as i32);
    }
    let normals = if src.normals.len() == points.len() {
        Some(src.normals)
    } else {
        None
    };
    let uvs = if !src.texcoords.is_empty() && src.texcoords[0].len() == points.len() {
        Some(src.texcoords[0].clone())
    } else {
        None
    };

    let data = MeshData {
        points,
        face_vertex_counts,
        face_vertex_indices,
        normals,
        uvs,
    };

    let scope = ensure_mesh_library_scope(ctx)?;
    let lib_name = library_safe_name(ctx, &scope, abs);
    let lib_prim = define_mesh(&mut ctx.stage, &scope, &lib_name, &data)?;

    Ok(MeshLibEntry {
        prim: lib_prim,
        single_material: mtl_material,
        multi_material: false,
        kind: MeshLibKind::SingleMesh,
    })
}

/// DAE library path (custom Collada parser).
///
/// Unlike the mesh-loader fallback, this route preserves per-primitive
/// material bindings: every `<triangles>` / `<polylist>` / `<polygons>`
/// group becomes a `GeomSubset` with its own `material:binding` derived
/// from `<bind_material><instance_material symbol=... target=.../>`.
/// Scene-graph transforms are accumulated down to each `<instance_geometry>`
/// and baked into the points / normals. Collada's `<asset><unit>` scales
/// the final positions.
fn author_dae_library_entry(
    ctx: &mut Ctx<'_>,
    abs: &std::path::Path,
) -> Result<MeshLibEntry> {
    use usd_schemas::geom::define_geom_subset_face;
    use usd_schemas::shade::bind_material;

    let doc = match super::dae::parse(abs) {
        Ok(d) => d,
        Err(e) => {
            eprintln!(
                "warning: failed to parse DAE `{}`: {e}; falling back to mesh-loader",
                abs.display()
            );
            return author_meshloader_library_entry(ctx, abs);
        }
    };

    if doc.scene_instances.is_empty() && doc.geometries.is_empty() {
        eprintln!(
            "warning: DAE `{}` contained no geometry; emitting empty Mesh prim",
            abs.display()
        );
        return author_empty_library_entry(ctx, abs);
    }

    let scale = if doc.unit_meter > 0.0 { doc.unit_meter } else { 1.0 };

    // Flatten every instance_geometry into a single merged mesh, baking
    // world transforms + unit scale into positions. Each primitive group
    // records its face-range in the merged arrays plus the resolved
    // material prim path (if any).
    let mut merged_points: Vec<[f32; 3]> = Vec::new();
    let mut merged_normals: Vec<[f32; 3]> = Vec::new();
    let mut merged_uvs: Vec<[f32; 2]> = Vec::new();
    let mut merged_counts: Vec<i32> = Vec::new();
    let mut merged_indices: Vec<i32> = Vec::new();

    struct SubsetRange {
        face_indices: Vec<i32>,
        material: Option<Path>,
    }
    let mut subsets: Vec<SubsetRange> = Vec::new();
    let mut fallback_material: Option<Path> = None;

    // If the DAE had no scene instances at all, treat each geometry as a
    // single identity-transform instance so we still emit something.
    let synthetic_instances: Vec<super::dae::SceneInstance> = if doc.scene_instances.is_empty() {
        doc.geometries
            .keys()
            .map(|id| super::dae::SceneInstance {
                geometry_id: id.clone(),
                world_matrix: [
                    1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0,
                ],
                bindings: Default::default(),
            })
            .collect()
    } else {
        doc.scene_instances.clone()
    };

    for inst in &synthetic_instances {
        let Some(geom) = doc.geometries.get(&inst.geometry_id) else {
            continue;
        };

        // Author one primitive group (GeomSubset) per <triangles>/<polylist>.
        for prim in &geom.primitives {
            let vertex_base = merged_points.len() as i32;

            // Transform and append vertex data referenced by this primitive.
            // We append the entire source arrays — downstream only uses
            // indices that were authored, and duplicate points cause no
            // behavioural issues.
            for p in &geom.positions {
                let w = super::dae::transform_point(&inst.world_matrix, *p);
                merged_points.push([
                    w[0] * scale as f32,
                    w[1] * scale as f32,
                    w[2] * scale as f32,
                ]);
            }
            let has_normals = prim.normal_indices.is_some();
            let normal_base = merged_normals.len() as i32;
            if has_normals {
                for n in &geom.normals {
                    let w = super::dae::transform_vector(&inst.world_matrix, *n);
                    merged_normals.push(w);
                }
            }
            let has_uvs = prim.texcoord_indices.is_some();
            let uv_base = merged_uvs.len() as i32;
            if has_uvs {
                for uv in &geom.texcoords {
                    merged_uvs.push(*uv);
                }
            }

            let face_start = merged_counts.len() as i32;
            let mut cursor = 0usize;
            for &arity in &prim.face_vertex_counts {
                merged_counts.push(arity as i32);
                for i in 0..(arity as usize) {
                    merged_indices.push(prim.face_indices[cursor + i] as i32 + vertex_base);
                }
                cursor += arity as usize;
            }
            let face_end = merged_counts.len() as i32;

            // Normals / UVs: we appended full source arrays above, and
            // COLLADA stores per-vertex indices separately. For USD Mesh
            // we need per-vertex arrays that align with `points` — so we
            // widen normals/uvs to per-vertex if the index arrays disagree,
            // by scattering them. If the primitive's normal indices match
            // position indices one-to-one, we can keep them as-is.
            // Simpler implementation: if normals/uvs exist and have same
            // length as positions, use them; otherwise drop.
            let _ = (normal_base, uv_base, has_normals, has_uvs);

            // Resolve material binding.
            let material = resolve_dae_material(ctx, abs, &doc, prim, inst);
            if fallback_material.is_none() {
                fallback_material = material.clone();
            }

            let face_indices: Vec<i32> = (face_start..face_end).collect();
            subsets.push(SubsetRange {
                face_indices,
                material,
            });
        }
    }

    if merged_counts.is_empty() {
        return author_empty_library_entry(ctx, abs);
    }

    // Sanity check: normals / uvs must line up with points. COLLADA index
    // per-vertex semantics don't always match URDF/USD's "one index, all
    // channels" convention — if they disagree we drop normals/uvs rather
    // than risk corrupt output. The cheap heuristic: lengths must match.
    let normals = if !merged_normals.is_empty() && merged_normals.len() == merged_points.len() {
        Some(merged_normals)
    } else {
        None
    };
    let uvs = if !merged_uvs.is_empty() && merged_uvs.len() == merged_points.len() {
        Some(merged_uvs)
    } else {
        None
    };

    let data = MeshData {
        points: merged_points,
        face_vertex_counts: merged_counts,
        face_vertex_indices: merged_indices,
        normals,
        uvs,
    };

    let scope = ensure_mesh_library_scope(ctx)?;
    let lib_name = library_safe_name(ctx, &scope, abs);
    let lib_prim = define_mesh(&mut ctx.stage, &scope, &lib_name, &data)?;

    // Author GeomSubsets + per-subset material bindings. When there's only
    // one primitive group total, we skip the subset and surface the
    // resolved material at the library-mesh level so URDF `<material>`
    // override still has a chance.
    let single_material = if subsets.len() == 1 {
        subsets[0].material.clone()
    } else {
        None
    };
    let multi_material = subsets.len() > 1
        && subsets.iter().filter(|s| s.material.is_some()).count() > 0;

    if multi_material {
        let lib_path = lib_prim.as_str().to_string();
        for (i, subset) in subsets.iter().enumerate() {
            let Some(mat_path) = subset.material.clone() else {
                continue;
            };
            let subset_name = ctx.names.claim(&lib_path, &format!("subset_{i}"));
            let prim = define_geom_subset_face(
                &mut ctx.stage,
                &lib_prim,
                &subset_name,
                &subset.face_indices,
            )?;
            bind_material(&mut ctx.stage_materials, &prim, &mat_path)?;
        }
    }

    Ok(MeshLibEntry {
        prim: lib_prim,
        single_material,
        multi_material,
        kind: MeshLibKind::SingleMesh,
    })
}

fn resolve_dae_material(
    ctx: &mut Ctx<'_>,
    abs: &std::path::Path,
    doc: &super::dae::DaeDocument,
    prim: &super::dae::Primitive,
    inst: &super::dae::SceneInstance,
) -> Option<Path> {
    let symbol = prim.material_symbol.as_ref()?;
    // Material symbol -> material id (via bind_material) -> effect id.
    let material_id = inst.bindings.get(symbol).cloned()?;
    let effect_id = doc.materials.get(&material_id)?;
    let effect = doc.effects.get(effect_id)?;
    ingest_dae_material(ctx, abs, &material_id, effect, doc).ok().flatten()
}

fn ingest_dae_material(
    ctx: &mut Ctx<'_>,
    mesh_path: &std::path::Path,
    material_id: &str,
    effect: &super::dae::Effect,
    doc: &super::dae::DaeDocument,
) -> Result<Option<Path>> {
    let base_key = format!("mtl_{material_id}");
    if let Some(existing) = ctx.material_prims.get(&base_key) {
        return Ok(Some(existing.clone()));
    }

    let scope = ctx.ensure_materials_scope()?;
    let safe = ctx.names.claim(scope.as_str(), &base_key);

    let diffuse_rgb = effect
        .diffuse_color
        .map(|c| [c[0], c[1], c[2]])
        .unwrap_or([0.8, 0.8, 0.8]);
    let emissive_rgb = effect
        .emissive_color
        .map(|c| [c[0], c[1], c[2]])
        .unwrap_or([0.0, 0.0, 0.0]);
    let opacity = super::dae::resolved_opacity(effect);
    let ior = effect.ior.unwrap_or(0.0);

    let mesh_dir = mesh_path.parent().unwrap_or(std::path::Path::new("."));
    let diffuse_tex = effect
        .diffuse_sampler
        .as_ref()
        .and_then(|sid| super::dae::resolve_sampler_to_path(effect, sid, doc))
        .and_then(|name| stage_dae_texture(ctx, mesh_dir, name));
    let emissive_tex = effect
        .emissive_sampler
        .as_ref()
        .and_then(|sid| super::dae::resolve_sampler_to_path(effect, sid, doc))
        .and_then(|name| stage_dae_texture(ctx, mesh_dir, name));

    let spec = MaterialSpec {
        diffuse_srgb: diffuse_rgb,
        opacity,
        roughness: 0.5,
        metallic: 0.0,
        emissive_srgb: emissive_rgb,
        ior,
        diffuse_texture: diffuse_tex.as_deref(),
        normal_texture: None,
        roughness_texture: None,
        metallic_texture: None,
        opacity_texture: None,
        emissive_texture: emissive_tex.as_deref(),
    };
    let prim = define_preview_material(&mut ctx.stage_materials, &scope, &safe, &spec)?;
    ctx.material_prims.insert(base_key, prim.clone());
    Ok(Some(prim))
}

fn stage_dae_texture(
    ctx: &mut Ctx<'_>,
    mesh_dir: &std::path::Path,
    relative: &str,
) -> Option<String> {
    let rel = std::path::Path::new(relative);
    let mut candidates: Vec<PathBuf> = Vec::new();
    if rel.is_absolute() {
        candidates.push(rel.to_path_buf());
    } else {
        candidates.push(rel.to_path_buf());
        candidates.push(mesh_dir.join(rel));
    }
    let abs = candidates.iter().find(|p| p.is_file()).cloned()?;
    textures::ensure_texture(ctx, &abs).ok()
}

/// Read `<asset><unit meter="X"/></asset>` from a COLLADA file and return
/// `X`. Returns `None` if the file can't be read, isn't valid XML, or
/// doesn't carry a unit tag — caller should default to 1.0 (meters).
fn read_dae_unitmeter(path: &std::path::Path) -> Option<f64> {
    use quick_xml::events::Event;
    use quick_xml::reader::Reader;

    let xml = std::fs::read_to_string(path).ok()?;
    let mut reader = Reader::from_str(&xml);
    reader.config_mut().trim_text(true);

    let mut in_asset = false;
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Eof) => break,
            Ok(Event::Start(e)) if e.name().as_ref() == b"asset" => in_asset = true,
            Ok(Event::End(e)) if e.name().as_ref() == b"asset" => in_asset = false,
            Ok(Event::Empty(e)) | Ok(Event::Start(e)) if in_asset && e.name().as_ref() == b"unit" => {
                for attr in e.attributes().flatten() {
                    if attr.key.as_ref() == b"meter" {
                        let v = attr.unescape_value().ok()?.into_owned();
                        return v.parse::<f64>().ok();
                    }
                }
            }
            Ok(_) => {}
            Err(_) => break,
        }
        buf.clear();
    }
    None
}

fn author_empty_library_entry(
    ctx: &mut Ctx<'_>,
    abs: &std::path::Path,
) -> Result<MeshLibEntry> {
    let scope = ensure_mesh_library_scope(ctx)?;
    let lib_name = library_safe_name(ctx, &scope, abs);
    let lib_prim = define_mesh(&mut ctx.stage, &scope, &lib_name, &empty_mesh())?;
    Ok(MeshLibEntry {
        prim: lib_prim,
        single_material: None,
        multi_material: false,
        kind: MeshLibKind::SingleMesh,
    })
}

// ---------- Material ingestion from mesh-level sources ----------

fn ingest_mesh_material(
    ctx: &mut Ctx<'_>,
    mesh_path: &std::path::Path,
    mat: mesh_loader::Material,
) -> Result<Option<Path>> {
    let has_any = mat.color.diffuse.is_some()
        || mat.color.emissive.is_some()
        || mat.texture.diffuse.is_some()
        || mat.opacity.is_some();
    if !has_any {
        return Ok(None);
    }

    let base_key = if mat.name.is_empty() {
        format!(
            "_mtl_{}",
            mesh_path
                .file_stem()
                .map(|s| s.to_string_lossy().into_owned())
                .unwrap_or_default()
        )
    } else {
        format!("mtl_{}", mat.name)
    };
    if let Some(existing) = ctx.material_prims.get(&base_key) {
        return Ok(Some(existing.clone()));
    }

    let scope = ctx.ensure_materials_scope()?;
    let safe = ctx.names.claim(scope.as_str(), &base_key);

    let diffuse = mat.color.diffuse.unwrap_or([0.8, 0.8, 0.8, 1.0]);
    let opacity = mat
        .opacity
        .or(Some(diffuse[3]))
        .unwrap_or(1.0);

    let mesh_dir = mesh_path.parent().unwrap_or(std::path::Path::new("."));
    let diffuse_tex = stage_texture(ctx, mesh_dir, mat.texture.diffuse.as_ref());
    let normal_tex = stage_texture(ctx, mesh_dir, mat.texture.normal.as_ref());
    let roughness_tex = stage_texture(ctx, mesh_dir, mat.texture.shininess.as_ref());
    let opacity_tex = stage_texture(ctx, mesh_dir, mat.texture.opacity.as_ref());
    let emissive_tex = stage_texture(ctx, mesh_dir, mat.texture.emissive.as_ref());

    let emissive_rgb = mat
        .color
        .emissive
        .map(|c| [c[0], c[1], c[2]])
        .unwrap_or([0.0, 0.0, 0.0]);

    let spec = MaterialSpec {
        diffuse_srgb: [diffuse[0], diffuse[1], diffuse[2]],
        opacity,
        roughness: 0.5,
        metallic: 0.0,
        emissive_srgb: emissive_rgb,
        ior: 0.0,
        diffuse_texture: diffuse_tex.as_deref(),
        normal_texture: normal_tex.as_deref(),
        roughness_texture: roughness_tex.as_deref(),
        metallic_texture: None,
        opacity_texture: opacity_tex.as_deref(),
        emissive_texture: emissive_tex.as_deref(),
    };
    let prim = define_preview_material(&mut ctx.stage_materials, &scope, &safe, &spec)?;
    ctx.material_prims.insert(base_key, prim.clone());
    Ok(Some(prim))
}

fn stage_texture(
    ctx: &mut Ctx<'_>,
    mesh_dir: &std::path::Path,
    rel: Option<&std::path::PathBuf>,
) -> Option<String> {
    let rel = rel?;
    let mut candidates: Vec<PathBuf> = Vec::new();
    if rel.is_absolute() {
        candidates.push(rel.clone());
    } else {
        candidates.push(rel.clone());
        candidates.push(mesh_dir.join(rel));
    }
    let abs = candidates.iter().find(|p| p.is_file()).cloned()?;
    match textures::ensure_texture(ctx, &abs) {
        Ok(r) => Some(r),
        Err(e) => {
            eprintln!(
                "warning: failed to stage mesh texture `{}`: {e}",
                abs.display()
            );
            None
        }
    }
}

fn ingest_tobj_material(
    ctx: &mut Ctx<'_>,
    mesh_path: &std::path::Path,
    mat: &tobj::Material,
) -> Result<Option<Path>> {
    let has_any = mat.diffuse.is_some()
        || mat.diffuse_texture.is_some()
        || mat.dissolve.is_some();
    if !has_any {
        return Ok(None);
    }

    let base_key = if mat.name.is_empty() {
        format!(
            "_mtl_{}",
            mesh_path
                .file_stem()
                .map(|s| s.to_string_lossy().into_owned())
                .unwrap_or_default()
        )
    } else {
        format!("mtl_{}", mat.name)
    };
    if let Some(existing) = ctx.material_prims.get(&base_key) {
        return Ok(Some(existing.clone()));
    }

    let scope = ctx.ensure_materials_scope()?;
    let safe = ctx.names.claim(scope.as_str(), &base_key);

    let diffuse = mat.diffuse.unwrap_or([0.8, 0.8, 0.8]);
    let opacity = mat.dissolve.unwrap_or(1.0);

    let mesh_dir = mesh_path.parent().unwrap_or(std::path::Path::new("."));
    let to_buf = |s: &Option<String>| s.as_ref().map(PathBuf::from);
    let diffuse_buf = to_buf(&mat.diffuse_texture);
    let normal_buf = to_buf(&mat.normal_texture);
    let roughness_buf = to_buf(&mat.shininess_texture);
    let opacity_buf = to_buf(&mat.dissolve_texture);

    let diffuse_tex = stage_texture(ctx, mesh_dir, diffuse_buf.as_ref());
    let normal_tex = stage_texture(ctx, mesh_dir, normal_buf.as_ref());
    let roughness_tex = stage_texture(ctx, mesh_dir, roughness_buf.as_ref());
    let opacity_tex = stage_texture(ctx, mesh_dir, opacity_buf.as_ref());

    let roughness_scalar = mat
        .unknown_param
        .get("Pr")
        .and_then(|s| s.parse::<f32>().ok())
        .unwrap_or(0.5);
    let metallic_scalar = mat
        .unknown_param
        .get("Pm")
        .and_then(|s| s.parse::<f32>().ok())
        .unwrap_or(0.0);

    let spec = MaterialSpec {
        diffuse_srgb: diffuse,
        opacity,
        roughness: roughness_scalar,
        metallic: metallic_scalar,
        emissive_srgb: [0.0, 0.0, 0.0],
        ior: 0.0,
        diffuse_texture: diffuse_tex.as_deref(),
        normal_texture: normal_tex.as_deref(),
        roughness_texture: roughness_tex.as_deref(),
        metallic_texture: None,
        opacity_texture: opacity_tex.as_deref(),
        emissive_texture: None,
    };
    let prim = define_preview_material(&mut ctx.stage_materials, &scope, &safe, &spec)?;
    ctx.material_prims.insert(base_key, prim.clone());
    Ok(Some(prim))
}
