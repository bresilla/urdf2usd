//! COLLADA (.dae) parser for URDF mesh conversion.
//!
//! Scope (enough to cover URDF use cases, not full pycollada parity):
//! - `<asset><unit meter="X"/></asset>` + `<up_axis>` → stage-level scale
//! - `<library_images>` → image id → filename
//! - `<library_effects>` → effect colors (diffuse / emissive / transparent)
//!   + scalar transparency + `opaque` mode (`A_ONE` / `A_ZERO` /
//!   `RGB_ZERO` / `RGB_ONE`) + per-effect sampler-to-image map
//! - `<library_materials>` → material id → effect id
//! - `<library_geometries>` → per-geometry source arrays + primitive
//!   groups (`<triangles>`, `<polylist>`, `<polygons>`) with material
//!   symbols and `<input>` offsets
//! - `<library_visual_scenes>` → node tree with accumulated transforms
//!   (matrix / translate / rotate / scale) and `<instance_geometry>` +
//!   `<bind_material>` bindings
//!
//! The parser is stream-based (quick-xml events). We use a simple
//! position stack (`Vec<&[u8]>` of open tag names) rather than building
//! an intermediate DOM, so even moderately large DAE files stay cheap to
//! parse.
//!
//! The output is [`DaeDocument`], which the caller walks to author the
//! equivalent USD prim structure.

use std::collections::HashMap;
use std::path::Path;

use quick_xml::events::{BytesStart, Event};
use quick_xml::reader::Reader;

use crate::{Error, Result};

// --------------------------------------------------------------------
//                           Public data
// --------------------------------------------------------------------

#[derive(Debug, Default)]
pub struct DaeDocument {
    pub unit_meter: f64,
    pub up_axis: UpAxis,
    /// image_id -> resolved filename (value of `<init_from>`). Paths are
    /// as-authored (relative/absolute); callers are responsible for
    /// resolving them against the DAE's own directory.
    pub images: HashMap<String, String>,
    pub effects: HashMap<String, Effect>,
    /// material_id -> effect_id
    pub materials: HashMap<String, String>,
    /// geometry_id -> mesh
    pub geometries: HashMap<String, Geometry>,
    /// Instances of geometries discovered under `<library_visual_scenes>`,
    /// each with its accumulated world transform and bind-material
    /// mapping. Multiple entries means the DAE placed the geometry more
    /// than once with different transforms.
    pub scene_instances: Vec<SceneInstance>,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum UpAxis {
    X,
    #[default]
    Y,
    Z,
}

#[derive(Debug, Default, Clone)]
pub struct Effect {
    pub diffuse_color: Option<[f32; 4]>,
    pub diffuse_sampler: Option<String>,
    pub emissive_color: Option<[f32; 4]>,
    pub emissive_sampler: Option<String>,
    pub transparent_color: Option<[f32; 4]>,
    pub transparent_sampler: Option<String>,
    pub transparency: Option<f32>,
    pub opaque_mode: OpaqueMode,
    pub ior: Option<f32>,
    /// Sampler surface IDs (from `<newparam sid="foo"><sampler2D><source>surf</source>`)
    /// → surface SID (from `<newparam sid="surf"><surface><init_from>img</init_from>`).
    /// Callers resolve sampler SIDs to image IDs through this map.
    pub sampler_to_image: HashMap<String, String>,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum OpaqueMode {
    #[default]
    AOne,
    AZero,
    RgbZero,
    RgbOne,
}

#[derive(Debug, Default, Clone)]
pub struct Geometry {
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub texcoords: Vec<[f32; 2]>,
    /// Pre-resolved per-face data, one entry per primitive group
    /// (triangles/polylist/polygons). Each entry already has per-vertex
    /// indices referencing `positions`/`normals`/`texcoords` directly
    /// (inputs deinterleaved).
    pub primitives: Vec<Primitive>,
}

#[derive(Debug, Clone)]
pub struct Primitive {
    /// `material="symbol"` attribute from the primitive element. Resolved
    /// to a material id through `<bind_material>` at scene instantiation.
    pub material_symbol: Option<String>,
    /// Vertex count per face. All 3 for `<triangles>`. Variable for
    /// `<polylist>` / `<polygons>`.
    pub face_vertex_counts: Vec<u32>,
    /// Vertex indices, flattened. `positions[face_indices[i]]` gives the
    /// vertex. Normals/UVs also use these indices directly (deinterleaved).
    pub face_indices: Vec<u32>,
    pub normal_indices: Option<Vec<u32>>,
    pub texcoord_indices: Option<Vec<u32>>,
}

#[derive(Debug, Clone)]
pub struct SceneInstance {
    pub geometry_id: String,
    /// Row-major 4×4 matrix stacking node transforms from scene root down
    /// to this instance.
    pub world_matrix: [f64; 16],
    /// material symbol → material id (resolved from `<bind_material>`).
    pub bindings: HashMap<String, String>,
}

// --------------------------------------------------------------------
//                              Parser
// --------------------------------------------------------------------

pub fn parse(path: &Path) -> Result<DaeDocument> {
    let xml = std::fs::read_to_string(path)?;
    parse_str(&xml)
}

pub fn parse_str(xml: &str) -> Result<DaeDocument> {
    let mut reader = Reader::from_str(xml);
    reader.config_mut().trim_text(true);

    let mut doc = DaeDocument::default();
    let mut st = State::default();
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(e)) => on_open(&mut st, &mut doc, &e, false),
            Ok(Event::Empty(e)) => {
                on_open(&mut st, &mut doc, &e, true);
                on_close(&mut st, &mut doc, e.name().as_ref());
            }
            Ok(Event::End(e)) => on_close(&mut st, &mut doc, e.name().as_ref()),
            Ok(Event::Text(t)) => {
                let s = t.unescape().map(|c| c.into_owned()).unwrap_or_default();
                let s = s.trim().to_string();
                if !s.is_empty() {
                    st.current_text.push_str(&s);
                }
            }
            Ok(Event::Eof) => break,
            Ok(_) => {}
            Err(e) => return Err(Error::msg(format!("DAE parse error: {e}"))),
        }
        buf.clear();
    }

    Ok(doc)
}

// Parser state is a stack of nested element contexts. Each element type
// we care about captures attributes on open, text on close. Anything we
// don't know about is ignored.
#[derive(Default)]
struct State {
    stack: Vec<String>,
    current_text: String,

    // current *things under construction*, keyed by outer element id.
    // Writes land on the top of the stack.
    current_effect_id: Option<String>,
    current_effect: Effect,

    current_material_id: Option<String>,
    current_material_effect: Option<String>,

    current_image_id: Option<String>,

    current_geometry_id: Option<String>,
    current_geometry: Geometry,

    // Source arrays inside the current geometry's <mesh>.
    sources: HashMap<String, DaeSource>,
    current_source_id: Option<String>,
    current_source: DaeSource,

    // `<vertices id="..."><input semantic="POSITION" source="#positions"/>`
    // maps the `<vertices>` input id (referenced from primitives via
    // `semantic="VERTEX"`) to the underlying source id.
    vertex_aliases: HashMap<String, String>,
    current_vertices_id: Option<String>,

    // Primitive group under construction.
    current_primitive: Option<PrimitiveBuilder>,

    // Effect common-profile context (e.g. `<lambert>` / `<phong>`).
    in_effect_profile_common: bool,
    // Which channel of the current profile block we're inside.
    current_channel: Option<EffectChannel>,

    // Newparam stack for sampler2D / surface surface-id → image-id resolution.
    newparam_sid: Option<String>,
    newparam_kind: NewParamKind,
    newparam_source: String,   // <sampler2D><source>...</source>
    newparam_init_from: String,// <surface><init_from>...</init_from>
    // SID -> surface image_id
    surface_to_image: HashMap<String, String>,

    // Visual-scene traversal state.
    transform_stack: Vec<[f64; 16]>,
    current_instance_geometry: Option<InstanceGeometryBuilder>,
    inside_bind_material: bool,
}

#[derive(Debug, Default, Clone)]
struct DaeSource {
    floats: Vec<f64>,
    stride: usize,
    count: usize,
}

#[derive(Debug, Clone, Copy)]
enum NewParamKind {
    None,
    Sampler2D,
    Surface,
}

impl Default for NewParamKind {
    fn default() -> Self {
        NewParamKind::None
    }
}

#[derive(Debug, Clone, Copy)]
enum EffectChannel {
    Diffuse,
    Emission,
    Transparent,
    Transparency,
    IndexOfRefraction,
}

#[derive(Debug, Clone)]
struct PrimitiveBuilder {
    kind: PrimKind,
    material_symbol: Option<String>,
    /// `<input semantic="X" source="#src" offset="N"/>` rows.
    inputs: Vec<InputRef>,
    /// Values from `<p>` elements. For triangles: one p per group of
    /// 3×stride ints. For polylist: one concatenated list.
    p_values: Vec<u32>,
    /// `<vcount>` for polylist (face vertex counts).
    vcount: Vec<u32>,
    /// One `<p>` per face for `<polygons>`.
    polygons_ps: Vec<Vec<u32>>,
    /// Total stride = max(offset) + 1 across all inputs.
    stride: usize,
}

#[derive(Debug, Clone, Copy)]
enum PrimKind {
    Triangles,
    Polylist,
    Polygons,
}

#[derive(Debug, Clone)]
struct InputRef {
    semantic: String,
    source_id: String, // already stripped of `#`
    offset: usize,
}

#[derive(Debug, Clone)]
struct InstanceGeometryBuilder {
    geometry_id: String,
    bindings: HashMap<String, String>,
}

// --------------------------------------------------------------------
//                          Event dispatch
// --------------------------------------------------------------------

fn on_open(st: &mut State, doc: &mut DaeDocument, e: &BytesStart<'_>, _is_empty: bool) {
    let name_raw = e.name();
    let name = bytes_to_string(name_raw.as_ref());
    let attrs = attrs_as_map(e);
    st.current_text.clear();

    match name.as_str() {
        "unit" => {
            if let Some(v) = attrs.get("meter").and_then(|s| s.parse::<f64>().ok()) {
                doc.unit_meter = v;
            }
        }
        "up_axis" => {
            // <up_axis>X_UP|Y_UP|Z_UP</up_axis> — text handled on close.
        }

        // ---- images ----
        "image" => {
            if st.in_library("image") {
                st.current_image_id = attrs.get("id").cloned();
            }
        }

        // ---- effects ----
        "effect" => {
            st.current_effect_id = attrs.get("id").cloned();
            st.current_effect = Effect::default();
            st.surface_to_image.clear();
        }
        "profile_COMMON" => st.in_effect_profile_common = true,
        "newparam" => {
            st.newparam_sid = attrs.get("sid").cloned();
            st.newparam_kind = NewParamKind::None;
            st.newparam_source.clear();
            st.newparam_init_from.clear();
        }
        "sampler2D" if st.newparam_sid.is_some() => st.newparam_kind = NewParamKind::Sampler2D,
        "surface" if st.newparam_sid.is_some() => st.newparam_kind = NewParamKind::Surface,

        "diffuse" if st.in_effect_profile_common => st.current_channel = Some(EffectChannel::Diffuse),
        "emission" if st.in_effect_profile_common => st.current_channel = Some(EffectChannel::Emission),
        "transparent" if st.in_effect_profile_common => {
            st.current_channel = Some(EffectChannel::Transparent);
            if let Some(mode) = attrs.get("opaque") {
                st.current_effect.opaque_mode = parse_opaque_mode(mode);
            }
        }
        "transparency" if st.in_effect_profile_common => {
            st.current_channel = Some(EffectChannel::Transparency)
        }
        "index_of_refraction" if st.in_effect_profile_common => {
            st.current_channel = Some(EffectChannel::IndexOfRefraction)
        }
        "texture" if st.current_channel.is_some() => {
            if let Some(sampler) = attrs.get("texture") {
                match st.current_channel {
                    Some(EffectChannel::Diffuse) => {
                        st.current_effect.diffuse_sampler = Some(sampler.clone());
                    }
                    Some(EffectChannel::Emission) => {
                        st.current_effect.emissive_sampler = Some(sampler.clone());
                    }
                    Some(EffectChannel::Transparent) => {
                        st.current_effect.transparent_sampler = Some(sampler.clone());
                    }
                    _ => {}
                }
            }
        }

        // ---- materials ----
        "material" => {
            if st.in_library("material") {
                st.current_material_id = attrs.get("id").cloned();
                st.current_material_effect = None;
            }
        }
        "instance_effect" if st.current_material_id.is_some() => {
            if let Some(url) = attrs.get("url") {
                st.current_material_effect = Some(strip_hash(url));
            }
        }

        // ---- geometries ----
        "geometry" => {
            st.current_geometry_id = attrs.get("id").cloned();
            st.current_geometry = Geometry::default();
            st.sources.clear();
            st.vertex_aliases.clear();
        }
        "source" if st.current_geometry_id.is_some() => {
            st.current_source_id = attrs.get("id").cloned();
            st.current_source = DaeSource::default();
        }
        "accessor" if st.current_source_id.is_some() => {
            if let Some(c) = attrs.get("count").and_then(|s| s.parse::<usize>().ok()) {
                st.current_source.count = c;
            }
            if let Some(s) = attrs.get("stride").and_then(|s| s.parse::<usize>().ok()) {
                st.current_source.stride = s;
            }
        }
        "vertices" if st.current_geometry_id.is_some() => {
            st.current_vertices_id = attrs.get("id").cloned();
        }
        "input" => {
            // Inputs live in two contexts:
            //   <vertices><input semantic="POSITION" source="#src"/>
            //   <triangles|polylist|polygons><input ... offset="N"/>
            let semantic = attrs.get("semantic").cloned().unwrap_or_default();
            let source = attrs.get("source").map(|s| strip_hash(s)).unwrap_or_default();
            if let Some(vid) = &st.current_vertices_id {
                if semantic == "POSITION" {
                    st.vertex_aliases.insert(vid.clone(), source);
                }
            } else if let Some(prim) = st.current_primitive.as_mut() {
                let offset = attrs
                    .get("offset")
                    .and_then(|s| s.parse::<usize>().ok())
                    .unwrap_or(0);
                prim.inputs.push(InputRef {
                    semantic,
                    source_id: source,
                    offset,
                });
                prim.stride = prim.stride.max(offset + 1);
            }
        }
        "triangles" | "polylist" | "polygons" => {
            let kind = match name.as_str() {
                "triangles" => PrimKind::Triangles,
                "polylist" => PrimKind::Polylist,
                _ => PrimKind::Polygons,
            };
            st.current_primitive = Some(PrimitiveBuilder {
                kind,
                material_symbol: attrs.get("material").cloned(),
                inputs: Vec::new(),
                p_values: Vec::new(),
                vcount: Vec::new(),
                polygons_ps: Vec::new(),
                stride: 0,
            });
        }

        // ---- visual scenes ----
        "visual_scene" | "library_visual_scenes" => {
            st.transform_stack.clear();
        }
        "node" => {
            let parent = st.transform_stack.last().copied().unwrap_or(IDENTITY4);
            st.transform_stack.push(parent);
        }
        "instance_geometry" if !st.transform_stack.is_empty() => {
            if let Some(url) = attrs.get("url") {
                st.current_instance_geometry = Some(InstanceGeometryBuilder {
                    geometry_id: strip_hash(url),
                    bindings: HashMap::new(),
                });
            }
        }
        "bind_material" => st.inside_bind_material = true,
        "instance_material" if st.inside_bind_material => {
            let symbol = attrs.get("symbol").cloned();
            let target = attrs.get("target").map(|s| strip_hash(s));
            if let (Some(sym), Some(tgt)) = (symbol, target) {
                if let Some(ig) = st.current_instance_geometry.as_mut() {
                    ig.bindings.insert(sym, tgt);
                }
            }
        }

        _ => {}
    }

    st.stack.push(name);
}

fn on_close(st: &mut State, doc: &mut DaeDocument, name_raw: &[u8]) {
    let name = bytes_to_string(name_raw);
    let text = std::mem::take(&mut st.current_text);

    match name.as_str() {
        "up_axis" => {
            doc.up_axis = match text.as_str() {
                "X_UP" => UpAxis::X,
                "Z_UP" => UpAxis::Z,
                _ => UpAxis::Y,
            };
        }

        // ---- images ----
        "init_from" => {
            if let NewParamKind::Surface = st.newparam_kind {
                st.newparam_init_from = text.clone();
            } else if let Some(id) = &st.current_image_id {
                doc.images.insert(id.clone(), text.clone());
            }
        }
        "image" if st.in_library("image") => st.current_image_id = None,

        // ---- effects ----
        "newparam" => {
            if let Some(sid) = st.newparam_sid.take() {
                match st.newparam_kind {
                    NewParamKind::Sampler2D => {
                        // Resolve the sampler's underlying surface sid later
                        // via surface_to_image.
                        if let Some(img) = st.surface_to_image.get(&st.newparam_source) {
                            st.current_effect
                                .sampler_to_image
                                .insert(sid, img.clone());
                        }
                    }
                    NewParamKind::Surface => {
                        st.surface_to_image.insert(sid, st.newparam_init_from.clone());
                    }
                    NewParamKind::None => {}
                }
            }
            st.newparam_kind = NewParamKind::None;
            st.newparam_source.clear();
            st.newparam_init_from.clear();
        }
        "source" => {
            // Inside <newparam><sampler2D> the text is the surface SID.
            if let NewParamKind::Sampler2D = st.newparam_kind {
                st.newparam_source = text.clone();
            }
            // Inside <geometry>: finish a source array.
            if let (Some(id), None) = (
                st.current_source_id.as_ref(),
                st.stack.iter().rev().find(|s| s.as_str() == "newparam"),
            ) {
                st.sources.insert(id.clone(), std::mem::take(&mut st.current_source));
            }
            st.current_source_id = None;
        }
        "float_array" if st.current_source_id.is_some() => {
            st.current_source.floats = text
                .split_ascii_whitespace()
                .filter_map(|s| s.parse::<f64>().ok())
                .collect();
        }
        "color" if st.current_channel.is_some() => {
            let rgba = parse_rgba_text(&text);
            match st.current_channel {
                Some(EffectChannel::Diffuse) => st.current_effect.diffuse_color = Some(rgba),
                Some(EffectChannel::Emission) => st.current_effect.emissive_color = Some(rgba),
                Some(EffectChannel::Transparent) => st.current_effect.transparent_color = Some(rgba),
                _ => {}
            }
        }
        "float" if st.current_channel.is_some() => {
            let v = text.parse::<f32>().ok();
            match st.current_channel {
                Some(EffectChannel::Transparency) => st.current_effect.transparency = v,
                Some(EffectChannel::IndexOfRefraction) => st.current_effect.ior = v,
                _ => {}
            }
        }
        "diffuse" | "emission" | "transparent" | "transparency" | "index_of_refraction" => {
            st.current_channel = None;
        }
        "profile_COMMON" => st.in_effect_profile_common = false,
        "effect" => {
            if let Some(id) = st.current_effect_id.take() {
                doc.effects.insert(id, std::mem::take(&mut st.current_effect));
            }
        }

        // ---- materials ----
        "material" if st.in_library("material") => {
            if let (Some(id), Some(effect)) = (
                st.current_material_id.take(),
                st.current_material_effect.take(),
            ) {
                doc.materials.insert(id, effect);
            }
        }

        // ---- geometries ----
        "vertices" => st.current_vertices_id = None,
        "p" => {
            let vals: Vec<u32> = text
                .split_ascii_whitespace()
                .filter_map(|s| s.parse::<u32>().ok())
                .collect();
            if let Some(prim) = st.current_primitive.as_mut() {
                match prim.kind {
                    PrimKind::Polygons => prim.polygons_ps.push(vals),
                    _ => prim.p_values.extend(vals),
                }
            }
        }
        "vcount" => {
            if let Some(prim) = st.current_primitive.as_mut() {
                prim.vcount = text
                    .split_ascii_whitespace()
                    .filter_map(|s| s.parse::<u32>().ok())
                    .collect();
            }
        }
        "triangles" | "polylist" | "polygons" => {
            if let Some(prim) = st.current_primitive.take() {
                finalize_primitive(
                    prim,
                    &st.sources,
                    &st.vertex_aliases,
                    &mut st.current_geometry,
                );
            }
        }
        "geometry" => {
            if let Some(id) = st.current_geometry_id.take() {
                doc.geometries
                    .insert(id, std::mem::take(&mut st.current_geometry));
            }
        }

        // ---- visual scenes ----
        "matrix" | "translate" | "rotate" | "scale" => {
            if !st.transform_stack.is_empty() {
                apply_node_transform(&mut st.transform_stack, &name, &text);
            }
        }
        "node" => {
            st.transform_stack.pop();
        }
        "instance_geometry" => {
            if let Some(ig) = st.current_instance_geometry.take() {
                let m = st.transform_stack.last().copied().unwrap_or(IDENTITY4);
                doc.scene_instances.push(SceneInstance {
                    geometry_id: ig.geometry_id,
                    world_matrix: m,
                    bindings: ig.bindings,
                });
            }
        }
        "bind_material" => st.inside_bind_material = false,

        _ => {}
    }

    st.stack.pop();
}

// --------------------------------------------------------------------
//                        Primitive finalization
// --------------------------------------------------------------------

/// Walk a parsed `<triangles>` / `<polylist>` / `<polygons>` block,
/// deinterleave its `<p>` indices into per-channel index arrays, merge
/// the referenced source arrays into the geometry's shared position /
/// normal / texcoord buffers (deduped by source id), and append the
/// resulting `Primitive` to the geometry.
fn finalize_primitive(
    p: PrimitiveBuilder,
    sources: &HashMap<String, DaeSource>,
    vertex_aliases: &HashMap<String, String>,
    geom: &mut Geometry,
) {
    let mut vertex_offset: Option<usize> = None;
    let mut vertex_source: Option<String> = None;
    let mut normal_offset: Option<usize> = None;
    let mut normal_source: Option<String> = None;
    let mut tex_offset: Option<usize> = None;
    let mut tex_source: Option<String> = None;

    for input in &p.inputs {
        match input.semantic.as_str() {
            "VERTEX" => {
                let src = vertex_aliases
                    .get(&input.source_id)
                    .cloned()
                    .unwrap_or_else(|| input.source_id.clone());
                vertex_offset = Some(input.offset);
                vertex_source = Some(src);
            }
            "NORMAL" => {
                normal_offset = Some(input.offset);
                normal_source = Some(input.source_id.clone());
            }
            "TEXCOORD" if tex_offset.is_none() => {
                tex_offset = Some(input.offset);
                tex_source = Some(input.source_id.clone());
            }
            _ => {}
        }
    }

    let Some(vertex_offset) = vertex_offset else { return };
    let Some(vertex_source) = vertex_source else { return };

    // Merge source buffers into the shared geometry arrays, tracking the
    // base offset we assigned each source inside the merged arrays.
    let pos_base = merge_vec3_source(&vertex_source, sources, &mut geom.positions);
    let nrm_base = normal_source
        .as_ref()
        .map(|id| merge_vec3_source(id, sources, &mut geom.normals));
    let uv_base = tex_source
        .as_ref()
        .map(|id| merge_vec2_source(id, sources, &mut geom.texcoords));

    let face_vertex_counts: Vec<u32> = match p.kind {
        PrimKind::Triangles => {
            let face_count = p.p_values.len() / p.stride.max(1) / 3;
            vec![3u32; face_count]
        }
        PrimKind::Polylist => p.vcount.clone(),
        PrimKind::Polygons => p
            .polygons_ps
            .iter()
            .map(|v| (v.len() / p.stride.max(1)) as u32)
            .collect(),
    };

    let mut pos_indices: Vec<u32> = Vec::new();
    let mut nrm_indices: Vec<u32> = Vec::new();
    let mut uv_indices: Vec<u32> = Vec::new();

    let stride = p.stride.max(1);

    let walk = |indices: &[u32],
                pos: &mut Vec<u32>,
                nrm: &mut Vec<u32>,
                uv: &mut Vec<u32>| {
        let mut c = 0usize;
        while c + stride <= indices.len() {
            pos.push(indices[c + vertex_offset] + pos_base);
            if let (Some(n_off), Some(base)) = (normal_offset, nrm_base) {
                nrm.push(indices[c + n_off] + base);
            }
            if let (Some(t_off), Some(base)) = (tex_offset, uv_base) {
                uv.push(indices[c + t_off] + base);
            }
            c += stride;
        }
    };

    match p.kind {
        PrimKind::Triangles | PrimKind::Polylist => {
            walk(
                &p.p_values,
                &mut pos_indices,
                &mut nrm_indices,
                &mut uv_indices,
            );
        }
        PrimKind::Polygons => {
            for poly in &p.polygons_ps {
                walk(poly, &mut pos_indices, &mut nrm_indices, &mut uv_indices);
            }
        }
    }

    geom.primitives.push(Primitive {
        material_symbol: p.material_symbol,
        face_vertex_counts,
        face_indices: pos_indices,
        normal_indices: nrm_base.map(|_| nrm_indices),
        texcoord_indices: uv_base.map(|_| uv_indices),
    });
}

/// Append floats from the source with id `src_id` as `[f32; 3]`s into
/// `target`; return the starting index (so downstream indices can be
/// remapped by adding this offset).
fn merge_vec3_source(
    src_id: &str,
    sources: &HashMap<String, DaeSource>,
    target: &mut Vec<[f32; 3]>,
) -> u32 {
    let base = target.len() as u32;
    let Some(src) = sources.get(src_id) else {
        return base;
    };
    let stride = src.stride.max(3);
    let mut i = 0usize;
    while i + 3 <= src.floats.len() {
        target.push([
            src.floats[i] as f32,
            src.floats[i + 1] as f32,
            src.floats[i + 2] as f32,
        ]);
        i += stride;
    }
    base
}

fn merge_vec2_source(
    src_id: &str,
    sources: &HashMap<String, DaeSource>,
    target: &mut Vec<[f32; 2]>,
) -> u32 {
    let base = target.len() as u32;
    let Some(src) = sources.get(src_id) else {
        return base;
    };
    let stride = src.stride.max(2);
    let mut i = 0usize;
    while i + 2 <= src.floats.len() {
        target.push([src.floats[i] as f32, src.floats[i + 1] as f32]);
        i += stride;
    }
    base
}

// --------------------------------------------------------------------
//                        Transform math
// --------------------------------------------------------------------

const IDENTITY4: [f64; 16] = [
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
];

fn apply_node_transform(stack: &mut [[f64; 16]], kind: &str, text: &str) {
    let Some(top) = stack.last_mut() else {
        return;
    };
    let new_m = match kind {
        "matrix" => parse_matrix(text),
        "translate" => translation_matrix(text),
        "rotate" => rotate_matrix(text),
        "scale" => scale_matrix(text),
        _ => return,
    };
    *top = mat_mul(*top, new_m);
}

fn parse_matrix(text: &str) -> [f64; 16] {
    let mut m = IDENTITY4;
    let vals: Vec<f64> = text
        .split_ascii_whitespace()
        .filter_map(|s| s.parse::<f64>().ok())
        .collect();
    for i in 0..16.min(vals.len()) {
        m[i] = vals[i];
    }
    m
}

fn translation_matrix(text: &str) -> [f64; 16] {
    let v: Vec<f64> = text
        .split_ascii_whitespace()
        .filter_map(|s| s.parse::<f64>().ok())
        .collect();
    let [x, y, z] = [
        *v.first().unwrap_or(&0.0),
        *v.get(1).unwrap_or(&0.0),
        *v.get(2).unwrap_or(&0.0),
    ];
    let mut m = IDENTITY4;
    m[3] = x;
    m[7] = y;
    m[11] = z;
    m
}

fn rotate_matrix(text: &str) -> [f64; 16] {
    let v: Vec<f64> = text
        .split_ascii_whitespace()
        .filter_map(|s| s.parse::<f64>().ok())
        .collect();
    let (ax, ay, az, deg) = (
        *v.first().unwrap_or(&0.0),
        *v.get(1).unwrap_or(&0.0),
        *v.get(2).unwrap_or(&0.0),
        *v.get(3).unwrap_or(&0.0),
    );
    let rad = deg.to_radians();
    let n = (ax * ax + ay * ay + az * az).sqrt().max(f64::EPSILON);
    let (x, y, z) = (ax / n, ay / n, az / n);
    let c = rad.cos();
    let s = rad.sin();
    let t = 1.0 - c;

    [
        t * x * x + c,     t * x * y - s * z, t * x * z + s * y, 0.0,
        t * x * y + s * z, t * y * y + c,     t * y * z - s * x, 0.0,
        t * x * z - s * y, t * y * z + s * x, t * z * z + c,     0.0,
        0.0,               0.0,               0.0,               1.0,
    ]
}

fn scale_matrix(text: &str) -> [f64; 16] {
    let v: Vec<f64> = text
        .split_ascii_whitespace()
        .filter_map(|s| s.parse::<f64>().ok())
        .collect();
    let [sx, sy, sz] = [
        *v.first().unwrap_or(&1.0),
        *v.get(1).unwrap_or(&1.0),
        *v.get(2).unwrap_or(&1.0),
    ];
    let mut m = IDENTITY4;
    m[0] = sx;
    m[5] = sy;
    m[10] = sz;
    m
}

fn mat_mul(a: [f64; 16], b: [f64; 16]) -> [f64; 16] {
    let mut o = [0.0f64; 16];
    for i in 0..4 {
        for j in 0..4 {
            let mut v = 0.0;
            for k in 0..4 {
                v += a[i * 4 + k] * b[k * 4 + j];
            }
            o[i * 4 + j] = v;
        }
    }
    o
}

/// Row-major 4×4 matrix applied to a point (w=1).
pub fn transform_point(m: &[f64; 16], p: [f32; 3]) -> [f32; 3] {
    let (x, y, z) = (p[0] as f64, p[1] as f64, p[2] as f64);
    [
        (m[0] * x + m[1] * y + m[2] * z + m[3]) as f32,
        (m[4] * x + m[5] * y + m[6] * z + m[7]) as f32,
        (m[8] * x + m[9] * y + m[10] * z + m[11]) as f32,
    ]
}

/// Row-major 4×4 matrix applied to a direction (w=0, treated as vector).
pub fn transform_vector(m: &[f64; 16], v: [f32; 3]) -> [f32; 3] {
    let (x, y, z) = (v[0] as f64, v[1] as f64, v[2] as f64);
    [
        (m[0] * x + m[1] * y + m[2] * z) as f32,
        (m[4] * x + m[5] * y + m[6] * z) as f32,
        (m[8] * x + m[9] * y + m[10] * z) as f32,
    ]
}

// --------------------------------------------------------------------
//                        Opacity / transparency
// --------------------------------------------------------------------

/// Resolve the final opacity scalar from the four COLLADA opacity
/// ingredients. Mirrors Python converter's `material.py:372-397`.
///
/// When an effect carries neither `<transparency>` nor `<transparent>`,
/// COLLADA treats it as fully opaque — don't run the mode formula with
/// the naked defaults (which would collapse A_ONE to 0).
pub fn resolved_opacity(effect: &Effect) -> f32 {
    if effect.transparency.is_none() && effect.transparent_color.is_none() {
        return 1.0;
    }
    let trans = effect.transparency.unwrap_or(1.0);
    let color = effect.transparent_color.unwrap_or([1.0, 1.0, 1.0, 1.0]);
    match effect.opaque_mode {
        OpaqueMode::AOne => 1.0 - trans * color[3],
        OpaqueMode::AZero => trans * color[3],
        OpaqueMode::RgbZero => {
            let lum = 0.212671 * color[0] + 0.715160 * color[1] + 0.072169 * color[2];
            1.0 - trans * lum
        }
        OpaqueMode::RgbOne => {
            let lum = 0.212671 * color[0] + 0.715160 * color[1] + 0.072169 * color[2];
            trans * lum
        }
    }
}

/// Resolve a `<texture texture="sid"/>` reference on an effect to the
/// underlying image filename, going `sampler sid → surface sid → image id
/// → filename` through the effect's own sampler map and the document's
/// `library_images`.
pub fn resolve_sampler_to_path<'a>(
    effect: &Effect,
    sampler_sid: &str,
    doc: &'a DaeDocument,
) -> Option<&'a str> {
    let image_id = effect.sampler_to_image.get(sampler_sid)?;
    doc.images.get(image_id).map(|s| s.as_str())
}

// --------------------------------------------------------------------
//                             Helpers
// --------------------------------------------------------------------

fn bytes_to_string(b: &[u8]) -> String {
    String::from_utf8_lossy(b).into_owned()
}

fn attrs_as_map(e: &BytesStart<'_>) -> HashMap<String, String> {
    let mut m = HashMap::new();
    for res in e.attributes() {
        let Ok(a) = res else { continue };
        let k = String::from_utf8_lossy(a.key.as_ref()).into_owned();
        let v = a
            .unescape_value()
            .map(|c| c.into_owned())
            .unwrap_or_default();
        m.insert(k, v);
    }
    m
}

fn strip_hash(s: &str) -> String {
    s.strip_prefix('#').unwrap_or(s).to_string()
}

fn parse_rgba_text(text: &str) -> [f32; 4] {
    let v: Vec<f32> = text
        .split_ascii_whitespace()
        .filter_map(|s| s.parse::<f32>().ok())
        .collect();
    [
        *v.first().unwrap_or(&0.0),
        *v.get(1).unwrap_or(&0.0),
        *v.get(2).unwrap_or(&0.0),
        *v.get(3).unwrap_or(&1.0),
    ]
}

fn parse_opaque_mode(s: &str) -> OpaqueMode {
    match s {
        "A_ZERO" => OpaqueMode::AZero,
        "RGB_ZERO" => OpaqueMode::RgbZero,
        "RGB_ONE" => OpaqueMode::RgbOne,
        _ => OpaqueMode::AOne,
    }
}

impl State {
    fn in_library(&self, inner_tag: &str) -> bool {
        // "library_<inner>s" (plural). Confirm our open ancestors include
        // `library_<inner>s` — a cheap guard to avoid misinterpreting
        // nested blocks.
        let expected = format!("library_{inner_tag}s");
        self.stack.iter().any(|t| t == &expected)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_unitmeter_and_up_axis() {
        let xml = r#"<COLLADA>
            <asset>
                <unit name="meter" meter="0.01"/>
                <up_axis>Z_UP</up_axis>
            </asset>
        </COLLADA>"#;
        let doc = parse_str(xml).unwrap();
        assert!((doc.unit_meter - 0.01).abs() < 1e-9);
        assert_eq!(doc.up_axis, UpAxis::Z);
    }

    #[test]
    fn parses_material_and_effect_with_transparency() {
        let xml = r##"<COLLADA>
            <library_effects>
                <effect id="mat_effect">
                    <profile_COMMON>
                        <technique sid="common">
                            <lambert>
                                <diffuse><color>1 0 0 1</color></diffuse>
                                <transparent opaque="A_ONE"><color>1 1 1 1</color></transparent>
                                <transparency><float>0.5</float></transparency>
                            </lambert>
                        </technique>
                    </profile_COMMON>
                </effect>
            </library_effects>
            <library_materials>
                <material id="red"><instance_effect url="#mat_effect"/></material>
            </library_materials>
        </COLLADA>"##;
        let doc = parse_str(xml).unwrap();
        assert_eq!(doc.materials.get("red"), Some(&"mat_effect".to_string()));
        let e = doc.effects.get("mat_effect").expect("effect");
        assert_eq!(e.diffuse_color, Some([1.0, 0.0, 0.0, 1.0]));
        assert!(matches!(e.opaque_mode, OpaqueMode::AOne));
        // A_ONE with trans=0.5 + color.a=1 → opacity = 1 - 0.5*1 = 0.5
        assert!((resolved_opacity(e) - 0.5).abs() < 1e-6);
    }

    #[test]
    fn resolved_opacity_modes() {
        let mut e = Effect::default();
        e.transparency = Some(0.3);
        e.transparent_color = Some([0.8, 0.8, 0.8, 1.0]);
        e.opaque_mode = OpaqueMode::AOne;
        assert!((resolved_opacity(&e) - 0.7).abs() < 1e-6);
        e.opaque_mode = OpaqueMode::AZero;
        assert!((resolved_opacity(&e) - 0.3).abs() < 1e-6);
    }
}
