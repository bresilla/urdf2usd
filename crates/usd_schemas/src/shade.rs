//! UsdShade authoring: `Material` + `UsdPreviewSurface` + per-channel
//! `UsdUVTexture` shaders.
//!
//! We author the **PreviewMaterial interface pattern**: every shader
//! input that can be overridden (diffuseColor, opacity, roughness,
//! metallic, emissiveColor, ior, normal) is promoted up to the
//! `Material` prim as `inputs:<name>`, and the shader's own input is a
//! connection to the Material input. Downstream users can tweak a
//! material's look by overriding `Material.inputs:diffuseColor` without
//! touching the shader graph.
//!
//! - Scalar channels → `Material.inputs:X = <value>` (default value) and
//!   `Shader.inputs:X.connect = </Material.inputs:X>`.
//! - Textured channels → `Material.inputs:X.connect = </Material/Tex.outputs:rgb>`
//!   and `Shader.inputs:X.connect = </Material.inputs:X>`. The texture
//!   output flows through the Material's interface so overriding the
//!   Material input also cuts the texture off cleanly.

use openusd::sdf::{Path, Value};

use anyhow::Result;

use super::Stage;
use super::tokens::*;

pub struct MaterialSpec<'a> {
    pub diffuse_srgb: [f32; 3],
    pub opacity: f32,
    pub roughness: f32,
    pub metallic: f32,
    pub emissive_srgb: [f32; 3],
    /// Zero means "don't author `inputs:ior` at all — let the renderer use
    /// the PreviewSurface default".
    pub ior: f32,

    pub diffuse_texture: Option<&'a str>,
    pub normal_texture: Option<&'a str>,
    pub roughness_texture: Option<&'a str>,
    pub metallic_texture: Option<&'a str>,
    pub opacity_texture: Option<&'a str>,
    pub emissive_texture: Option<&'a str>,
}

impl<'a> Default for MaterialSpec<'a> {
    fn default() -> Self {
        Self {
            diffuse_srgb: [0.8, 0.8, 0.8],
            opacity: 1.0,
            roughness: 0.5,
            metallic: 0.0,
            emissive_srgb: [0.0, 0.0, 0.0],
            ior: 0.0,
            diffuse_texture: None,
            normal_texture: None,
            roughness_texture: None,
            metallic_texture: None,
            opacity_texture: None,
            emissive_texture: None,
        }
    }
}

pub fn define_preview_material(
    stage: &mut Stage,
    parent: &Path,
    name: &str,
    spec: &MaterialSpec<'_>,
) -> Result<Path> {
    let mat = stage.define_prim(parent, name, T_MATERIAL)?;
    stage.set_prim_metadata(&mat, "instanceable", Value::Bool(true))?;

    let shader = stage.define_prim(&mat, "Surface", T_SHADER)?;
    stage.define_attribute(
        &shader,
        "info:id",
        "token",
        Value::Token("UsdPreviewSurface".into()),
        true,
    )?;

    let any_texture = spec.diffuse_texture.is_some()
        || spec.normal_texture.is_some()
        || spec.roughness_texture.is_some()
        || spec.metallic_texture.is_some()
        || spec.opacity_texture.is_some()
        || spec.emissive_texture.is_some();

    // Shared `st` primvar reader used by every UsdUVTexture.
    let st_out = if any_texture {
        Some(define_st_reader(stage, &mat)?)
    } else {
        None
    };

    // Author each textured channel first so we can reference the texture's
    // output prim path from the Material interface input.
    let diffuse_linear = srgb_to_linear(spec.diffuse_srgb);
    let diffuse_tex_out = if let (Some(path), Some(st)) = (spec.diffuse_texture, st_out.as_ref()) {
        let fallback = [
            diffuse_linear[0],
            diffuse_linear[1],
            diffuse_linear[2],
            spec.opacity,
        ];
        let t = author_uv_texture(stage, &mat, "DiffuseTex", path, "sRGB", fallback, st)?;
        Some(stage.attribute_path(&t, "outputs:rgb")?)
    } else {
        None
    };
    let normal_tex_out = if let (Some(path), Some(st)) = (spec.normal_texture, st_out.as_ref()) {
        let t = author_uv_texture(
            stage,
            &mat,
            "NormalTex",
            path,
            "raw",
            [0.5, 0.5, 1.0, 1.0],
            st,
        )?;
        stage.define_attribute(
            &t,
            "inputs:scale",
            "float4",
            Value::Vec4f([2.0, 2.0, 2.0, 1.0]),
            false,
        )?;
        stage.define_attribute(
            &t,
            "inputs:bias",
            "float4",
            Value::Vec4f([-1.0, -1.0, -1.0, 0.0]),
            false,
        )?;
        Some(stage.attribute_path(&t, "outputs:rgb")?)
    } else {
        None
    };
    let roughness_tex_out =
        if let (Some(path), Some(st)) = (spec.roughness_texture, st_out.as_ref()) {
            let t = author_uv_texture(
                stage,
                &mat,
                "RoughnessTex",
                path,
                "raw",
                [spec.roughness, spec.roughness, spec.roughness, 1.0],
                st,
            )?;
            Some(stage.attribute_path(&t, "outputs:r")?)
        } else {
            None
        };
    let metallic_tex_out =
        if let (Some(path), Some(st)) = (spec.metallic_texture, st_out.as_ref()) {
            let t = author_uv_texture(
                stage,
                &mat,
                "MetallicTex",
                path,
                "raw",
                [spec.metallic, spec.metallic, spec.metallic, 1.0],
                st,
            )?;
            Some(stage.attribute_path(&t, "outputs:r")?)
        } else {
            None
        };
    let opacity_tex_out = if let (Some(path), Some(st)) = (spec.opacity_texture, st_out.as_ref()) {
        let t = author_uv_texture(
            stage,
            &mat,
            "OpacityTex",
            path,
            "raw",
            [spec.opacity, spec.opacity, spec.opacity, 1.0],
            st,
        )?;
        Some(stage.attribute_path(&t, "outputs:r")?)
    } else {
        None
    };
    let emissive_linear = srgb_to_linear(spec.emissive_srgb);
    let emissive_tex_out = if let (Some(path), Some(st)) = (spec.emissive_texture, st_out.as_ref())
    {
        let fallback = [emissive_linear[0], emissive_linear[1], emissive_linear[2], 1.0];
        let t = author_uv_texture(stage, &mat, "EmissiveTex", path, "sRGB", fallback, st)?;
        Some(stage.attribute_path(&t, "outputs:rgb")?)
    } else {
        None
    };

    // Now promote each input to the Material interface + wire the shader's
    // input as a connection to the Material interface.
    promote_scalar(stage, &mat, &shader, "diffuseColor", "color3f", Value::Vec3f(diffuse_linear), diffuse_tex_out)?;
    promote_scalar(stage, &mat, &shader, "opacity", "float", Value::Float(spec.opacity), opacity_tex_out)?;
    promote_scalar(stage, &mat, &shader, "roughness", "float", Value::Float(spec.roughness), roughness_tex_out)?;
    promote_scalar(stage, &mat, &shader, "metallic", "float", Value::Float(spec.metallic), metallic_tex_out)?;
    if spec.emissive_srgb != [0.0, 0.0, 0.0] || spec.emissive_texture.is_some() {
        promote_scalar(
            stage,
            &mat,
            &shader,
            "emissiveColor",
            "color3f",
            Value::Vec3f(emissive_linear),
            emissive_tex_out,
        )?;
    }
    if spec.ior > 0.0 {
        promote_scalar(stage, &mat, &shader, "ior", "float", Value::Float(spec.ior), None)?;
    }
    if let Some(out) = normal_tex_out {
        // Normal map doesn't have a scalar fallback on PreviewSurface — it
        // flows only through the texture. Still promote to Material interface
        // for symmetry.
        let mat_attr = stage.attribute_path(&mat, "inputs:normal")?;
        stage.define_connection(&mat, "inputs:normal", "normal3f", out)?;
        stage.define_connection(&shader, "inputs:normal", "normal3f", mat_attr)?;
    }

    // Shader output + material output wiring.
    stage.define_attribute(
        &shader,
        "outputs:surface",
        "token",
        Value::Token(String::new()),
        false,
    )?;
    let shader_surface = stage.attribute_path(&shader, "outputs:surface")?;
    stage.define_connection(&mat, "outputs:surface", "token", shader_surface)?;

    Ok(mat)
}

/// Author `Material.inputs:<channel>` (scalar default *or* connection to
/// the texture output) and `Shader.inputs:<channel>.connect = </mat.inputs:<channel>>`.
fn promote_scalar(
    stage: &mut Stage,
    mat: &Path,
    shader: &Path,
    channel: &str,
    type_name: &str,
    scalar_default: Value,
    texture_out: Option<Path>,
) -> Result<()> {
    let mat_attr_name = format!("inputs:{channel}");

    if let Some(tex_out) = texture_out {
        // Material.inputs:X connects to the texture output; it carries no
        // scalar default (the texture is the value).
        stage.define_connection(mat, &mat_attr_name, type_name, tex_out)?;
    } else {
        // Material.inputs:X holds the scalar default directly.
        stage.define_attribute(mat, &mat_attr_name, type_name, scalar_default, false)?;
    }

    // Shader.inputs:X always reads from the Material interface input.
    let mat_input_path = stage.attribute_path(mat, &mat_attr_name)?;
    stage.define_connection(shader, &mat_attr_name, type_name, mat_input_path)?;
    Ok(())
}

fn define_st_reader(stage: &mut Stage, mat: &Path) -> Result<Path> {
    let st_reader = stage.define_prim(mat, "stReader", T_SHADER)?;
    stage.define_attribute(
        &st_reader,
        "info:id",
        "token",
        Value::Token("UsdPrimvarReader_float2".into()),
        true,
    )?;
    stage.define_attribute(
        &st_reader,
        "inputs:varname",
        "token",
        Value::Token("st".into()),
        false,
    )?;
    stage.define_attribute(
        &st_reader,
        "outputs:result",
        "float2",
        Value::Vec2f([0.0, 0.0]),
        false,
    )?;
    stage.attribute_path(&st_reader, "outputs:result")
}

fn author_uv_texture(
    stage: &mut Stage,
    material: &Path,
    name: &str,
    asset_path: &str,
    source_color_space: &str,
    fallback: [f32; 4],
    st_out: &Path,
) -> Result<Path> {
    let tex = stage.define_prim(material, name, T_SHADER)?;
    stage.define_attribute(
        &tex,
        "info:id",
        "token",
        Value::Token("UsdUVTexture".into()),
        true,
    )?;
    stage.define_attribute(
        &tex,
        "inputs:file",
        "asset",
        Value::AssetPath(asset_path.to_string()),
        false,
    )?;
    stage.define_attribute(
        &tex,
        "inputs:sourceColorSpace",
        "token",
        Value::Token(source_color_space.into()),
        false,
    )?;
    stage.define_attribute(
        &tex,
        "inputs:wrapS",
        "token",
        Value::Token("repeat".into()),
        false,
    )?;
    stage.define_attribute(
        &tex,
        "inputs:wrapT",
        "token",
        Value::Token("repeat".into()),
        false,
    )?;
    stage.define_attribute(
        &tex,
        "inputs:fallback",
        "float4",
        Value::Vec4f(fallback),
        false,
    )?;
    stage.define_attribute(
        &tex,
        "outputs:rgb",
        "float3",
        Value::Vec3f([0.0, 0.0, 0.0]),
        false,
    )?;
    stage.define_attribute(&tex, "outputs:r", "float", Value::Float(0.0), false)?;
    stage.define_connection(&tex, "inputs:st", "float2", st_out.clone())?;
    Ok(tex)
}

pub fn bind_material(stage: &mut Stage, prim: &Path, material: &Path) -> Result<()> {
    stage.apply_api_schemas(prim, &[API_MATERIAL_BINDING])?;
    stage.define_relationship(prim, "material:binding", vec![material.clone()])?;
    Ok(())
}

fn srgb_to_linear(c: [f32; 3]) -> [f32; 3] {
    let f = |v: f32| -> f32 {
        if v <= 0.04045 {
            v / 12.92
        } else {
            ((v + 0.055) / 1.055).powf(2.4)
        }
    };
    [f(c[0]), f(c[1]), f(c[2])]
}
