//! URDF XML tolerance preprocessor.
//!
//! `urdf-rs` is strict — `<material>` must have a `name`, `<limit>` a
//! `velocity`, `<visual>` / `<collision>` a well-formed `<geometry>`, etc.
//! Real-world URDFs and deliberately sloppy test fixtures often omit or
//! mis-type these. This module streams the XML through `quick-xml` and
//! injects safe defaults so the downstream parse succeeds.
//!
//! Two patches happen:
//! 1. Missing-required-attribute injection on known tags (`<material>`,
//!    `<limit>`, `<safety_controller>`, `<mass>`, `<inertia>`, `<color>`,
//!    `<mesh>`).
//! 2. A tiny state machine to recover malformed geometry:
//!    - `<visual>` / `<collision>` without any `<geometry>` child gets a
//!      placeholder `<geometry><box size="0 0 0"/></geometry>`.
//!    - `<geometry></geometry>` gets a placeholder `<box size="0 0 0"/>`.
//!    - `<geometry><unknown_tag/></geometry>` has the unknown child
//!      replaced with a placeholder box (content of the unknown tag, if
//!      any, is dropped).

use std::io::Cursor;

use quick_xml::events::{BytesStart, Event};
use quick_xml::reader::Reader;
use quick_xml::writer::Writer;

use crate::{Error, Result};

const GEOM_SHAPES: &[&[u8]] = &[
    b"box",
    b"sphere",
    b"cylinder",
    b"capsule",
    b"mesh",
];

/// Patch the URDF XML text to inject attributes urdf-rs requires but the
/// file omitted, and to recover from a handful of well-known structural
/// mistakes in the geometry subtree.
pub fn tolerate(xml: &str) -> Result<String> {
    let mut reader = Reader::from_str(xml);
    reader.config_mut().trim_text(false);

    let mut writer = Writer::new(Cursor::new(Vec::<u8>::new()));

    let mut state = State::default();
    let mut anon_mat = 0u32;
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Eof) => break,
            Ok(Event::Start(e)) => handle_start(&mut state, &mut writer, e, &mut anon_mat)?,
            Ok(Event::Empty(e)) => handle_empty(&mut state, &mut writer, e, &mut anon_mat)?,
            Ok(Event::End(e)) => handle_end(&mut state, &mut writer, e)?,
            Ok(ev) => {
                if state.skip_depth == 0 {
                    writer.write_event(ev).map_err(xml_err)?;
                }
            }
            Err(e) => return Err(Error::msg(format!("XML parse error: {e}"))),
        }
        buf.clear();
    }

    let out = writer.into_inner().into_inner();
    String::from_utf8(out).map_err(|e| Error::msg(format!("XML not UTF-8 after rewrite: {e}")))
}

#[derive(Default)]
struct State {
    in_visual: bool,
    in_collision: bool,
    visual_has_geometry: bool,
    collision_has_geometry: bool,
    in_geometry: bool,
    geometry_has_child: bool,
    /// When > 0 we are inside an unknown element under `<geometry>` and
    /// we're dropping its events (we already emitted a placeholder box).
    skip_depth: u32,
}

fn handle_start(
    state: &mut State,
    writer: &mut Writer<Cursor<Vec<u8>>>,
    e: BytesStart<'_>,
    anon_mat: &mut u32,
) -> Result<()> {
    let tag = e.name();
    let name = tag.as_ref();

    if state.skip_depth > 0 {
        state.skip_depth += 1;
        return Ok(());
    }

    if state.in_geometry && !is_known_shape(name) {
        write_placeholder_box(writer)?;
        state.geometry_has_child = true;
        state.skip_depth = 1;
        return Ok(());
    }

    match name {
        b"visual" => {
            state.in_visual = true;
            state.visual_has_geometry = false;
        }
        b"collision" => {
            state.in_collision = true;
            state.collision_has_geometry = false;
        }
        b"geometry" => {
            state.in_geometry = true;
            state.geometry_has_child = false;
            if state.in_visual {
                state.visual_has_geometry = true;
            }
            if state.in_collision {
                state.collision_has_geometry = true;
            }
        }
        _ => {}
    }

    let patched = patch_start(e, anon_mat);
    writer.write_event(Event::Start(patched)).map_err(xml_err)?;
    Ok(())
}

fn handle_empty(
    state: &mut State,
    writer: &mut Writer<Cursor<Vec<u8>>>,
    e: BytesStart<'_>,
    anon_mat: &mut u32,
) -> Result<()> {
    if state.skip_depth > 0 {
        return Ok(());
    }
    let tag = e.name();
    let name = tag.as_ref();

    if state.in_geometry && !is_known_shape(name) {
        write_placeholder_box(writer)?;
        state.geometry_has_child = true;
        return Ok(());
    }
    if state.in_geometry && is_known_shape(name) {
        state.geometry_has_child = true;
    }

    let patched = patch_start(e, anon_mat);
    writer.write_event(Event::Empty(patched)).map_err(xml_err)?;
    Ok(())
}

fn handle_end(
    state: &mut State,
    writer: &mut Writer<Cursor<Vec<u8>>>,
    e: quick_xml::events::BytesEnd<'_>,
) -> Result<()> {
    if state.skip_depth > 0 {
        state.skip_depth -= 1;
        return Ok(());
    }
    let tag = e.name();
    let name = tag.as_ref();

    match name {
        b"geometry" => {
            if state.in_geometry && !state.geometry_has_child {
                write_placeholder_box(writer)?;
            }
            state.in_geometry = false;
        }
        b"visual" => {
            if state.in_visual && !state.visual_has_geometry {
                write_placeholder_geometry(writer)?;
            }
            state.in_visual = false;
        }
        b"collision" => {
            if state.in_collision && !state.collision_has_geometry {
                write_placeholder_geometry(writer)?;
            }
            state.in_collision = false;
        }
        _ => {}
    }
    writer.write_event(Event::End(e)).map_err(xml_err)?;
    Ok(())
}

fn patch_start<'a>(e: BytesStart<'a>, anon_mat: &mut u32) -> BytesStart<'a> {
    let tag = e.name();
    match tag.as_ref() {
        b"material" if !has_attr(&e, b"name") => {
            let label = format!("_anon_material_{}", *anon_mat);
            *anon_mat += 1;
            clone_with_attr(&e, "name", &label)
        }
        b"limit" if !has_attr(&e, b"velocity") => clone_with_attr(&e, "velocity", "0"),
        b"safety_controller" if !has_attr(&e, b"k_velocity") => {
            clone_with_attr(&e, "k_velocity", "0")
        }
        b"mass" if !has_attr(&e, b"value") => clone_with_attr(&e, "value", "0"),
        b"inertia" => ensure_inertia_attrs(e),
        b"color" if !has_attr(&e, b"rgba") => clone_with_attr(&e, "rgba", "1 1 1 1"),
        b"mesh" if !has_attr(&e, b"filename") => clone_with_attr(&e, "filename", ""),
        _ => e,
    }
}

fn ensure_inertia_attrs(mut e: BytesStart<'_>) -> BytesStart<'_> {
    const REQUIRED: &[&str] = &["ixx", "ixy", "ixz", "iyy", "iyz", "izz"];
    for k in REQUIRED {
        if !has_attr(&e, k.as_bytes()) {
            e.push_attribute((*k, "0"));
        }
    }
    e
}

fn is_known_shape(name: &[u8]) -> bool {
    GEOM_SHAPES.iter().any(|s| *s == name)
}

fn write_placeholder_box(writer: &mut Writer<Cursor<Vec<u8>>>) -> Result<()> {
    let mut e = BytesStart::new("box");
    e.push_attribute(("size", "0 0 0"));
    writer.write_event(Event::Empty(e)).map_err(xml_err)
}

fn write_placeholder_geometry(writer: &mut Writer<Cursor<Vec<u8>>>) -> Result<()> {
    let open = BytesStart::new("geometry");
    writer.write_event(Event::Start(open)).map_err(xml_err)?;
    write_placeholder_box(writer)?;
    writer
        .write_event(Event::End(quick_xml::events::BytesEnd::new("geometry")))
        .map_err(xml_err)
}

fn has_attr(e: &BytesStart<'_>, key: &[u8]) -> bool {
    e.attributes().flatten().any(|a| a.key.as_ref() == key)
}

fn clone_with_attr<'a>(e: &BytesStart<'a>, key: &str, val: &str) -> BytesStart<'a> {
    let mut out = e.clone();
    out.push_attribute((key, val));
    out
}

fn xml_err(e: quick_xml::Error) -> Error {
    Error::msg(format!("XML write error: {e}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn injects_material_name() {
        let input = r#"<robot name="x"><material><color rgba="1 0 0 1"/></material></robot>"#;
        let out = tolerate(input).unwrap();
        assert!(out.contains(r#"name="_anon_material_0""#), "got: {out}");
    }

    #[test]
    fn injects_limit_velocity() {
        let input = r#"<robot name="x"><joint name="j" type="revolute"><limit lower="0" upper="1" effort="1"/></joint></robot>"#;
        let out = tolerate(input).unwrap();
        assert!(out.contains(r#"velocity="0""#), "got: {out}");
    }

    #[test]
    fn leaves_valid_xml_alone() {
        let input = r#"<robot name="x"><material name="red"><color rgba="1 0 0 1"/></material></robot>"#;
        let out = tolerate(input).unwrap();
        assert_eq!(out, input);
    }

    #[test]
    fn fills_empty_geometry() {
        let input = r#"<robot name="x"><link name="l"><visual><geometry></geometry></visual></link></robot>"#;
        let out = tolerate(input).unwrap();
        assert!(out.contains(r#"<box size="0 0 0""#), "got: {out}");
    }

    #[test]
    fn replaces_unknown_shape() {
        let input = r#"<robot name="x"><link name="l"><visual><geometry><foo size="1 1 1"/></geometry></visual></link></robot>"#;
        let out = tolerate(input).unwrap();
        assert!(out.contains(r#"<box size="0 0 0""#), "got: {out}");
        assert!(!out.contains("<foo"), "got: {out}");
    }

    #[test]
    fn fills_missing_visual_geometry() {
        let input = r#"<robot name="x"><link name="l"><visual><material name="m"/></visual></link></robot>"#;
        let out = tolerate(input).unwrap();
        assert!(out.contains("<geometry>"), "got: {out}");
        assert!(out.contains(r#"<box size="0 0 0""#), "got: {out}");
    }

    #[test]
    fn drops_nested_unknown_content() {
        let input = r#"<robot name="x"><link name="l"><visual><geometry><foo><bar/></foo></geometry></visual></link></robot>"#;
        let out = tolerate(input).unwrap();
        assert!(!out.contains("<foo") && !out.contains("<bar"), "got: {out}");
        assert!(out.contains(r#"<box size="0 0 0""#));
    }
}
