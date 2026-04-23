//! Capture URDF elements / attributes that urdf-rs doesn't model and
//! stash them as `urdf:*` custom attributes on the corresponding USD prims.
//!
//! urdf-rs is a strict serde mapping: any tag or attribute it doesn't know
//! about gets silently dropped. Real-world URDFs routinely carry
//! extensions (Gazebo `<gazebo reference=...>` blocks, MoveIt semantic
//! descriptors, vendor-specific children). This module streams the URDF
//! XML with `quick-xml`, tracks whether we're inside `<link>` / `<joint>`
//! / `<robot>`, and for every element that isn't part of the standard
//! URDF schema records its attributes + text-leaf children under
//! `urdf:<tag[/nested]>:<attr>` keys.
//!
//! The output is attached to the right prim later in `convert::*`.

use std::collections::HashMap;

use quick_xml::events::Event;
use quick_xml::reader::Reader;

use crate::{Error, Result};

/// One attribute or text leaf extracted from an unknown URDF element.
#[derive(Debug, Clone)]
pub struct UndefinedEntry {
    /// Full USD attribute name, e.g. `"urdf:gazebo:reference"` or
    /// `"urdf:gazebo:material"`.
    pub attr_name: String,
    /// Textual value. We author these as `string` attrs since URDF
    /// extensions don't carry a type.
    pub value: String,
}

/// Where each undefined-element entry should land in USD.
#[derive(Debug, Default)]
pub struct UndefinedMap {
    pub robot_level: Vec<UndefinedEntry>,
    pub by_link: HashMap<String, Vec<UndefinedEntry>>,
    pub by_joint: HashMap<String, Vec<UndefinedEntry>>,
}

/// Schema-recognised tags we do not want to report as "undefined" at each
/// level. Anything else under these scopes is extension content.
const KNOWN_ROBOT: &[&str] = &["material", "link", "joint", "transmission"];
const KNOWN_LINK: &[&str] = &["inertial", "visual", "collision"];
const KNOWN_JOINT: &[&str] = &[
    "origin",
    "parent",
    "child",
    "axis",
    "limit",
    "dynamics",
    "mimic",
    "calibration",
    "safety_controller",
];

pub fn collect(xml: &str) -> Result<UndefinedMap> {
    let mut reader = Reader::from_str(xml);
    reader.config_mut().trim_text(true);

    let mut out = UndefinedMap::default();
    let mut ctx = Context::default();
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Eof) => break,
            Ok(Event::Start(e)) => {
                let name = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                let attrs = attrs_as_map(&e);
                on_open(&mut ctx, &name, &attrs, &mut out);
            }
            Ok(Event::Empty(e)) => {
                let name = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                let attrs = attrs_as_map(&e);
                on_open(&mut ctx, &name, &attrs, &mut out);
                on_close(&mut ctx, &name, None, &mut out);
            }
            Ok(Event::Text(t)) => {
                let text = t.unescape().map(|c| c.into_owned()).unwrap_or_default();
                let text = text.trim().to_string();
                if !text.is_empty() {
                    ctx.current_text = Some(text);
                }
            }
            Ok(Event::End(e)) => {
                let name = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                let text = ctx.current_text.take();
                on_close(&mut ctx, &name, text, &mut out);
            }
            Ok(_) => {}
            Err(e) => return Err(Error::msg(format!("undefined-pass XML error: {e}"))),
        }
        buf.clear();
    }

    Ok(out)
}

#[derive(Default)]
struct Context {
    /// Stack of open tag names.
    stack: Vec<String>,
    /// Current `<link>` name if any (top-level).
    current_link: Option<String>,
    /// Current `<joint>` name if any (top-level).
    current_joint: Option<String>,
    /// Depth at which we entered an unknown element, together with the
    /// path segments (relative to the unknown root) accumulated so far.
    /// Only the outermost unknown is tracked — nested unknown descendants
    /// collapse into its path.
    unknown_root: Option<UnknownFrame>,
    /// Text that just closed from the last `Text` event; consumed by the
    /// `End` handler when it fires.
    current_text: Option<String>,
}

struct UnknownFrame {
    /// Depth of the `stack` when the unknown element was first opened.
    enter_depth: usize,
    /// Dot-separated path from the unknown root down to the current
    /// descendant. `["gazebo", "material"]` becomes `"gazebo.material"`.
    path: Vec<String>,
    /// Which bucket to drop entries into when this frame closes.
    bucket: Bucket,
}

#[derive(Clone, Copy)]
enum Bucket {
    Robot,
    Link,
    Joint,
}

fn on_open(
    ctx: &mut Context,
    name: &str,
    attrs: &HashMap<String, String>,
    out: &mut UndefinedMap,
) {
    // Track standard scopes first so nested tags inside a known link/joint
    // know which bucket they belong to.
    let depth = ctx.stack.len();
    if ctx.unknown_root.is_none() {
        if name == "link" && depth == 1 {
            if let Some(n) = attrs.get("name") {
                ctx.current_link = Some(n.clone());
            }
        } else if name == "joint" && depth == 1 {
            if let Some(n) = attrs.get("name") {
                ctx.current_joint = Some(n.clone());
            }
        }
    }

    let is_unknown = ctx.unknown_root.is_some() || is_undefined_at_depth(name, depth, ctx);

    if is_unknown {
        if ctx.unknown_root.is_none() {
            let bucket = if ctx.current_link.is_some() {
                Bucket::Link
            } else if ctx.current_joint.is_some() {
                Bucket::Joint
            } else {
                Bucket::Robot
            };
            ctx.unknown_root = Some(UnknownFrame {
                enter_depth: depth,
                path: vec![sanitize_segment(name)],
                bucket,
            });
        } else {
            // Nested inside an existing unknown — extend the path.
            let frame = ctx.unknown_root.as_mut().unwrap();
            frame.path.push(sanitize_segment(name));
        }

        // Emit attributes of this unknown element.
        let frame = ctx.unknown_root.as_ref().unwrap();
        for (k, v) in attrs {
            let attr_name = format!("urdf:{}:{}", frame.path.join(":"), sanitize_segment(k));
            push_entry(
                out,
                frame.bucket,
                ctx.current_link.as_deref(),
                ctx.current_joint.as_deref(),
                UndefinedEntry {
                    attr_name,
                    value: v.clone(),
                },
            );
        }
    }

    ctx.stack.push(name.to_string());
}

fn on_close(
    ctx: &mut Context,
    name: &str,
    text: Option<String>,
    out: &mut UndefinedMap,
) {
    // If this close corresponds to a text-carrying leaf inside an unknown
    // block, record that as `urdf:<path>`.
    if let Some(t) = text {
        if let Some(frame) = ctx.unknown_root.as_ref() {
            let attr_name = format!("urdf:{}", frame.path.join(":"));
            push_entry(
                out,
                frame.bucket,
                ctx.current_link.as_deref(),
                ctx.current_joint.as_deref(),
                UndefinedEntry {
                    attr_name,
                    value: t,
                },
            );
        }
    }

    // Pop the unknown-path if we're inside one.
    if let Some(frame) = ctx.unknown_root.as_mut() {
        if ctx.stack.len() > frame.enter_depth + 1 {
            frame.path.pop();
        } else {
            // Closing the outermost unknown element.
            ctx.unknown_root = None;
        }
    }

    ctx.stack.pop();

    // Clear scope markers when the <link>/<joint> actually closes.
    if ctx.unknown_root.is_none() {
        if name == "link" && ctx.stack.len() == 1 {
            ctx.current_link = None;
        } else if name == "joint" && ctx.stack.len() == 1 {
            ctx.current_joint = None;
        }
    }
}

fn is_undefined_at_depth(name: &str, depth: usize, ctx: &Context) -> bool {
    // depth = number of parents above this tag.
    match depth {
        0 => false, // the <robot> element itself
        1 => !KNOWN_ROBOT.contains(&name),
        _ => {
            let parent = ctx.stack.last().map(String::as_str).unwrap_or("");
            match parent {
                "link" => !KNOWN_LINK.contains(&name),
                "joint" => !KNOWN_JOINT.contains(&name),
                _ => false,
            }
        }
    }
}

fn push_entry(
    out: &mut UndefinedMap,
    bucket: Bucket,
    link: Option<&str>,
    joint: Option<&str>,
    entry: UndefinedEntry,
) {
    match bucket {
        Bucket::Robot => out.robot_level.push(entry),
        Bucket::Link => {
            if let Some(n) = link {
                out.by_link.entry(n.to_string()).or_default().push(entry);
            }
        }
        Bucket::Joint => {
            if let Some(n) = joint {
                out.by_joint.entry(n.to_string()).or_default().push(entry);
            }
        }
    }
}

fn attrs_as_map(e: &quick_xml::events::BytesStart<'_>) -> HashMap<String, String> {
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

fn sanitize_segment(raw: &str) -> String {
    raw.chars()
        .map(|c| if c.is_ascii_alphanumeric() { c } else { '_' })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn captures_gazebo_on_link() {
        let xml = r#"<robot name="r">
            <link name="base"/>
            <gazebo reference="base">
                <material>Gazebo/Red</material>
                <mu1>100.0</mu1>
            </gazebo>
        </robot>"#;
        // gazebo at robot-level attached to base via reference= attribute
        let out = collect(xml).unwrap();
        assert!(!out.robot_level.is_empty(), "{:?}", out);
    }

    #[test]
    fn captures_unknown_on_joint() {
        let xml = r#"<robot name="r">
            <joint name="j" type="fixed">
                <parent link="a"/>
                <child link="b"/>
                <vendor speed="fast"/>
            </joint>
        </robot>"#;
        let out = collect(xml).unwrap();
        let entries = out.by_joint.get("j").expect("joint entries");
        assert!(
            entries
                .iter()
                .any(|e| e.attr_name == "urdf:vendor:speed" && e.value == "fast"),
            "{:?}",
            entries
        );
    }

    #[test]
    fn leaves_known_alone() {
        let xml = r#"<robot name="r">
            <link name="a"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
            <joint name="j" type="fixed"><parent link="a"/><child link="a"/></joint>
        </robot>"#;
        let out = collect(xml).unwrap();
        assert!(out.robot_level.is_empty());
        assert!(out.by_link.is_empty());
        assert!(out.by_joint.is_empty());
    }
}
