//! URDF → USD → URDF structural round-trip test.
//!
//! Rather than byte-compare the reconstructed URDF against the input
//! (tolerance-inserted placeholders, float precision, and attribute
//! ordering all differ), we assert that the *structural* invariants
//! match: link names, joint names + types, parent/child topology, the
//! per-link count of visuals / collisions, and the primary geometry
//! kind of each shape. That's enough to prove the round-trip preserves
//! everything a downstream consumer (bevy_urdf, rapier, ROS tooling)
//! would key off.

use std::collections::{HashMap, HashSet};
use std::path::Path;

use urdf2usd::{Options, urdf_to_usd, usd_to_urdf};

/// Parse a URDF, run it through the forward path, then the reverse path,
/// and return (original, reconstructed).
fn roundtrip(urdf_path: &Path) -> (urdf_rs::Robot, urdf_rs::Robot) {
    let out_dir = tempdir(urdf_path);
    let usd_top = urdf_to_usd(urdf_path, &out_dir, &Options::new()).expect("forward");

    // Use tolerated input as the "original" for comparison — tolerance is
    // expected behaviour of the forward path, not noise.
    let raw = std::fs::read_to_string(urdf_path).unwrap();
    let patched = urdf2usd::tolerate::tolerate(&raw).unwrap();
    let original = urdf_rs::read_from_string(&patched).unwrap();

    let reconstructed_path = out_dir.join("reconstructed.urdf");
    usd_to_urdf(&usd_top, &reconstructed_path).expect("reverse");
    let reconstructed_xml = std::fs::read_to_string(&reconstructed_path).unwrap();
    let reconstructed = urdf_rs::read_from_string(&reconstructed_xml).unwrap();

    (original, reconstructed)
}

fn tempdir(input: &Path) -> std::path::PathBuf {
    let stem = input
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("test");
    let dir = std::env::temp_dir().join(format!("urdf2usd_rt_{stem}"));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn joint_kind_name(t: &urdf_rs::JointType) -> &'static str {
    match t {
        urdf_rs::JointType::Fixed => "fixed",
        urdf_rs::JointType::Revolute => "revolute",
        urdf_rs::JointType::Continuous => "continuous",
        urdf_rs::JointType::Prismatic => "prismatic",
        urdf_rs::JointType::Floating => "floating",
        urdf_rs::JointType::Planar => "planar",
        urdf_rs::JointType::Spherical => "spherical",
    }
}

fn geom_kind(g: &urdf_rs::Geometry) -> &'static str {
    match g {
        urdf_rs::Geometry::Box { .. } => "box",
        urdf_rs::Geometry::Cylinder { .. } => "cylinder",
        urdf_rs::Geometry::Capsule { .. } => "capsule",
        urdf_rs::Geometry::Sphere { .. } => "sphere",
        urdf_rs::Geometry::Mesh { .. } => "mesh",
    }
}

fn assert_structural_parity(original: &urdf_rs::Robot, reconstructed: &urdf_rs::Robot) {
    assert_eq!(original.name, reconstructed.name, "robot name");

    let orig_links: HashSet<&str> = original.links.iter().map(|l| l.name.as_str()).collect();
    let recon_links: HashSet<&str> =
        reconstructed.links.iter().map(|l| l.name.as_str()).collect();
    assert_eq!(orig_links, recon_links, "link name set");

    // Joint topology: name -> (kind, parent, child)
    let orig_joints: HashMap<&str, (&str, &str, &str)> = original
        .joints
        .iter()
        .map(|j| {
            (
                j.name.as_str(),
                (
                    joint_kind_name(&j.joint_type),
                    j.parent.link.as_str(),
                    j.child.link.as_str(),
                ),
            )
        })
        .collect();
    let recon_joints: HashMap<&str, (&str, &str, &str)> = reconstructed
        .joints
        .iter()
        .map(|j| {
            (
                j.name.as_str(),
                (
                    joint_kind_name(&j.joint_type),
                    j.parent.link.as_str(),
                    j.child.link.as_str(),
                ),
            )
        })
        .collect();
    assert_eq!(orig_joints, recon_joints, "joint topology");

    // Per-link visual/collision counts + leading geometry kind.
    let recon_links_by_name: HashMap<&str, &urdf_rs::Link> = reconstructed
        .links
        .iter()
        .map(|l| (l.name.as_str(), l))
        .collect();
    for orig in &original.links {
        let recon = recon_links_by_name
            .get(orig.name.as_str())
            .expect("link present");
        assert_eq!(
            orig.visual.len(),
            recon.visual.len(),
            "link `{}` visual count",
            orig.name
        );
        assert_eq!(
            orig.collision.len(),
            recon.collision.len(),
            "link `{}` collision count",
            orig.name
        );
        if let (Some(ov), Some(rv)) = (orig.visual.first(), recon.visual.first()) {
            assert_eq!(
                geom_kind(&ov.geometry),
                geom_kind(&rv.geometry),
                "link `{}` first visual geometry kind",
                orig.name
            );
        }
    }
}

#[test]
fn simple_box_roundtrip() {
    let p = Path::new("xtra/urdf-usd-converter/tests/data/simple_box.urdf");
    let (a, b) = roundtrip(p);
    assert_structural_parity(&a, &b);
}

#[test]
fn fixed_continuous_joints_roundtrip() {
    let p = Path::new("xtra/urdf-usd-converter/tests/data/fixed_continuous_joints.urdf");
    let (a, b) = roundtrip(p);
    assert_structural_parity(&a, &b);
}

#[test]
fn revolute_joints_roundtrip() {
    let p = Path::new("xtra/urdf-usd-converter/tests/data/revolute_joints.urdf");
    let (a, b) = roundtrip(p);
    assert_structural_parity(&a, &b);
}

#[test]
fn prismatic_joints_roundtrip() {
    let p = Path::new("xtra/urdf-usd-converter/tests/data/prismatic_joints.urdf");
    let (a, b) = roundtrip(p);
    assert_structural_parity(&a, &b);
}

#[test]
fn material_mesh_color_roundtrip() {
    let p = Path::new("xtra/urdf-usd-converter/tests/data/material_mesh_color.urdf");
    let (a, b) = roundtrip(p);
    assert_structural_parity(&a, &b);
}

#[test]
fn inertia_roundtrip() {
    let p = Path::new("xtra/urdf-usd-converter/tests/data/inertia.urdf");
    let (a, b) = roundtrip(p);
    assert_structural_parity(&a, &b);
}

/// Joint axis + origin rpy must be preserved through the round-trip.
/// Previously the axis-correction rotation (which we fold into
/// `localRot0` on the forward path to emit `physics:axis = "X"`)
/// leaked into `origin.rpy` on the reverse path, rotating child links
/// like robot wheels by 90° and tilting everything downstream.
#[test]
fn joint_axis_preservation() {
    let tmp = std::env::temp_dir().join("urdf2usd_axis_rt");
    let _ = std::fs::remove_dir_all(&tmp);
    std::fs::create_dir_all(&tmp).unwrap();

    // Two-link robot with a Y-axis revolute joint (the canonical wheel
    // case) and zero origin rotation — both should come back exactly.
    let input_urdf = tmp.join("axis.urdf");
    std::fs::write(
        &input_urdf,
        r#"<?xml version="1.0"?>
<robot name="axis">
  <link name="base">
    <visual><geometry><box size="1 1 0.1"/></geometry></visual>
  </link>
  <link name="wheel">
    <visual><geometry><cylinder radius="0.2" length="0.05"/></geometry></visual>
  </link>
  <joint name="hinge" type="continuous">
    <origin xyz="0.5 0 0" rpy="0 0 0"/>
    <parent link="base"/>
    <child link="wheel"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>"#,
    )
    .unwrap();

    let (_, b) = roundtrip(&input_urdf);
    let hinge = b
        .joints
        .iter()
        .find(|j| j.name == "hinge")
        .expect("hinge joint present");

    // Axis should be recovered as (0, 1, 0) — no leakage from
    // forward-path axis correction.
    let ax = hinge.axis.xyz.0;
    assert!(
        (ax[0]).abs() < 1e-4 && (ax[1] - 1.0).abs() < 1e-4 && (ax[2]).abs() < 1e-4,
        "axis was not preserved, got {:?}",
        ax
    );

    // Origin rpy should stay zero — axis correction should not have
    // leaked into it.
    let rpy = hinge.origin.rpy.0;
    assert!(
        rpy.iter().all(|v| v.abs() < 1e-4),
        "origin.rpy leaked axis correction, got {:?}",
        rpy
    );
}
