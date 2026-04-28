//! URDF ↔ OpenUSD converter.
//!
//! Both directions are first-class:
//! - [`urdf_to_usd`] reads a URDF file and writes a USD asset (single
//!   `.usda` or layered `<robot>.usda` + Geometry/Physics/Materials
//!   sublayers).
//! - [`usd_to_urdf`] reads the USD asset back and reconstructs a URDF
//!   file, closing the round-trip so tools that only consume URDF
//!   (bevy_urdf, rapier3d-urdf, ROS tooling) can read the output.
//!
//! The CLI in `main.rs` dispatches on the input file extension.

pub mod hierarchy;
pub mod math;
pub mod opts;
pub mod resolve;
pub mod to_urdf;
pub mod to_usd;
pub mod to_usdc;
pub mod tolerate;
pub mod undefined;
pub mod usdz;

pub use opts::Options;
pub use to_usdc::convert_layered_dir_to_usdc;
pub use usdz::pack_dir_to_usdz;

use std::path::{Path, PathBuf};

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("URDF parse failed: {0}")]
    Urdf(#[from] urdf_rs::UrdfError),

    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("USD write failed: {0}")]
    Usd(#[from] anyhow::Error),

    #[error("conversion error: {0}")]
    Msg(String),
}

impl Error {
    pub fn msg(s: impl Into<String>) -> Self {
        Error::Msg(s.into())
    }
}

pub type Result<T> = std::result::Result<T, Error>;

/// Read a URDF file and write a USD asset under `output_dir`. Returns the
/// path of the primary `.usda` file written.
pub fn urdf_to_usd(
    input_file: impl AsRef<Path>,
    output_dir: impl AsRef<Path>,
    opts: &Options,
) -> Result<PathBuf> {
    let input_file = input_file.as_ref();
    let output_dir = output_dir.as_ref();

    if !input_file.is_file() {
        return Err(Error::msg(format!(
            "input file does not exist or is not a file: {}",
            input_file.display()
        )));
    }

    std::fs::create_dir_all(output_dir)?;

    let raw_xml = std::fs::read_to_string(input_file)?;
    let patched = tolerate::tolerate(&raw_xml)?;
    let robot = urdf_rs::read_from_string(&patched)?;

    check_duplicate_names(&robot)?;

    let undefined = undefined::collect(&raw_xml).unwrap_or_else(|e| {
        eprintln!("warning: undefined-element pass failed: {e}");
        undefined::UndefinedMap::default()
    });

    let resolver = resolve::PackageResolver::new(input_file, &opts.packages);
    let hierarchy = hierarchy::LinkHierarchy::build(&robot)?;

    to_usd::run(&robot, &hierarchy, &resolver, output_dir, opts, &undefined)
}

/// Read a USD asset and write a URDF file.
///
/// `input_file` is the top USD file (`.usda` / `.usdc`); it may reference
/// sublayers (Geometry / Physics / Materials) — openusd composes them all
/// automatically. `output_file` is the destination `.urdf` path.
pub fn usd_to_urdf(
    input_file: impl AsRef<Path>,
    output_file: impl AsRef<Path>,
) -> Result<PathBuf> {
    let input_file = input_file.as_ref();
    let output_file = output_file.as_ref();

    if !input_file.is_file() {
        return Err(Error::msg(format!(
            "input file does not exist or is not a file: {}",
            input_file.display()
        )));
    }
    if let Some(parent) = output_file.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent)?;
        }
    }

    let robot = to_urdf::run(input_file)?;
    let xml = urdf_rs::write_to_string(&robot)
        .map_err(|e| Error::msg(format!("URDF serialize failed: {e}")))?;
    std::fs::write(output_file, xml)?;
    Ok(output_file.to_path_buf())
}

/// Back-compat: old entry point. Calls [`urdf_to_usd`].
#[deprecated(note = "use urdf_to_usd()")]
pub fn convert(
    input_file: impl AsRef<Path>,
    output_dir: impl AsRef<Path>,
    opts: &Options,
) -> Result<PathBuf> {
    urdf_to_usd(input_file, output_dir, opts)
}

fn check_duplicate_names(robot: &urdf_rs::Robot) -> Result<()> {
    use std::collections::HashSet;

    let mut seen: HashSet<&str> = HashSet::new();
    for link in &robot.links {
        if !seen.insert(link.name.as_str()) {
            return Err(Error::msg(format!(
                "duplicate link name `{}`",
                link.name
            )));
        }
    }

    let mut seen: HashSet<&str> = HashSet::new();
    for joint in &robot.joints {
        if !seen.insert(joint.name.as_str()) {
            return Err(Error::msg(format!(
                "duplicate joint name `{}`",
                joint.name
            )));
        }
    }

    let mut seen: HashSet<&str> = HashSet::new();
    for mat in &robot.materials {
        if mat.name.is_empty() {
            continue;
        }
        if !seen.insert(mat.name.as_str()) {
            return Err(Error::msg(format!(
                "duplicate material name `{}`",
                mat.name
            )));
        }
    }

    Ok(())
}
