use std::path::PathBuf;

/// Converter options. Mirrors the Python converter's CLI flags.
#[derive(Debug, Clone, Default)]
pub struct Options {
    /// Emit the asset as an Atomic Component (Asset Interface layer +
    /// payloaded contents + Geometry/Physics/Materials sublayers).
    ///
    /// When `false`, everything is flattened into a single `.usdc` file.
    /// Not yet implemented — M5. For now we always emit a single `.usda`.
    pub layer_structure: bool,

    /// Emit a `UsdPhysics.Scene` prim with the root fixed joint.
    pub scene: bool,

    /// Optional authoring comment stamped into stage metadata.
    pub comment: String,

    /// `--package name=path` overrides for resolving `package://` URIs
    /// in URDF mesh/texture filenames.
    pub packages: Vec<PackageOverride>,
}

#[derive(Debug, Clone)]
pub struct PackageOverride {
    pub name: String,
    pub path: PathBuf,
}

impl Options {
    pub fn new() -> Self {
        Self {
            layer_structure: true,
            scene: true,
            comment: String::new(),
            packages: Vec::new(),
        }
    }
}
