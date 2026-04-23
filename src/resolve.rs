//! Resolve URDF filename URIs (`package://`, `file://`, relative, absolute)
//! to concrete filesystem paths.
//!
//! Mirrors the Python converter's `_impl/ros_package.py`: explicit `--package`
//! overrides first, then upward directory search from the URDF file.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use crate::opts::PackageOverride;

#[derive(Debug)]
pub struct PackageResolver {
    /// Directory containing the URDF file. All relative paths resolve here.
    urdf_dir: PathBuf,
    /// Explicit `package name -> absolute path` overrides from CLI.
    overrides: HashMap<String, PathBuf>,
    /// Cache of previously-resolved package names to speed up repeated hits.
    cache: std::cell::RefCell<HashMap<String, PathBuf>>,
}

impl PackageResolver {
    pub fn new(urdf_file: &Path, overrides: &[PackageOverride]) -> Self {
        let urdf_dir = urdf_file
            .parent()
            .map(Path::to_path_buf)
            .unwrap_or_else(|| PathBuf::from("."));

        let mut map = HashMap::new();
        for ov in overrides {
            let abs = if ov.path.is_absolute() {
                ov.path.clone()
            } else {
                urdf_dir.join(&ov.path)
            };
            map.insert(ov.name.clone(), abs);
        }

        Self {
            urdf_dir,
            overrides: map,
            cache: Default::default(),
        }
    }

    /// Package name → filesystem path pairs that have been discovered so
    /// far (either via `--package` overrides or the upward package.xml /
    /// directory walk). Useful for handing off to downstream viewers
    /// (e.g. bevy_urdf's `PackageMap`) so they see the same resolution.
    pub fn known_packages(&self) -> Vec<(String, PathBuf)> {
        let mut out: Vec<(String, PathBuf)> = self
            .overrides
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect();
        for (k, v) in self.cache.borrow().iter() {
            if !out.iter().any(|(n, _)| n == k) {
                out.push((k.clone(), v.clone()));
            }
        }
        out
    }

    /// Resolve a filename that may be `package://pkg/path`, `file:///abs`,
    /// an absolute filesystem path, or a path relative to the URDF file.
    pub fn resolve(&self, filename: &str) -> Option<PathBuf> {
        // https:// / http:// / ftp:// and other remote schemes — we don't
        // fetch over the network. Caller decides what to do with None.
        if filename.starts_with("http://")
            || filename.starts_with("https://")
            || filename.starts_with("ftp://")
        {
            return None;
        }

        if let Some(rest) = filename.strip_prefix("package://") {
            let (pkg, tail) = rest.split_once('/')?;
            let pkg_path = self.package_path(pkg, tail)?;
            Some(pkg_path.join(tail))
        } else if let Some(rest) = filename.strip_prefix("file://") {
            // URDF convention: `file:///abs` -> absolute, `file://rel` -> relative
            // to the URDF's directory. `file:///` leaves `rest = "/abs/..."`.
            let p = PathBuf::from(rest);
            if p.is_absolute() {
                Some(p)
            } else {
                Some(self.urdf_dir.join(p))
            }
        } else {
            let p = PathBuf::from(filename);
            if p.is_absolute() {
                Some(p)
            } else {
                Some(self.urdf_dir.join(p))
            }
        }
    }

    fn package_path(&self, name: &str, tail: &str) -> Option<PathBuf> {
        if let Some(p) = self.overrides.get(name) {
            return Some(p.clone());
        }
        if let Some(p) = self.cache.borrow().get(name) {
            return Some(p.clone());
        }
        // Walk up from the URDF dir looking for a directory named `name`.
        // Preference order: (1) a `name` dir containing a `package.xml`
        // (canonical ROS package layout); (2) any `name` dir where the
        // requested `tail` file actually exists (handles loose URDF
        // drops that don't ship a ROS manifest — e.g. `husky_description/`
        // repackaged under a user dir).
        let mut cur = self.urdf_dir.as_path();
        let mut fallback: Option<PathBuf> = None;
        loop {
            let candidate = cur.join(name);
            if candidate.join("package.xml").is_file() {
                self.cache
                    .borrow_mut()
                    .insert(name.to_string(), candidate.clone());
                return Some(candidate);
            }
            if fallback.is_none()
                && candidate.is_dir()
                && candidate.join(tail).is_file()
            {
                fallback = Some(candidate);
            }
            match cur.parent() {
                Some(p) => cur = p,
                None => break,
            }
        }
        if let Some(p) = fallback {
            self.cache.borrow_mut().insert(name.to_string(), p.clone());
            return Some(p);
        }
        None
    }
}
