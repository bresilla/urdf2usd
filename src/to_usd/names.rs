//! Name sanitization + dedup for USD prim names.
//!
//! USD prim names must match `[a-zA-Z_][a-zA-Z0-9_]*`. URDF is looser.
//! We also dedupe within each namespace scope (global + per-parent).

use std::collections::{HashMap, HashSet};

#[derive(Default)]
pub struct NameCache {
    global: HashSet<String>,
    per_scope: HashMap<String, HashSet<String>>,
}

impl NameCache {
    pub fn new() -> Self {
        Self::default()
    }

    /// Claim a unique name at the global (stage-root) level.
    pub fn claim_global(&mut self, raw: &str) -> String {
        let base = sanitize(raw);
        let unique = uniquify(&base, &self.global);
        self.global.insert(unique.clone());
        unique
    }

    /// Claim a unique name under a given scope (e.g. a parent prim path).
    pub fn claim(&mut self, scope: &str, raw: &str) -> String {
        let base = sanitize(raw);
        let set = self.per_scope.entry(scope.to_string()).or_default();
        let unique = uniquify(&base, set);
        set.insert(unique.clone());
        unique
    }
}

fn uniquify(base: &str, taken: &HashSet<String>) -> String {
    if !taken.contains(base) {
        return base.to_string();
    }
    let mut i = 1u32;
    loop {
        let cand = format!("{base}_{i}");
        if !taken.contains(&cand) {
            return cand;
        }
        i += 1;
    }
}

pub fn sanitize(raw: &str) -> String {
    if raw.is_empty() {
        return "unnamed".to_string();
    }
    let mut out = String::with_capacity(raw.len() + 1);
    for ch in raw.chars() {
        if ch.is_ascii_alphanumeric() || ch == '_' {
            out.push(ch);
        } else {
            out.push('_');
        }
    }
    match out.chars().next() {
        Some(c) if c.is_ascii_alphabetic() || c == '_' => out,
        Some(_) => {
            out.insert(0, '_');
            out
        }
        None => "unnamed".to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sanitizes_punctuation() {
        assert_eq!(sanitize("base-link"), "base_link");
        assert_eq!(sanitize("arm.0"), "arm_0");
        assert_eq!(sanitize("7link"), "_7link");
        assert_eq!(sanitize(""), "unnamed");
    }

    #[test]
    fn dedupes_within_scope() {
        let mut c = NameCache::new();
        assert_eq!(c.claim("/root", "foo"), "foo");
        assert_eq!(c.claim("/root", "foo"), "foo_1");
        assert_eq!(c.claim("/root", "foo"), "foo_2");
        // Different scope resets the counter.
        assert_eq!(c.claim("/other", "foo"), "foo");
    }
}
