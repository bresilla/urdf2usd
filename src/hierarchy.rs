//! Link/joint graph on top of `urdf-rs::Robot`.
//!
//! Mirrors `_impl/link_hierarchy.py`: root-link detection + children
//! iteration + ghost-link flags + "remove rigid body" propagation along
//! chains of ghost links connected by fixed joints.

use std::collections::{HashMap, HashSet};

use crate::{Error, Result};

pub struct LinkHierarchy<'a> {
    /// Index of each link by name for O(1) lookup.
    pub link_by_name: HashMap<&'a str, &'a urdf_rs::Link>,

    /// Joints indexed by parent link name. `Vec` because a parent can have
    /// multiple children. Order preserves authored URDF order.
    pub joints_by_parent: HashMap<&'a str, Vec<&'a urdf_rs::Joint>>,

    /// Root link names. Exactly one in valid, non-cyclic URDFs.
    pub roots: Vec<&'a str>,

    /// Link names flagged as "ghost": no inertial, no visuals, no collisions.
    pub ghost_links: HashSet<&'a str>,

    /// Link names for which we should *not* author a rigid body. Populated by
    /// `mark_ghost_chains`. A link ends up here when it is a ghost link
    /// attached to its parent via a fixed joint and sits on a chain that
    /// ends in ghost links (matching Python's `_fill_all_remove_rigid_body_flag`).
    pub skip_rigid_body: HashSet<String>,
}

impl<'a> LinkHierarchy<'a> {
    pub fn build(robot: &'a urdf_rs::Robot) -> Result<Self> {
        let mut link_by_name = HashMap::new();
        for link in &robot.links {
            link_by_name.insert(link.name.as_str(), link);
        }

        let mut joints_by_parent: HashMap<&str, Vec<&urdf_rs::Joint>> = HashMap::new();
        let mut has_parent: HashSet<&str> = HashSet::new();
        for joint in &robot.joints {
            joints_by_parent
                .entry(joint.parent.link.as_str())
                .or_default()
                .push(joint);
            has_parent.insert(joint.child.link.as_str());
        }

        let mut roots: Vec<&str> = robot
            .links
            .iter()
            .map(|l| l.name.as_str())
            .filter(|n| !has_parent.contains(n))
            .collect();

        if roots.is_empty() {
            if robot.links.is_empty() {
                return Err(Error::msg("URDF has no links"));
            }
            if !robot.joints.is_empty() {
                return Err(Error::msg(
                    "closed-loop articulation: every link has a parent joint",
                ));
            }
            // No joints at all — all links are implicitly roots.
            roots.push(robot.links[0].name.as_str());
        }

        let ghost_links: HashSet<&str> = robot
            .links
            .iter()
            .filter(|l| is_ghost(l))
            .map(|l| l.name.as_str())
            .collect();

        let mut h = Self {
            link_by_name,
            joints_by_parent,
            roots,
            ghost_links,
            skip_rigid_body: HashSet::new(),
        };
        h.mark_ghost_chains(robot);
        Ok(h)
    }

    pub fn children_of(&self, link: &str) -> &[&'a urdf_rs::Joint] {
        self.joints_by_parent
            .get(link)
            .map(Vec::as_slice)
            .unwrap_or(&[])
    }

    pub fn should_skip_rigid_body(&self, link: &str) -> bool {
        self.skip_rigid_body.contains(link)
    }

    /// Walk from each root link in order, returning the first link that is
    /// neither a ghost link nor suppressed by the ghost-chain rule. This is
    /// where the articulation root API should live (matches Python's
    /// "first non-ghost link during the tree walk" semantics).
    pub fn first_non_ghost_link(&self) -> Option<&'a str> {
        let mut stack: Vec<&str> = self.roots.iter().copied().collect();
        stack.reverse();
        while let Some(link) = stack.pop() {
            if !self.ghost_links.contains(link)
                && !self.skip_rigid_body.contains(link)
            {
                return Some(link);
            }
            // Push children in reverse so we visit in authored order.
            let children: Vec<&str> = self
                .joints_by_parent
                .get(link)
                .map(|v| v.iter().map(|j| j.child.link.as_str()).collect())
                .unwrap_or_default();
            for child in children.into_iter().rev() {
                stack.push(child);
            }
        }
        None
    }

    fn mark_ghost_chains(&mut self, robot: &'a urdf_rs::Robot) {
        // Links referenced by a mimic joint whose target joint is fixed must
        // keep their rigid body even if they're otherwise ghost. This preserves
        // the mimic chain's resolution semantics.
        let pinned: HashSet<&str> = robot
            .joints
            .iter()
            .filter_map(|j| {
                let m = j.mimic.as_ref()?;
                let target = robot.joints.iter().find(|t| t.name == m.joint)?;
                if target.joint_type == urdf_rs::JointType::Fixed {
                    Some(target.child.link.as_str())
                } else {
                    None
                }
            })
            .collect();

        // `belongs_to_fixed_joint[link] = parent joint is fixed`.
        let mut via_fixed: HashSet<&str> = HashSet::new();
        for j in &robot.joints {
            if j.joint_type == urdf_rs::JointType::Fixed {
                via_fixed.insert(j.child.link.as_str());
            }
        }

        // For each non-root link, walk its subtree. If the link and every
        // descendant satisfies (ghost && attached-by-fixed && not pinned),
        // the entire subtree gets its rigid bodies suppressed.
        let roots = self.roots.clone();
        for root in roots {
            self.walk_and_mark(root, &via_fixed, &pinned);
        }
    }

    fn walk_and_mark(
        &mut self,
        link: &'a str,
        via_fixed: &HashSet<&str>,
        pinned: &HashSet<&str>,
    ) {
        if self.all_ghost_chain(link, via_fixed, pinned) {
            self.fill_subtree(link);
            return;
        }
        for joint in self
            .joints_by_parent
            .get(link)
            .cloned()
            .unwrap_or_default()
        {
            self.walk_and_mark(joint.child.link.as_str(), via_fixed, pinned);
        }
    }

    fn all_ghost_chain(
        &self,
        link: &str,
        via_fixed: &HashSet<&str>,
        pinned: &HashSet<&str>,
    ) -> bool {
        if !self.ghost_links.contains(link)
            || !via_fixed.contains(link)
            || pinned.contains(link)
        {
            return false;
        }
        let children = self
            .joints_by_parent
            .get(link)
            .map(Vec::as_slice)
            .unwrap_or(&[]);
        for j in children {
            if !self.all_ghost_chain(j.child.link.as_str(), via_fixed, pinned) {
                return false;
            }
        }
        true
    }

    fn fill_subtree(&mut self, link: &str) {
        self.skip_rigid_body.insert(link.to_string());
        let children: Vec<String> = self
            .joints_by_parent
            .get(link)
            .map(|v| v.iter().map(|j| j.child.link.clone()).collect())
            .unwrap_or_default();
        for c in children {
            self.fill_subtree(&c);
        }
    }
}

fn is_ghost(link: &urdf_rs::Link) -> bool {
    let inertial = &link.inertial;
    let no_inertial = inertial.mass.value == 0.0
        && inertial.inertia.ixx == 0.0
        && inertial.inertia.ixy == 0.0
        && inertial.inertia.ixz == 0.0
        && inertial.inertia.iyy == 0.0
        && inertial.inertia.iyz == 0.0
        && inertial.inertia.izz == 0.0;
    no_inertial && link.visual.is_empty() && link.collision.is_empty()
}
