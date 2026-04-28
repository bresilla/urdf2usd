//! Native URDF viewer — same UI shell as the `viewer` example
//! (Frost ribbons + panels, Glacial camera/gizmos/grid/selection-ring),
//! but loads the URDF directly through `bevy_urdf` (no URDF→USD
//! conversion step). Useful for side-by-side comparing the
//! `urdf2usd` USD pipeline against the native-URDF render.
//!
//! ```
//! cargo run --example urdf_viewer -- <path/to/robot.urdf>
//! ```
//!
//! `package://name/...` URIs are resolved by walking the URDF's
//! parent chain looking for a directory whose name matches the
//! package — works for the standard `<workspace>/<pkg>/urdf/foo.urdf`
//! layout used by the `urdf_collection` repo.

use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};

use bevy::gizmos::config::{DefaultGizmoConfigGroup, GizmoConfigStore};
use bevy::prelude::*;
use bevy_egui::{
    EguiContexts, EguiFullOutput, EguiPlugin, EguiPostUpdateSet, EguiPrimaryContextPass, egui,
};
use bevy_frost::prelude::*;
use bevy_glacial::prelude::*;
use bevy_urdf::{
    JointKind, LoadRobot, PackageMap, Robot, RobotJoint, RobotLink, RobotRoot, UrdfPlugin,
};

const APP_NAME: &str = "u2u_urdf_viewer";

// ── Ribbon layout ─────────────────────────────────────────────────────
// Mirrors the USD viewer's ribbon — three icon buttons toggle the
// Joints / Overlays / Tree panes. Different ribbon ids than the USD
// viewer so the two examples can run side-by-side without ribbon
// state collisions through the persisted config.
const RIBBON_LEFT: &str = "u2urdf_ribbon_left";
const RIBBON_RIGHT: &str = "u2urdf_ribbon_right";

const MENU_JOINTS: &str = "u2urdf_menu_joints";
const MENU_OVERLAYS: &str = "u2urdf_menu_overlays";
const MENU_TREE: &str = "u2urdf_menu_tree";

const RIBBONS: &[RibbonDef] = &[
    RibbonDef {
        id: RIBBON_LEFT,
        edge: RibbonEdge::Left,
        role: RibbonRole::Panel,
        mode: RibbonMode::ThreeSided,
        draggable: true,
        accepts: &[RIBBON_RIGHT],
    },
    RibbonDef {
        id: RIBBON_RIGHT,
        edge: RibbonEdge::Right,
        role: RibbonRole::Panel,
        mode: RibbonMode::ThreeSided,
        draggable: true,
        accepts: &[RIBBON_LEFT],
    },
];

const RIBBON_ITEMS: &[RibbonItem] = &[
    RibbonItem {
        id: MENU_JOINTS,
        ribbon: RIBBON_LEFT,
        cluster: RibbonCluster::Start,
        slot: 0,
        glyph: RibbonGlyph::Icon("gauge"),
        tooltip: "Joints — sliders per movable joint",
        child_ribbon: None,
    },
    RibbonItem {
        id: MENU_OVERLAYS,
        ribbon: RIBBON_LEFT,
        cluster: RibbonCluster::Start,
        slot: 1,
        glyph: RibbonGlyph::Icon("eye"),
        tooltip: "Overlays — gizmos, world, body opacity",
        child_ribbon: None,
    },
    RibbonItem {
        id: MENU_TREE,
        ribbon: RIBBON_LEFT,
        cluster: RibbonCluster::Start,
        slot: 2,
        glyph: RibbonGlyph::Icon("list"),
        tooltip: "Tree — links + joints from the loaded URDF",
        child_ribbon: None,
    },
];

/// Build and run the native-URDF App against `urdf_path`. Caller is
/// responsible for verifying the file exists and is a URDF / XML.
/// Used by both the `urdf_viewer` example and the dispatching
/// `viewer` example.
pub fn run(urdf_path: PathBuf) {
    let stem = urdf_path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("robot")
        .to_string();

    // Walk the URDF for `package://name/...` URIs and build a
    // best-effort PackageMap by searching parent directories.
    let package_map = build_package_map(&urdf_path);
    println!("input  : {}", urdf_path.display());
    if !package_map.is_empty() {
        println!("packages:");
        for (n, p) in &package_map {
            println!("  {n} -> {}", p.display());
        }
    }

    let geometry = WindowGeometry::load(APP_NAME);

    println!("→ launching native URDF viewer (bevy_urdf + bevy_glacial + bevy_frost) …");
    App::new()
        .insert_resource(InitialUrdf(urdf_path))
        .insert_resource(InitialPackages(package_map))
        .insert_resource(JointAngles::default())
        .insert_resource(BodyMaterials::default())
        .insert_resource(Overlays::default())
        .insert_resource(EguiDebugConfigured(false))
        .insert_resource(ClearColor(Color::srgb(0.06, 0.07, 0.09)))
        .insert_resource(SelectionRing {
            anchor: None,
            outer_radius: 1.5,
            color: Color::srgba(0.45, 0.85, 1.0, 0.4),
        })
        .insert_resource(GizmoAutoScale {
            object_fraction: 0.3,
            max_pixels: 80.0,
            min_pixels: 4.0,
            fallback_world_radius: 0.5,
        })
        .add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(geometry.to_window(&format!("urdf2usd[urdf]: {stem}"))),
                    ..default()
                }),
        )
        .add_plugins(UrdfPlugin)
        .add_plugins(GlacialPlugins)
        .add_plugins(WindowSettingsPlugin::new(APP_NAME))
        .add_plugins(EguiPlugin::default())
        .add_plugins(FrostPlugin)
        .add_systems(
            Startup,
            (
                spawn_world,
                send_load_robot,
                open_default_panel,
                configure_gizmo_depth,
            ),
        )
        .add_systems(
            Update,
            (
                track_selection,
                patch_selection_ring_alpha,
                apply_joint_angles,
                apply_frame_overlays,
                apply_main_gizmo_toggle,
                apply_world_axes_toggle,
                apply_world_grid_toggle,
                auto_scale_gizmo_to_target,
                snapshot_body_materials,
                apply_body_opacity,
            ),
        )
        .add_systems(
            EguiPrimaryContextPass,
            (configure_egui_debug, draw_ribbons, draw_panels).chain(),
        )
        .add_systems(
            bevy::app::PostUpdate,
            log_egui_id_clashes
                .after(EguiPostUpdateSet::EndPass)
                .before(EguiPostUpdateSet::ProcessOutput),
        )
        .run();
}

#[derive(Resource)]
struct InitialUrdf(PathBuf);

#[derive(Resource)]
struct InitialPackages(HashMap<String, PathBuf>);

/// Per-joint slider value (radians for revolute/continuous, meters
/// for prismatic). Keyed by joint name. UI writes; `apply_joint_angles`
/// pushes into `Robot.chain` via `set_joint_position`.
#[derive(Resource, Default)]
struct JointAngles {
    values: HashMap<String, f32>,
}

#[derive(Resource, Default)]
struct BodyMaterials {
    originals: HashMap<bevy::asset::AssetId<StandardMaterial>, (Color, AlphaMode)>,
    last_applied: Option<f32>,
}

#[derive(Resource)]
struct Overlays {
    show_link_frames: bool,
    show_joint_frames: bool,
    show_main_gizmo: bool,
    show_world_axes: bool,
    show_world_grid: bool,
    show_selection_ring: bool,
    body_opacity: f32,
}

impl Default for Overlays {
    fn default() -> Self {
        Self {
            show_link_frames: false,
            show_joint_frames: false,
            show_main_gizmo: true,
            show_world_axes: true,
            show_world_grid: true,
            show_selection_ring: true,
            body_opacity: 1.0,
        }
    }
}

#[derive(Resource)]
struct EguiDebugConfigured(bool);

#[derive(Component)]
struct FrameOverlay;

#[derive(Component)]
struct WorldAxisGizmo;

fn spawn_world(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        ChaseCamera {
            focus: Vec3::ZERO,
            distance: 4.0,
            yaw: 35f32.to_radians(),
            elevation: 25f32.to_radians(),
            ..default()
        },
        GizmoCamera,
    ));

    commands.spawn((
        Transform::from_xyz(2.0, 0.0, 0.0),
        GlobalTransform::default(),
        AxisGizmo::new(0.4),
        WorldAxisGizmo,
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 6000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 3.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

fn send_load_robot(
    mut commands: Commands,
    urdf: Res<InitialUrdf>,
    pkgs: Res<InitialPackages>,
    mut writer: MessageWriter<LoadRobot>,
) {
    // Spawn a root entity carrying the GizmoTarget + a marker so the
    // selection ring can track it. The URDF loader will attach the
    // Robot component + child link entities to this entity.
    let root = commands
        .spawn((
            Transform::default(),
            GlobalTransform::default(),
            Visibility::default(),
            GizmoTarget::default(),
            UrdfRoot,
        ))
        .id();

    // Pre-populate the PackageMap before LoadRobot fires so the mesh
    // loader's `package://` resolution finds every referenced package.
    let mut map = PackageMap::new();
    for (name, root) in &pkgs.0 {
        map.insert(name.clone(), root.clone());
    }
    commands.insert_resource(map);

    writer.write(LoadRobot {
        path: urdf.0.clone(),
        root: Some(root),
    });
}

/// Marker on the entity we hand to bevy_urdf's `LoadRobot`. Equivalent
/// to the `LoadedRobot` marker in the USD viewer; lets the selection-
/// ring tracker + main-gizmo toggle find the spawn point.
#[derive(Component)]
struct UrdfRoot;

fn open_default_panel(mut open: ResMut<RibbonOpen>) {
    open.toggle(RIBBON_LEFT, MENU_JOINTS);
}

fn configure_gizmo_depth(mut store: ResMut<GizmoConfigStore>) {
    let (cfg, _) = store.config_mut::<DefaultGizmoConfigGroup>();
    cfg.depth_bias = -1.0;
}

fn track_selection(
    overlays: Res<Overlays>,
    robots: Query<&GlobalTransform, With<UrdfRoot>>,
    mut ring: ResMut<SelectionRing>,
) {
    if !overlays.show_selection_ring {
        ring.anchor = None;
        return;
    }
    if let Ok(tf) = robots.single() {
        ring.anchor = Some(tf.translation());
    } else {
        ring.anchor = None;
    }
}

fn patch_selection_ring_alpha(
    ring: Query<&SelectionRingEntity>,
    mut materials: ResMut<Assets<SelectionRingMaterial>>,
) {
    let Ok(rk) = ring.single() else {
        return;
    };
    if let Some(mat) = materials.get_mut(&rk.material) {
        mat.extension.alpha = 0.1;
    }
}

/// Drive `Robot.chain` joint positions from the slider state. `k::Chain`
/// uses interior mutability — `set_joint_position` takes `&self` and
/// uses `RefCell` internally — so we can iterate the chain without
/// `&mut Robot`. The kinematics plugin's `sync_fk_to_transforms` runs
/// in `PostUpdate` and turns the new joint state into per-link
/// `Transform`s.
fn apply_joint_angles(
    robots: Query<&Robot, With<RobotRoot>>,
    angles: Res<JointAngles>,
) {
    for robot in robots.iter() {
        for node in robot.chain.iter() {
            let joint_name = node.joint().name.clone();
            if let Some(angle) = angles.values.get(&joint_name).copied() {
                let _ = node.set_joint_position(angle);
            }
        }
    }
}

fn apply_frame_overlays(
    mut commands: Commands,
    overlays: Res<Overlays>,
    existing: Query<Entity, With<FrameOverlay>>,
    links: Query<Entity, With<RobotLink>>,
    joints: Query<Entity, With<RobotJoint>>,
) {
    let want_link = overlays.show_link_frames;
    let want_joint = overlays.show_joint_frames;

    if !want_link && !want_joint {
        for e in existing.iter() {
            commands.entity(e).despawn();
        }
        return;
    }

    if !existing.is_empty() {
        return;
    }

    if want_link {
        for entity in links.iter() {
            commands.entity(entity).with_children(|c| {
                c.spawn((
                    Transform::default(),
                    GlobalTransform::default(),
                    AxisGizmo::new(0.15),
                    FrameOverlay,
                ));
            });
        }
    }

    if want_joint {
        for entity in joints.iter() {
            commands.entity(entity).with_children(|c| {
                c.spawn((
                    Transform::default(),
                    GlobalTransform::default(),
                    AxisGizmo::new(0.12),
                    FrameOverlay,
                ));
            });
        }
    }
}

fn apply_main_gizmo_toggle(
    mut commands: Commands,
    overlays: Res<Overlays>,
    with_target: Query<Entity, (With<UrdfRoot>, With<GizmoTarget>)>,
    without_target: Query<Entity, (With<UrdfRoot>, Without<GizmoTarget>)>,
) {
    if overlays.show_main_gizmo {
        for e in without_target.iter() {
            commands.entity(e).insert(GizmoTarget::default());
        }
    } else {
        for e in with_target.iter() {
            commands.entity(e).remove::<GizmoTarget>();
        }
    }
}

fn apply_world_axes_toggle(
    mut commands: Commands,
    overlays: Res<Overlays>,
    with: Query<Entity, (With<WorldAxisGizmo>, With<AxisGizmo>)>,
    without: Query<Entity, (With<WorldAxisGizmo>, Without<AxisGizmo>)>,
) {
    if overlays.show_world_axes {
        for e in without.iter() {
            commands.entity(e).insert(AxisGizmo::new(0.4));
        }
    } else {
        for e in with.iter() {
            commands.entity(e).remove::<AxisGizmo>();
        }
    }
}

fn apply_world_grid_toggle(overlays: Res<Overlays>, mut grid: ResMut<GroundGrid>) {
    if grid.visible != overlays.show_world_grid {
        grid.visible = overlays.show_world_grid;
    }
}

fn snapshot_body_materials(
    materials: Res<Assets<StandardMaterial>>,
    handles: Query<&MeshMaterial3d<StandardMaterial>>,
    mut cache: ResMut<BodyMaterials>,
) {
    for h in handles.iter() {
        let id = h.0.id();
        if cache.originals.contains_key(&id) {
            continue;
        }
        if let Some(m) = materials.get(&h.0) {
            cache.originals.insert(id, (m.base_color, m.alpha_mode));
        }
    }
}

fn apply_body_opacity(
    overlays: Res<Overlays>,
    mut cache: ResMut<BodyMaterials>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let opacity = overlays.body_opacity.clamp(0.0, 1.0);
    if let Some(prev) = cache.last_applied {
        if (prev - opacity).abs() < 1e-4 {
            return;
        }
    }
    let restore = (opacity - 1.0).abs() < 1e-4;
    let originals: Vec<(bevy::asset::AssetId<StandardMaterial>, Color, AlphaMode)> = cache
        .originals
        .iter()
        .map(|(id, (c, m))| (*id, *c, *m))
        .collect();
    for (id, original_color, original_mode) in originals {
        let Some(m) = materials.get_mut(id) else {
            continue;
        };
        let mut linear = original_color.to_linear();
        linear.alpha *= opacity;
        m.base_color = linear.into();
        m.alpha_mode = if restore { original_mode } else { AlphaMode::Blend };
    }
    cache.last_applied = Some(opacity);
}

fn configure_egui_debug(mut contexts: EguiContexts, mut done: ResMut<EguiDebugConfigured>) {
    if done.0 {
        return;
    }
    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };
    ctx.options_mut(|o| {
        o.warn_on_id_clash = true;
    });
    done.0 = true;
}

fn log_egui_id_clashes(
    out: Query<&EguiFullOutput>,
    mut last_seen: Local<HashMap<String, u32>>,
) {
    let mut found_this_frame: HashMap<String, u32> = HashMap::new();
    for full in out.iter() {
        let Some(fo) = &full.0 else {
            continue;
        };
        for clipped in &fo.shapes {
            scan_shape(&clipped.shape, &mut found_this_frame);
        }
    }
    for (text, count) in &found_this_frame {
        let prev = last_seen.get(text).copied().unwrap_or(0);
        if prev == 0 {
            warn!("egui clash painted: {text} (this frame: {count})");
        }
        last_seen.insert(text.clone(), prev + count);
    }
}

fn scan_shape(shape: &egui::epaint::Shape, out: &mut HashMap<String, u32>) {
    use egui::epaint::Shape;
    match shape {
        Shape::Text(t) => {
            let s = t.galley.text();
            if s.contains("use of") && s.contains("ID") {
                let pos = t.pos;
                let key = format!("{} @ ({:.0},{:.0})", s, pos.x, pos.y);
                *out.entry(key).or_insert(0) += 1;
            }
        }
        Shape::Vec(v) => {
            for s in v {
                scan_shape(s, out);
            }
        }
        _ => {}
    }
}

fn pseudo_rand(seed: &str) -> f32 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in seed.as_bytes() {
        h = h.wrapping_mul(0x100_0000_01b3);
        h ^= *b as u64;
    }
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0);
    h ^= now.rotate_left(13);
    h = h.wrapping_mul(0x100_0000_01b3);
    let frac = ((h >> 32) & 0x00ff_ffff) as f32 / 0x0100_0000 as f32;
    frac.clamp(0.0, 1.0)
}

fn draw_ribbons(
    mut contexts: EguiContexts,
    accent: Res<AccentColor>,
    mut open: ResMut<RibbonOpen>,
    mut placement: ResMut<RibbonPlacement>,
    mut drag: ResMut<RibbonDrag>,
) {
    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };
    let _ = draw_assembly(
        ctx,
        accent.0,
        RIBBONS,
        RIBBON_ITEMS,
        &mut open,
        &mut placement,
        &mut drag,
        |_| false,
    );
}

fn draw_panels(
    mut contexts: EguiContexts,
    mut overlays: ResMut<Overlays>,
    mut angles: ResMut<JointAngles>,
    accent: Res<AccentColor>,
    open: Res<RibbonOpen>,
    placement: Res<RibbonPlacement>,
    joints_q: Query<&RobotJoint>,
    links_q: Query<&RobotLink>,
) {
    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };
    let accent_color: egui::Color32 = accent.0;
    let is_open = |id: &'static str| -> bool {
        let Some(item) = find_item(RIBBON_ITEMS, id) else {
            return false;
        };
        let (rid, _, _) = placement.resolve(item);
        open.is_open(rid, id)
    };
    let mut keep_open = true;

    // Collect joints once so we can sort + paint in a stable order.
    let mut joints: Vec<RobotJoint> = joints_q.iter().cloned().collect();
    joints.sort_by_key(|j| j.joint_index);

    if is_open(MENU_JOINTS) {
        floating_window_for_item(
            ctx,
            RIBBONS,
            RIBBON_ITEMS,
            &placement,
            MENU_JOINTS,
            "Joints",
            egui::vec2(320.0, 460.0),
            &mut keep_open,
            accent_color,
            |pane| {
                pane.section("movable_joints", "Movable joints", true, |ui| {
                    if joints.is_empty() {
                        ui.label("No joints loaded yet.");
                        return;
                    }
                    let mut shown = 0usize;
                    for (i, j) in joints.iter().enumerate() {
                        if !matches!(
                            j.kind,
                            JointKind::Revolute | JointKind::Continuous | JointKind::Prismatic
                        ) {
                            continue;
                        }
                        shown += 1;
                        let val_entry = angles.values.entry(j.name.clone()).or_insert(0.0);
                        let mut val = *val_entry as f64;
                        let (lo, hi, suffix, decimals) = match j.kind {
                            JointKind::Revolute | JointKind::Continuous => {
                                let (lo, hi) = j
                                    .limits
                                    .map(|(a, b)| (a as f64, b as f64))
                                    .unwrap_or((
                                        -std::f32::consts::PI as f64,
                                        std::f32::consts::PI as f64,
                                    ));
                                (lo, hi, " rad", 3)
                            }
                            JointKind::Prismatic => {
                                let (lo, hi) = j
                                    .limits
                                    .map(|(a, b)| (a as f64, b as f64))
                                    .unwrap_or((-1.0, 1.0));
                                (lo, hi, " m", 3)
                            }
                            _ => unreachable!(),
                        };
                        ui.push_id(("jslider", i), |ui| {
                            pretty_slider(
                                ui,
                                &j.name,
                                &mut val,
                                lo..=hi,
                                decimals,
                                suffix,
                                accent_color,
                            );
                        });
                        *val_entry = val as f32;
                    }
                    if shown == 0 {
                        ui.label(
                            "(only fixed / floating / planar joints — nothing to drive)",
                        );
                    }
                });
                pane.section("actions", "Actions", true, |ui| {
                    ui.horizontal(|ui| {
                        if ui.button("Reset all to 0").clicked() {
                            for v in angles.values.values_mut() {
                                *v = 0.0;
                            }
                        }
                        if ui.button("Randomize").clicked() {
                            for j in &joints {
                                if !matches!(
                                    j.kind,
                                    JointKind::Revolute
                                        | JointKind::Continuous
                                        | JointKind::Prismatic
                                ) {
                                    continue;
                                }
                                let (lo, hi) = match j.kind {
                                    JointKind::Revolute | JointKind::Continuous => j
                                        .limits
                                        .unwrap_or((
                                            -std::f32::consts::PI,
                                            std::f32::consts::PI,
                                        )),
                                    JointKind::Prismatic => j.limits.unwrap_or((-1.0, 1.0)),
                                    _ => continue,
                                };
                                let t = pseudo_rand(&j.name);
                                angles.values.insert(j.name.clone(), lo + t * (hi - lo));
                            }
                        }
                    });
                });
            },
        );
    }

    if is_open(MENU_OVERLAYS) {
        floating_window_for_item(
            ctx,
            RIBBONS,
            RIBBON_ITEMS,
            &placement,
            MENU_OVERLAYS,
            "Overlays",
            egui::vec2(280.0, 240.0),
            &mut keep_open,
            accent_color,
            |pane| {
                pane.section("gizmos", "Gizmos", true, |ui| {
                    toggle(
                        ui,
                        "Main gizmo",
                        &mut overlays.show_main_gizmo,
                        accent_color,
                    );
                    toggle(
                        ui,
                        "Selection ring",
                        &mut overlays.show_selection_ring,
                        accent_color,
                    );
                    toggle(
                        ui,
                        "Link frames",
                        &mut overlays.show_link_frames,
                        accent_color,
                    );
                    toggle(
                        ui,
                        "Joint frames",
                        &mut overlays.show_joint_frames,
                        accent_color,
                    );
                });
                pane.section("world", "World", true, |ui| {
                    toggle(
                        ui,
                        "World axes",
                        &mut overlays.show_world_axes,
                        accent_color,
                    );
                    toggle(
                        ui,
                        "Ground grid",
                        &mut overlays.show_world_grid,
                        accent_color,
                    );
                });
                pane.section("body", "Body", true, |ui| {
                    let mut op = overlays.body_opacity as f64;
                    pretty_slider(ui, "Opacity", &mut op, 0.0..=1.0, 2, "", accent_color);
                    overlays.body_opacity = op as f32;
                });
            },
        );
    }

    if is_open(MENU_TREE) {
        floating_window_for_item(
            ctx,
            RIBBONS,
            RIBBON_ITEMS,
            &placement,
            MENU_TREE,
            "Tree",
            egui::vec2(320.0, 380.0),
            &mut keep_open,
            accent_color,
            |pane| {
                pane.section("links", "Links", true, |ui| {
                    let mut links: Vec<&RobotLink> = links_q.iter().collect();
                    links.sort_by_key(|l| l.link_index);
                    if links.is_empty() {
                        ui.label("No links loaded yet.");
                        return;
                    }
                    for (i, l) in links.iter().enumerate() {
                        ui.push_id(("link", i), |ui| {
                            ui.label(l.name.as_str());
                        });
                    }
                });
                pane.section("joints_list", "Joints", true, |ui| {
                    if joints.is_empty() {
                        ui.label("No joints loaded yet.");
                        return;
                    }
                    for (i, j) in joints.iter().enumerate() {
                        ui.push_id(("jt", i), |ui| {
                            ui.label(format!("{}  ({:?})", j.name, j.kind));
                        });
                    }
                });
            },
        );
    }
}

// ── Package map heuristic ────────────────────────────────────────────
//
// Scan the URDF text for `package://name/...` occurrences; for each
// distinct name walk up from the URDF directory looking for a sibling
// directory named that. Works for the standard
// `<workspace>/<pkg>/urdf/foo.urdf` layout (Husky, UR, etc.) where
// `<pkg>` literally matches the package name in the URI.
fn build_package_map(urdf_path: &Path) -> HashMap<String, PathBuf> {
    let Ok(text) = std::fs::read_to_string(urdf_path) else {
        return HashMap::new();
    };
    let mut packages: HashSet<String> = HashSet::new();
    let mut cursor = 0usize;
    while let Some(idx) = text[cursor..].find("package://") {
        let start = cursor + idx + "package://".len();
        let end = text[start..]
            .find(|c: char| c == '/' || c == '"' || c == '\'' || c.is_whitespace())
            .map(|n| start + n)
            .unwrap_or(text.len());
        if start < end {
            packages.insert(text[start..end].to_string());
        }
        cursor = end;
    }

    let mut map = HashMap::new();
    let mut search_root = urdf_path.parent().unwrap_or(Path::new(".")).to_path_buf();
    for _ in 0..6 {
        let parent = match search_root.parent() {
            Some(p) => p.to_path_buf(),
            None => break,
        };
        for pkg in &packages {
            if map.contains_key(pkg) {
                continue;
            }
            let candidate = parent.join(pkg);
            if candidate.is_dir() {
                map.insert(pkg.clone(), candidate);
            } else if search_root.file_name().map(|n| n.to_string_lossy() == *pkg)
                .unwrap_or(false)
            {
                map.insert(pkg.clone(), search_root.clone());
            }
        }
        if map.len() == packages.len() {
            break;
        }
        search_root = parent;
    }
    map
}
