//! USD → Bevy viewer with a Frost glass UI on top of the
//! `bevy_openusd` loader and the world chrome from `bevy_glacial`.
//!
//! ```
//! cargo run --example viewer -- <path/to/robot.usda>
//! ```
//!
//! USD-only: the input must be a `.usda` / `.usdc` / `.usd` file. Use
//! the `u2u` CLI (`cargo run -p urdf2usd-cli -- to-usd …`) ahead of
//! time to convert URDFs into the layered USD format this viewer
//! consumes — typically into `../usd_collection/`. For native URDF
//! rendering (no conversion) use the `urdf_viewer` example instead.
//!
//! Pipeline:
//! 1. `bevy_openusd::UsdLoader` opens the asset and projects every
//!    composed prim into a Bevy entity (each tagged with `UsdPrimRef`).
//! 2. `bevy_glacial::GlacialPlugins` provides:
//!     - [`ChaseCamera`] — mouse pan / orbit / zoom.
//!     - [`GroundGridPlugin`] — LOD ground grid following the camera.
//!     - [`AxisGizmoPlugin`] — R/G/B arrow triads (world origin + per
//!       link/joint frame when the overlay is enabled).
//!     - [`TransformGizmoPlugin`] — translate / rotate / scale handles
//!       on `GizmoTarget` (the loaded scene root).
//!     - [`SelectionRingPlugin`] — animated ground ring driven by the
//!       [`SelectionRing`] resource.
//!     - [`WindowSettingsPlugin`] — persists window size + position to
//!       `~/.config/u2u_viewer/window.txt`.
//! 4. `bevy_frost::FrostPlugin` provides the glass-themed editor UI.
//!    Three ribbon icon-buttons (left rail) toggle the Joints,
//!    Overlays, and Tree panes — same pattern as the Frost demo
//!    example (`draw_assembly` + `floating_window_for_item`). Drag
//!    a button across rails and its panel re-anchors automatically.
//!
//! Mouse navigation (from `bevy_glacial::camera`):
//! - Scroll              → zoom (logarithmic)
//! - Middle drag         → pan in world XZ
//! - Middle + L/R drag   → lift focus along world Y
//! - Left + Right drag   → orbit
//! - Double middle-click → snap focus to cursor's world point

use std::collections::HashMap;
use std::path::PathBuf;

use bevy::asset::LoadState;
use bevy::gizmos::config::{DefaultGizmoConfigGroup, GizmoConfigStore};
use bevy::prelude::*;
use bevy_egui::{
    EguiContexts, EguiFullOutput, EguiPlugin, EguiPostUpdateSet, EguiPrimaryContextPass, egui,
};
use bevy_frost::prelude::*;
use bevy_glacial::prelude::*;
use bevy_openusd::{UsdAsset, UsdLoaderSettings, UsdPlugin, UsdPrimRef};
use usd_schemas::physics::{JointKind, ReadJoint};

const APP_NAME: &str = "u2u_viewer";

// ── Ribbon layout ─────────────────────────────────────────────────────
// Two side rails (LEFT / RIGHT). Each rail is `ThreeSided` (Start /
// Middle / End clusters) so buttons can be dragged anywhere on either
// rail. The Joints / Overlays / Tree buttons start on the LEFT rail's
// Start cluster.
const RIBBON_LEFT: &str = "u2u_ribbon_left";
const RIBBON_RIGHT: &str = "u2u_ribbon_right";

const MENU_JOINTS: &str = "u2u_menu_joints";
const MENU_OVERLAYS: &str = "u2u_menu_overlays";
const MENU_TREE: &str = "u2u_menu_tree";

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
        tooltip: "Overlays — link / joint frame triads, body opacity",
        child_ribbon: None,
    },
    RibbonItem {
        id: MENU_TREE,
        ribbon: RIBBON_LEFT,
        cluster: RibbonCluster::Start,
        slot: 2,
        glyph: RibbonGlyph::Icon("list"),
        tooltip: "Tree — rigid bodies + joints from the loaded asset",
        child_ribbon: None,
    },
];

/// Build and run the USD-loading App against `input_path`. Caller is
/// responsible for verifying the file exists and has a USD extension.
/// Used both by the `usd_viewer` example (single-purpose) and by the
/// dispatching `viewer` example (auto-routes URDF vs USD).
pub fn run(input_path: PathBuf) {
    let ext = input_path
        .extension()
        .and_then(|s| s.to_str())
        .map(|s| s.to_ascii_lowercase())
        .unwrap_or_default();
    if !matches!(ext.as_str(), "usda" | "usdc" | "usd") {
        eprintln!(
            "error: input must be a USD file (.usda / .usdc / .usd) — got `.{ext}`."
        );
        std::process::exit(1);
    }

    // bevy_openusd loads relative to the AssetPlugin's `file_path`. We
    // anchor the asset root at the USD's parent directory and feed
    // just the basename to the loader, so sublayer references like
    // `./Geometry.usda` resolve correctly against the same directory.
    let asset_root = input_path
        .parent()
        .expect("usd file has a parent")
        .to_path_buf();
    let usd_rel = PathBuf::from(
        input_path
            .file_name()
            .expect("usd file has a filename"),
    );
    let stem = input_path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("scene")
        .to_string();

    // Sublayer fallback search path — bevy_openusd's loader stages
    // the top file through a tempfile, so feed the real output dir to
    // openusd's resolver as well.
    let search_paths = vec![asset_root.clone()];

    let geometry = WindowGeometry::load(APP_NAME);

    println!("input  : {}", input_path.display());
    println!("→ launching viewer (bevy_openusd + bevy_glacial + bevy_frost) …");
    App::new()
        .insert_resource(LoadedUsd(usd_rel))
        .insert_resource(LoaderSearchPaths(search_paths))
        .insert_resource(SpawnedScene::default())
        .insert_resource(RobotJoints::default())
        .insert_resource(JointAngles::default())
        .insert_resource(LinkRest::default())
        .insert_resource(BodyMaterials::default())
        .insert_resource(Overlays::default())
        .insert_resource(ClearColor(Color::srgb(0.06, 0.07, 0.09)))
        // 60% transparent (alpha=0.4) so the ring reads as a soft halo
        // rather than a saturated band over the geometry behind it.
        .insert_resource(SelectionRing {
            anchor: None,
            outer_radius: 1.5,
            color: Color::srgba(0.45, 0.85, 1.0, 0.4),
        })
        .add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(geometry.to_window(&format!("urdf2usd: {stem}"))),
                    ..default()
                })
                .set(AssetPlugin {
                    file_path: asset_root.to_string_lossy().into_owned(),
                    ..default()
                }),
        )
        .add_plugins(UsdPlugin)
        .add_plugins(GlacialPlugins)
        .add_plugins(WindowSettingsPlugin::new(APP_NAME))
        .add_plugins(EguiPlugin::default())
        .add_plugins(FrostPlugin)
        .insert_resource(EguiDebugConfigured(false))
        // Override `GizmoAutoScale` defaults so the transform handles
        // shrink to ~30% of the robot's apparent screen size and
        // never balloon past 80px when zoomed in. `min_pixels=4`
        // keeps the gizmo barely visible (rather than disappearing)
        // when the camera is far. `auto_scale_gizmo_to_target`
        // (registered below) reads this every frame.
        .insert_resource(GizmoAutoScale {
            object_fraction: 0.3,
            max_pixels: 80.0,
            min_pixels: 4.0,
            fallback_world_radius: 0.5,
        })
        .add_systems(
            Startup,
            (
                spawn_world,
                request_usd,
                open_default_panel,
                configure_gizmo_depth,
            ),
        )
        .add_systems(
            Update,
            (
                spawn_when_ready,
                snapshot_link_rest,
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
        // After the egui pass produces the frame's paint shapes, walk
        // them and surface any ID-clash text that egui paints to the
        // screen by also routing it through the `log` crate. Must run
        // AFTER `EndPass` (which populates `EguiFullOutput`) and
        // BEFORE `ProcessOutput` (which `take()`s it).
        .add_systems(
            bevy::app::PostUpdate,
            log_egui_id_clashes
                .after(EguiPostUpdateSet::EndPass)
                .before(EguiPostUpdateSet::ProcessOutput),
        )
        .run();
}

#[derive(Resource)]
struct LoadedUsd(PathBuf);

#[derive(Resource)]
struct LoaderSearchPaths(Vec<PathBuf>);

#[derive(Resource)]
struct StageHandle(Handle<UsdAsset>);

#[derive(Resource, Default)]
struct SpawnedScene(bool);

/// Marker on the entity holding `SceneRoot(usd.scene)` — used by
/// `track_selection` to anchor the selection ring + by future systems
/// that want to look up "the loaded model" without iterating every
/// `SceneRoot`.
#[derive(Component)]
struct LoadedRobot;

/// Cached joint metadata pulled out of `UsdAsset.joints` once the asset
/// finishes loading. UI sliders read this for kind / axis / limits;
/// `apply_joint_angles` reads it to drive the body1 link transform.
#[derive(Resource, Default)]
struct RobotJoints {
    joints: Vec<ReadJoint>,
    rigid_body_paths: Vec<String>,
}

/// Per-joint angle (radians for revolute, meters for prismatic). Keyed
/// by joint prim path. UI writes, `apply_joint_angles` reads.
#[derive(Resource, Default)]
struct JointAngles {
    values: HashMap<String, f32>,
}

/// Snapshot of each link entity's local Transform at load time. Joint
/// motion is applied as `rest * joint_displacement` so dragging a slider
/// to zero returns the link to its authored pose.
#[derive(Resource, Default)]
struct LinkRest {
    rest: HashMap<String, Transform>,
    snapshotted: bool,
}

/// Cached "originals" for every `StandardMaterial` used by the loaded
/// scene's body. We snapshot the authored `base_color` and
/// `alpha_mode` once on load, then `apply_body_opacity` patches each
/// material's alpha to `original.alpha * overlays.body_opacity`. When
/// the slider is back at 1.0 we restore the original `alpha_mode`
/// (Opaque / Mask) so opaque materials don't keep paying for blend
/// composition. Keyed by `AssetId` so we patch each material exactly
/// once even if many meshes reference the same handle.
#[derive(Resource, Default)]
struct BodyMaterials {
    originals: HashMap<bevy::asset::AssetId<StandardMaterial>, (Color, AlphaMode)>,
    last_applied: Option<f32>,
}

/// Toggle state for overlay rendering.
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

/// Marker on the AxisGizmo entities we spawn for link/joint frame
/// overlays so we can despawn them when toggled off.
#[derive(Component)]
struct FrameOverlay;

/// Marker on the world-origin / off-axis triad spawned in `spawn_world`,
/// so we can toggle its visibility from the Overlays pane.
#[derive(Component)]
struct WorldAxisGizmo;

#[derive(Resource)]
struct EguiDebugConfigured(bool);

/// Reads `EguiFullOutput` after the egui pass and forwards the on-
/// screen "First/Second/Double use of {what} ID {id}" warnings (which
/// egui only paints, never logs) into the `log` crate. Also prints a
/// once-per-startup summary of every Text shape we observed, so if
/// the walker never sees the collision strings we still know whether
/// it can read the painted output at all.
fn log_egui_id_clashes(
    out: Query<&EguiFullOutput>,
    mut last_seen: Local<HashMap<String, u32>>,
    mut frame: Local<u32>,
    mut summary_printed: Local<bool>,
) {
    *frame += 1;
    let mut found_this_frame: HashMap<String, u32> = HashMap::new();
    let mut all_text_samples: Vec<String> = Vec::new();
    let mut total_shapes = 0usize;
    let mut total_text = 0usize;
    for full in out.iter() {
        let Some(fo) = &full.0 else {
            continue;
        };
        total_shapes += fo.shapes.len();
        for clipped in &fo.shapes {
            scan_shape_for_collisions(
                &clipped.shape,
                &mut found_this_frame,
                &mut all_text_samples,
                &mut total_text,
            );
        }
    }
    for (text, count) in &found_this_frame {
        let prev = last_seen.get(text).copied().unwrap_or(0);
        if prev == 0 {
            warn!("egui clash painted: {text} (this frame: {count})");
        }
        last_seen.insert(text.clone(), prev + count);
    }
    // After 90 frames (~1.5 s at 60Hz) print one summary so we know
    // the walker is functioning even if no clashes were found.
    if !*summary_printed && *frame == 90 {
        *summary_printed = true;
        info!(
            "egui shape walker: {} top-level shapes / {} text shapes seen this frame; \
             {} clash messages so far. First 30 text samples: {:?}",
            total_shapes,
            total_text,
            last_seen.len(),
            all_text_samples.iter().take(30).collect::<Vec<_>>(),
        );
    }
}

fn scan_shape_for_collisions(
    shape: &egui::epaint::Shape,
    clashes: &mut HashMap<String, u32>,
    samples: &mut Vec<String>,
    total_text: &mut usize,
) {
    use egui::epaint::Shape;
    match shape {
        Shape::Text(t) => {
            *total_text += 1;
            let s = t.galley.text();
            if samples.len() < 40 {
                samples.push(s.to_string());
            }
            if s.contains("use of") && s.contains("ID") {
                // The galley's screen position is where the warning
                // text was painted — i.e. right at the colliding
                // widget. Stash it inside the key so the log line
                // tells you which screen rect to look at.
                let pos = t.pos;
                let key = format!("{} @ ({:.0},{:.0})", s, pos.x, pos.y);
                *clashes.entry(key).or_insert(0) += 1;
            }
        }
        Shape::Vec(v) => {
            for s in v {
                scan_shape_for_collisions(s, clashes, samples, total_text);
            }
        }
        _ => {}
    }
}

/// One-shot egui setup. Only flips `warn_on_id_clash` (egui's debug-
/// build default; we make it explicit). The visual overlays
/// (`debug_on_hover`, `show_widget_hits`, `show_interactive_widgets`)
/// paint per-widget debug text *over* the rest of the UI and were
/// useful while chasing the ID clashes — now that they're fixed we
/// keep them OFF so the actual UI is visible. The
/// `log_egui_id_clashes` system already routes any clash text into
/// `RUST_LOG=warn` if anything regresses.
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

/// Open the Joints pane by default so the user sees the slider stack
/// the moment the asset finishes loading. `RibbonOpen::toggle` flips
/// the per-ribbon "active button" slot.
fn open_default_panel(mut open: ResMut<RibbonOpen>) {
    open.toggle(RIBBON_LEFT, MENU_JOINTS);
}

/// Set the default Bevy gizmo group's `depth_bias` so every gizmo
/// drawn through the immediate-mode `Gizmos` API (the `AxisGizmo`
/// triads we spawn for joint / link / world-origin frames) renders
/// **on top of** every mesh, regardless of depth. Without this the
/// per-link triads disappear inside the wheel cylinders. The
/// transform-gizmo handles use a separate render pipeline (their own
/// material with `always_on_top`), so they're unaffected by this
/// setting.
fn configure_gizmo_depth(mut store: ResMut<GizmoConfigStore>) {
    let (cfg, _) = store.config_mut::<DefaultGizmoConfigGroup>();
    cfg.depth_bias = -1.0;
}

/// Add or remove `GizmoTarget` on the loaded robot based on the
/// Overlays toggle. With the component absent the transform-gizmo
/// renderer skips that entity entirely — handles disappear.
fn apply_main_gizmo_toggle(
    mut commands: Commands,
    overlays: Res<Overlays>,
    with_target: Query<Entity, (With<LoadedRobot>, With<GizmoTarget>)>,
    without_target: Query<Entity, (With<LoadedRobot>, Without<GizmoTarget>)>,
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

/// Toggle the world-origin axis triad's visibility. Hides the triad
/// + the entity itself by setting its `Visibility`. Bevy's gizmos
/// API doesn't directly read `Visibility`, but the `AxisGizmoPlugin`
/// queries by component, so we instead add/remove the `AxisGizmo`
/// component when toggled.
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

/// Drive the `GroundGrid.visible` flag from the Overlays toggle.
fn apply_world_grid_toggle(overlays: Res<Overlays>, mut grid: ResMut<GroundGrid>) {
    if grid.visible != overlays.show_world_grid {
        grid.visible = overlays.show_world_grid;
    }
}

/// Walk every mesh that came in with the loaded asset and snapshot
/// its `StandardMaterial`'s authored `base_color` + `alpha_mode`. We
/// run every frame but no-op once a material is already cached;
/// new materials added later (e.g. lazy-loaded textures) get picked
/// up automatically.
fn snapshot_body_materials(
    spawned: Res<SpawnedScene>,
    materials: Res<Assets<StandardMaterial>>,
    handles: Query<&MeshMaterial3d<StandardMaterial>>,
    mut cache: ResMut<BodyMaterials>,
) {
    if !spawned.0 {
        return;
    }
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

/// Apply the Overlays opacity slider to every body material. We
/// short-circuit when the slider hasn't changed (cheap epsilon
/// compare) so the system isn't poking material assets every frame
/// at idle. When the slider is at 1.0 we restore each material's
/// authored `base_color` + `alpha_mode` exactly — no residual blend
/// cost on opaque materials.
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
    // Snapshot the keys we want to mutate up front, so we can borrow
    // `cache.originals` while iterating without conflicting with the
    // mutable `materials` borrow.
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
        m.alpha_mode = if restore {
            original_mode
        } else {
            AlphaMode::Blend
        };
    }
    cache.last_applied = Some(opacity);
}

/// bevy_glacial's `update_selection_ring` hardcodes the ring
/// material's `alpha = 0.9` every frame, ignoring the alpha channel
/// of `SelectionRing.color`. We run AFTER that system (no
/// `.before`/`.after` constraint needed because Bevy's parallel
/// scheduler can interleave them, but the ECS resource graph still
/// resolves writes in submit order — our system reads the same
/// material through `Assets<SelectionRingMaterial>` and overwrites
/// alpha last). Net effect: selection ring renders at 40% alpha
/// (60% transparent), matching what the user asked for.
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

    // World axes — off-center to avoid overlapping the loaded robot at origin.
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

fn request_usd(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    usd: Res<LoadedUsd>,
    search: Res<LoaderSearchPaths>,
) {
    let path = usd.0.to_string_lossy().into_owned();
    let search_paths = search.0.clone();
    let handle: Handle<UsdAsset> =
        asset_server.load_with_settings::<UsdAsset, _>(path, move |s: &mut UsdLoaderSettings| {
            s.search_paths = search_paths.clone();
        });
    commands.insert_resource(StageHandle(handle));
}

fn spawn_when_ready(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    assets: Res<Assets<UsdAsset>>,
    stage: Option<Res<StageHandle>>,
    mut spawned: ResMut<SpawnedScene>,
    mut robot: ResMut<RobotJoints>,
    mut angles: ResMut<JointAngles>,
) {
    if spawned.0 {
        return;
    }
    let Some(stage) = stage else {
        return;
    };
    match asset_server.get_load_state(&stage.0) {
        Some(LoadState::Loaded) => {
            let Some(asset) = assets.get(&stage.0) else {
                return;
            };
            info!(
                "loaded UsdAsset: default_prim={:?}, layer_count={}, joints={}, rigid_bodies={}",
                asset.default_prim,
                asset.layer_count,
                asset.joints.len(),
                asset.rigid_body_prims.len(),
            );

            // Cache joint + rigid-body metadata so the UI doesn't keep
            // borrowing the asset every frame.
            robot.joints = asset.joints.clone();
            robot.rigid_body_paths = asset.rigid_body_prims.clone();

            // Initialise every joint angle to 0. The slider midpoint
            // for a symmetric ±π revolute joint already sits at 0; for
            // asymmetric joints the user lands at the authored "zero
            // pose" rather than the slider middle.
            angles.values.clear();
            for j in &robot.joints {
                angles.values.insert(j.path.clone(), 0.0);
            }

            commands.spawn((
                SceneRoot(asset.scene.clone()),
                Transform::default(),
                LoadedRobot,
                GizmoTarget::default(),
            ));
            spawned.0 = true;
        }
        Some(LoadState::Failed(err)) => {
            error!("UsdAsset load failed: {err}");
            spawned.0 = true;
        }
        _ => {}
    }
}

/// Capture each link entity's local Transform exactly once, the first
/// frame after the scene spawns. `apply_joint_angles` rebuilds the
/// link's local transform as `rest * displacement` so resetting the
/// slider to zero returns the authored pose verbatim.
fn snapshot_link_rest(
    mut rest: ResMut<LinkRest>,
    spawned: Res<SpawnedScene>,
    robot: Res<RobotJoints>,
    q: Query<(&UsdPrimRef, &Transform), Without<LoadedRobot>>,
) {
    if rest.snapshotted || !spawned.0 {
        return;
    }
    if robot.rigid_body_paths.is_empty() && robot.joints.is_empty() {
        return;
    }
    // Wait until the loader has actually projected the scene — the
    // SceneRoot spawn this frame doesn't include the children yet, so
    // try-and-skip until we see at least one prim entity.
    if q.is_empty() {
        return;
    }
    for (pr, tf) in q.iter() {
        rest.rest.insert(pr.path.clone(), *tf);
    }
    rest.snapshotted = true;
    info!("snapshotted {} link rest transforms", rest.rest.len());
}

/// Each frame, anchor the SelectionRing on the loaded robot's current
/// world position so dragging it via the transform gizmo also slides
/// the ring under it. Hides the ring (`anchor = None`) whenever the
/// Overlays toggle disables it.
fn track_selection(
    overlays: Res<Overlays>,
    robots: Query<&GlobalTransform, With<LoadedRobot>>,
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

/// Cheap deterministic-but-jittered "random" in [0,1) seeded by the
/// joint's path string. The Randomize button uses this so that hits
/// land on roughly different angles per joint without pulling in
/// `rand` as a dep.
fn pseudo_rand(seed: &str) -> f32 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in seed.as_bytes() {
        h = h.wrapping_mul(0x100_0000_01b3);
        h ^= *b as u64;
    }
    // Mix the wallclock so each click yields a fresh value.
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0);
    h ^= now.rotate_left(13);
    h = h.wrapping_mul(0x100_0000_01b3);
    let frac = ((h >> 32) & 0x00ff_ffff) as f32 / 0x0100_0000 as f32;
    frac.clamp(0.0, 1.0)
}

/// Convert "X" / "Y" / "Z" tokens into the unit axis used by the
/// joint. Falls back to X when the token is missing or malformed.
fn axis_vec(axis: Option<&str>) -> Vec3 {
    match axis.unwrap_or("X") {
        "Y" => Vec3::Y,
        "Z" => Vec3::Z,
        _ => Vec3::X,
    }
}

/// Drive child link transforms from the slider state. The math:
///
/// - Joint frame in body1 = (local_pos1, local_rot1).
/// - For a revolute joint the displacement is a rotation by `angle`
///   around the joint axis, expressed in the joint frame, applied at
///   the joint frame's origin.
/// - The body1 entity's local Transform is the *rest* transform
///   composed with the displacement on the body1 side: if the URDF
///   author placed body1 such that its rest pose corresponds to
///   angle=0 of the joint, then `rest * disp_in_body1_frame` re-poses
///   the link.
/// - `disp_in_body1_frame = T(local_pos1) * R(local_rot1) * R(axis,
///   angle) * R(local_rot1)⁻¹ * T(-local_pos1)`. We apply that as
///   `rest * disp` in the parent frame because the rest already
///   encodes the body0→body1 transform.
fn apply_joint_angles(
    robot: Res<RobotJoints>,
    angles: Res<JointAngles>,
    rest: Res<LinkRest>,
    mut q: Query<(&UsdPrimRef, &mut Transform), Without<LoadedRobot>>,
) {
    if !rest.snapshotted || robot.joints.is_empty() {
        return;
    }
    // Build a path → Transform delta map first so we can apply once
    // per body1 even if multiple joints touch the same link (we win
    // the last; URDF / USD doesn't allow that anyway).
    let mut disp_by_body: HashMap<&str, Transform> = HashMap::new();
    for j in &robot.joints {
        let Some(body1) = j.body1.as_deref() else {
            continue;
        };
        let angle = angles.values.get(&j.path).copied().unwrap_or(0.0);
        let displacement = match j.kind {
            JointKind::Revolute => {
                let axis_local = axis_vec(j.axis.as_deref());
                let pivot = Vec3::from(j.local_pos1);
                let frame_rot = Quat::from_xyzw(
                    j.local_rot1[1],
                    j.local_rot1[2],
                    j.local_rot1[3],
                    j.local_rot1[0],
                );
                let axis_world = (frame_rot * axis_local).normalize_or_zero();
                if axis_world == Vec3::ZERO {
                    Transform::IDENTITY
                } else {
                    let rot = Quat::from_axis_angle(axis_world, angle);
                    Transform::from_translation(pivot)
                        * Transform::from_rotation(rot)
                        * Transform::from_translation(-pivot)
                }
            }
            JointKind::Prismatic => {
                let axis_local = axis_vec(j.axis.as_deref());
                let frame_rot = Quat::from_xyzw(
                    j.local_rot1[1],
                    j.local_rot1[2],
                    j.local_rot1[3],
                    j.local_rot1[0],
                );
                let axis_world = (frame_rot * axis_local).normalize_or_zero();
                Transform::from_translation(axis_world * angle)
            }
            // Fixed / Spherical / Generic — slider has no effect for
            // now (Spherical would need three-axis input).
            _ => Transform::IDENTITY,
        };
        disp_by_body.insert(body1.as_ref(), displacement);
    }

    for (pr, mut tf) in q.iter_mut() {
        let Some(disp) = disp_by_body.get(pr.path.as_str()) else {
            continue;
        };
        let Some(rest_tf) = rest.rest.get(&pr.path) else {
            continue;
        };
        // Compose: rest * displacement in body0's parent frame.
        *tf = rest_tf.mul_transform(*disp);
    }
}

/// Spawn / despawn `AxisGizmo` children on link and joint frame
/// entities depending on overlay toggles. We piggyback on existing
/// link entities by adding a child entity carrying just an
/// AxisGizmo — that way `AxisGizmoPlugin`'s draw system picks it up
/// without us having to modify the link entity itself.
fn apply_frame_overlays(
    mut commands: Commands,
    overlays: Res<Overlays>,
    robot: Res<RobotJoints>,
    spawned: Res<SpawnedScene>,
    existing: Query<Entity, With<FrameOverlay>>,
    prims: Query<(Entity, &UsdPrimRef), Without<LoadedRobot>>,
) {
    if !spawned.0 {
        return;
    }

    let want_link = overlays.show_link_frames;
    let want_joint = overlays.show_joint_frames;

    if !want_link && !want_joint {
        for e in existing.iter() {
            commands.entity(e).despawn();
        }
        return;
    }

    // Despawn-then-respawn: cheap because the set is small (a few dozen
    // links/joints). Avoids tracking which gizmo lives where.
    if !existing.is_empty() {
        return;
    }

    if want_link {
        let bodies: std::collections::HashSet<&str> = robot
            .rigid_body_paths
            .iter()
            .map(|s| s.as_str())
            .collect();
        for (entity, pr) in prims.iter() {
            if !bodies.contains(pr.path.as_str()) {
                continue;
            }
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
        // Joints don't get their own ECS entity — anchor a gizmo on
        // body1 at (local_pos1, local_rot1) so the triad sits on the
        // joint's child-side frame. Same body can host multiple
        // joints; that's fine, gizmos are independent.
        let mut by_path: HashMap<&str, Entity> = HashMap::new();
        for (entity, pr) in prims.iter() {
            by_path.insert(pr.path.as_str(), entity);
        }
        for j in &robot.joints {
            let Some(body1) = j.body1.as_deref() else {
                continue;
            };
            let Some(parent) = by_path.get(body1) else {
                continue;
            };
            let pivot = Vec3::from(j.local_pos1);
            let rot = Quat::from_xyzw(
                j.local_rot1[1],
                j.local_rot1[2],
                j.local_rot1[3],
                j.local_rot1[0],
            );
            let local = Transform {
                translation: pivot,
                rotation: rot,
                scale: Vec3::ONE,
            };
            commands.entity(*parent).with_children(|c| {
                c.spawn((
                    local,
                    GlobalTransform::default(),
                    AxisGizmo::new(0.12),
                    FrameOverlay,
                ));
            });
        }
    }
}

/// Paint the side rails. `draw_assembly` handles every aspect of the
/// rail buttons — paint, drag-to-swap, exclusivity (one open menu per
/// rail), and click→toggle routing into [`RibbonOpen`]. The fourth
/// closure arg returns whether a button is "active" for visual
/// emphasis; we always say `false` since none of our buttons act as a
/// persistent state indicator (they're pure menu toggles).
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
    let _clicks = draw_assembly(
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

/// Paint the panes for the currently-open ribbon buttons. Each pane
/// goes through [`floating_window_for_item`], which:
///
/// * looks the button up in `RIBBONS` / `RIBBON_ITEMS` (so dragging
///   the button across rails re-anchors the pane automatically),
/// * routes through `floating_window_scoped` with a width-scope
///   derived from the ribbon item id (which is unique per button),
///   so the resize-handle Area id stays unique across panes — no
///   egui ID clashes the way the bare `floating_window` would
///   produce when two same-side panes share `frost_panel_width_*`.
fn draw_panels(
    mut contexts: EguiContexts,
    mut overlays: ResMut<Overlays>,
    robot: Res<RobotJoints>,
    mut angles: ResMut<JointAngles>,
    accent: Res<AccentColor>,
    open: Res<RibbonOpen>,
    placement: Res<RibbonPlacement>,
) {
    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };
    let accent_color: egui::Color32 = accent.0;

    // Closure: "is this menu currently open?" — checks against the
    // ribbon the button currently lives on (which may differ from its
    // declaration if the user dragged it).
    let is_open = |id: &'static str| -> bool {
        let Some(item) = find_item(RIBBON_ITEMS, id) else {
            return false;
        };
        let (rid, _, _) = placement.resolve(item);
        open.is_open(rid, id)
    };
    let mut keep_open = true;

    // ── Joints ──────────────────────────────────────────────────────
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
                    if robot.joints.is_empty() {
                        ui.label("No joints in this asset.");
                        return;
                    }
                    let mut shown = 0usize;
                    for (i, j) in robot.joints.iter().enumerate() {
                        if !matches!(j.kind, JointKind::Revolute | JointKind::Prismatic) {
                            continue;
                        }
                        shown += 1;
                        let leaf = j
                            .path
                            .rsplit('/')
                            .next()
                            .unwrap_or(j.path.as_str())
                            .to_string();
                        let val_entry = angles.values.entry(j.path.clone()).or_insert(0.0);
                        let mut val = *val_entry as f64;
                        let (lo, hi, suffix, decimals) = match j.kind {
                            JointKind::Revolute => {
                                let lo = j
                                    .lower_limit
                                    .map(|d| d.to_radians() as f64)
                                    .unwrap_or(-std::f32::consts::PI as f64);
                                let hi = j
                                    .upper_limit
                                    .map(|d| d.to_radians() as f64)
                                    .unwrap_or(std::f32::consts::PI as f64);
                                (lo, hi, " rad", 3)
                            }
                            JointKind::Prismatic => {
                                let lo = j.lower_limit.unwrap_or(-1.0) as f64;
                                let hi = j.upper_limit.unwrap_or(1.0) as f64;
                                (lo, hi, " m", 3)
                            }
                            _ => unreachable!(),
                        };
                        ui.push_id(("jslider", i), |ui| {
                            pretty_slider(
                                ui,
                                &leaf,
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
                            "(only fixed / spherical / generic joints — nothing to drive)",
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
                            for j in &robot.joints {
                                if !matches!(
                                    j.kind,
                                    JointKind::Revolute | JointKind::Prismatic
                                ) {
                                    continue;
                                }
                                let (lo, hi) = match j.kind {
                                    JointKind::Revolute => (
                                        j.lower_limit
                                            .map(|d| d.to_radians())
                                            .unwrap_or(-std::f32::consts::PI),
                                        j.upper_limit
                                            .map(|d| d.to_radians())
                                            .unwrap_or(std::f32::consts::PI),
                                    ),
                                    JointKind::Prismatic => (
                                        j.lower_limit.unwrap_or(-1.0),
                                        j.upper_limit.unwrap_or(1.0),
                                    ),
                                    _ => continue,
                                };
                                let t: f32 = pseudo_rand(j.path.as_str());
                                angles
                                    .values
                                    .insert(j.path.clone(), lo + t * (hi - lo));
                            }
                        }
                    });
                });
            },
        );
    }

    // ── Overlays ────────────────────────────────────────────────────
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

    // ── Tree ────────────────────────────────────────────────────────
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
                pane.section("rigid_bodies", "Rigid bodies", true, |ui| {
                    if robot.rigid_body_paths.is_empty() {
                        ui.label("No rigid bodies authored.");
                        return;
                    }
                    for (i, path) in robot.rigid_body_paths.iter().enumerate() {
                        ui.push_id(("rb", i), |ui| {
                            ui.label(path.as_str());
                        });
                    }
                });
                pane.section("joints_list", "Joints", true, |ui| {
                    if robot.joints.is_empty() {
                        ui.label("No joints authored.");
                        return;
                    }
                    for (i, j) in robot.joints.iter().enumerate() {
                        ui.push_id(("jt", i), |ui| {
                            let leaf = j.path.rsplit('/').next().unwrap_or(j.path.as_str());
                            ui.label(format!("{leaf}  ({:?})", j.kind));
                        });
                    }
                });
            },
        );
    }
}
