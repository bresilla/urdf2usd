//! URDF → USD → URDF round-trip viewer with a joints + overlays panel
//! so you can sanity-check the round-trip visually *and* interactively.
//!
//! ```
//! cargo run --example view_roundtrip -- <path/to/robot.urdf>
//! ```
//!
//! With no argument the example runs against
//! `xtra/urdf-usd-converter/tests/data/fixed_continuous_joints.urdf`.
//!
//! Pipeline:
//! 1. Parse the URDF with urdf-rs.
//! 2. Discover `package://` roots via
//!    [`urdf2usd::resolve::PackageResolver`] and hand them to bevy_urdf's
//!    [`PackageMap`] so its mesh loader resolves `package://` URIs.
//! 3. [`urdf2usd::urdf_to_usd`] writes a layered USD asset under
//!    `<input_dir>/.urdf2usd_rt/`.
//! 4. [`urdf2usd::usd_to_urdf`] reads it back and writes
//!    `<input_dir>/<stem>_roundtrip.urdf` next to the input.
//! 5. Bevy window opens with bevy_urdf loading the reconstructed URDF
//!    plus a left sidebar with tabs:
//!     - **Joints** — drives FK via `draw_joint_controls` so you can
//!       verify every joint's type/axis/limits survived.
//!     - **Overlays** — toggle visual / collision / frames / grid.
//!     - **Info** — paths of every artifact produced.

use std::path::{Path, PathBuf};

use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPrimaryContextPass, egui};
use bevy_urdf::keyboard::DisplayToggles;
use bevy_urdf::manipulation::RobotArms;
use bevy_urdf::robot::{LoadRobot, Robot, RobotRoot};
use bevy_urdf::ui::{UiState, draw_joint_controls};
use bevy_urdf::urdf::PackageMap;
use bevy_urdf::UrdfPlugin;
use urdf2usd::resolve::PackageResolver;
use urdf2usd::{Options, urdf_to_usd, usd_to_urdf};

const DEFAULT_URDF: &str =
    "xtra/urdf-usd-converter/tests/data/fixed_continuous_joints.urdf";

fn main() {
    let input = std::env::args()
        .nth(1)
        .unwrap_or_else(|| DEFAULT_URDF.to_string());
    let input_path = PathBuf::from(&input).canonicalize().unwrap_or_else(|_| {
        eprintln!("error: input URDF does not exist: {input}");
        std::process::exit(1);
    });

    let input_dir = input_path.parent().unwrap_or(Path::new(".")).to_path_buf();
    let stem = input_path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("robot")
        .to_string();

    let usd_dir = input_dir.join(".urdf2usd_rt");
    std::fs::create_dir_all(&usd_dir).expect("create temp usd dir");

    println!("input      : {}", input_path.display());

    let package_map = build_package_map(&input_path);

    println!("→ URDF → USD …");
    let usd_top =
        urdf_to_usd(&input_path, &usd_dir, &Options::new()).expect("URDF → USD failed");
    println!("  wrote    : {}", usd_top.display());

    let reconstructed = input_dir.join(format!("{stem}_roundtrip.urdf"));
    println!("→ USD → URDF …");
    usd_to_urdf(&usd_top, &reconstructed).expect("USD → URDF failed");
    println!("  wrote    : {}", reconstructed.display());

    println!("→ opening bevy_urdf viewer — left sidebar tabs for joints/overlays/info.");
    App::new()
        .insert_resource(ArtifactPaths {
            input: input_path.clone(),
            usd: usd_top,
            reconstructed: reconstructed.clone(),
        })
        .insert_resource(LeftTab::Joints)
        .insert_resource(ReconstructedPath(reconstructed))
        .insert_resource(package_map)
        .insert_resource(ClearColor(Color::srgb(0.08, 0.08, 0.09)))
        .add_plugins(
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    title: format!("urdf2usd: {stem}_roundtrip"),
                    resolution: (1400u32, 900u32).into(),
                    ..default()
                }),
                ..default()
            }),
        )
        // UrdfPlugin wires in RobotPlugin + MeshPlugin + KinematicsPlugin
        // + ArcballCameraPlugin + UI/keyboard/overlays, and auto-adds
        // EguiPlugin if not already present.
        .add_plugins(UrdfPlugin)
        .add_systems(Startup, (spawn_scene, spawn_robot))
        .add_systems(EguiPrimaryContextPass, draw_sidebar)
        .run();
}

#[derive(Resource)]
struct ReconstructedPath(PathBuf);

#[derive(Resource)]
struct ArtifactPaths {
    input: PathBuf,
    usd: PathBuf,
    reconstructed: PathBuf,
}

#[derive(Resource, Debug, Clone, Copy, PartialEq, Eq)]
enum LeftTab {
    Joints,
    Overlays,
    Info,
}

fn build_package_map(urdf_path: &Path) -> PackageMap {
    let mut pm = PackageMap::new();
    let raw_xml = match std::fs::read_to_string(urdf_path) {
        Ok(s) => s,
        Err(_) => return pm,
    };
    let patched = urdf2usd::tolerate::tolerate(&raw_xml).unwrap_or_default();
    let Ok(robot) = urdf_rs::read_from_string(&patched) else {
        return pm;
    };

    let resolver = PackageResolver::new(urdf_path, &[]);
    for link in &robot.links {
        for v in &link.visual {
            if let urdf_rs::Geometry::Mesh { filename, .. } = &v.geometry {
                let _ = resolver.resolve(filename);
            }
        }
        for c in &link.collision {
            if let urdf_rs::Geometry::Mesh { filename, .. } = &c.geometry {
                let _ = resolver.resolve(filename);
            }
        }
    }
    for m in &robot.materials {
        if let Some(t) = &m.texture {
            let _ = resolver.resolve(&t.filename);
        }
    }
    for (name, path) in resolver.known_packages() {
        pm.insert(name, path);
    }
    pm
}

fn spawn_scene(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(2.5, 1.8, 2.5).looking_at(Vec3::ZERO, Vec3::Y),
        bevy_urdf::ArcballCamera {
            focus: Vec3::ZERO,
            distance: 3.5,
            ..default()
        },
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

fn spawn_robot(mut commands: Commands, path: Res<ReconstructedPath>) {
    commands.write_message(LoadRobot {
        path: path.0.clone(),
        root: None,
    });
}

/// Sidebar panel with tab selector + three views. Runs in
/// [`EguiPrimaryContextPass`] so egui's context is valid.
fn draw_sidebar(
    mut contexts: EguiContexts,
    mut active: ResMut<LeftTab>,
    mut ui_state: ResMut<UiState>,
    mut toggles: ResMut<DisplayToggles>,
    artifacts: Res<ArtifactPaths>,
    robots: Query<&Robot, With<RobotRoot>>,
    arms_q: Query<&RobotArms, With<RobotRoot>>,
) {
    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };

    egui::SidePanel::left("roundtrip_sidebar")
        .resizable(true)
        .default_width(320.0)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                tab_button(ui, &mut active, LeftTab::Joints, "Joints");
                tab_button(ui, &mut active, LeftTab::Overlays, "Overlays");
                tab_button(ui, &mut active, LeftTab::Info, "Info");
            });
            ui.separator();

            egui::ScrollArea::vertical().show(ui, |ui| match *active {
                LeftTab::Joints => {
                    if let Ok(robot) = robots.single() {
                        let arms = arms_q.single().ok();
                        draw_joint_controls(ui, robot, arms, &mut ui_state);
                    } else {
                        ui.label("No robot loaded.");
                    }
                }
                LeftTab::Overlays => draw_overlays(ui, &mut toggles),
                LeftTab::Info => draw_info(ui, &artifacts),
            });
        });
}

fn tab_button(ui: &mut egui::Ui, active: &mut LeftTab, tab: LeftTab, label: &str) {
    let selected = *active == tab;
    if ui
        .selectable_label(selected, label)
        .on_hover_text(format!("Show the {label} tab"))
        .clicked()
    {
        *active = tab;
    }
}

fn draw_overlays(ui: &mut egui::Ui, t: &mut DisplayToggles) {
    ui.heading("Display");
    ui.checkbox(&mut t.show_visual, "Visual geometry");
    ui.checkbox(&mut t.show_collision, "Collision geometry");
    ui.separator();
    ui.heading("Frames");
    ui.checkbox(&mut t.show_link_frames, "Link frames");
    ui.checkbox(&mut t.show_joint_frames, "Joint frames");
    ui.checkbox(&mut t.show_link_names, "Link name labels");
    ui.separator();
    ui.heading("World");
    ui.checkbox(&mut t.show_world_grid, "Grid");
    ui.checkbox(&mut t.show_world_axes, "Axes");
}

fn draw_info(ui: &mut egui::Ui, artifacts: &ArtifactPaths) {
    ui.heading("Round-trip artifacts");
    labelled(ui, "Input URDF", &artifacts.input);
    labelled(ui, "USD top", &artifacts.usd);
    labelled(ui, "Reconstructed URDF", &artifacts.reconstructed);
    ui.separator();
    ui.label(
        "The viewer is loaded from the *reconstructed* URDF — anything you \
         see here survived URDF → USD → URDF. Use the Joints tab to wiggle \
         each DOF and confirm axes, limits, and parent/child topology \
         came through intact.",
    );
}

fn labelled(ui: &mut egui::Ui, label: &str, path: &Path) {
    ui.label(egui::RichText::new(label).strong());
    let s = path.display().to_string();
    ui.add(egui::Label::new(egui::RichText::new(&s).monospace()).wrap());
    ui.add_space(6.0);
}
