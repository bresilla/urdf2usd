//! Auto-dispatching viewer — picks the URDF or USD pipeline based on
//! the input file extension and runs the matching App.
//!
//! ```
//! cargo run --example viewer -- <robot.urdf>     # → bevy_urdf path
//! cargo run --example viewer -- <scene.usda>     # → bevy_openusd path
//! ```
//!
//! Single-purpose alternatives: `urdf_viewer` and `usd_viewer`.

use std::path::PathBuf;

#[path = "_shared/usd_app.rs"]
mod usd_app;
#[path = "_shared/urdf_app.rs"]
mod urdf_app;

fn main() {
    let input = std::env::args().nth(1).unwrap_or_else(|| {
        eprintln!(
            "usage: viewer <path/to/robot.urdf | scene.usda>\n\
             URDF (.urdf / .xml) → native bevy_urdf path.\n\
             USD  (.usda / .usdc / .usd) → bevy_openusd path."
        );
        std::process::exit(2);
    });
    let path = PathBuf::from(&input).canonicalize().unwrap_or_else(|_| {
        eprintln!("error: input does not exist: {input}");
        std::process::exit(1);
    });
    let ext = path
        .extension()
        .and_then(|s| s.to_str())
        .map(|s| s.to_ascii_lowercase())
        .unwrap_or_default();
    match ext.as_str() {
        "urdf" | "xml" => urdf_app::run(path),
        "usda" | "usdc" | "usd" => usd_app::run(path),
        other => {
            eprintln!(
                "error: unsupported extension `.{other}` — use .urdf / .xml or .usda / .usdc / .usd"
            );
            std::process::exit(1);
        }
    }
}
