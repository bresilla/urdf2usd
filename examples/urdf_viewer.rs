//! Native URDF viewer. Loads a URDF directly through `bevy_urdf` (no
//! conversion). Use `usd_viewer` for USD assets.
//!
//! ```
//! cargo run --example urdf_viewer -- <path/to/robot.urdf>
//! ```

use std::path::PathBuf;

#[path = "_shared/urdf_app.rs"]
mod app;

fn main() {
    let input = std::env::args().nth(1).unwrap_or_else(|| {
        eprintln!("usage: urdf_viewer <path/to/robot.urdf>");
        std::process::exit(2);
    });
    let path = PathBuf::from(&input).canonicalize().unwrap_or_else(|_| {
        eprintln!("error: input URDF does not exist: {input}");
        std::process::exit(1);
    });
    app::run(path);
}
