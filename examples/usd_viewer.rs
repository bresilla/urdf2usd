//! USD-only viewer. Loads a USD asset directly through `bevy_openusd`;
//! reject non-USD inputs. Convert URDFs ahead of time with `u2u to-usd`.
//!
//! ```
//! cargo run --example usd_viewer -- <path/to/scene.usda>
//! ```

use std::path::PathBuf;

#[path = "_shared/usd_app.rs"]
mod app;

fn main() {
    let input = std::env::args().nth(1).unwrap_or_else(|| {
        eprintln!("usage: usd_viewer <path/to/scene.usda>");
        std::process::exit(2);
    });
    let path = PathBuf::from(&input).canonicalize().unwrap_or_else(|_| {
        eprintln!("error: input USD does not exist: {input}");
        std::process::exit(1);
    });
    app::run(path);
}
