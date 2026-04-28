//! `u2u` — bidirectional URDF ↔ OpenUSD command-line converter.
//!
//! Two ways to invoke it:
//!
//! 1. Auto-dispatch on extension (the common case):
//!    ```text
//!    u2u robot.urdf ./out/          # URDF → USD (layered asset)
//!    u2u ./out/robot.usda back.urdf # USD  → URDF
//!    ```
//!
//! 2. Explicit subcommand if you want to be sure:
//!    ```text
//!    u2u to-usd robot.urdf ./out/
//!    u2u to-urdf ./out/robot.usda back.urdf
//!    ```
//!
//! The library (`urdf2usd` crate) has no clap dependency — it's entirely
//! a binary-only concern, wired up here.

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use clap::{Parser, Subcommand};

use urdf2usd::opts::PackageOverride;
use urdf2usd::{
    Options, convert_layered_dir_to_usdc, pack_dir_to_usdz, urdf_to_usd, usd_to_urdf,
};

/// Convert between URDF and OpenUSD assets in either direction.
#[derive(Debug, Parser)]
#[command(name = "u2u", version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Option<Command>,

    /// Input file — positional fallback when no subcommand is given.
    /// Extension chooses direction: `.urdf` / `.xml` → USD; `.usda` /
    /// `.usdc` / `.usd` → URDF.
    #[arg(global = false, required = false)]
    input_file: Option<PathBuf>,

    /// Output path (positional fallback). Directory for URDF → USD,
    /// file (or directory) for USD → URDF.
    #[arg(global = false, required = false)]
    output: Option<PathBuf>,

    /// Print each conversion step + the path of every file written.
    #[arg(short, long, global = true)]
    verbose: bool,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// URDF → USD. Writes a layered Atomic Component by default
    /// (`<robot>.usda` + `Geometry.usda` / `Physics.usda` / `Materials.usda`
    /// sublayers). Use `--flat` to emit a single `.usda`.
    ToUsd(ToUsdArgs),

    /// USD → URDF. Reads any USD asset authored by `to-usd` (or any USD
    /// that conforms to the same conventions) and reconstructs a URDF
    /// file.
    ToUrdf(ToUrdfArgs),
}

#[derive(Debug, clap::Args)]
struct ToUsdArgs {
    /// Input `.urdf` (or `.xml`) file.
    input_file: PathBuf,

    /// Output directory. `<output>/<robot>.usda` is the top file.
    output_dir: PathBuf,

    /// Emit a single flattened `.usda` instead of an Atomic Component.
    #[arg(long, alias = "no-layer-structure")]
    flat: bool,

    /// Pack the layered output into a single `.usdz` archive
    /// (`<output_dir>/<robot>.usdz`) and remove the loose layered
    /// files. The robot stays addressable as one self-contained
    /// asset — sublayers and textures live inside the zip.
    #[arg(long)]
    usdz: bool,

    /// After writing the layered `.usda`, post-process every layer
    /// to binary `.usdc` and rewrite the top file's `subLayers`
    /// references accordingly. Combine with `--usdz` to ship a USDZ
    /// archive whose internal layers are USDC (the standard
    /// production layout — smaller and faster to parse than text).
    #[arg(long)]
    usdc: bool,

    /// Skip authoring the `UsdPhysics.Scene` prim.
    #[arg(long, alias = "no-physics-scene")]
    no_physics: bool,

    /// Authoring comment stamped on the asset layer.
    #[arg(short, long, default_value = "")]
    comment: String,

    /// ROS package overrides for `package://…` URIs. Use repeated
    /// `--package name=/abs/path` flags.
    #[arg(short, long, value_parser = parse_package)]
    package: Vec<PackageOverride>,
}

#[derive(Debug, clap::Args)]
struct ToUrdfArgs {
    /// Input USD file (`.usda` / `.usdc` / `.usd`). Sublayers are
    /// composed automatically.
    input_file: PathBuf,

    /// Output path. If it's a directory, the URDF is named
    /// `<input_stem>.urdf` inside it.
    output: PathBuf,
}

fn parse_package(raw: &str) -> Result<PackageOverride, String> {
    let (name, path) = raw
        .split_once('=')
        .ok_or_else(|| format!("expected <name>=<path>, got `{raw}`"))?;
    if name.is_empty() || path.is_empty() {
        return Err(format!("expected <name>=<path>, got `{raw}`"));
    }
    Ok(PackageOverride {
        name: name.to_string(),
        path: PathBuf::from(path),
    })
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    let verbose = cli.verbose;

    let result = match (cli.command, cli.input_file, cli.output) {
        // Explicit subcommands.
        (Some(Command::ToUsd(args)), _, _) => run_to_usd(args, verbose),
        (Some(Command::ToUrdf(args)), _, _) => run_to_urdf(args, verbose),

        // Auto-dispatch: no subcommand, positional input + output given.
        (None, Some(input), Some(output)) => auto_dispatch(&input, &output, verbose),

        // Not enough args — re-run clap with `--help` equivalent.
        _ => {
            let _ = <Cli as clap::CommandFactory>::command().print_help();
            eprintln!();
            eprintln!("error: missing input file and output path. See usage above.");
            return ExitCode::FAILURE;
        }
    };

    match result {
        Ok(written) => {
            if verbose {
                eprintln!("wrote {}", written.display());
            } else {
                println!("{}", written.display());
            }
            ExitCode::SUCCESS
        }
        Err(err) => {
            eprintln!("u2u: {err}");
            ExitCode::FAILURE
        }
    }
}

/// When invoked as `u2u <in> <out>`, infer direction from the input
/// extension. Keeps the UX simple for the common case.
fn auto_dispatch(input: &Path, output: &Path, verbose: bool) -> urdf2usd::Result<PathBuf> {
    let ext = input
        .extension()
        .and_then(|e| e.to_str())
        .map(|s| s.to_ascii_lowercase())
        .unwrap_or_default();
    match ext.as_str() {
        "urdf" | "xml" => run_to_usd(
            ToUsdArgs {
                input_file: input.to_path_buf(),
                output_dir: output.to_path_buf(),
                flat: false,
                no_physics: false,
                comment: String::new(),
                package: Vec::new(),
                usdz: false,
                usdc: false,
            },
            verbose,
        ),
        "usda" | "usdc" | "usd" => run_to_urdf(
            ToUrdfArgs {
                input_file: input.to_path_buf(),
                output: output.to_path_buf(),
            },
            verbose,
        ),
        other => Err(urdf2usd::Error::msg(format!(
            "unsupported input extension `{other}` — use `.urdf` for URDF input or `.usda` / `.usdc` / `.usd` for USD input"
        ))),
    }
}

fn run_to_usd(args: ToUsdArgs, verbose: bool) -> urdf2usd::Result<PathBuf> {
    if verbose {
        eprintln!("u2u: URDF → USD");
        eprintln!("  input      = {}", args.input_file.display());
        eprintln!("  output dir = {}", args.output_dir.display());
        eprintln!("  layered    = {}", !args.flat);
        eprintln!("  scene      = {}", !args.no_physics);
        eprintln!("  usdz       = {}", args.usdz);
        if !args.comment.is_empty() {
            eprintln!("  comment    = {:?}", args.comment);
        }
        for p in &args.package {
            eprintln!("  package    = {} -> {}", p.name, p.path.display());
        }
    }
    // `--usdz` always emits a single flattened layer inside the
    // archive — that's the whole point of USDZ ("one self-contained
    // file"). Forcing `flat = true` here means the archive holds one
    // `.usd*` + textures instead of a 4-layer atomic component the
    // consumer would have to recompose.
    let flat = args.flat || args.usdz;
    let opts = Options {
        layer_structure: !flat,
        scene: !args.no_physics,
        comment: args.comment,
        packages: args.package,
    };

    if !args.usdz {
        // Loose-layered output. If `--usdc` is set, convert the
        // freshly-written `.usda` files in place to `.usdc` so the
        // user's directory ends up with binary layers.
        let top = urdf_to_usd(&args.input_file, &args.output_dir, &opts)?;
        if args.usdc {
            convert_layered_dir_to_usdc(&args.output_dir)?;
            return Ok(top.with_extension("usdc"));
        }
        return Ok(top);
    }

    // USDZ path: write the layered asset into a per-robot subdir
    // under output_dir, optionally rewrite text→binary, pack to
    // `<output_dir>/<robot>.usdz`, then remove the loose subdir so
    // the caller is left with one file.
    let stem = args
        .input_file
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("robot")
        .to_string();
    let stage_dir = args.output_dir.join(format!("{stem}.usdz_staging"));
    if stage_dir.exists() {
        std::fs::remove_dir_all(&stage_dir)
            .map_err(urdf2usd::Error::Io)?;
    }
    let _ = urdf_to_usd(&args.input_file, &stage_dir, &opts)?;
    if args.usdc {
        convert_layered_dir_to_usdc(&stage_dir)?;
    }
    let usdz_path = args.output_dir.join(format!("{stem}.usdz"));
    let written = pack_dir_to_usdz(&stage_dir, &usdz_path)?;
    // Best-effort cleanup; if it fails we still keep the .usdz.
    if let Err(e) = std::fs::remove_dir_all(&stage_dir) {
        eprintln!(
            "warning: failed to remove staging dir {}: {e}",
            stage_dir.display()
        );
    }
    Ok(written)
}

fn run_to_urdf(args: ToUrdfArgs, verbose: bool) -> urdf2usd::Result<PathBuf> {
    let out_path = if args.output.is_dir() {
        let stem = args
            .input_file
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("robot");
        args.output.join(format!("{stem}.urdf"))
    } else {
        args.output.clone()
    };
    if verbose {
        eprintln!("u2u: USD → URDF");
        eprintln!("  input   = {}", args.input_file.display());
        eprintln!("  output  = {}", out_path.display());
    }
    usd_to_urdf(&args.input_file, &out_path)
}
