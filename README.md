# urdf2usd

Bidirectional URDF ↔ OpenUSD converter in pure Rust — no Pixar USD C++
libraries, no Python.

Converts a URDF robot description to an OpenUSD asset (layered into
`<robot>.usda` + `Geometry.usda` + `Physics.usda` + `Materials.usda`
sublayers, or flattened to a single file), and converts USD back to URDF
so tools that only speak URDF (ROS, `rapier3d-urdf`, `bevy_urdf`) can
consume the output of the first leg.

Built on top of [`mxpv/openusd`](https://github.com/mxpv/openusd) for
the spec-level (Sdf) reader/writer and [`urdf-rs`](https://crates.io/crates/urdf-rs)
for URDF parsing.

## Workspace layout

```
urdf2usd/
├── Cargo.toml                workspace + library crate
├── src/                      urdf2usd library
│   ├── to_usd/               URDF → USD pipeline
│   ├── to_urdf/              USD → URDF back-converter
│   ├── tolerate.rs           URDF tolerance (injects missing required attrs)
│   ├── undefined.rs          Passthrough for unknown URDF extensions
│   ├── hierarchy.rs          Link/joint graph + ghost-link chain
│   ├── resolve.rs            package:// and file:// URI resolver
│   ├── math/                 RPY ↔ quat, inertia eigendecomposition
│   └── opts.rs               Conversion options
├── tests/roundtrip.rs        URDF → USD → URDF structural round-trip tests
├── examples/view_in_bevy.rs  Convert + render in Bevy via bevy_openusd
└── crates/
    └── urdf2usd-cli/         The `u2u` command-line binary
                               (clap lives here, not in the library).
```

Two crates, two concerns:

| crate | scope |
|---|---|
| `urdf2usd` | The converter pipelines (both directions). No CLI, no clap. |
| `urdf2usd-cli` | The `u2u` binary: clap wiring around the library. |

Schema authoring + reading helpers (`UsdGeom.Mesh`, `UsdPhysics.RevoluteJoint`,
`UsdShade.Material`, etc.) live in [`bevy_openusd`'s `usd_schemas`
crate](https://github.com/bresilla/bevy_openusd) — what was originally
extracted from this repo has grown into a much richer crate (added camera
/ light / skel / anim / NURBS / curves / typed readers). We consume that
canonical version via a path dependency.

## Install

From source:

```bash
git clone https://github.com/bresilla/urdf2usd
cd urdf2usd
cargo install --path crates/urdf2usd-cli   # installs `u2u` on $PATH
```

## CLI usage

Two forms — auto-dispatch on extension, or explicit subcommand.

### Auto-dispatch

```bash
u2u robot.urdf ./out/                   # URDF → USD (layered)
u2u ./out/robot.usda robot_back.urdf    # USD  → URDF
```

Extension picks direction: `.urdf` / `.xml` emits USD; `.usda` / `.usdc`
/ `.usd` emits URDF. The output argument can be a directory or a file.

### Explicit subcommand

```bash
u2u to-usd  robot.urdf ./out/ --comment "asset v1"
u2u to-usd  robot.urdf ./out/ --flat --no-physics
u2u to-urdf ./out/robot.usda ./out/
u2u -v to-usd robot.urdf ./out/         # verbose — print every step
```

### Flags (URDF → USD)

| flag | default | effect |
|---|---|---|
| `--flat` | off | Emit a single flattened `.usda` instead of an Atomic Component. |
| `--no-physics` | off | Skip authoring `UsdPhysics.Scene`. |
| `--comment "..."` | empty | Stamp a comment on the asset layer. |
| `--package name=/path` | — | Resolve `package://name/…` URIs. Repeat for multiple packages. |
| `--verbose` / `-v` | off | Print each step and the path of every file written. |

### Output layout (layered)

```
out/
├── robot.usda              # top asset — defaultPrim + stage metadata + subLayers
├── Geometry.usda           # def Xform tree, primitives, MeshLibrary
├── Physics.usda            # overs adding PhysicsRigidBodyAPI/MassAPI/Joints
├── Materials.usda          # Materials scope + material bindings
└── Textures/               # texture files copied + renamed on collision
```

## Library usage

```rust
use urdf2usd::{Options, urdf_to_usd, usd_to_urdf};

// URDF → USD
let usd_top = urdf_to_usd(
    "robot.urdf",
    "./out",
    &Options::new(),    // layered + physics scene, no comment, no package overrides
)?;

// USD → URDF (round-trip validation or pipeline integration)
let urdf = usd_to_urdf(&usd_top, "./out/robot_back.urdf")?;
```

Both functions return `Result<PathBuf>` with the path of the primary
file written.

## Coverage

### URDF → USD

- **Primitives**: box / sphere / cylinder / capsule.
- **Meshes**: STL / OBJ / DAE. Shared `/MeshLibrary` scope with internal
  references so identical mesh files are stored once. OBJ n-gons
  preserved (not force-triangulated). Multi-shape OBJs become
  `Xform`s containing per-shape `Mesh` children.
- **Materials**: URDF colors + textures, OBJ `.mtl` (diffuse / normal /
  roughness / metallic / opacity), DAE effects with transparency modes
  (`A_ONE` / `A_ZERO` / `RGB_ZERO` / `RGB_ONE`). Full
  `UsdPreviewSurface` graph with per-channel `UsdUVTexture` +
  `UsdPrimvarReader_float2` + `sourceColorSpace` tokens. PreviewMaterial
  input interface promotion on the Material prim.
- **Physics**: `PhysicsScene` + `NewtonSceneAPI`, `PhysicsRigidBodyAPI`,
  `PhysicsMassAPI` with principal-axes eigendecomposition (deterministic
  canonicalisation for degenerate inertia tensors), `ArticulationRootAPI`
  + `NewtonArticulationRootAPI` on the first non-ghost link, world-frame
  root fixed joint.
- **Joints**: fixed / revolute / continuous / prismatic / planar /
  floating / spherical. Non-canonical URDF axes are absorbed into the
  joint frame so downstream solvers see a uniform `physics:axis = "X"`.
  Mimic joints via `NewtonMimicAPI`. Planar joints lock the
  normal-translation + two non-axis rotations via multi-apply
  `PhysicsLimitAPI`. Unsupported URDF joint attributes (effort,
  velocity, calibration, dynamics, safety_controller) survive round-trip
  as `urdf:*` custom floats.
- **Ghost-link removal**: chains of empty links connected by fixed
  joints at the tail of the tree get their rigid bodies suppressed.
- **URDF tolerance**: missing required attributes (material `name`,
  limit `velocity`, mass `value`, inertia fields, color `rgba`,
  safety_controller `k_velocity`, mesh `filename`) are injected with
  safe defaults before `urdf-rs` sees the XML.
- **Malformed XML recovery**: empty `<geometry></geometry>`, unknown
  geometry child tags, and missing `<geometry>` under `<visual>` /
  `<collision>` get replaced with a placeholder box.
- **Extension passthrough**: Gazebo blocks, MoveIt semantic descriptors,
  vendor-specific children under `<robot>` / `<link>` / `<joint>` are
  captured as `urdf:<tag>:<attr>` custom-string attributes on the
  corresponding USD prim.
- **Duplicate name detection**: duplicate link / joint / material names
  are rejected with an explicit error.
- **Layer structure**: layered by default (4 files + textures dir);
  `--flat` merges everything into one `.usda`.

### USD → URDF

The reverse pipeline reads any USD asset authored by the forward path
(or any USD that follows the same conventions) and reconstructs a
`urdf_rs::Robot` which gets serialised back to XML via
`urdf-rs::write_to_string`.

Structurally lossless for everything the forward path writes:

- Link names, joint names, joint types, parent/child topology
- URDF joint origins recovered exactly by peeling the axis-correction
  rotation back off `localRot0` using `localRot1`. Wheels rotate around
  their original axis; no tilt leakage.
- Visual / collision geometry kinds (box / sphere / cylinder / capsule /
  mesh). Mesh filenames survive verbatim via a `urdf:sourceFilename`
  custom attribute on each library Mesh prim.
- Material diffuse color (via the `Material.inputs:diffuseColor`
  interface input).
- Inertial mass + principal moments (diagonalised — off-diagonal
  components collapse to zero, orientation carried in `origin.rpy`).
- Original joint type distinction (revolute vs continuous, planar vs
  spherical) preserved through a `urdf:jointType` custom token.

## Verified via round-trip tests

7 `tests/roundtrip.rs` integration tests do URDF → USD → URDF and
assert structural parity: robot name, link-name set, joint topology,
joint type + parent + child, per-link visual/collision counts, leading
geometry kind, and — critically — that joint axis and origin.rpy survive
without leakage.

```bash
cargo test                  # workspace: 26 unit + 7 round-trip
cargo test -p usd_schemas   # just the schema crate's math tests
```

## Bevy viewer example

For visual verification, an example converts the URDF to USD and renders
it natively in Bevy through [`bevy_openusd`](https://github.com/bresilla/bevy_openusd)'s
`UsdPlugin`:

```bash
cargo run --example view_in_bevy -- path/to/robot.urdf
# default: uses xtra/urdf-usd-converter/tests/data/fixed_continuous_joints.urdf
```

The example:

1. Parses the URDF.
2. Calls `urdf_to_usd` — writes layered `<robot>.usda` + sublayers under
   `<input_dir>/.urdf2usd_rt/`.
3. Spawns a Bevy app with `bevy_openusd::UsdPlugin` and
   `SceneRoot(asset_server.load(usd_path))`. The same Stage projection
   `bevy_openusd`'s viewer uses — one entity per composed prim, basis
   fix at the root, mesh + material loading via the asset pipeline.

For the fully-featured viewer (VS-Code chrome + variant switcher +
overlays panel) point `bevy_openusd`'s own viewer at the converted USD:

```bash
cargo run -p bevy_openusd -- /path/to/converted.usda
```

Pulls in Bevy as a dev-dependency — heavy, kept out of normal builds.

## Architecture notes

### openusd is schema-agnostic

`openusd` is a **spec-level** (Sdf) reader/writer — it gives you
`sdf::Data`, readers / writers for `.usda` / `.usdc` / `.usdz`, and the
composition engine, but no typed `UsdGeom.Mesh`, `UsdPhysics.Scene`, or
`UsdShade.Material` builders. Those are what Pixar's C++ schema libs
and NVIDIA's `usdex.core` provide on top of the Sdf layer.

The `usd_schemas` crate fills that gap with hand-rolled Rust equivalents
— Pixar's C++ `UsdGeomXform::Define()` becomes our `xform::define_xform()`,
and so on. It stamps the correct `TypeName` tokens, `apiSchemas` list
ops, and attribute defaults that downstream tools (usdview, Omniverse,
Newton, bevy_urdf, Unreal's USD importer) recognise.

The schema crate is deliberately portable — depends only on `openusd`
and `anyhow` — so anyone authoring or reading USD from Rust can use it
without pulling in either this converter or `bevy_openusd`. See
[`bevy_openusd/crates/usd_schemas/src/lib.rs`](https://github.com/bresilla/bevy_openusd/blob/main/crates/usd_schemas/src/lib.rs)
for the API surface.

### Forward-path axis correction

URDF lets joint axes be any unit 3-vector; UsdPhysics expects a token
(`"X"` / `"Y"` / `"Z"`). We fold the rotation that takes `+X` to the
URDF axis into both `localRot0` and `localRot1` on the forward path, so
every revolute / prismatic / planar joint emits `physics:axis = "X"`
uniformly. The reverse path peels the factor back off via
`axis_rotation = localRot1` and `origin_rotation = localRot0 ·
axis_rotation⁻¹`, recovering the original URDF axis + origin.rpy
exactly.

### URDF tolerance + undefined-element passthrough

Two XML pre-passes happen before `urdf-rs` sees the file:

1. **Tolerance** (`src/tolerate.rs`) — streaming `quick-xml` rewriter
   that injects required-but-missing attributes + recovers malformed
   geometry. Real-world URDFs often omit `<material name>` or `<limit
   velocity>`; rather than rejecting them we make the XML parseable.

2. **Undefined capture** (`src/undefined.rs`) — second pass that
   captures every unknown tag under `<robot>` / `<link>` / `<joint>`
   (Gazebo blocks, MoveIt extensions, vendor tags) into a map keyed by
   link/joint name. These surface as `urdf:<tag>[:<sub>]:<attr>` custom
   attributes on the corresponding USD prim, so they survive round-trip
   and remain inspectable in any USD tool.

### Mesh library with references

Each unique resolved mesh file is authored exactly once under
`/<robot>/MeshLibrary/<safe_name>`. Each visual / collision site emits a
thin `def Mesh (references = </…/MeshLibrary/X>) { xformOp:scale = … }`
prim that composes the library entry through USD's references arc.
Single-shape libraries are `def Mesh`; multi-shape OBJ / DAE libraries
are `def Xform` with per-shape `Mesh` children (matching Python's
pycollada-based converter's shape nesting).

## Development

```bash
cargo build                                 # library + all workspace crates
cargo build -p urdf2usd-cli                 # just the CLI (`u2u` binary)
cargo test --workspace --lib                # unit tests
cargo test -p urdf2usd --test roundtrip     # full-pipeline round-trip tests
cargo run --example view_in_bevy -- <f>     # convert URDF and render in Bevy via bevy_openusd
```

## Dependencies

Runtime (library):

- [`openusd`](https://github.com/mxpv/openusd) (git, `main`) — spec-level USD.
- [`usd_schemas`](https://github.com/bresilla/bevy_openusd) (path dep into `../bevy_openusd`) — typed schema authoring + reading helpers.
- [`urdf-rs`](https://crates.io/crates/urdf-rs) 0.9 — URDF parser.
- [`mesh-loader`](https://crates.io/crates/mesh-loader) 0.1 — STL / DAE loading.
- [`tobj`](https://crates.io/crates/tobj) 4 — OBJ loading with per-material face IDs.
- [`nalgebra`](https://crates.io/crates/nalgebra) 0.33 — symmetric eigendecomposition.
- [`quick-xml`](https://crates.io/crates/quick-xml) 0.36 — URDF tolerance + undefined-element passes.
- `anyhow`, `thiserror` — error plumbing.

Binary-only:

- [`clap`](https://crates.io/crates/clap) 4 (derive) — in `crates/urdf2usd-cli`.

Example-only (dev-dependency):

- [`bevy_openusd`](https://github.com/bresilla/bevy_openusd) (path dep) — `UsdPlugin` for the example viewer.
- `bevy` 0.18.

## License

MIT.
