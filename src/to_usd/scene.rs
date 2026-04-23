//! UsdPhysics.Scene authoring. Lives in the physics sublayer.

use crate::Result;
use usd_schemas::physics::define_scene as define_usd_scene;

use super::Ctx;

pub fn define_scene(ctx: &mut Ctx<'_>) -> Result<()> {
    let robot_prim = ctx.robot_prim.clone();
    define_usd_scene(&mut ctx.stage_physics, &robot_prim, "PhysicsScene")?;
    Ok(())
}
