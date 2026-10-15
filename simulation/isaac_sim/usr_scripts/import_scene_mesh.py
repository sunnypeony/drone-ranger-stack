# Import the simplified reconstruction mesh into Isaac Sim as a static collider.
# Run in the Isaac Sim Script Editor (Window -> Script Editor).
#
# 1. Converts the OBJ to USD (OBJ cannot be referenced directly by USD).
# 2. References it under SCENE_PRIM_PATH/mesh and aligns SCENE_PRIM_PATH to Isaac Sim's Z-up world.
# 3. Adds static triangle-mesh collision to every Mesh prim inside it.
#
# Save the stage afterwards (File -> Save As) to get a .usd that
# PegasusInterface().load_environment() can load directly.

import asyncio
import os

import omni.kit.asset_converter
import omni.kit.commands
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics

MESH_PATH = os.path.expanduser("~/backyard_reconstruction/mesh/scene_clean.obj")   # .obj or .usd
SCENE_PRIM_PATH = "/World/BackyardScene"   # Xform owned by this script; the mesh is referenced below it

# COLMAP's world frame is arbitrary and unscaled: adjust these until the ground is
# horizontal at z = 0 and a known distance (e.g. a 1 m gate) measures correctly.
ROTATE_XYZ_DEG = Gf.Vec3f(-90.0, 0.0, 0.0)
SCALE = 1.0
TRANSLATE = Gf.Vec3d(0.0, 0.0, 0.0)

# "none" = exact triangle mesh, the standard choice for static environment geometry.
# "convexDecomposition" is cheaper but fills in concave shapes.
# ("sdf" is meant for dynamic bodies and is not needed for a static scene.)
COLLISION_APPROXIMATION = "none"


async def _convert_to_usd(src_path):
    dst_path = os.path.splitext(src_path)[0] + ".usd"
    converter = omni.kit.asset_converter.get_instance()
    task = converter.create_converter_task(src_path, dst_path, None)
    if not await task.wait_until_finished():
        raise RuntimeError(f"Mesh conversion failed: {task.get_error_message()}")
    print(f"Converted {src_path} -> {dst_path}")
    return dst_path


async def import_scene_mesh():
    stage = omni.usd.get_context().get_stage()
    root = UsdGeom.Xform.Define(stage, SCENE_PRIM_PATH).GetPrim()
    mesh_root_path = SCENE_PRIM_PATH + "/mesh"

    # Re-running only re-applies the transform and collision settings below
    if not stage.GetPrimAtPath(mesh_root_path).IsValid():
        if not os.path.isfile(MESH_PATH):
            raise FileNotFoundError(MESH_PATH)
        usd_path = MESH_PATH if MESH_PATH.endswith((".usd", ".usda", ".usdc")) else await _convert_to_usd(MESH_PATH)
        omni.kit.commands.execute(
            "CreateReferenceCommand",
            usd_context=omni.usd.get_context(),
            path_to=mesh_root_path,
            asset_path=usd_path,
            instanceable=False,
        )

    # The transform goes on our own parent Xform, so it never clashes with ops inside the asset
    xform = UsdGeom.Xformable(root)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(TRANSLATE)
    xform.AddRotateXYZOp().Set(ROTATE_XYZ_DEG)
    xform.AddScaleOp().Set(Gf.Vec3f(SCALE, SCALE, SCALE))

    # Collision APIs must sit on the Mesh prims; on the parent Xform they have no effect.
    # No RigidBodyAPI: a collider without a rigid body is static.
    meshes = [prim for prim in Usd.PrimRange(root) if prim.IsA(UsdGeom.Mesh)]
    for prim in meshes:
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set(COLLISION_APPROXIMATION)

    if not meshes:
        print(f"Warning: no Mesh prims found under {SCENE_PRIM_PATH}")
    print(f"Scene at {SCENE_PRIM_PATH}: collision ({COLLISION_APPROXIMATION}) on {len(meshes)} mesh(es).")


asyncio.ensure_future(import_scene_mesh())
