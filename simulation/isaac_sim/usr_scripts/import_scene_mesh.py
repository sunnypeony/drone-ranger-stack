# import the simplified mesh into Isaac Sim and set up collision properties


import omni.kit.commands
import omni.usd
from pxr import UsdPhysics, PhysxSchema, UsdGeom, Gf

stage = omni.usd.get_context().get_stage()

# 1. Import OBJ(Isaac Sim automatically convert into USD)
omni.kit.commands.execute(
    "CreateReferenceCommand",
    usd_context=omni.usd.get_context(),
    path_to="/World/BackyardMesh",
    asset_path="/root/scene_clean.obj",
    instanceable=False)
print("Mesh imported into Isaac Sim.")


# 2. Find the imported mesh prim
mesh_prim = stage.GetPrimAtPath("/World/BackyardMesh")

# 3. Align coordinate system (COLMAP Y-up → Isaac Sim Z-up)
xform = UsdGeom.Xformable(mesh_prim)
xform.ClearXformOpOrder()
xform.AddRotateXYZOp().Set(Gf.Vec3f(-90, 0, 0))
xform.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))  # 按需调整尺度

# 4. Add collision properties
UsdPhysics.CollisionAPI.Apply(mesh_prim)

mesh_col = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
mesh_col.GetApproximationAttr().Set("sdf")  # 最精确，保留凹面

# 设为静态（不受重力）
rb = UsdPhysics.RigidBodyAPI.Apply(mesh_prim)
rb.GetRigidBodyEnabledAttr().Set(False)

print("Scene imported with SDF collision.")
