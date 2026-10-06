

import omni
import omni.kit.viewport.utility as vp_util
from pxr import Gf, UsdGeom

QUADROTOR_PRIM_PATH = "/World/quadrotor/body"   
VIEWPORT_CAMERA_PATH = "/World/FollowCam"


_follow_cam_sub = None

def _on_update(_):
    try:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        prim = stage.GetPrimAtPath(QUADROTOR_PRIM_PATH)
        if not prim.IsValid():
            return
        cam_prim = stage.GetPrimAtPath(VIEWPORT_CAMERA_PATH)
        if not cam_prim.IsValid():
            return
        xform = UsdGeom.Xformable(prim)
        world_xform = xform.ComputeLocalToWorldTransform(0)
        # 相机放在机体后方 2 米（机体 -Z），这样能看到飞机
        pos = world_xform.Transform(Gf.Vec3d(0, 0, -2))
        rot = world_xform.ExtractRotation().GetQuat()
        cam_xform = UsdGeom.Xformable(cam_prim)
        op = cam_xform.GetTransformOp()
        if not op:
            op = cam_xform.AddtransformOp()
        mat = Gf.Matrix4d(1.0)
        mat.SetRotate(Gf.Rotation(rot))
        mat.SetTranslateOnly(pos)
        op.Set(mat)
    except Exception:
        pass

# Intitialize: Let FollowCam only on transform op, avoid conflict every frame
stage = omni.usd.get_context().get_stage()
if stage:
        cam_prim = stage.GetPrimAtPath(VIEWPORT_CAMERA_PATH)
        if cam_prim.IsValid():
            cx = UsdGeom.Xformable(cam_prim)
            cx.ClearXformOpOrder()
            cx.AddTranslateOp()
            

stream = omni.kit.app.get_app().get_update_event_stream()
_follow_cam_sub = stream.create_subscription_to_pop(_on_update, name="follow_cam")
viewport = vp_util.get_active_viewport()
if viewport:
    viewport.camera_path = VIEWPORT_CAMERA_PATH
print("[Follow Cam] Started. To stop: run the stop snippet.")