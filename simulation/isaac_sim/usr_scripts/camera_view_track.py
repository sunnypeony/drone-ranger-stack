# Chase camera that follows the Pegasus quadrotor in the viewport.
# Run in the Isaac Sim Script Editor (Window -> Script Editor) after the vehicle is spawned.
# Running it again replaces the previous follower; call stop_follow_cam() to stop it.

import builtins

import omni.kit.app
import omni.kit.viewport.utility as vp_util
import omni.usd
from pxr import Gf, Usd, UsdGeom

QUADROTOR_PRIM_PATH = "/World/quadrotor/body"   # Pegasus rigid body of the vehicle
VIEWPORT_CAMERA_PATH = "/World/FollowCam"
FOLLOW_DISTANCE = 3.0   # metres behind the vehicle
FOLLOW_HEIGHT = 1.5     # metres above the vehicle

_SUB_KEY = "_drone_ranger_follow_cam_sub"

try:
    from omni.isaac.dynamic_control import _dynamic_control
    _dc = _dynamic_control.acquire_dynamic_control_interface()
except ImportError:
    _dc = None


def _vehicle_pose(stage):
    """World position and forward axis of the vehicle, or None if it is not available."""
    # While the simulation is playing, read the pose from PhysX: with Fabric enabled
    # the rigid-body pose is not always written back to USD.
    if _dc is not None:
        handle = _dc.get_rigid_body(QUADROTOR_PRIM_PATH)
        if handle != _dynamic_control.INVALID_HANDLE:
            pose = _dc.get_rigid_body_pose(handle)
            rot = Gf.Rotation(Gf.Quatd(pose.r[3], pose.r[0], pose.r[1], pose.r[2]))   # dc quat is (x, y, z, w)
            return Gf.Vec3d(pose.p[0], pose.p[1], pose.p[2]), rot.TransformDir(Gf.Vec3d(1, 0, 0))

    prim = stage.GetPrimAtPath(QUADROTOR_PRIM_PATH)
    if not prim.IsValid():
        return None
    world_xform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return world_xform.ExtractTranslation(), world_xform.TransformDir(Gf.Vec3d(1, 0, 0))


def _on_update(_):
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return
    cam_prim = stage.GetPrimAtPath(VIEWPORT_CAMERA_PATH)
    pose = _vehicle_pose(stage)
    if pose is None or not cam_prim.IsValid():
        return
    pos, forward = pose

    # Follow yaw only, so the view does not roll and pitch with the vehicle
    forward_xy = Gf.Vec3d(forward[0], forward[1], 0.0)
    if forward_xy.GetLength() < 1e-6:
        forward_xy = Gf.Vec3d(1, 0, 0)
    forward_xy.Normalize()

    eye = pos - forward_xy * FOLLOW_DISTANCE + Gf.Vec3d(0, 0, FOLLOW_HEIGHT)
    # SetLookAt builds the world->camera (view) matrix; the prim needs camera->world
    view = Gf.Matrix4d().SetLookAt(eye, pos, Gf.Vec3d(0, 0, 1))
    _cam_transform_op.Set(view.GetInverse())


def stop_follow_cam():
    sub = getattr(builtins, _SUB_KEY, None)
    if sub is not None:
        sub.unsubscribe()
        setattr(builtins, _SUB_KEY, None)
        print("[Follow Cam] Stopped.")


# Replace a follower started by a previous run of this script
stop_follow_cam()

stage = omni.usd.get_context().get_stage()
cam_prim = stage.GetPrimAtPath(VIEWPORT_CAMERA_PATH)
if not cam_prim.IsValid():
    cam_prim = UsdGeom.Camera.Define(stage, VIEWPORT_CAMERA_PATH).GetPrim()

# A single matrix op that is overwritten every frame
_cam_transform_op = UsdGeom.Xformable(cam_prim).MakeMatrixXform()

setattr(builtins, _SUB_KEY,
        omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(_on_update, name="follow_cam"))

viewport = vp_util.get_active_viewport()
if viewport:
    viewport.camera_path = VIEWPORT_CAMERA_PATH
print(f"[Follow Cam] Following {QUADROTOR_PRIM_PATH}. Call stop_follow_cam() to stop.")
