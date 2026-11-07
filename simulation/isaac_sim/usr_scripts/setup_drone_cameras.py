# Mount a forward-facing RGB-D camera on the Pegasus quadrotor and publish it to ROS 2.
# Run in the Isaac Sim Script Editor after Pegasus "Load Vehicle" (safe to run again).
#
# Publishes:
#   /drone/rgb          sensor_msgs/Image       (rgb8)
#   /drone/depth        sensor_msgs/Image       (32FC1, metres, aligned to RGB)
#   /drone/camera_info  sensor_msgs/CameraInfo

import omni.graph.core as og
import omni.usd
from pxr import Gf, UsdGeom

BODY_PATH = "/World/quadrotor/body/body"   # moves with the Pegasus vehicle
CAMERA_PATH = BODY_PATH + "/Camera"
GRAPH_PATH = BODY_PATH + "/CameraActionGraph"

WIDTH, HEIGHT = 640, 480
FRAME_ID = "drone_camera"

# Mount pose in the body frame (x forward, y left, z up). Move the camera out of
# the airframe so the body does not block the view.
CAMERA_TRANSLATE = Gf.Vec3d(0.15, 0.0, 0.0)
# A USD camera looks along its -Z axis with +Y up. rotateXYZ (90, 0, -90) maps the
# view direction to body +X (forward) and camera up to body +Z.
CAMERA_ROTATE_XYZ_DEG = Gf.Vec3f(90.0, 0.0, -90.0)


def setup_drone_cameras():
    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath(BODY_PATH).IsValid():
        raise RuntimeError(f"{BODY_PATH} not found: load the Pegasus vehicle first")

    # One camera for both streams, so RGB and depth share the same pose and intrinsics
    camera = UsdGeom.Camera.Define(stage, CAMERA_PATH)
    xform = UsdGeom.Xformable(camera)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(CAMERA_TRANSLATE)
    xform.AddRotateXYZOp().Set(CAMERA_ROTATE_XYZ_DEG)

    # Rebuild the graph on re-runs instead of failing on existing nodes
    if stage.GetPrimAtPath(GRAPH_PATH).IsValid():
        stage.RemovePrim(GRAPH_PATH)

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick",           "omni.graph.action.OnPlaybackTick"),
                ("RenderProduct",    "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RGBHelper",        "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("DepthHelper",      "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraInfoHelper", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                ("RenderProduct.inputs:cameraPrim",    [CAMERA_PATH]),
                ("RenderProduct.inputs:width",         WIDTH),
                ("RenderProduct.inputs:height",        HEIGHT),
                ("RGBHelper.inputs:topicName",         "/drone/rgb"),
                ("RGBHelper.inputs:type",              "rgb"),
                ("RGBHelper.inputs:frameId",           FRAME_ID),
                ("DepthHelper.inputs:topicName",       "/drone/depth"),
                ("DepthHelper.inputs:type",            "depth"),
                ("DepthHelper.inputs:frameId",         FRAME_ID),
                ("CameraInfoHelper.inputs:topicName",  "/drone/camera_info"),
                ("CameraInfoHelper.inputs:frameId",    FRAME_ID),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick",                     "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut",           "RGBHelper.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "RGBHelper.inputs:renderProductPath"),
                ("RenderProduct.outputs:execOut",           "DepthHelper.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "DepthHelper.inputs:renderProductPath"),
                ("RenderProduct.outputs:execOut",           "CameraInfoHelper.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "CameraInfoHelper.inputs:renderProductPath"),
            ],
        }
    )
    print(f"Camera mounted at {CAMERA_PATH}, Action Graph at {GRAPH_PATH}")
    print("   RGB         -> /drone/rgb")
    print("   Depth       -> /drone/depth")
    print("   CameraInfo  -> /drone/camera_info")


setup_drone_cameras()
