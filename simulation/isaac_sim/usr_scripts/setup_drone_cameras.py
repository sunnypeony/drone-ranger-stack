import omni.kit.commands
import omni.graph.core as og
import omni.usd

def setup_drone_cameras():
    body_path = "/World/quadrotor/body/body"
    rgb_cam_path = body_path + "/RGBCamera"
    depth_cam_path = body_path + "/DepthCamera"

    omni.kit.commands.execute("CreatePrim", prim_path=rgb_cam_path, prim_type="Camera")
    omni.kit.commands.execute("CreatePrim", prim_path=depth_cam_path, prim_type="Camera")

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": body_path + "/CameraActionGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick",          "omni.graph.action.OnPlaybackTick"),
                ("RGBRenderProd",   "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("DepthRenderProd", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RGBHelper",       "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("DepthHelper",     "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.SET_VALUES: [
                ("RGBRenderProd.inputs:cameraPrim",   [rgb_cam_path]),
                ("RGBRenderProd.inputs:width",        640),
                ("RGBRenderProd.inputs:height",       480),
                ("DepthRenderProd.inputs:cameraPrim", [depth_cam_path]),
                ("DepthRenderProd.inputs:width",      640),
                ("DepthRenderProd.inputs:height",     480),
                ("RGBHelper.inputs:topicName",        "/drone/rgb"),
                ("RGBHelper.inputs:type",             "rgb"),
                ("RGBHelper.inputs:frameId",          "drone_camera"),
                ("DepthHelper.inputs:topicName",      "/drone/depth"),
                ("DepthHelper.inputs:type",           "depth"),
                ("DepthHelper.inputs:frameId",        "drone_camera"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick",                       "RGBRenderProd.inputs:execIn"),
                ("RGBRenderProd.outputs:execOut",             "RGBHelper.inputs:execIn"),
                ("RGBRenderProd.outputs:renderProductPath",   "RGBHelper.inputs:renderProductPath"),
                ("OnTick.outputs:tick",                       "DepthRenderProd.inputs:execIn"),
                ("DepthRenderProd.outputs:execOut",           "DepthHelper.inputs:execIn"),
                ("DepthRenderProd.outputs:renderProductPath", "DepthHelper.inputs:renderProductPath"),
            ],
        }
    )
    print("Cameras and Action Graph created!")
    print("   RGB   -> /drone/rgb")
    print("   Depth -> /drone/depth")

setup_drone_cameras()
