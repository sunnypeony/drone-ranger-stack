#!/usr/bin/env python3

from datetime import timedelta

import depthai as dai

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


# ============================================================
# Configuration
# ============================================================

RGB_W = 640
RGB_H = 400

STEREO_W = 640
STEREO_H = 400

FPS = 30.0

RGB_FRAME = "oak_rgb_camera_optical_frame"

RGB_TOPIC = "/oak/rgb/image_raw"
RGB_INFO_TOPIC = "/oak/rgb/camera_info"

DEPTH_TOPIC = "/oak/depth/image_raw"
DEPTH_INFO_TOPIC = "/oak/depth/camera_info"


class OakRgbdPublisher(Node):

    def __init__(self):
        super().__init__("oak_rgbd_python_v2")

        self.bridge = CvBridge()

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transforms()

        # ----------------------------------------------------
        # Sensor-data QoS
        #
        # BEST_EFFORT + KEEP_LAST is normally preferable for
        # high-rate camera streams: don't accumulate old frames.
        # ----------------------------------------------------

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.rgb_pub = self.create_publisher(
            Image,
            RGB_TOPIC,
            sensor_qos
        )

        self.depth_pub = self.create_publisher(
            Image,
            DEPTH_TOPIC,
            sensor_qos
        )

        self.rgb_info_pub = self.create_publisher(
            CameraInfo,
            RGB_INFO_TOPIC,
            sensor_qos
        )

        self.depth_info_pub = self.create_publisher(
            CameraInfo,
            DEPTH_INFO_TOPIC,
            sensor_qos
        )

        # ====================================================
        # DepthAI pipeline
        # ====================================================

        self.pipeline = dai.Pipeline()

        platform = self.pipeline.getDefaultDevice().getPlatform()

        self.get_logger().info(
            f"DepthAI platform: {platform}"
        )

        # OAK-D Lite should be RVC2.
        if platform == dai.Platform.RVC4:
            raise RuntimeError(
                "This bridge version is written for OAK-D Lite / RVC2."
            )

        # ----------------------------------------------------
        # Cameras
        # CAM_A = RGB
        # CAM_B = left mono
        # CAM_C = right mono
        # ----------------------------------------------------

        self.rgb_cam = self.pipeline.create(
            dai.node.Camera
        ).build(
            dai.CameraBoardSocket.CAM_A
        )

        self.left_cam = self.pipeline.create(
            dai.node.Camera
        ).build(
            dai.CameraBoardSocket.CAM_B
        )

        self.right_cam = self.pipeline.create(
            dai.node.Camera
        ).build(
            dai.CameraBoardSocket.CAM_C
        )

        self.stereo = self.pipeline.create(
            dai.node.StereoDepth
        )

        self.sync = self.pipeline.create(
            dai.node.Sync
        )

        # Sync tolerance = half one frame interval.
        # At 30 FPS: ~16.7 ms.
        self.sync.setSyncThreshold(
            timedelta(seconds=1.0 / (2.0 * FPS))
        )

        # ----------------------------------------------------
        # RGB output
        #
        # CROP gives deterministic 640x400 geometry.
        # enableUndistortion=True outputs an undistorted RGB
        # image suitable for a pinhole-style reconstruction
        # pipeline.
        # ----------------------------------------------------

        self.rgb_out = self.rgb_cam.requestOutput(
            size=(RGB_W, RGB_H),
            type=dai.ImgFrame.Type.BGR888p,
            resizeMode=dai.ImgResizeMode.CROP,
            fps=FPS,
            enableUndistortion=True,
        )

        # ----------------------------------------------------
        # Stereo inputs
        # ----------------------------------------------------

        self.left_out = self.left_cam.requestOutput(
            size=(STEREO_W, STEREO_H),
            resizeMode=dai.ImgResizeMode.CROP,
            fps=FPS,
        )

        self.right_out = self.right_cam.requestOutput(
            size=(STEREO_W, STEREO_H),
            resizeMode=dai.ImgResizeMode.CROP,
            fps=FPS,
        )

        self.left_out.link(self.stereo.left)
        self.right_out.link(self.stereo.right)

        # More robust depth around occlusion boundaries.
        self.stereo.setLeftRightCheck(True)

        # Subpixel improves depth precision, especially farther away.
        self.stereo.setSubpixel(True)

        # ----------------------------------------------------
        # CRITICAL:
        # Align StereoDepth output to RGB camera geometry.
        #
        # On RVC2 this is the official v3 approach.
        # ----------------------------------------------------

        self.rgb_out.link(
            self.stereo.inputAlignTo
        )

        # ----------------------------------------------------
        # Device-side synchronization
        # ----------------------------------------------------

        self.rgb_out.link(
            self.sync.inputs["rgb"]
        )

        self.stereo.depth.link(
            self.sync.inputs["depth_aligned"]
        )

        self.sync_queue = (
            self.sync.out.createOutputQueue(
                maxSize=4,
                blocking=False
            )
        )

        # ====================================================
        # Start pipeline
        # ====================================================

        self.pipeline.start()

        self.get_logger().info(
            "OAK-D RGB + aligned StereoDepth + Sync pipeline started"
        )

        # ----------------------------------------------------
        # Calibration
        # ----------------------------------------------------

        device = self.pipeline.getDefaultDevice()
        self.calib = device.readCalibration()

        # RGB intrinsics at our requested output size.
        self.rgb_info_template = self.make_aligned_camera_info(
            dai.CameraBoardSocket.CAM_A,
            RGB_W,
            RGB_H,
            RGB_FRAME
        )

        # Because depth is aligned to RGB:
        #
        # RGB and depth share the SAME output camera model.
        #
        # Therefore depth CameraInfo is a copy of RGB CameraInfo.
        self.depth_info_template = CameraInfo()
        self.depth_info_template = self.copy_camera_info(
            self.rgb_info_template
        )

        # Poll output queue frequently without blocking ROS executor.
        self.timer = self.create_timer(
            0.001,
            self.publish_synced_pair
        )

    # ========================================================
    # CameraInfo
    # ========================================================

    def make_aligned_camera_info(
        self,
        socket,
        width,
        height,
        frame_id
    ):

        intrinsics = self.calib.getCameraIntrinsics(
            socket,
            width,
            height
        )

        fx = float(intrinsics[0][0])
        fy = float(intrinsics[1][1])
        cx = float(intrinsics[0][2])
        cy = float(intrinsics[1][2])

        msg = CameraInfo()

        msg.width = width
        msg.height = height
        msg.header.frame_id = frame_id

        # ----------------------------------------------------
        # We publish an UNDISTORTED RGB stream.
        #
        # For downstream ROS/Open3D usage, represent the
        # resulting image as an ideal pinhole image.
        # ----------------------------------------------------

        msg.distortion_model = "plumb_bob"

        msg.d = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]

        msg.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0,
        ]

        # Identity rectification matrix.
        msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]

        # Standard monocular projection matrix.
        msg.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        msg.binning_x = 0
        msg.binning_y = 0

        msg.roi.x_offset = 0
        msg.roi.y_offset = 0
        msg.roi.height = 0
        msg.roi.width = 0
        msg.roi.do_rectify = False

        self.get_logger().info(
            "RGB CameraInfo: "
            f"fx={fx:.3f}, fy={fy:.3f}, "
            f"cx={cx:.3f}, cy={cy:.3f}"
        )

        return msg

    def copy_camera_info(self, src):

        dst = CameraInfo()

        dst.width = src.width
        dst.height = src.height
        dst.distortion_model = src.distortion_model

        dst.d = list(src.d)
        dst.k = list(src.k)
        dst.r = list(src.r)
        dst.p = list(src.p)

        dst.binning_x = src.binning_x
        dst.binning_y = src.binning_y

        dst.roi.x_offset = src.roi.x_offset
        dst.roi.y_offset = src.roi.y_offset
        dst.roi.height = src.roi.height
        dst.roi.width = src.roi.width
        dst.roi.do_rectify = src.roi.do_rectify

        dst.header.frame_id = src.header.frame_id

        return dst

    def publish_static_transforms(self):
        transforms = []

        # --------------------------------------------------------
        # base_link -> oak_rgb_camera_frame
        #
        # TEMPORARY EXTRINSIC:
        # Replace with measured/calibrated values.
        # --------------------------------------------------------
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "oak_rgb_camera_frame"
        t.transform.translation.x = 0.125
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        transforms.append(t)

        # ========================================================
        # oak_rgb_camera_frame -> oak_rgb_camera_optical_frame
        # ========================================================

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "oak_rgb_camera_frame"
        t.child_frame_id = "oak_rgb_camera_optical_frame"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        # ROS camera frame -> optical frame
        # RPY = (-pi/2, 0, -pi/2)
        # Quaternion xyzw:
        # [0.5, -0.5, 0.5, -0.5]
        t.transform.rotation.x = 0.5
        t.transform.rotation.y = -0.5
        t.transform.rotation.z = 0.5
        t.transform.rotation.w = -0.5
        transforms.append(t)

        # --------------------------------------------------------
        # base_link -> laser_frame
        #
        # TEMPORARY EXTRINSIC:
        # Replace with measured/calibrated values.
        # --------------------------------------------------------
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "laser_frame"
        t.transform.translation.x = 0.125
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.025
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        transforms.append(t)

        self.tf_static_broadcaster.sendTransform(transforms)

        self.get_logger().info(
            "Published static transforms: "
            "base_link -> oak_rgb_camera_frame, "
            "oak_rgb_camera_frame -> oak_rgb_camera_optical_frame, "
            "base_link -> laser_frame"
        )

    # ========================================================
    # Publishing
    # ========================================================

    def publish_synced_pair(self):

        group = self.sync_queue.tryGet()

        if group is None:
            return

        if not isinstance(group, dai.MessageGroup):
            self.get_logger().warning(
                "Sync queue returned non-MessageGroup"
            )
            return

        try:
            rgb_packet = group["rgb"]
            depth_packet = group["depth_aligned"]

        except Exception as exc:
            self.get_logger().warning(
                f"Incomplete synchronized message group: {exc}"
            )
            return

        if rgb_packet is None or depth_packet is None:
            return

        # ----------------------------------------------------
        # Convert frames
        # ----------------------------------------------------

        rgb_frame = rgb_packet.getCvFrame()
        depth_frame = depth_packet.getFrame()

        # One ROS timestamp for this synchronized RGB-D pair.
        stamp = self.get_clock().now().to_msg()

        # ----------------------------------------------------
        # RGB
        # ----------------------------------------------------

        rgb_msg = self.bridge.cv2_to_imgmsg(
            rgb_frame,
            encoding="bgr8"
        )

        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = RGB_FRAME

        # ----------------------------------------------------
        # Depth
        #
        # DepthAI StereoDepth depth output is RAW16.
        # Values are depth distance, conventionally mm on RVC2
        # unless explicitly configured otherwise.
        # ----------------------------------------------------

        depth_msg = self.bridge.cv2_to_imgmsg(
            depth_frame,
            encoding="16UC1"
        )

        depth_msg.header.stamp = stamp

        # CRITICAL:
        # aligned depth is expressed in RGB optical geometry.
        depth_msg.header.frame_id = RGB_FRAME

        # ----------------------------------------------------
        # CameraInfo
        # ----------------------------------------------------

        rgb_info = self.copy_camera_info(
            self.rgb_info_template
        )

        depth_info = self.copy_camera_info(
            self.depth_info_template
        )

        rgb_info.header.stamp = stamp
        depth_info.header.stamp = stamp

        rgb_info.header.frame_id = RGB_FRAME
        depth_info.header.frame_id = RGB_FRAME

        # ----------------------------------------------------
        # Publish
        # ----------------------------------------------------

        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)

        self.rgb_info_pub.publish(rgb_info)
        self.depth_info_pub.publish(depth_info)

    # ========================================================
    # Shutdown
    # ========================================================

    def destroy_node(self):

        try:
            self.pipeline.stop()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = OakRgbdPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()