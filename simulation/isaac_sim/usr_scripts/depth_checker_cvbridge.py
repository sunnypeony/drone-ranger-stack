# this file is used to check the depth image from Isaac Sim, with using cv_bridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

from cv_bridge import CvBridge

class DepthChecker(Node):
    def __init__(self):
        super().__init__('depth_checker')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/depth', self.callback, 10)

    def callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        h,w = depth_image.shape
        cx,cy = w // 2, h // 2
        center_val = depth_image[cy,cx]
        
        # 过滤掉 inf 和 nan
        valid = depth_image[np.isfinite(depth_image)]
        
        # cx, cy = msg.width // 2, msg.height // 2
        print(f"image size: {w} X {h}")
        print(f"中心像素: {center_val:.3f} 米")
        print(f"有效像素数: {len(valid)} / {depth_image.size}")
        print(f"深度范围: {valid.min():.3f} ~ {valid.max():.3f} 米")
        print(f"平均深度: {valid.mean():.3f} 米")
        print("---")

def main():
    rclpy.init()
    node = DepthChecker()
    rclpy.spin(node)

if __name__ == '__main__':
    main()