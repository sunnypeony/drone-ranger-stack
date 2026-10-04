

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np



class DepthChecker(Node):
    def __init__(self):
        super().__init__('depth_checker')
        
        self.sub = self.create_subscription(
            Image, '/depth', self.callback, 10)

    def callback(self, msg):
        # decode from raw bytes, do not use cv_bridge
        img = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        
        valid = img[np.isfinite(img)]
        cx, cy = msg.width // 2, msg.height // 2
        
        print(f"Resolution: {msg.width}x{msg.height}")
        print(f"Central Pixel: {img[cy, cx]:.3f} meter")
        print(f"Valid Pixel Count: {len(valid)} / {img.size}")
        if len(valid) > 0:
            print(f"Depth Range: {valid.min():.3f} ~ {valid.max():.3f} meter")
            print(f"Average Depth: {valid.mean():.3f} meter")
        else:
            print("Warning: No valid pixel, all is inf or nan!")
        print("---")

def main():
    rclpy.init()
    rclpy.spin(DepthChecker())

if __name__ == '__main__':
    main()