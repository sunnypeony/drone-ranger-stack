

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np



class DepthRepublisher(Node):
    def __init__(self):
        super().__init__('depth_republisher')
        self.sub = self.create_subscription(
            Image, '/depth', self.callback, 10)
        self.pub = self.create_publisher(
            Image, '/depth_clean', 10)

    def callback(self, msg):
        img = np.frombuffer(msg.data, dtype=np.float32).reshape(
            msg.height, msg.width).copy()
        
        # inf 和 nan 替换成 0
        img[~np.isfinite(img)] = 0.0
        
        new_msg = Image()
        new_msg.header = msg.header
        new_msg.height = msg.height
        new_msg.width = msg.width
        new_msg.encoding = '32FC1'
        new_msg.is_bigendian = msg.is_bigendian
        new_msg.step = msg.step
        new_msg.data = img.tobytes()
        self.pub.publish(new_msg)

def main():
    rclpy.init()
    rclpy.spin(DepthRepublisher())

if __name__ == '__main__':
    main()