
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class EdgeDetector(Node):
    def __init__(self):
        super().__init__('edge_detector')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Image, '/camera/edge', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 100, 200)
        edge_msg = self.bridge.cv2_to_imgmsg(edges, 'mono8')
        self.pub.publish(edge_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EdgeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
