import os, mujoco, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data

class MujocoImageNode(Node):
    def __init__(self):
        super().__init__('mujoco_image_pub')
        self.pub = self.create_publisher(Image, '/image_raw', qos_profile_sensor_data)
        self.jpub = self.create_publisher(JointState, '/joint_states', 10)
        self.br = CvBridge()
        self.m = mujoco.MjModel.from_xml_path(os.path.expanduser("~/mujoco_w3/simple.xml"))
        self.d = mujoco.MjData(self.m)
        self.r = mujoco.Renderer(self.m, height=240, width=320)
        self.period = 1.0 / 30.0
        self.next_t = 0.0
        self.create_timer(self.period, self.on_timer)

    def on_timer(self):
        while self.d.time < self.next_t:
            mujoco.mj_step(self.m, self.d)
        self.next_t += self.period
        self.r.update_scene(self.d, camera="cam")
        msg = self.br.cv2_to_imgmsg(self.r.render(), encoding="rgb8")
        msg.header.frame_id = "camera_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['free']
        js.position = [float(x) for x in self.d.qpos[:7]]
        js.velocity = [float(x) for x in self.d.qvel[:6]]
        self.jpub.publish(js)

rclpy.init()
node = MujocoImageNode()
rclpy.spin(node)
