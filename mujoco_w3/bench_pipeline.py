import os, time, mujoco
from cv_bridge import CvBridge
m = mujoco.MjModel.from_xml_path(os.path.expanduser("~/mujoco_w3/simple.xml"))
d = mujoco.MjData(m); br = CvBridge()
r = mujoco.Renderer(m, height=480, width=640)
for _ in range(20):
    r.update_scene(d, camera="cam"); r.render()
t1 = t2 = 0.0
for _ in range(20):
    a = time.time()
    r.update_scene(d, camera="cam"); rgb = r.render()
    b = time.time(); br.cv2_to_imgmsg(rgb, encoding="rgb8"); c = time.time()
    t1 += b - a; t2 += c - b
    mujoco.mj_step(m, d)
print(f"render {(t1/20)*1000:.1f} ms | to_imgmsg {(t2/20)*1000:.1f} ms")
