import os, mujoco, numpy as np
m = mujoco.MjModel.from_xml_path(os.path.expanduser("~/mujoco_w3/simple.xml"))
d = mujoco.MjData(m)
for _ in range(300):
    mujoco.mj_step(m, d)
r = mujoco.Renderer(m, height=480, width=640)
r.update_scene(d, camera="cam")
r.enable_depth_rendering()
depth = r.render()
r.disable_depth_rendering()
print("shape =", depth.shape, "dtype =", depth.dtype)
print("min/max =", depth.min(), depth.max())
print("画面中心 =", depth[240, 320], "米")
np.save(os.path.expanduser("~/mujoco_w3/evidence/w3-04-depth.npy"), depth)
