import os, mujoco, numpy as np, cv2
m = mujoco.MjModel.from_xml_path(os.path.expanduser("~/mujoco_w3/simple.xml"))
d = mujoco.MjData(m)
for _ in range(300):
    mujoco.mj_step(m, d)
r = mujoco.Renderer(m, height=480, width=640)
r.update_scene(d, camera="cam")
r.enable_depth_rendering(); depth = r.render(); r.disable_depth_rendering()
near, far = 1.6, 4.5
vis = (255 * (1 - (np.clip(depth, near, far) - near) / (far - near))).clip(0, 255).astype(np.uint8)
cv2.imwrite(os.path.expanduser("~/mujoco_w3/evidence/w3-04-depth.png"), vis)
print("盒子处深度 =", round(float(depth[250, 320]), 3))
