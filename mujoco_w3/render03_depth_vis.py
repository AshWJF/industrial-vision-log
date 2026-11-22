import os, mujoco, numpy as np, cv2
m = mujoco.MjModel.from_xml_path(os.path.expanduser("~/mujoco_w3/simple.xml"))
d = mujoco.MjData(m)
for _ in range(300):
    mujoco.mj_step(m, d)
r = mujoco.Renderer(m, height=480, width=640)
r.update_scene(d, camera="cam")
r.enable_depth_rendering(); depth = r.render(); r.disable_depth_rendering()
near, far = depth.min(), np.percentile(depth, 99)
vis = (255 * (1 - (depth - near) / (far - near))).clip(0, 255).astype(np.uint8)
cv2.imwrite(os.path.expanduser("~/mujoco_w3/evidence/w3-04-depth.png"), vis)
print("near/far =", round(float(near),3), round(float(far),3), "| shape =", vis.shape)
