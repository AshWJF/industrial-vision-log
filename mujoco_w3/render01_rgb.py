import os, mujoco, cv2
m = mujoco.MjModel.from_xml_path(os.path.expanduser("~/mujoco_w3/simple.xml"))
d = mujoco.MjData(m)
for _ in range(300):
    mujoco.mj_step(m, d)
r = mujoco.Renderer(m, height=480, width=640)
r.update_scene(d, camera="cam")
rgb = r.render()
print("shape =", rgb.shape, "dtype =", rgb.dtype)
print("min/max =", rgb.min(), rgb.max())
out = os.path.expanduser("~/mujoco_w3/evidence/w3-02b-camera.png")
cv2.imwrite(out, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
print("saved:", out)
