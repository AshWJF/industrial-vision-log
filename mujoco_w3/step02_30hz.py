import os, mujoco
p = os.path.expanduser("~/mujoco_w3/simple.xml")
for per_frame, label in [(1, "只 step 1 次"), (16, "step 16 次")]:
    m = mujoco.MjModel.from_xml_path(p); d = mujoco.MjData(m)
    for f in range(60):
        for _ in range(per_frame):
            mujoco.mj_step(m, d)
    print(f"{label}: 60 帧后 t={d.time:.3f}s  z={d.qpos[2]:.4f}  vz={d.qvel[2]:.3f}")
