import os, mujoco
m = mujoco.MjModel.from_xml_path(os.path.expanduser("~/mujoco_w3/simple.xml"))
d = mujoco.MjData(m)
print("nq =", m.nq, "| nv =", m.nv, "| nbody =", m.nbody, "| ngeom =", m.ngeom)
print("timestep =", m.opt.timestep)
print("body_mass =", m.body_mass)
print("geom_friction =", m.geom_friction.tolist())
for i in range(401):
    if i % 40 == 0:
        print(f"step={i:3d}  t={d.time:.3f}  z={d.qpos[2]:.4f}  vz={d.qvel[2]:.3f}")
    mujoco.mj_step(m, d)
