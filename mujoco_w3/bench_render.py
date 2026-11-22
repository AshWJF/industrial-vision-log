import os, time, mujoco
m = mujoco.MjModel.from_xml_path(os.path.expanduser("~/mujoco_w3/simple.xml"))
d = mujoco.MjData(m)
for _ in range(300):
    mujoco.mj_step(m, d)
for h, w in [(480, 640), (240, 320)]:
    r = mujoco.Renderer(m, height=h, width=w)
    t0 = time.time()
    for _ in range(20):
        r.update_scene(d, camera="cam"); r.render()
    print(f"{w}x{h}: {(time.time()-t0)/20*1000:.1f} ms/帧")
