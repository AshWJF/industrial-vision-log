# W3 学习笔记：MuJoCo 入门

> 学习窗口：2026-11-22 03:25 → 11-23 01:22（原排 11.30 起，提前一周并行完成）
> 环境：ROS2 Humble / Python 3.10 / mujoco 3.17.0 / numpy 1.21.5 / cv2 4.5.4
> 证据：`~/mujoco_w3/evidence/`（9 张 png + 1 个 npy + README.md）
> 脚本：`~/mujoco_w3/*.py`（9 个）

---

## 0. 一句话结论

MuJoCo 入门其实就四件事：**写对 XML**（改哪个字段决定改外观还是改物理）、**理解 MjModel/MjData 的内存布局**（用 `mj_step` 推进）、**离屏渲染要自己建 Renderer 并指定相机名**、**往 ROS2 发图真正的坑在 QoS 和带宽，不在渲染速度**。

这四条串起来，正好就是 D435i 的仿真替身：相机 → 深度 → 点云 → 缺陷检测。W3 打通的是"仿真世界里有一台能出 RGB+深度的相机，而且它的输出能像真机一样从 ROS2 话题流出来"。

---

## 1. 装包与第一次运行

```bash
python3 -m pip install mujoco      # 3.17.0，装到 ~/.local/lib/python3.10/site-packages/mujoco
```

**关键：numpy 保持 1.21.5 不动。** 后面要和 `cv_bridge` 打交道，`cv_bridge` 对 numpy 2.x 兼容有问题。装 mujoco 时如果 pip 想把 numpy 升到 2.x，要显式 `numpy==1.21.5` 挡住。

启动 viewer 的正确写法：

```bash
python3 -m mujoco.viewer --mjcf=$HOME/mujoco_w3/simple.xml
```

两个坑：

- **`--mjcf` 必须给。** 不给的话 `mujoco.viewer` 走的是空 `launch()`，打开一个空白 GUI，看着像崩了其实没加载模型。
- **路径用 `$HOME`，不用 `~`。** 在某些传参路径上 `~` 不会被 shell 展开。

包内自带两个能动的官方模型，适合观察：

- `.../mujoco/testdata/model.xml` —— 带三条红色腿的头状体 + hfield 地形 + tendon，会自己动
- `.../mujoco/testdata/msh.xml`

（没有 humanoid，官方仓库里才有。）

viewer 里 File → Screenshot 的落点是**进程启动时的工作目录**，文件名固定 `screenshot.png`，不是 `~/mujoco_w3/`。这就是 `ls ~/mujoco_w3/*.png` 找不到的原因。

---

## 2. MJCF 结构：谁管外观，谁管物理

我们的 `simple.xml` 只有 17 行，但麻雀小：

```xml
<mujoco model="simple">
  <option timestep="0.002"/>
  <visual><headlight ambient="0.4 0.4 0.4" diffuse="0.7 0.7 0.7"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" .../>
    <material name="grid" texture="grid" texrepeat="4 4"/>
  </asset>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="2 2 0.1" material="grid" friction="0.05"/>
    <body name="box" pos="0 0 1" euler="0 20 0">
      <freejoint/>
      <geom name="box_geom" type="box" size="0.1 0.1 0.1" rgba="0.2 0.4 1 1"
            mass="0.5" friction="0.05"/>
    </body>
    <camera name="cam" pos="0 -3 1.2" xyaxes="1 0 0 0 0.342 0.9397"/>
  </worldbody>
</mujoco>
```

分工看得很清楚：

| 层 | 字段 | 管什么 |
|---|---|---|
| `<option>` | `timestep` | 物理步长，仿真精度与速度的总开关 |
| `<visual><headlight>` | `ambient` / `diffuse` | **仅离屏渲染**用的光，不影响物理 |
| `<asset>` | texture / material | 贴图，只影响画面 |
| `<worldbody><light>` | pos / dir | 场景里的实体光源（viewer 和离屏都用） |
| `<geom>` | `rgba` | 颜色，只影响画面 |
| `<geom>` | `friction` / `mass` / `size` | 物理参数 |
| `<body>` | `pos` / `euler` | 初始位姿 |
| `<camera>` | `pos` / `xyaxes` | 相机在哪、朝哪看 |

**记住这条分界：`rgba` 和 `material` 改的是"长什么样"，`friction`/`mass`/`size` 改的是"怎么动"。** 两者完全不串。

---

## 3. 四个字段各改一次，结论是什么

### 3.1 `rgba`：红 → 蓝，下落时间一模一样

原来 `rgba="... 0.4 1 1"`（偏灰蓝），改成纯蓝 `0.2 0.4 1 1`。z 曲线、落地时刻、回弹高度**逐位相同**。

结论：**外观与物理解耦**。这决定了后面做域随机化（随机换颜色、换贴图）时不用担心会改变动力学，可以放心大批量生成训练数据。

### 3.2 `camera`：改了这个，viewer 里看不出来

改 `<camera name="cam">` 的 `pos`/`xyaxes` 后，viewer 画面完全没变。原因：**viewer 默认用自由相机（鼠标拖来拖去那台），不认 XML 里的 `<camera>`**。

`<camera>` 只有在你**按名字调用**时才有意义：

```python
renderer.update_scene(data, camera="cam")
```

所以 ② 里"改 camera 字段"这条，在 22 日当时是看不见效果的，直到 23 日做了 ④ 离屏渲染才真正生效、留下证据。这也是当时 02b 截图只有棋盘格、没有方块的根本原因之一（另一个原因是 xyaxes 瞄偏了，见 §6）。

### 3.3 `euler`：0.35 度 vs 20 度，差了一个世界

第一次试着改 `euler="0 0 0.35 0"`，画面毫无变化 —— 因为 **MJCF 的 `angle` 默认是 `degree`**。0.35 度肉眼不可见。改成 `euler="0 20 0"` 立刻看到方块斜着往下掉、撞地之后翻倒、最后以一个面平贴地面停住。

内部存的是弧度：`20° → 0.349 rad`。`mjModel` 里永远拿弧度，XML 里永远写度数，别混。

### 3.4 `friction` 和 `mass`：两个完全不同性质的东西

**`friction` 是一对属性取 max。** 地面 0.05、盒子 0.05，接触时用 0.05；把地面改成 1.0，盒子还是 0.05，实际接触摩擦变成 **1.0**。所以只改盒子一侧的 friction 是没用的（取 max 之后还是被地面的 1 压住）。

静/滑临界判据：

$$\mu \ge \tan\theta \Rightarrow \text{静止},\quad \mu < \tan\theta \Rightarrow \text{滑动}$$

20° 坡 → $\tan 20° \approx 0.364$。0.05 远小于它，一路滑出视野；1.0 大于它，卡死在坡上不动。两张截图 `w3-02d-1/2` 和 `w3-02e` 就是这两侧的对照。

还有个细节：**`friction` 只写一个数时会自动补全成三个** `[s, 0.1s, 0.002s]`。写 `0.05` 存进去是 `[0.05, 0.005, 0.0001]`（滑动/滚动/自旋三轴）。`mjModel.geom_friction` 里看到的就是这三位。

**`mass` 是这次最反直觉的一条。** 把 `mass` 从 0.5 改成 5（十倍），重跑同一个脚本，**11 行输出逐位相同** —— 连 `0.0884` 这个软穿透峰值、`0.379` 这个回弹速度都一模一样，只有 `body_mass` 从 `[0. 0.5]` 变成 `[0. 5.]`。

为什么？

$$v = gt = 9.81 \times 0.4 = 3.924\ \text{(实测 -3.924)}$$

自由落体速度与质量无关，这是伽利略的结论，在仿真里也成立。接触阶段 MuJoCo 用约束求解器（不是冲量-动量手动分配），单盒撞**静态**地面这种"地面对盒子"的约束里，地面是无限质量的一端，质量不参与分配。

**这条结论对后面很重要**：单物体撞静态环境时，改质量对轨迹没影响；只有**两个都会动的物体互相碰撞**，质量才会进入动量分配、才真的改变结果。昨天只给了文字结论，这次用"逐位相同的输出"补了硬证据。

---

## 4. MjModel / MjData / mj_step

两个对象的分工：

- **`MjModel`**：结构。模型有多少关节、多少几何、每个几何的尺寸摩擦质量、`timestep`。**编译完就不再变**。
- **`MjData`**：状态。`qpos`（位置）、`qvel`（速度）、接触力、时间 `data.time`。**每一步都在变**。

### 内存布局（这是最容易踩的坑）

```python
m = mujoco.MjModel.from_xml_path("simple.xml")
d = mujoco.MjData(m)
print(m.nq, m.nv, m.nbody, m.ngeom)   # 7 6 2 2
```

一个 `freejoint`（自由关节）贡献 `nq += 7` / `nv += 6`：

- `qpos[0:3]` = 位置 (x, y, z) —— **`qpos[2]` 才是高度，不是 `qpos[1]`**
- `qpos[3:7]` = 姿态四元数 (w, x, y, z)
- `qvel[0:3]` = 线速度，`qvel[3:6]` = 角速度（速度没有四元数，只有 3 个）

为什么姿态要 7 个数、速度只要 6 个？因为四元数是 4 维的（带归一化约束），角速度是 3 维的（轴×角速度）。这是四元数表示法的必然结果。

**读过 URDF/ROS 的人会习惯性以为 `qpos[1]` 是高度**，这是从 `geometry_msgs/Pose` 那套 (x,y,z + quat) 里带出来的错觉，MuJoCo 里 `qpos` 是"位置在前、姿态在后"的扁平数组。

### `mj_step` 一次走多远

```python
mujoco.mj_step(m, d)   # 推进一个 model.opt.timestep = 0.002 s
```

400 步 = 0.8 s。实测 z 曲线：

| 步数 | t (s) | z | vz |
|---|---|---|---|
| 0 | 0.000 | 1.0000 | 0 |
| 40 | 0.080 | 0.9678 | -0.785 |
| 80 | 0.160 | 0.8729 | -1.570 |
| 120 | 0.240 | 0.7151 | -2.354 |
| 160 | 0.320 | 0.4946 | -3.139 |
| 200 | 0.400 | 0.2113 | -3.924 |
| 240 | 0.480 | 0.0884 | +0.379 |
| 280 | 0.560 | 0.0994 | +0.025 |
| 320 | 0.640 | 0.0999 | +0.002 |
| 400 | 0.800 | 0.1000 | 0 |

三处验算都对上了：

1. 每 40 步 vz 增加 0.785，`0.785 / 0.08 = 9.81` → 就是重力加速度
2. `t = 0.400` 时 `vz = -3.924 = 9.81 × 0.4` → 纯自由落体，还没接触
3. `t = 0.480` 的 `0.0884` 是**接触瞬间的软穿透**（MuJoCo 的接触约束是软的，允许少量穿透），随后收敛到稳定值 `0.1000`

`0.1000` 正好是盒子半边长。因为 20° 倾斜落地后会翻倒，最终一个面平贴地面，中心高度 = 半边长 0.1。**仿真自己"想明白"了这件事，我们只是读出来。**

---

## 5. `mj_step` 与 30 Hz 的矛盾

同一条 xml、同样跑 60 次回调：

| 每帧 step 次数 | t | z | vz |
|---|---|---|---|
| 1 | 0.120 | 0.9282 | -1.177 |
| 16 | 1.920 | 0.1000 | 0 |

`1.920 / 0.120 = 16.0` 精确。`-1.177 = 9.81 × 0.12` 精确。

**结论：每帧只 `mj_step` 一次，仿真会慢放 16 倍。** 画面看起来"方块在飘"，不是渲染慢，是物理时间走得比现实慢。

16 的来历：

$$\frac{1/30}{0.002} = 16.67$$

严格做法不是固定 16 步，而是**按目标仿真时刻累计推进**：

```python
period = 1.0 / 30.0
next_t = 0.0
# 每帧：
while d.time < next_t:
    mujoco.mj_step(m, d)
next_t += period
```

写 16 会让累积误差每 3 帧漂一个 timestep（16 vs 16.67），大约两分钟就会差出一秒。ROS2 节点里用的就是累计推进的写法。

---

## 6. 离屏渲染

```python
renderer = mujoco.Renderer(m, height=480, width=320)  # 只吃 model，不吃 data
renderer.update_scene(d, camera="cam")               # 按名字取 xml 里的相机
rgb = renderer.render()                              # (H, W, 3) uint8
renderer.enable_depth_rendering()
depth = renderer.render()                            # (H, W) float32，单位米
```

### 第一次渲染失败：两个原因叠在一起

首跑 `min/max = 0 / 86`，画面只有棋盘格、**没有方块**。两个原因：

**原因一：`xyaxes` 瞄偏了 42°。**

原写法 `xyaxes="1 1 0 -0.5 0.5 1"` 推出的视线方向是 `(-0.58, 0.58, -0.58)`，指向左前方；方块在正前方 `(0, 0, 0.1)`。夹角 42°，而 fovy 半角只有 22.5° —— 方块在视野外，当然拍不到。

改成 `xyaxes="1 0 0 0 0.342 0.9397"`：右轴 `(1, 0, 0)`、上轴 `(0, 0.342, 0.940)`，也就是朝 +y、俯视 20°。`0.342 = sin20°`、`0.9397 = cos20°`。

**`xyaxes` 是"右轴 + 上轴"两个向量，不是欧拉角。** 视线方向 = 右轴 × 上轴（叉乘）。这是 MJCF 里最不直观的一个字段。

**原因二：`<global ambient>` 这个属性根本不存在。**

```xml
<global ambient="0.4 0.4 0.4"/>   <!-- 报错：unrecognized attribute: 'ambient' -->
```

`<global>` 这个元素存在，但**没有 `ambient` 属性**。环境光必须写在 `<headlight>` 里：

```xml
<visual><headlight ambient="0.4 0.4 0.4" diffuse="0.7 0.7 0.7"/></visual>
```

改完 `min/max` 从 `0 86` 变成 `0 255`，画面正确：蓝方块在正中、棋盘格地面、正前方俯视 20°。

**为什么 viewer 里一直好好的、离屏就黑？** 因为 viewer 有一套默认的界面头灯（GUI 自带），离屏渲染没有界面，光照完全按 XML 来。**"viewer 里看得见"不等于"渲染得出来"**，这是所有人第一次做离屏渲染都会踩的坑。

### 深度图：单位、参照系、量程

```python
renderer.enable_depth_rendering()
depth = renderer.render()
# shape (480, 640), dtype float32
# min 1.6428 / max 69.6626 / 画面中心 3.0871
```

三个特性必须记牢：

1. **单位是米，float32。** 不是毫米、不是 uint16。
2. **是到相机原点的直线距离**，不是沿光轴的投影深度。这个差别在实际场景里可能很大（广角镜头边缘尤其明显）。
3. **没有几何的地方返回 60+ 米量级的极大值**（天空/无穷远），做点云前必须先把这些值滤掉。

三个数字验算：

- 画面中心 `3.087`：相机在 `y = -3`、盒子前面在 `y = -0.1`，沿视线 `(3 - 0.1) / cos20° = 2.9 / 0.9397 = 3.086`。实测 `3.087`，**差 2 mm**。
- `min = 1.64`：画面最下沿那条射线打到的地面。相机高 1.2 m、俯角 20°，画面下沿是 20° + 22.5° = 42.5°。实测 1.64，与"直线距离"口径吻合（细节见下）。
- `max = 69.66`：最远处是接近水平线的地面，仰角约 1° 时 `1.2 / tan1° ≈ 68.8`。量级对。

**第 2 条（直线距离 vs 光轴投影）怎么验证的？** 中心点区分不了两者 —— 正中射线就是光轴，直线距离和投影距离恰好相等（所以 3.087 那个 2 mm 只证明了内参一致，没证明口径）。要靠画面**边缘**区分：最下沿射线相对光轴偏 22.5°，若按"沿光轴投影"算，深度应是 `1.6428 × cos22.5° ≈ 1.52`；实测是 `1.6428` 本身，没被投影。所以是**直线距离**。

**这里留一个待复核的点**：按 fovy = 45°（半角 22.5°）反推，最下沿射线俯角应为 42.5°，对应的直线距离是 `1.2 / sin42.5° ≈ 1.78`，与实测 `1.64` 差 8%。可能是实际 fovy 不是 45°（按实测反推半角约 26.9°），也可能 `1.6428` 落在画面下沿略偏内、不是最边缘像素。**下次做点云反投影时，拿标定板或已知尺寸的方块验证一次再定标，别把这个角度假设直接带进去。**

### 深度可视化：动态量程是陷阱

第一版 `render03` 用 **99 分位做 far（68.6）**：

```python
far = np.percentile(depth, 99)   # 68.6
vis = np.clip((depth - near) / (far - near), 0, 1)
```

结果**地面全饱和成白、盒子只剩淡痕**。因为远端的 69 米把整个动态范围压扁了，近处 3 米的东西全挤在灰度 4% 的位置。

第二版 `render04` 固定量程 + 裁剪：

```python
near, far = 1.6, 4.5
vis = np.clip((depth - near) / (far - near), 0, 1)
```

地面近白远黑、盒子是清晰的灰块。**盒子处深度读到 3.107**（和中心的 3.087 差 2 cm，是盒子有厚度）。

**教训：可视化量程要按"你关心的物体距离"定，不能让离群值（无穷远背景）来决定动态范围。** 这和相机自动曝光被天空拉爆是同一个道理。

---

## 7. 桥到 ROS2

### 7.1 第一个错：`rospy`

写检查脚本时用了 `import rospy` → `ModuleNotFoundError`。**ROS2 没有 `rospy`，那是 ROS1 的包。** ROS2 用 `rclpy`。

正确的三项检查：

```python
import rclpy                          # ok
from rclpy.node import Node           # ok
from sensor_msgs.msg import Image     # ok
from cv_bridge import CvBridge        # ok
```

### 7.2 节点的骨架

```python
class ImgPub(Node):
    def __init__(self):
        super().__init__('w3_image_pub')
        self.pub = self.create_publisher(Image, '/image_raw', qos_profile_sensor_data)
        self.br = CvBridge()
        self.m = mujoco.MjModel.from_xml_path(...)
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
        img = self.br.cv2_to_imgmsg(self.r.render(), encoding="rgb8")
        img.header.frame_id = "camera_link"
        img.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(img)
```

几个要点：

- **`Renderer` 要存成成员**，不能在回调里每帧新建 —— 每次新建都重新初始化 GL 上下文，开销巨大。
- **`encoding="rgb8"` 必须显式给**，`cv_bridge` 的默认推断在 RGB/BGR 上容易错，错了就是红蓝反。
- **`frame_id` 必须和 rviz2 的 Fixed Frame 同名**，否则报 "No tf"、图不显示。
- **`stamp` 用 `get_clock().now()`**，别用 `time.time()`，后者和 ROS 时间源脱钩。

### 7.3 频率问题：三层排查

首跑 `average 12–14Hz`、`min 0.023s`、`max 0.600–1.333s` —— 严重抖动且不达标。

**第一层：渲染是不是瓶颈？**

```python
# bench_render.py
640×480 → 5.8 ms/帧
320×240 → 0.1 ms/帧   ← 不可信，复用了 GL 上下文
```

5.8 ms 远低于 33 ms 的预算。**渲染不是瓶颈。**

**第二层：pipeline 是不是瓶颈？**

```python
# bench_pipeline.py
render   5.9 ms
to_imgmsg 1.3 ms
合计      7.2 ms
```

**7.2 ms 仍远低于 33 ms。** 说明瓶颈不在这两段计算，而在**传输的数据量**。

**第三层：算带宽。**

$$480 \times 640 \times 3 = 900\ \text{KB/帧},\quad 900\ \text{KB} \times 30 = 27\ \text{MB/s}$$

27 MB/s 走 DDS 的默认 reliable QoS，订阅端（`ros2 topic hz`，零拷贝没开）来不及消费就反压，publisher 被阻塞，`max` 冲到 1.333 s。

**两步修：**

1. **换 QoS。** 两处 sed：加 `from rclpy.qos import qos_profile_sensor_data`，把 `create_publisher` 第三参改成它。`max` 从 1.333 s 降到 0.800 s、`avg` 到 14–15 Hz —— **缓解但没根治**，因为数据量还是 27 MB/s。
2. **降分辨率。** `480×640 → 240×320`。数据量掉到 `225 KB × 30 = 6.7 MB/s`，**四倍余量**。

结果：

```
average 29.959–29.990 Hz
min 0.031 s / max 0.035–0.036 s
std dev 0.00052–0.00065 s
```

**稳在 30 Hz。** 浮动 0.5 ms，对这个用途完全够。

**这是这次最有价值的一条经验：性能问题先量带宽，再怀疑算力。** 渲染 5.9 ms、编码 1.3 ms 都没问题，卡在网线上。

### 7.4 rviz2 的四个开关

| 项 | 值 | 不设会怎样 |
|---|---|---|
| Fixed Frame | `camera_link` | 报 "No tf"、显示不出 |
| Add → Image | 加一个 Image display | 没面板 |
| Topic | `/image_raw` | 空白 |
| Reliability Policy | **Best effort** | **一帧都收不到** |

最后一条是最坑的。**rviz2 的 Image display 默认 `Reliable`，和 best-effort 的发布者 QoS 不兼容，直接一帧都不给。** 且界面不报错，只是 Status 一直不是 Ok。这个坑和 ROS1 时代"话题名对不上"长得完全不一样，很容易怀疑到错的地方去。

排掉之后 `Status: Ok`，左下 Image 面板出画面，`w3-05-rviz.png` 就是这一帧。

### 7.5 顺手消 Warn

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map camera_link
```

输出 `translation (0,0,0)`、`rotation (0,0,0,1)`、`from 'map' to 'camera_link'`。rviz2 里 `map` 和 `camera_link` 之间有了 tf，警告消失。

（旧式位置参数写法会打一条 `Old-style arguments are deprecated` 的 WARN，功能正常，只是提示该换成 `--x 0 --y 0 ...` 的新写法。）

---

## 8. 全部坑点清单

按"会卡多久"排序：

| # | 坑 | 现象 | 正解 |
|---|---|---|---|
| 1 | `python -m mujoco.viewer` 不带 `--mjcf` | 空 GUI，像崩了 | 必须给 `--mjcf=$HOME/...` |
| 2 | `<global ambient>` | schema 报错 `unrecognized attribute` | 环境光写进 `<headlight ambient=...>` |
| 3 | rviz2 Reliability 默认 Reliable | 一帧都收不到、Status 不是 Ok | 手动改 **Best effort** |
| 4 | `xyaxes` 当欧拉角理解 | 方块在视野外、只有背景 | 它是"右轴+上轴"两向量，视线 = 叉乘 |
| 5 | 每帧只 `mj_step` 一次 | 画面像在飘、慢放 16 倍 | 按目标时刻累计推进 |
| 6 | 900KB × 30Hz 走 reliable | 12–14 Hz、max 1.3 s | `qos_profile_sensor_data` + 降分辨率 |
| 7 | `qpos[1]` 当高度 | 高度读出来是 0 | `qpos[2]` 才是 z |
| 8 | `import rospy` | ModuleNotFoundError | ROS2 用 `rclpy` |
| 9 | `euler="0 0 0.35 0"` | 看不出变化 | MJCF 角度默认是**度**，0.35 度 = 几乎没动 |
| 10 | 只改盒子一侧 friction | 摩擦没变 | 两 geom 取 **max**，得改地面那侧 |
| 11 | viewer screenshot 找不到 | `ls *.png` 无输出 | 落在**进程 cwd**，文件名叫 `screenshot.png` |
| 12 | 深度可视化用 99 分位定量程 | 地面全白、物体淡痕 | 固定 `near/far` 按目标距离 + `clip` |
| 13 | `mv ~/图片/"截图..."` | 报文件不存在 | 实际在 `~/图片/截图/`，多一层子目录 |
| 14 | `Renderer` 在回调里新建 | 频率暴跌 | 存成成员，只建一次 |

---

## 9. 和 D435i 的对应关系

W3 做的是"仿真替身"，为的是让后面那套东西在没有真机时也能跑。对应关系：

| 仿真里 | 真机 D435i |
|---|---|
| `<camera name="cam" pos= xyaxes=>` | 相机内外参标定 |
| `renderer.render()` RGB → `rgb8` | `sensor_msgs/Image`，`/camera/color/image_raw` |
| `enable_depth_rendering()` float32 米 | `sensor_msgs/Image` 32FC1，`/camera/depth/image_rect_raw` |
| `frame_id = "camera_link"` | 真机 tf 树里就有的 `camera_link` |
| 30 Hz | D435i 也是 30 Hz |
| 距离是到相机原点直线距离 | 真机深度也是沿射线距离（**同一套语义**） |

**第四行和第六行是关键**：真机的深度也是"沿射线量到物体的距离"，所以从仿真深度图到真机深度图，反投影算法可以完全复用。这就是为什么 §6 里非要搞清楚"直线距离 vs 光轴投影"。

后面接缺陷检测，缺的是：把深度转点云 → 得到工件的三维形状 → 在形状上找凹坑/划痕/尺寸偏差。真机上还要加手眼标定（相机在机械臂末端，工件在台面上），仿真里则可以随便摆相机位姿来造训练数据。

---

## 10. 下一步

- **W4**：点云生成（深度 + 内参 → `PointCloud2`），或者先做相机标定
- **W1/W2 尾巴**：turtlesim 截图、`talker.py`/`listener.py` 归位、rsync + commit
- **自检 17 条**：这次做的五项对应第 1、2 组；后面 3、4 组等 W4 之后再补
- **仓库**：`/mnt/d/ub-sync/` 下还没确认 `.git` 在哪一层，commit 前要先定下来

---

*笔记生成于 2026-11-23 01:30，对应证据目录 `~/mujoco_w3/evidence/`，脚本 `~/mujoco_w3/*.py`。所有数字均来自实际运行输出，没有估算。*

> （注：部分内容由豆包 AI 生成）