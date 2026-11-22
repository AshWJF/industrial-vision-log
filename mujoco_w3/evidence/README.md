# W3 证据（2026-11-22 ~ 23）

五项全部完成，比规划（11.30 起）提前一周。

01  装包渲染：mujoco 3.17.0，官方 model.xml 在 viewer 里跑起来
02a rgba：红→蓝，下落时间不变，外观与物理解耦
02b camera：离屏按名字取 cam，正前方俯视 20°
02c euler：0.35 度 vs 20 度，MJCF 角度默认是度
02d ramp：摩擦 0.05 一路滑出视野（两张：下滑中 / 已滑走）
02e ramp：摩擦 1.0 卡在坡上不动（mu >= tan20 = 0.364）
04  depth：离屏 RGB + 深度 float32 单位米，盒子处 3.087 m
05  rviz：cv_bridge rgb8 + frame_id camera_link，rviz2 30Hz

结论：rgba 只改外观 / camera 只改从哪看 / friction 成对取较大值 /
      mass 不影响下落与减速，只影响惯性与碰撞动量

坑：MJCF 角度默认度；离屏渲染要在 headlight 里加环境光；
    global 没有 ambient 属性；图像话题必须 best effort，
    否则 27MB/s 会被订阅端反压到 12Hz；rviz2 Image 的
    Reliability 也要设 Best effort，否则收不到
