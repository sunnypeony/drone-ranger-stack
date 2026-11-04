# 2026-11 RGB-D + 2D LiDAR → TSDF mesh → OBJ / 带碰撞的 USD（多数据集）

**2026-10 实验定下来的 v1 流水线**（RGB-D odometry + 地面约束 + Open3D 位姿图 → TSDF → 网格 → USD），
代码复制自 `experiments/2026-10-rgbd-lidar-sync`（分支 `exp/2026-10-rgbd-tsdf_thread`），在此基础上：
数据集改成纯配置选择；TSDF 加了远距离第二层（重建围栏 / 房子墙）；修了几个问题（见“代码修改”）。
为什么是 v1 而不是 v2/v3、各阶段的检查方法、常见失败现象：见 2026-10 README，这里不重复。

| 数据集（`configs/<名字>.yaml`） | bag | 时长 | 结果 |
|---|---|---|---|
| **`backyard_table_0914_1840`** | `/home/natasha/rosbags/rosbag2_trimmed`（= 2026-10 的数据，HF 上也有） | 3 min，手动截取，截止 175 s | **可用**：围栏、房子墙和窗、树冠、桌上的花盆都在；见下 |
| `backyard_0914_1657` | `/home/natasha/rosbags/rosbag2_2026_09_14-16_57_35_0` | 11.6 min，未截取 | **不可用**：逆光 → 位姿漂移，网格是碎块 |

两个 bag 是同一天同一个后院（16:57 与 18:40）。

## 运行 / 选择数据集
```bash
uv venv -p 3.12 .venv && VIRTUAL_ENV=$PWD/.venv uv pip install -r requirements.txt
./run_all.sh --dataset backyard_table_0914_1840 --from 1 --with-lidar   # 01 检查 → 02–06（+07）
./run_all.sh --dataset backyard_0914_1657 --from 4                     # 只重跑 04 之后
./run_all.sh --dataset backyard_table_0914_1840 --from 4 --set tsdf.far.depth_trunc=9.0   # 临时改参数
./run_all.sh                                                           # 不带参数：列出可选的数据集
```
- `--dataset NAME` = `--config configs/NAME.yaml`。单独跑某个脚本：`.venv/bin/python scripts/04_integrate_tsdf.py --config configs/NAME.yaml`。
- **加一个新数据集 = 加一个 yaml，不改代码**：
  ```yaml
  base: default.yaml
  dataset: my_bag                 # 输出到 outputs/my_bag/，交付物到 results/my_bag/
  paths: {bag_dir: /path/to/bag}  # 或 null + download.repo_id（00 从 Hugging Face 下载到 data/my_bag/raw）
  cutoff_s: null                  # 手动截取的 bag：只用前 N 秒（02 就丢掉之后的帧）
  ```
  其余按需覆盖 `default.yaml`（它是所有数据集共用的基础，不能单独运行）。配置里 `${key}` 引用其他键，
  如 `outputs_dir: outputs/${dataset}`。
- 辅助脚本（同样只接 `--config`）：`fit_lidar_tilt.py`（LiDAR 安装角，免位姿）、`render_views.py`（网格从真实位姿
  渲染 vs RGB）、`table_check.py NAME=outputs/NAME`（桌面覆盖率）、`verify_usd.py`、`eval_poses.py`。

## 代码修改（相对 2026-10）
| 改动 | 文件 | 原因 |
|---|---|---|
| `import open3d` 移到 `load_config` 之后 | 04、05 | 原来 `runtime.omp_threads.tsdf: 16` 在正式运行中从未生效（约 29 线程）：TSDF 1.6 s/帧 → 约 22 帧/s。2026-10 分支已同步修复（`f0b39a1`） |
| 数据集配置、`${key}` 引用、`--dataset` / `--set`、日志和 USD 路径跟随 `paths.outputs_dir` | common、run_all.sh、configs | 选数据集只改配置 |
| `cutoff_s` 在 02 生效 | 02 | 之前 v1 的 03/04 不认 `cutoff_s`（只有 v3 脚本认） |
| `tsdf.far`：远距离第二个 TSDF 体 | 04 | 见下“远处为什么需要第二层” |
| `mesh.min_cluster_area_m2`：按绝对面积保留连通块 | 05 | 只按总面积比例时，远处几何让总面积变成 7 倍，桌子（1.38 m²）被当成碎块删掉 |
| `lidar.T_base_laser` 可带 pitch / roll | common | 18:40 bag 的 LiDAR 向下倾 6°（见下） |
| `fit_lidar_tilt.py`、`render_views.py`；`test_scene` 配置节 | 新增 | 免位姿检查 LiDAR 安装角 / fx；渲染对比；`table_check.py` 之前缺配置 |

## 数据集 `backyard_table_0914_1840`（可用）

### 之前（2026-10 v1）为什么没有围栏和房子
不是数据问题，是截断距离：v1 抽帧和 TSDF 都只到 **5 m**，odometry 只用 4 m。桌子在院子中间，从桌边看过去
房子墙约 **9.5 m**、门和围栏约 **8 m**、树 4–10 m。这份 bag 的有效深度里 ≤ 5 m 只占 46%，≤ 8 m 70%，≤ 10 m 81%
（抽 91 帧统计）——远处的东西在 02 就被丢了。数据本身确实好：背光（阴影里）、各列有效深度均匀（无暗列）。

### 远处为什么需要第二层（参数怎么定的）
立体深度噪声随距离平方增长。在单帧里拟合平面测得：地面 4.9 m 处残差 **5 cm**，房子墙 9.5 m 处 **20–24 cm**。
v1 的 `voxel 0.02 / sdf_trunc 0.08` 对 9 m 处的墙太窄（大部分测量落在截断带外 → 碎、空洞），而把整体调粗
（如 `0.05 / 0.25`）又会把桌子、花盆糊掉。所以分两层（同一遍读帧）：

| | 深度范围 | voxel | sdf_trunc | 用途 |
|---|---|---|---|---|
| 近层（`tsdf.*`） | ≤ 4.5 m | 0.02 | 0.08 | 桌子、花盆、椅子（= v1） |
| 远层（`tsdf.far`） | 4.0–10.5 m | 0.05 | 0.25（≈ 9.5 m 单帧噪声 1σ，TSDF 再对几百个视角平均） | 围栏、墙、树 |

远层网格中离近层网格 < 0.10 m 的三角形删掉（两层都有时用近层），其余拼进同一个 `mesh_raw.ply`。
其他：`extract.depth_max 11`、`cutoff_s 175`、`mesh.min_cluster_area_m2 0.3`（= v1 上比例阈值实际对应的面积）、
`mesh.target_triangles 1,000,000`（面积是 v1 的 7 倍）。odometry 仍只用 ≤ 4 m 深度（远处噪声对位姿只有坏处）。

### 结果（`results/backyard_table_0914_1840/`）
`scene_collision.usdc`（22 MB，静态碰撞，`verify_usd` 通过）、`mesh_clean.obj`（74 MB）/ `.ply`（38 MB，100 万三角形）、
**`render_vs_rgb.jpg`（网格从 6 个真实相机位姿渲染，上 = 照片，下 = 网格）**、`mesh_views.png`、`trajectory_topdown.png`、
`accumulated_preview.png`、`table_check.png/json`、`tilt_fit.json`、`lidar_vs_mesh.png`，以及各阶段 report json。

| 指标 | 本次 | 2026-10 v1 |
|---|---|---|
| odometry 失败 / ICP 兜底 | 4 / 31（2625 对） | 4 |
| 回环（剪枝后） | 202（未强制首尾） | 含强制首尾回环（后来证明是错的） |
| 倾斜残差（最终） | 中位 0.44°，p90 1.0° | — |
| 网格范围 | x −7.5…13.5，y −9.5…9.2 m（整个院子） | 约 −2…7 × −4…4 m |
| 原始 → 保留 → 交付三角形 | 447 万（近 185 + 远 262）→ 286 万（137 块）→ 100 万 | 263 万 → 20 万 |
| 桌面覆盖率 / 桌子半径（`table_check`） | 0.67 / 0.35 m | 0.68 / 0.30 m |
| 用时（03 / 04 / 05） | 2739 s / 96 s / 117 s | — |

看得到的（`render_vs_rgb.jpg`）：
- **围栏**：连续的木板墙，纹理和颜色对得上（第 0、385、1157 帧）。
- **房子墙和窗户**：墙面平整，窗户是墙上的深色矩形；玻璃本身没有可靠的立体深度，所以窗是“洞 / 窗帘颜色”，
  不是平整玻璃面。空调外机、门的位置对得上（第 771、2314 帧）。
- **树**：树冠是成团的块（树叶在 8 m 处每帧噪声 ~20 cm，体素 5 cm），树干可辨认，细枝没有。
- **桌子和花盆**：花盆、植物在（偏胖、细节少，100 万三角形分给整个院子）；桌面薄、相机几乎平视，
  渲染里看不出来，但 `table_check` 覆盖率与 v1 相同。桌腿（约 2 cm）没有，v1 也没有。
- **空洞**：地面有白色空洞（草地 + 远处掠射角），天空当然没有。

### LiDAR：安装是倾斜的，07 在这份数据上不能验证墙
07 的“LiDAR 到网格距离”中位 0.63 m（p90 1.33 m，比较到 10 m）看起来很差，但原因不是网格：
- `fit_lidar_tilt.py`（不用位姿：每帧深度图 vs 同一时刻的 /scan，扫描投进深度图比距离）发现
  **LiDAR 相对相机向下倾 6.0°**（roll 0.25°）。按 `/tf_static` 的水平安装，深度/LiDAR 比值在 4–6 / 6–8 / 8–12 m 是
  1.24 / 1.26 / 1.19；加上 6° 后是 **1.08 / 1.01 / 0.96**。→ **相机深度的尺度是对的**，远处围栏、墙的位置可信。
- 同一拟合里焦距缩放 ×1.0 最好（×0.85 ≈ 2026-10 怀疑的 fx≈482 明显更差）→ **2026-10 “fx 可能错了”的推测被否定**，
  当时 LiDAR/视觉转角比 1.26 的现象更可能来自这个倾角。
- 0.8 m 高、向下 6° 的扫描线约在 7.6 m 处打到草地，`lidar_vs_mesh.png` 俯视图里那一圈“墙”就是地面交线，
  所以 07 基本扫不到 8–10 m 的墙，不能拿来评价远处几何。配置里已写入 `lidar.T_base_laser: [0.125, 0, 0.025, 0, 6.0, 0.25]`。
- **建议**：把 LiDAR 装平（或重新测外参），否则它对墙几乎没用。

## 数据集 `backyard_0914_1657`（不可用）

### 数据
- 696 s；RGB 20846 / depth 20848 @ 29.95 Hz（640x400，depth 16UC1，已对齐到 RGB），`/scan` 6958 @ 10 Hz，
  `/tf_static` 2 条；**没有 `/tf`、`/odom`**。同一台相机：K 相同（fx 553.78，D 全零）；`/tf_static` 同样把
  `base_link→laser_frame` 发布了两次，沿用 `tf_static_policy: first`。
- 深度有效像素平均 60%；有效深度中 ≤ 4 / 5 / 6 / 8 / 10 m 为 42 / 53 / 62 / 73 / 80%。
- **深度图有一条固定的弱区**：所有帧在 x ≈ 280–380 px 有效深度只有 29–49%（其余约 70%）。RGB 在这里没有异常 →
  很可能是某个双目单色相机被部分遮挡或镜头脏。
- 配置：`extract.depth_max 8`、`tsdf.depth_trunc 5`、`poses.loop_query_stride 10`（序列长 4 倍）、无远层。
- LiDAR 倾角拟合（`tilt_fit.json`）：pitch −2°、roll 2.5°，但吻合率只从 0.23 升到 0.24 → 证据弱（逆光深度太噪），
  未写入配置；不过和 18:40 的 6° 不同，说明两次录制之间 LiDAR 安装可能动过。

### 结果（`results/backyard_0914_1657/`）：**位姿漂移严重，网格是碎块**
| 指标 | 本数据 | 18:40 数据 |
|---|---|---|
| odometry 失败 | **843 / 10422（8%）**，ICP 兜底 426 | 4 / 2625 |
| 回环 | 候选 962 → 接受 66 → 剪枝后 57 | 202 |
| 首尾距离 / 路径长 | 5.81 m / 215 m（不知是否回到起点，不作误差） | — |
| 倾斜残差（最终） | 中位 0.6°，p90 3.3°，最大 41° | 中位 0.44°，最大 13° |
| 网格 | 1014 万三角形 → 64.8 万个连通块只留 62 个 → 20 万 | — |

- 三条轨迹（只 odometry / +地面 / PGO 后）彼此差几米；`accumulated_preview` 侧视图里地面不是一条细线；
  `render_vs_rgb.jpg` 里没有一个视角能认出东西。
- 失败集中在序列 30–50%（约 195–350 s；最长连续 121 对 = 8 s，从第 4001 对开始）。
- **原因是逆光**（`odometry_failure_frames.jpg`）：失败段相机正对太阳、常仰拍天空，深度变成横竖条纹，有效像素
  27–37%；再叠加上面的固定弱区。这是数据问题，调参救不回来。

建议：重新采集（背光或阴天，别仰拍天空，先检查双目镜头）；若只能用这份数据，可以只重建 odometry 正常的
前 20%（≈ 140 s，失败 15 次）：`./run_all.sh --dataset backyard_0914_1657 --from 2 --set cutoff_s=140`
（会覆盖这个数据集的 outputs，最好复制成新的数据集 yaml）。

OpenMP 线程（`runtime.omp_threads`：odometry 1 / loop_registration 1 / tsdf 16 / mesh 默认）沿用 2026-10 在本机实测的值。
