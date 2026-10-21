# 2026-10 RGB-D + 2D LiDAR → TSDF mesh → OBJ / 带碰撞的 USD（离线流水线 v1 + v2）

> v1：RGB-D 视觉 odometry + 地面约束 + Open3D 位姿图（下文前半部分，结果在 `results/`）。
> v2：在 v1 之上加入 2D LiDAR 约束（x/y/yaw），结果在 `results/v2/`，见文末 **v2** 一章。
> v3：时间截止 175 s + 仅用免位姿回环（留出评估）+ 模糊剔除，结果在 `results/v3/`，见文末 **v3** 一章。

手持 OAK-D（RGB + 设备端对齐深度）+ YDLIDAR T-mini Plus，在户外院子绕一张桌子走一圈（约 3 min）录制的
ROS 2 Humble rosbag，离线重建为三角网格，并导出 Isaac Sim 可直接使用的静态碰撞 USD。
不依赖 ROS / Isaac Sim 安装（`rosbags` 纯 Python 读 bag，`usd-core` 写 USD）。

数据：Hugging Face dataset `danyangh/static_table_rgbd_2dlaserscan_rosbag_3min`（`backyard_rosbag_001.tar.gz`）。

**数据说明（截取）**：这个 bag 是从一段更长的录制中**手动截取**的，末帧时刻是任意的，不代表实验结束，
也**不存在“首尾应闭合”的约束**。v1/v2 的强制首尾回环和“首尾距离”指标都不应再作为正确性依据。
v3 起位姿估计、TSDF 和评估只用前 `cutoff_s = 175.0` s（`configs/v3.yaml`），回环只来自免位姿测量。

## 流水线

```
 HF dataset ──00_download──► data/raw/<bag>/ (metadata.yaml + .db3)
                                   │
                          01_inspect_bag ──► outputs/inspect/report.json, rgb.png, depth_color.png, frame0.ply   [检查点 1]
                                   │
                        02_extract_frames ──► outputs/frames/{color,depth}/%06d.png, frames.csv, intrinsics.json
                                   │
                        03_estimate_poses ──► outputs/poses/poses.npz (T_world_optical), trajectory.csv (TUM),
                                   │           trajectory_topdown.png, pose_report.json, accumulated_preview.{ply,png}   [检查点 2]
                                   │
                        04_integrate_tsdf ──► outputs/tsdf/mesh_raw.ply, tsdf_pointcloud.ply
                                   │
                       05_postprocess_mesh ──► outputs/mesh/mesh_clean.{ply,obj}, T_align.npy, poses_aligned.npz
                                   │
                          06_export_usd ──► outputs/usd/scene_collision.usdc (+ _convexdecomp.usdc) → results/
                                   │
                     (07_lidar_check, 可选) ──► outputs/lidar/lidar_vs_mesh.png, lidar_report.json
```

核心公式：`world_P = T_world_optical(t) · camera_P`。Open3D `integrate()` 的 extrinsic 是 world→camera，
即 `np.linalg.inv(T_world_optical)`。约定 `T_a_b` 把 b 系的点变到 a 系。

## 运行

```bash
cd experiments/2026-10-rgbd-lidar-sync
python3.12 -m venv .venv          # open3d 0.19 只有 3.10–3.12 的 wheel（3.13 没有）
.venv/bin/pip install -r requirements.txt
# 无显示器的服务器若 import open3d 报 libGL/libgomp：sudo apt install libgl1 libgomp1 libegl1
./run_all.sh                      # 00–06，任何阶段失败即退出
./run_all.sh --with-lidar         # 再加 07
./run_all.sh --from 3             # 从某个阶段继续
.venv/bin/python scripts/03_estimate_poses.py --config configs/default.yaml   # 单独跑某一阶段
.venv/bin/python scripts/verify_usd.py outputs/usd/scene_collision.usdc
./run_v2.sh                       # v2（需先跑完 v1）：03b → 04–07 → 08 消融评估 → 09 Isaac 测试场景
```

私有 HF 仓库：`export HF_TOKEN=...`。所有路径、topic、frame 名、参数都在 `configs/default.yaml`。
`data/`、`outputs/` 不进 git；`results/` 只放最终交付物并提交。

长任务与断点续跑：
- 03：odometry 每 `checkpoint_every` 对写一次 `outputs/poses/ckpt_odometry.npz`，地面平面写
  `ckpt_ground.npz`，回环配准结果缓存到 `ckpt_loops.json`；重跑时跳过已完成部分。
  改了 odometry 门限后，已有结果会用新门限重新判定，只重算不合格的对。
  改了影响初值的参数（地面约束等）后应删除 `ckpt_loops.json`。
- 04：legacy `ScalableTSDFVolume` 不能序列化，因此“续跑” = 输入（tsdf 参数 + poses.npz）hash 不变时跳过整个阶段；
  中途中断只能重新融合（2700 帧约几分钟）。`progress.json` 只是进度快照。

## 各阶段说明与检查方法

| 阶段 | 关键参数 | 如何检查 |
|---|---|---|
| 00 | `download.*` | 终端打印 bag 目录与大小；`.tar.gz` 自动解压（拒绝路径穿越成员） |
| 01 | `inspect.*` | `report.json`：topic 频率、encoding（depth 必须 16UC1/32FC1，否则退出）、K/D、stamp 匹配率、tf_static、odom 覆盖率、深度有效率；看 `frame0_projections.png` 地面是否平、尺度是否合理 |
| 02 | `extract.*` | `extract_report.json`：配对数/丢弃数、保留帧数、磁盘占用 |
| 03 | `poses.*` | `trajectory_topdown.png`（起点绿、终点红）、`accumulated_preview.png`（第 3 幅高度切片按时间着色，物体不应出现重影；第 4 幅侧视图地面应是一条细线）、`pose_report.json` |
| 04 | `tsdf.*` | 终端/`done.json`：三角面数、bbox；可在 MeshLab 中打开 `mesh_raw.ply` |
| 05 | `mesh.*` | `mesh_report.json`、`mesh_views.png`（俯视+两个侧视，地面应在 z=0） |
| 06 | `usd.*` | `verify_usd.py` 打印 prim 树、CollisionAPI / MeshCollisionAPI、approximation |
| 07 | `lidar.*` | `lidar_vs_mesh.png`、`lidar_report.json`（scan 点到网格的距离中位数/90 分位） |

### 02 抽帧细节
- 按 `header.stamp` 精确配对（`match_tolerance_ns: 0`；>0 时对未精确匹配的 RGB 做最近邻 fallback）。
- 每 `frame_stride`（默认 2）对保留一帧；颜色按 encoding 处理（`bgr8`→RGB，写 PNG 时再转回 BGR 给 OpenCV）。
- 深度统一为 uint16 毫米：16UC1 按 mm（`depth_16uc1_scale`），32FC1 按 m（REP 117）；
  超出 `[depth_min, depth_max]` 或非有限值置 0。可选 median/bilateral 去噪，默认关。
- **去畸变** `undistort: auto`：D 非零时用 `initUndistortRectifyMap` 生成同一组 map，RGB 用线性插值，
  depth 用**最近邻**（避免前景/背景边缘处插值出不存在的“飞点”深度），并写出新 K（`intrinsics.json` 的 `K`，
  原始值在 `K_original`）；D 全零时跳过。本数据 D 全为零 → 未去畸变。

### 03 位姿
- `pose_source: auto | bag_tf | open3d`。auto：bag 中 odom→base_link（/tf 或 /odom）覆盖率 > 90% 用 bag_tf，否则 open3d。
  `compare_both: true` 时两者都可用就都算并输出对比（ATE）。
- bag_tf：`T_odom_optical = T_odom_base(t) · T_base_optical`，平移线性、旋转 slerp 插值，覆盖范围外或跨越
  `tf_interp_max_gap_s` 的帧丢弃；world = odom（Z 朝上）。
- open3d：
  1. 相邻关键帧 `compute_rgbd_odometry`（Hybrid term，金字塔 `[20,10,5]`），初值依次试单位阵和上一步速度。
     结果需通过步长门限（≤0.08 m、≤8°/步）和点云 fitness 校验；不通过时用 colored ICP 兜底，再不行用最近
     若干个好步的中位数做匀速外推（弱边，记为失败）。
  2. **地面平面约束**（`ground_constraint: true`，假设院子地面近似平）：每个关键帧在图像下半部 RANSAC 拟合地面，
     得到相机系地面法向和相机离地高度；先用长窗口中位数剔除离群拟合，再平滑，修正每帧 roll/pitch 使地面法向 = +Z，
     用修正后的旋转重新积分平移增量，并用离地高度固定 Z。yaw 和水平位置仍由 odometry + 回环决定。
     这是对“纯视觉 odometry 在户外俯仰/横滚漂移大”的针对性处理（见下文实测数据）。
  3. 回环：时间间隔 > `loop_min_gap_s`、当前估计距离 < 1 m、朝向差 < 45° 的关键帧对，
     先 RGB-D odometry 再多尺度 colored ICP，通过 fitness/RMSE 门限的作为 uncertain edge；
     并强制尝试首尾各 `loop_force_head_tail` 帧（子采样）。
  4. `PoseGraph` + `GlobalOptimizationLevenbergMarquardt`（line process 剪掉不一致回环），之后再做一次地面约束。
  5. 有 `T_base_optical` 时 world = t0 时刻 base_link（Z 朝上）；否则 world = 首帧 optical（05 再找平）。
- `pose_report.json`：首尾距离、回环边数（接受/剪枝后）、odometry 失败数、ICP 兜底数、路径长度、
  地面倾角残差（odometry / 地面修正后 / PGO 后 / 最终）。

### 04 TSDF
`ScalableTSDFVolume`（RGB8），voxel 0.02 m，sdf_trunc 0.08 m（=4×voxel），depth_scale 1000，
depth_trunc = `extract.depth_max`（5 m）。

### 05 网格后处理
去退化/重复三角形与顶点、非流形边 → `cluster_connected_triangles` 保留最大分量和面积 > 2% 的分量 →
`simplify_quadric_decimation` 到 `target_triangles`（200k）→ 地面找平（多次 RANSAC，取与当前上方向夹角
< 35° 且内点最多的平面；旋转到 +Z、平移到 z=0；检查相机轨迹在地面之上）→ `T_align.npy` 同时作用于 poses
（`poses_aligned.npz`）。`mesh_clean.ply` 含顶点色，`mesh_clean.obj` 只有几何+法向。
若交付文件 > `max_result_file_mb`（90 MB），自动把 target_triangles 减半直到满足，并记录在 `mesh_report.json`。

### 06 USD
`metersPerUnit=1`、`upAxis=Z`、`defaultPrim=/World`；`/World/Scene/mesh` 为 `UsdGeom.Mesh`
（points / faceVertexCounts / faceVertexIndices / normals(vertex) / `primvars:displayColor`(vertex) / extent）。
`UsdPhysics.CollisionAPI` + `UsdPhysics.MeshCollisionAPI`，`physics:approximation = "none"`（静态三角网格碰撞），
**不加 RigidBodyAPI**（静态环境）。另输出 `scene_collision_convexdecomp.usdc`（`convexDecomposition`）用于对比。

### 07 LiDAR 检查（可选）
用 scan 的 header.stamp 在相机轨迹上插值得到 `T_world_optical`，乘 `inv(T_base_optical)·T_base_laser` 得到
激光系位姿（已含 `T_align`），把扫描点投到 world，与网格在激光平面高度的切片做俯视叠加，并用
`RaycastingScene` 计算 scan 点到网格的距离。
**注意**：相机 stamp 是主机 `now()`，LiDAR 与相机之间的时间映射未标定，偏差属预期；
此检查用于发现大错误（外参错、轴向镜像、尺度错），不是精度评估。

### 辅助工具：首尾裁决与位姿评估
- `scripts/check_head_tail.py` → `outputs/head_tail/`：**不依赖任何位姿**判断是否回到起点。
  首段/尾段各取 5 帧，两两 FPFH + RANSAC（单位阵初值）+ 点到面 ICP；先去掉地平面（草地对绕法向的任意
  偏航都能拟合，带地面时 fitness 无法区分 180° 翻转）。首段相机静止，所以每个尾帧有 5 次重复测量，
  `per_tail_direct.spread_max_m` 就是它的可信度。另有对末尾 400 帧的扫描 `head_tail_sweep.png`。
  RANSAC 在多线程下不可复现，噪声帧多次运行会差 0.2–0.3 m，只信 spread 小的结果。
- `scripts/eval_poses.py [--poses name=path ...]` → `outputs/eval/compare.json`：对多套位姿算同一组指标。
  只有 `head_tail_reference_error`（与上面免位姿测量的差）能看到全局漂移；相机高度/地面带对用了地面约束的
  位姿天然有利；scan→mesh 用各自的网格且只取相机视场内的点，只反映局部一致性。

## 常见失败现象 → 原因

| 现象 | 最可能原因 | 排查 |
|---|---|---|
| 重影 / 物体出现两份 | 位姿错（odometry 漂移、错误回环） | `accumulated_preview.png` 切片图；`pose_report.json` 失败数、回环数 |
| 整体尺度错（桌子 10 倍大/小） | 深度单位错（16UC1 mm vs 32FC1 m） | `frame0_projections.png`；`extract.depth_*_scale` |
| 场景斜着 / 地面不水平 | 未找平或地面平面选错 | `mesh_views.png`；`mesh.level_ground`、`ground_max_angle_deg` |
| 地面分层、侧视有多条地面线 | 俯仰/横滚漂移 | `tilt_residual_*`；`poses.ground_constraint` |
| 空洞 | `sdf_trunc` 太小，或深度无效（天空、草地反光、超出 depth_max） | 01 的深度有效率；增大 `sdf_trunc` / `depth_max` |
| 网格外缘大量碎片/漂浮物 | 远距离深度噪声 | 降低 `depth_max`，或提高 `min_cluster_ratio` |
| rosbags 报 `Storage plugin '' not supported` | metadata.yaml 中 `storage_identifier: ""` | 已自动处理：在 `outputs/bag_metadata_fix/` 写修正副本 + symlink，不改原始数据 |

## Isaac Sim 验证步骤

1. Isaac Sim → File → Open `results/scene_collision.usdc`（或作为 reference 拖入已有 stage）。
2. 检查 Stage 单位为米、Z 朝上；`/World/Scene/mesh` 在 Property 面板中有 Physics → Collider，
   Approximation = None（Triangle Mesh）。
3. Window → Visual Settings / 右上角眼睛 → Show By Type → Physics → Colliders 打开碰撞可视化，确认碰撞体与网格重合。
4. Create → Physics → Physics Scene（若 stage 没有）；Create → Shape → Sphere，添加 Rigid Body with Colliders Preset，
   放到桌子上方约 1 m，按 Play：球应落在桌面/地面上而不是穿过去。
5. 对比 `scene_collision_convexdecomp.usdc`：凸分解会把桌腿间、桌下空间“填实”，一般用 none（三角网格）作为静态环境。
6. 若用 Pegasus/PX4 SITL：把 USD 作为环境 reference 加入，确认无人机初始位置不在网格内部。

## 已知局限

- **bag 中没有 /tf 动态变换、/odom、/odom_info**，只能走 open3d 视觉 odometry；bag_tf 分支与对比功能在本数据上未被实际验证。
- `/tf_static` 发布了两次，`base_link→laser_frame` 两次值不同：第一次 (0.125, 0, 0.025)，第二次 (0, 0, 0.02)
  （stamp 更早，疑似另一 launch 的默认值）。按采集者确认，以**第一次**为准（`frames.tf_static_policy: first`），
  冲突记录在 `report.json → tf_static.conflicts`。
- 户外：深度中位数约 5.7 m，[0.3, 5] m 内有效像素约 36%；天空与远处树木无深度。
- 后段（约 1850–2475 帧）有明显运动模糊，且部分帧近处只有桌子（有效像素约 15%），RGB-D odometry 在这里最弱。
- 地面约束假设地面近似水平面；坡地、台阶场景需关闭 `poses.ground_constraint`。
- 03 的 colored ICP / 回环门限是针对本数据调的，换场景可能需要调整。
- 04 不支持真正的中途续跑（见上）。
- 07 的时间偏差未标定，见上。
- 与任务书默认值的偏差：`depth_max` 4.0 → **5.0 m**（采集者要求）；`mesh.min_cluster_ratio` 0.02 → **0.001**
  （本数据中桌子+盆栽所在分量只占总面积 0.34%，2% 会把桌子删掉）；新增地面约束、odometry 几何校验与 colored ICP 兜底。

## 本次运行结果（2026-10-21，真实数据，CPU 4 核）

| 项 | 值 |
|---|---|
| bag | 6.5 GB sqlite3，180.0 s；RGB/depth/camera_info 各 5401 条 @30 Hz，/scan 1776 @10 Hz，无 /tf /odom |
| 深度 | 16UC1，640x400；D 全零 → 未去畸变；K fx=553.78 fy=553.90 cx=329.02 cy=189.53 |
| 02 | 5401 对全部精确匹配，丢弃 0；stride 2 → **2701 帧**，1.6 GB |
| 03 | pose_source=open3d（auto），2701 帧全部有位姿；odometry 2700 对：ICP 兜底 28、外推失败 4；回环候选 849+25（强制首尾），接受 334，剪枝后 215 |
| 03 地面倾角残差 | 纯 odometry 中位 15.6° / p90 23.3° → 地面约束后 0.45° / 1.04° |
| 03 首尾距离 | **1.43 m**（纯 odometry 1.83 m），路径 32.8 m |
| 04 | 2701 帧融合，2,576,912 三角面，用时 79 s |
| 05 | 保留 42 个分量（面积 63%），减面到 **200,000 三角面 / 103,458 顶点**；找平残余 0.19°，相机离地中位 0.81 m |
| 06 | `results/scene_collision.usdc`（4.5 MB），CollisionAPI + MeshCollisionAPI(approximation=none)，无 RigidBody |
| 07 | 4 m 内 scan 点到网格距离中位 **0.48 m**，p90 0.91 m |

**质量结论（如实）**：
- 首尾距离 1.43 m > 0.3 m。首尾帧之间的直接配准（强制回环边）测得两者相距约 1.4 m，且这些回环边优化后残差小
  （中位 2.3 cm / 0.4°），说明这很可能是采集者结束位置确实离起点约 1.4 m，而非单纯漂移；但仍不能排除整体漂移。
- `accumulated_preview.png` 的高度切片中仍可见**重影**（早/中段与后段的桌子、栅栏不完全重合），
  LiDAR 叠加图中栅栏呈双线、LiDAR 桌子与网格桌子偏约 0.4 m —— 剩余误差主要是 yaw / 水平位置。
- 网格：地面平整（z=0），栅栏、树干、桌面+盆栽可辨；**桌腿未重建**（细金属腿 + 2.5 m 处深度噪声 + 位姿误差）。
- 07 的 yaw/镜像扫描显示不存在大的激光外参错误（镜像、90°/180° 都更差）；最佳点 yaw≈−9°，证据很弱。

下一步建议：用 2D LiDAR scan matching 提供 yaw/xy 约束（与 RGB-D odometry 融合），或重新采集时放慢转身、避免运动模糊。

`results/` 中的文件：`scene_collision.usdc`、`mesh_clean.obj`、`mesh_clean.ply`、`trajectory_topdown.png`、
`lidar_vs_mesh.png`、`report.json`（01）、`pose_report.json`（03）。均 < 90 MB，未触发自动减面。


---

# v2：加入 2D LiDAR 约束（x / y / yaw）

```
v1 checkpoints (odometry, ground planes, visual loops) ──► 03b_lidar_refine ──► outputs/v2/poses (+ ablation/<variant>/poses)
                                                              │  blur 剔除 · LiDAR 预处理 · 标定诊断 · SE(2) 位姿图
                                                              ▼
                                      04 → 05 → 06 → 07（configs/v2.yaml：outputs/v2/，results/v2/）
                                                              ▼
                               08_evaluate_v2（每个变体建网格 + 留出评估 + 对比图） → 09_test_scene（Isaac 掉落测试）
```

所有 v2 参数在 `configs/v2.yaml`（`base: default.yaml` 继承 v1 配置，只写差异）；任何脚本都可用
`--set key=value` 临时覆盖参数（08 用它给每个消融变体建网格）。v1 的 `results/` 不受影响。

## 方法

**解耦优化**：roll / pitch / z 来自 v1 的地面平面约束（每帧 RANSAC 地面），保持不变；
x / y / yaw 在 SE(2) 位姿图中重新估计，最后合回 6DoF：`T_world_base = [Rz(yaw)·R_rp, (x, y, z)]`，
`T_world_optical = T_world_base · T_base_optical`。

为什么用 SE(2) + `scipy.optimize.least_squares` 而不是 Open3D 6DoF PoseGraph：LiDAR 只约束平面内 3 个自由度，
在 Open3D 里要把 roll/pitch/z 方向的信息置零，信息矩阵会奇异/病态，且 Open3D 的 line process 只对
“uncertain” 边做鲁棒化；scipy 版本可以对所有边统一用 Cauchy 核，并且残差/雅可比完全可控。

**一个实现上的坑（已修）**：最初在全部 2701 个位姿上直接优化，`trf + lsmr` 在 100 次评估内几乎不动
（status=0，视觉回环残差中位 15σ）；改为**关键帧图**：每 `pg_node_stride=10` 个位姿一个节点，节点间的
视觉 odometry 由链精确复合（σ 按 √步数增长），其他测量按“最近节点 + 链上偏移”挂到节点上，
优化后把每个节点的修正量 `X_n ∘ init_n⁻¹` 按时间插值回每一帧；`tr_solver=exact`，所有变体都收敛（status=2）。

边类型与 σ（`configs/v2.yaml`）：视觉 odometry 0.5 cm / 0.25° 每步（失败外推步 ×10）、视觉回环 5 cm / 1°、
LiDAR 相邻 2 cm / 0.5°、LiDAR 回环 5 cm / 1°；Cauchy，`f_scale=2`（白化残差）。

### 1. 模糊帧
每帧灰度 Laplacian 方差，阈值 = 全序列 10 分位（1507.8）：**270 帧**被标为模糊，分布在 113 段，
最长一段 15 帧；其中 107 帧落在 v1 发现的问题段 1850–2475（各 300 帧区间：0,0,26,42,52,17,23,83,27,0）。
模糊帧不做 odometry 匹配源（清晰帧之间重新做 odometry：2430 对，复用 v1 2317 对，ICP 兜底 39、外推失败 16）、
不进 TSDF（`poses.npz` 里的 `tsdf_mask`），位姿由相邻清晰帧插值。

### 2. LiDAR 预处理
- 范围 r ∈ [0.6, 8.0] m；外参固定为实测 `T_base_laser = (0.125, 0, 0.025), yaw 0`（`lidar.T_base_laser`，不再用 /tf_static）。
- “操作者身体”自动检测（5° 扇区内近距离回波 < 1 m 出现在 ≥ 50% 的 scan 中）：得到
  **[−135°, −85°]、[−75°, −55°]、[60°, 75°]、[120°, 135°]**。这些扇区的近距离回波中位距离只有 **0.06–0.22 m**，
  持续率达 100%，说明它们是**设备自身结构遮挡**（支架/手柄/相机壳），不是操作者；0.6–1 m 范围内没有任何扇区
  持续出现回波（其余扇区最高 0.32），即没检测到稳定的“人体”扇区。这些扇区整体剔除。
- 投影：scan 时刻（stamp + time_offset）在相机轨迹上插值，用该时刻已修正的 roll/pitch 把点变到水平化 base 系，
  丢掉离地 < 0.2 m 或 > 2.0 m 的点，取 xy。
- scan 按索引奇偶交替分成两份：**偶数用于约束，奇数只用于评估**（888 / 888）。

### 3. 标定网格搜索（诊断）
- **time_offset ∈ [−0.3, 0.3] s**：目标函数 = 相隔 1 s 的 scan 在轨迹投影下的互相一致性。曲线单峰、
  最优 **+0.10 s**（对比度 16%）→ **已采用**。（以 v1 网格为参照的目标函数基本平坦，不可用。）
- **yaw_offset ∈ [−10°, 10°]**：一致性目标几乎平坦（对比度 1.5%）、网格目标单调到边界（−9°/−10°）→
  **没有清晰峰，不提示修改，config 保持 0°**。另做了“同一时刻深度切片 vs scan”的无轨迹配准，也看不到大的 yaw 偏差。
- 曲线：`results/v2/lidar_calib.png`。

### 4. LiDAR 约束
- 关键帧：每 0.3 m 或 10°（共 76 个），每个关键帧用前后各 2 个约束 scan 拼成子图（视觉 odometry 局部精度足够）。
- 相邻关键帧：2D point-to-line ICP（numpy/scipy KD-tree 自实现，Huber），以视觉 odometry 为初值；
  接受 **71 / 75**（fitness 1、修正量过大 3 条被丢弃）。相对视觉 odometry 的修正中位 5.4 cm / **2.0°**。
- 回环：候选 = 当前估计距离 < 1.5 m 且时间间隔 > 20 s，每关键帧最多 3 个；correlative scan matching
  （±30°、±0.5 m，似然场）粗搜 + ICP 精配；接受条件：fitness ≥ 0.45、RMSE ≤ 5 cm、峰值唯一性（最优 / 峰外最优 ≥ 1.15）、
  与附近视觉回环一致（0.3 m / 5°）。2 轮（第 2 轮用第 1 轮优化后的位姿重新找候选）。
  `lidar_loops` 变体：候选 229，**接受 51**，匹配失败 56，**峰值不唯一 100**（户外栅栏/树叶重复结构），与视觉回环矛盾 22。
- 视觉回环审计（`full`）：对 334 条 v1 视觉回环，在两端附近取约束 scan 子图，以视觉回环为初值 ICP（失败再 CSM）；
  一致 → 保留，差 > 0.3 m / 5° → 剔除，LiDAR 无法确认 → σ×3 降权。结果：**保留 79、剔除 91、降权 139、
  端点为模糊帧而丢弃 25**。

### 5. 评估（避免自证）
所有变体用**同一批留出 scan**（奇数索引，从未进入约束）、同一批深度帧（每 10 帧、非模糊）、同一个 time_offset：
- `heldout_fov`：留出 scan 中**相机视场内**（|beam| ≤ 30°，由 K 算出）且 ≤ 4 m 的点到该变体网格的距离。
  这是主指标：LiDAR 是 360°，视场外的点处没有网格，会被量到 0.8 m 下面的地面上，所以“全角度”版本
  （`heldout_all`，也列出）对所有变体都约 0.4–0.5 m，几乎没有区分度。
- `crisp`：留出 scan 在该变体轨迹下拼成的 2D 地图的锐度（每点到 > 20 s 之外的留出点的最近距离中位数）。
- `closure`：用**留出 scan** 直接配准起点和终点（宽范围 CSM ±2 m/±45° + ICP，fitness 0.53）得到参考：
  终点相对起点 **1.02 m**。closure = 估计的起点→终点相对位姿与该参考之差。
  （首尾距离本身在这段录制里**不是误差指标**：采集者结束时离起点约 1 m。）
  独立交叉验证：纯相机、免位姿的 `scripts/check_head_tail.py`（去地面 FPFH+RANSAC+ICP，5 个静止首帧重复测量）
  给出：第 2651 帧回到距起点 0.17 m，之后离开；2663 帧距起点 0.60 m（spread 0.10 m，可信），末帧 0.80 m
  （spread 0.98 m，不可信）。两种独立方法都说明终点离起点约 0.6–1.0 m，既不是 SE(2) 变体的 0.1 m，也不是 v1 的 1.43 m。
- `slice_cells`：0.4–1.2 m 高度带投到 5 cm 栅格的占据格子数（越少越清晰）。
- `table_tris`：原始 TSDF 网格中桌子区域的三角面数（桌子被不一致视角“掏空”时会骤降）。
- `tilt`：倾角误差中位数。

## 消融结果（真实数据，2026-10-21）

| 变体 | heldout_fov 中位 / p90 [m] | heldout_all 中位 [m] | crisp [m] | closure xy [m] / yaw [°] | 首尾 [m] | slice_cells | table_tris | tilt [°] |
|---|---|---|---|---|---|---|---|---|
| v1（Open3D 6DoF PGO） | 0.309 / 0.595 | 0.489 | 0.026 | **0.50** / 11.1 | 1.43 | **5447** | 13147 | 0.72 |
| v1_se2（同 v1 图，SE(2) 优化，无 LiDAR） | 0.404 / 0.731 | 0.433 | 0.028 | 1.11 / 2.1 | 0.11 | 5614 | 10660 | 0.59 |
| lidar_adj（+ LiDAR 相邻边） | 0.232 / 0.680 | 0.515 | 0.023 | 1.10 / 1.9 | 0.11 | 5695 | 11494 | 0.61 |
| **lidar_loops（+ LiDAR 回环）← 交付** | **0.198** / 0.603 | **0.410** | 0.019 | 1.12 / 1.2 | 0.11 | 5897 | **17156** | 0.68 |
| lidar_loops_blur（+ 模糊剔除） | 0.766 / 0.869 | 0.645 | 0.018 | 1.09 / 1.7 | 0.09 | 5896 | **71** | 0.68 |
| full（+ 模糊剔除 + 视觉回环审计） | 0.770 / 0.891 | 0.497 | **0.015** | 1.04 / **0.4** | 0.10 | 5832 | **518** | 0.66 |

（`results/v2/report.json → ablation`；图：`slice_v1_vs_v2.png`、`slice_ablation.png`、`trajectory_comparison.png`、
`lidar_overlay_v1_vs_v2.png`。）

**交付选择**：`v2.final_variant: lidar_loops`（不是 `full`）。按留出指标选：相机视场内 LiDAR→网格距离
从 0.309 降到 **0.198 m（−36%）**，LiDAR 地图更锐（0.026 → 0.019），桌子区域网格更完整（13147 → 17156 面），
栅栏在叠加图里从双线变成单线。

**没有改善 / 变差的地方（如实）**：
1. **closure 变差**：v1 0.50 m → lidar_loops 1.12 m。所有 SE(2) 变体都把终点拉回起点附近（首尾 0.1 m），
   而留出 LiDAR 测得终点离起点约 1 m。原因是 v1 中“第 0 帧 ↔ 2605–2665 帧”的一批视觉回环（户外重复纹理，
   LiDAR 对这几对的 CSM 峰值唯一性只有 ≈1.0）在 SE(2) + Cauchy 下占了多数；v1 的 Open3D line process 剪掉了其中一部分。
   lidar_loops 的轨迹在起点附近因此有几段明显的跳变（`trajectory_comparison.png`）。
2. **slice_cells 没有变好**（5447 → 5897）。这个指标对深度噪声和视野外散点很敏感，和其他指标结论相反；我没有把它当主指标，但如实列出。
3. **模糊剔除和视觉回环审计都让结果变差**：单独加模糊剔除（lidar_loops_blur）或单独加审计（离线实验：全部帧 + 审计，
   table_tris 572）都会让桌子在 TSDF 里被“掏空”，heldout_fov 恶化到 0.77 m——虽然它们的 LiDAR 地图最锐（crisp 0.015–0.018）。
   也就是说：**LiDAR 越占主导，相机视角之间越不一致**。
4. `full` 对 LiDAR 回环的随机下采样敏感：两次运行（只是变体顺序不同）closure 分别为 0.28 m 和 1.04 m。
5. 1.5 cm 体素（`eval.extra_voxel`，outputs/v2/vox015）：桌面/盆栽更密，**仍然没有桌腿**（`table_region_voxel.png`）。

## 根因分析：LiDAR 与相机的旋转不一致

逐 30 s 统计相邻 LiDAR 关键帧边的 yaw 与同区间视觉 odometry 的 yaw：

| 时间段 [s] | 0 | 30 | 60 | 90 | 120 | 150 |
|---|---|---|---|---|---|---|
| LiDAR Σyaw [°] | 5 | −202 | −95 | +101 | +75 | +47 |
| 视觉 Σyaw [°] | 7 | −163 | −72 | +73 | +52 | +47 |

LiDAR 测到的转角系统性地比视觉 odometry 大约 **27%**（Σ|yaw| 比 1.27；与 time_offset 取 −0.3…+0.3 s 无关；
镜像 scan 则完全不一致）。这正好解释了上面的现象：LiDAR 权重越大，相机朝向被“多转”，从桌子两侧看到的桌子位置对不上，
TSDF 把它掏空。

两个独立证据指向 **CameraInfo 的焦距偏大**（fx = 554）：
- 无轨迹的“同一时刻深度切片 vs LiDAR scan”配准：把相机横向坐标缩放 s 后与 scan 的匹配度在 **s = 1.15** 处有清晰单峰
  （对比度 11%）→ 隐含 **fx ≈ 482**（`report.json → v2_pose_report.camera_lidar_lateral_scale`）。
  OAK-D 彩色相机 ~69° HFOV 在 640 px 宽时 fx ≈ 466，与此接近；554 对应 60° HFOV。
- 在 450–1350 帧上用 fx/1.3 重算视觉 odometry：LiDAR/视觉转角比从 1.26 降到 1.17，相邻关键帧 yaw 差中位从 2.2° 降到 1.0°。
  不过 fx 只能解释一部分（视觉 yaw 对 fx 不太敏感，深度几何项占主导），剩余差异原因未定。

**我没有修改内参**：这和 yaw 外参一样属于标定决定，需要你确认。建议 v3：用棋盘格重新标定 OAK-D 彩色相机
（或用 DepthAI 读取设备内参，核对 ROS 驱动发布的 CameraInfo 是否对应 640x400 输出），然后用新 K 重跑 02–03 和 v2。

## Isaac Sim 测试场景
`results/v2/test_drop.usda`（引用 `./scene_collision.usdc`，v2）和 `results/v2/test_drop_v1.usda`（引用 `../scene_collision.usdc`，v1），
均由 usd-core 生成：`/World/PhysicsScene`（重力 −Z 9.81）、3 个 0.2 m 刚体立方体（RigidBody + Collision + Mass 1 kg）、
`/World/DistantLight`，米制、Z 向上。立方体位置由网格中自动检测的桌子决定（v2：桌心 (3.05, −0.14)，半径 0.42 m），
向下射线检查其下方表面：桌心立方体下方 z=0.98（桌面/盆栽），桌边下方 z=1.03，地面立方体下方 z=0.04；
立方体初始高度为下方表面 + 1 m。在 Isaac Sim 中打开 usda 按 Play：红色立方体应停在桌上、蓝色在桌边、绿色落到地面。

## v2 交付物（results/v2/）
`scene_collision.usdc`、`mesh_clean.obj`、`mesh_clean.ply`（lidar_loops 变体，200k 三角面）、`test_drop.usda`、
`test_drop_v1.usda`、`report.json`（消融表 + 全部诊断）、`pose_report.json`、`trajectory_topdown.png`、
`trajectory_comparison.png`、`slice_v1_vs_v2.png`、`slice_ablation.png`、`lidar_overlay_v1_vs_v2.png`、
`lidar_vs_mesh.png`（07）、`lidar_calib.png`、`table_region_voxel.png`。均 < 90 MB。


---

# v3：时间截止 + 免位姿回环（`configs/v3.yaml`，`./run_v3.sh`）

```
pf_loop_edges（免位姿回环边，约束组 / 评估组） → 03b（消融：base/blur/pf/lidar/full/pf_blur）
  → eval_poses（同一套留出指标，含 v1） → 04 → 05 → 06 → 07 → 09（results/v3/test_drop.usda）
```
`configs/v3.yaml` 以 `base: v2.yaml` 继承，只写差异；`variants` / `pose_sets` 用 `_replace: true` 整体替换继承值。

## 做了什么
- **截止**：`cutoff_s: 175.0`，帧 0–2625（共 2626 帧）；scan 先按全序列索引分奇偶，再截止（评估与约束用同一索引）。
- **删除首尾强制约束**：v3 不使用 v1 的视觉回环（`use_v1_visual_loops: false`，其中含强制首尾对和户外重复纹理误匹配），
  不使用 LiDAR 回环（候选来自位姿估计，不是免位姿的）；`drop_forced_head_tail: true` 作为保险。
- **免位姿回环边**（`scripts/pf_loop_edges.py`）：160–175 s 每 3 帧一个候选（+ 第 2551 帧），对 5 个静止首帧
  各做一次去地面 FPFH+RANSAC+ICP，经首段内部链接换算成 `T_0_t` 的 5 个独立估计；**离散度 ≤ 3 cm / 1°** 才接受。
  75 个候选接受 **6 条**（全在 167.4–170.4 s；170.4–175 s 的都没达到离散度要求，没有加）。
  按帧号交替分组，2551 固定在评估组：约束组 2512 / 2548 / 2557，评估组 2545 / **2551** / 2554。
  **局限**：评估组和约束组在时间上只差 0.2 s，评估组不是强独立的（满足 2548/2557 几乎就满足 2551）。
- **模糊**：Laplacian 方差 < 全序列 10 分位（1520）→ 263 帧，位姿插值，不进 TSDF（`tsdf_mask`）。
- **LiDAR**：外参固定 (0.125, 0, 0.025)、旋转单位阵；time_offset 网格搜索单峰清晰 → 采用 **+0.06 s**（v2 为 +0.10 s）；
  yaw 无清晰峰，保持 0。只加相邻关键帧 x/y/yaw 边（73 关键帧，接受 68）。
- **OpenMP 线程按脚本设置**（`runtime.omp_threads_by_script`）：此前全局 1 线程的修复让 TSDF 积分从 1/60 s 变成 2.4 s 每帧；
  现在只有 03 / 03b 用 1 线程。

## 消融（0–175 s，留出指标；`results/v3/compare.json`、`table_check.json`）

| 变体 | 评估组残差 中位 [m] / [°] | 0→2551 [m] | 约束组残差 [m] | 留出 scan→mesh（视场内）中位 / p90 [m] | 切片格子 1850–2475 | 切片格子 全程 | 桌子半径 [m] / 桌面覆盖 |
|---|---|---|---|---|---|---|---|
| v1（云端 Open3D 6DoF PGO） | 0.084 / 2.3 | 0.084 | 0.108 | 0.085 / 0.226 | 4729 | 8775 | **0.30 / 0.68** |
| base（odometry + 地面，无回环） | 0.473 / 19.0 | 0.471 | 0.468 | 0.068 / 0.213 | 4925 | 8903 | 0.47 / 0.39 |
| + 模糊剔除（blur） | 2.128 / 46.3 | 2.128 | 2.118 | **0.060 / 0.183** | 4670 | 9806 | 0.35 / **0.75** |
| + 免位姿回环（pf） | 0.087 / 2.3 | 0.087 | 0.106 | 0.073 / 0.233 | 4848 | 8748 | 0.45 / 0.43 |
| + LiDAR（lidar） | 1.018 / 26.8 | 1.018 | 1.006 | 0.229 / 0.318 | 5040 | 9728 | （未建网格） |
| 全部（full） | 0.084 / 2.2 | 0.084 | 0.106 | 0.167 / 0.268 | 5127 | 9694 | **没检测到桌子** |
| pf + 模糊（pf_blur，**交付**） | **0.082** / 2.2 | **0.082** | 0.106 | 0.064 / 0.211 | **4525** | **8536** | 0.60 / 0.53 |

桌子：真实桌面直径约 0.7 m（第 0 帧像素宽度×深度估计），半径偏大 = 多份桌子叠加（重影）。图：`results/v3/table_check.png`。
切片格子数受评估中地面 RANSAC 影响，两次运行相差 1–2%（≈100 格），小于这个的差别不可信。

## 结论（如实）
1. **主指标上 v3 没有超过 v1**：评估组残差 0.082 m（pf_blur）vs 0.084 m（v1），在噪声内持平。
   v1 在截止前本来就和免位姿测量一致（v1 的问题主要在 175 s 之后）。
2. **LiDAR 让所有指标变差**：单独加 LiDAR 后 0→2551 从 0.47 m 恶化到 1.02 m、yaw 误差 19° → 27°，留出 scan→mesh 0.068 → 0.229 m；
   `full` 里桌子完全消失。原因与 v2 相同：LiDAR 测得的转角比视觉多约 27%（本次 1.277；无轨迹诊断仍给出 fx ≈ 482 而非 554）。
   **在内参问题解决前不应使用 LiDAR 约束。**
3. **模糊剔除**：单独使用时全局漂移最大（2.1 m / 46°），但局部指标最好、桌子最完整；与免位姿回环组合（pf_blur）后全局和
   留出 scan→mesh、切片格子都最好，**但桌子比 v1 明显重影**（半径 0.60 vs 0.30 m，覆盖 0.53 vs 0.68）。
4. **根因**：6 条免位姿回环边全部在轨迹末段（167–170 s），只约束“末段 vs 起点”。SE(2) 图把这 0.5 m / 19° 的修正
   **均匀摊到整条轨迹**（每步 odometry σ 相同），而真实误差集中在运动模糊/快速转身段；中段因此被拉弯，绕桌不同侧看到的桌子对不上。
   v1 有大量中段视觉回环，所以桌子更紧凑。“回环只来自免位姿测量”这一约束下，中段没有任何回环来源。
5. **交付**：`final_variant: pf_blur`（按留出指标选，不是 `full`）。`results/v3/test_drop.usda` 中桌心正好是网格的一个洞，
   09 现在会把桌面立方体挪到最近一个落在桌面高度带的点（本次挪了 1.7 cm，下方表面 0.89 m），并在报告里记录挪动距离。
   如果更看重桌子几何，`results/scene_collision.usdc`（v1）目前仍是更好的碰撞场景。

## 建议的下一步
- 让修正落到真正出错的地方：按模糊分数 / odometry fitness / ICP 兜底给每步 odometry 不同的 σ，而不是均匀分摊。
- 中段免位姿回环：把同一套“5 个独立估计离散度”方法从“对首段”推广到任意两个静止片段（需要先找到中段的静止/慢速片段）。
- 标定 OAK-D 彩色相机内参（或读取设备内参核对 CameraInfo），之后再重新评估 LiDAR 约束。

## v3 交付物（results/v3/，均 < 90 MB，共 27 MB）
`scene_collision.usdc`（199,999 面，Collision + MeshCollision approximation=none，无 RigidBody）、`mesh_clean.obj`、`mesh_clean.ply`、
`test_drop.usda`（引用 `./scene_collision.usdc`）、`test_drop_v1.usda`（引用 v1 的 `../scene_collision.usdc`）、`compare.json`、
`table_check.json/png`、`loop_edges.json/png`、`v3_pose_report.json`、`pose_report.json`、`trajectory_topdown.png`、`lidar_calib.png`、
`lidar_vs_mesh.png`、`test_scene_report.json`。
