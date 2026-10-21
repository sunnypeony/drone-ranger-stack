# 2026-10 RGB-D + 2D LiDAR → TSDF mesh → OBJ / 带碰撞的 USD（离线流水线 v1）

手持 OAK-D（RGB + 设备端对齐深度）+ YDLIDAR T-mini Plus，在户外院子绕一张桌子走一圈（约 3 min）录制的
ROS 2 Humble rosbag，离线重建为三角网格，并导出 Isaac Sim 可直接使用的静态碰撞 USD。
不依赖 ROS / Isaac Sim 安装（`rosbags` 纯 Python 读 bag，`usd-core` 写 USD）。

数据：Hugging Face dataset `danyangh/static_table_rgbd_2dlaserscan_rosbag_3min`（`backyard_rosbag_001.tar.gz`）。

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
