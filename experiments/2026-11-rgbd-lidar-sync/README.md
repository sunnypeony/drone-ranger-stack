# 2026-11 RGB-D + 2D LiDAR → TSDF mesh → OBJ / 带碰撞的 USD（新数据集：后院 11.6 min）

在新数据集上运行 **2026-10 实验定下来的 v1 流水线**，代码原样复制自
`experiments/2026-10-rgbd-lidar-sync`（分支 `exp/2026-10-rgbd-tsdf_thread` @ `0d3cfcd`），只改了配置。
为什么选 v1 而不是 v2/v3：见 2026-10 README 的 v2 @175 s 与 v3 两章（截止 175 s 后没有变体在主指标上明显
优于 v1；LiDAR 约束因相机内参疑似错误让所有指标变差；v3 的桌子比 v1 更重影）。方法细节、各阶段检查方法、
常见失败现象也都在 2026-10 的 README 里，这里不重复。

## 数据
- 本地 bag：`/home/natasha/rosbags/rosbag2_2026_09_14-16_57_35_0`（25 GB sqlite3，未上传 Hugging Face，**未截取**）。
- 696 s；RGB 20846 / depth 20848 @ 29.95 Hz（640x400，depth 16UC1，已对齐到 RGB），`/scan` 6958 @ 10 Hz，
  `/tf_static` 2 条；**没有 `/tf`、`/odom`** → 位姿由 Open3D RGB-D odometry 估计。
- 与 2026-10 数据同一台相机：K 相同（fx 553.78，D 全零 → 不去畸变）；`/tf_static` 同样把 `base_link→laser_frame`
  发布了两次，沿用 `tf_static_policy: first`（保留 (0.125, 0, 0.025)）。
- 深度：有效像素平均 60%；有效深度中 ≤ 4 / 5 / 6 / 8 / 10 m 的比例为 42 / 53 / 62 / 73 / 80%
  （帧中位深度平均 4.9 m，比 2026-10 数据远）。
- **深度图有一条固定的弱区**：所有帧在 x ≈ 280–380 px（图像中部偏右）的有效深度只有 29–49%，其余列约 70%；
  2026-10 数据各列均匀约 80%。RGB 在这里没有异常 → 很可能是**某个双目单色相机被部分遮挡或镜头脏**。
  不阻塞重建（其他视角会补上），但**下次采集前请检查设备**。

## 与 2026-10 默认配置的差异（`configs/default.yaml`）
| 键 | 2026-10 | 这里 | 原因 |
|---|---|---|---|
| `paths.bag_dir` | null（从 HF 下载） | 本地 bag 路径 | 数据只在本地；`run_all.sh --from 1` 跳过 00 |
| `download.repo_id` | HF 数据集 | null | 同上 |
| `poses.loop_force_head_tail` | 15 | **0** | 不知道这次是否回到起点；强制首尾回环在 2026-10 数据上被证明是错的 |
| `extract.depth_max` | 5.0 | **8.0** | 抽帧保留到 8 m，便于之后只重跑 04 试更远的截断 |
| `tsdf.depth_trunc` | null（= depth_max） | **5.0** | TSDF 仍只融合 5 m 内（远处立体深度噪声大）；odometry / 地面仍用 4 m |
| `poses.loop_query_stride` | 5 | **10** | 序列长 4 倍，回环候选随长度超线性增长；10 仍是每 0.67 s 查询一次 |
| `eval.pose_sets` | 2026-10 的位姿 | `v1: outputs/poses/poses.npz` | 本实验只有一套位姿 |
| `pf_loops`（v3） | 有 | 删除 | 对应脚本未复制 |

OpenMP 线程（`runtime.omp_threads`，odometry 1 / loop_registration 1 / tsdf 16 / mesh 默认）沿用 2026-10 在本机实测的值。

## 运行
```bash
uv venv -p 3.12 .venv && VIRTUAL_ENV=$PWD/.venv uv pip install -r requirements.txt
./run_all.sh --from 1 --with-lidar      # 01 检查 → 02–06（+07）；03 有断点续跑
```
本次 01 单独先跑（`outputs/inspect/report.json`），之后 `./run_all.sh --from 2 --with-lidar`。

## 代码修改（相对 2026-10）
- **04 / 05 的 `import open3d` 移到 `load_config` 之后**。原来先 import 再读配置，`runtime.omp_threads.tsdf: 16`
  在正式运行中从未生效（日志里有 `[warn] open3d already imported`，进程实际跑满约 29 线程）：
  本数据 TSDF 积分 1.6 s/帧（预计 4.7 h）→ 修复后约 22 帧/s，10423 帧 586 s。
  `bench_threads.py` 的测量不受影响（它在子进程启动前设环境变量），但 2026-10 分支上的 04 有同样的问题。

## 结果：**位姿漂移严重，网格只能看个大概，不能当可用场景**
`results/`：`scene_collision.usdc`（静态碰撞，`verify_usd` 通过）、`mesh_clean.obj/.ply`（20 万三角形）、
`mesh_views.png`、`trajectory_topdown.png`、`accumulated_preview.png`、`lidar_vs_mesh.png`、
`odometry_failure_frames.jpg`，以及 `pose_report / mesh_report / lidar_report / usd_report / report(01 检查).json`。

| 指标 | 本数据 | 2026-10 v1 |
|---|---|---|
| odometry 失败 | **843 / 10422（8%）**，ICP 兜底 426 | 4 |
| 回环 | 候选 962 → 接受 66 → 剪枝后 57 | — |
| 首尾距离 / 路径长 | 5.81 m / 215 m（不知是否回到起点，不作误差） | — |
| 倾斜残差（最终） | 中位 0.6°，p90 3.3°，最大 41° | — |
| LiDAR 到网格距离（0.6–4 m） | **中位 0.87 m，p90 1.56 m** | — |
| 网格 | 原始 1014 万三角形 → 64.8 万个连通块只留 62 个 → 抽稀到 20 万 | — |

现象：
- 三条轨迹（只 odometry / +地面 / PGO 后）彼此差几米，`accumulated_preview` 的侧视图里地面不是一条细线；
  `lidar_vs_mesh` 里同一面墙被扫成多条错开的扇形 → 位姿全局不一致，网格碎成块、有重影，看不到连续地面。
- 失败集中在序列 30–50%（约 195–350 s；最长连续 121 对 = 8 s，从第 4001 对开始），其余在 20–30%、90–100%。
- **原因是逆光**（`odometry_failure_frames.jpg`）：失败段相机正对太阳、经常仰拍天空，深度变成横竖条纹，
  有效像素只有 27–37%；再叠加上面说的 x≈280–380 px 固定弱区。这是数据问题，调参救不回来。

建议：
1. **重新采集**：背光或阴天，相机别仰拍天空，先清理/检查双目镜头（弱区）。这是最有效的一步。
2. 若只能用这份数据：可以截取 odometry 正常的片段单独重建（如前 20% ≈ 140 s，失败 15 次）；
   改 `tsdf.depth_trunc` 救不了位姿。
3. 本数据仍没有 `/odom`；若无人机有 VIO/飞控里程计，录进 bag 后流水线会直接使用（`odom_coverage`）。
