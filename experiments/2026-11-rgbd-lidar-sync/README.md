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

## 结果
（运行中）
