# PlotJuggler 4 · LCM 插件开发工作区

## 前言

本目录是 **PlotJuggler 4（PJ4）插件的本地开发工作区**，围绕 MagicLab 机器人的 LCM 数据链路构建。它主要解决两件事：

- **离线分析**：把机器人录制的 `.lcmlog` 文件加载进 PlotJuggler，按时间轴查看曲线、状态量和诊断信息；
- **在线调试**：通过 LCM 网络实时订阅机器人通道，在 PlotJuggler 中观察实时数据。

工作区从 PlotJuggler 官方插件仓库（`pj-official-plugins`）派生，使用 `plotjuggler_sdk` 的 **v4 插件 ABI**。与上游通用插件不同，这里的重点是：

- LCM 消息的解码与字段展开；
- 机器人各子系统（locomotion、manipulation、ecat、bms、hal 等）的 LCM 通道配置；
- 面向 X1 机器人的运行时分发与打包。

> 说明：`data_load_lcm` 的 `README.md` 和 `manifest.json` 目前仍保留了从 MCAP 加载器继承过来的文案。对外发布前，需要把它们替换为 LCM 加载器的实际说明和元数据。

---

## 目录结构

| 路径 | 说明 |
|---|---|
| `data_load_lcm/` | **离线 LCM Loader 插件**。读取 `.lcmlog`，通过 `lcmgen` / `tokenize` 解码 LCM 类型，再写入 PJ4 的 catalog 与曲线树。 |
| `data_stream_lcm/` | **在线 LCM Server 插件**。实时订阅 LCM 通道，支持单播/组播 IPv4/IPv6；包含 `robot_state_publisher`、`lidar_subscriber` 等机器人相关逻辑。 |
| `common/` | 多个插件共享的辅助库：`base64`、`can_dbc`、`grid_map`、`laser_scan`、`pointcloud_color`、`query`、`streaming_utils`、`transfer_rate`、`toolbox_preview` 等。 |
| `cmake/` | 构建辅助：manifest/UI 嵌入、导出符号检查、CPM 等。 |
| `scripts/` | 插件发布、扩展注册表提交、版本检查等脚本。 |
| `build/` | Conan/CMake 构建产物（本地生成，不应提交）。 |
| `pack/` | 运行时分发包：`lcm_plugin/`、`lcm_config/`、依赖库、文档等。 |
| `build.sh` | 构建入口，支持构建单个插件、ASan/TSan、`--sdk-local`。 |
| `pack.sh` | 构建两个 LCM 插件并打包为 `pack.tar.gz`。 |
| `SDK_VERSION` | 当前使用的 `plotjuggler_sdk` 版本（`0.34.1`）。 |

---

## 两个核心插件

### `data_load_lcm` — LCM Loader（离线）

- 入口：`LcmSource::importData()`
- 输入：`.lcmlog`（LCM 日志）
- 解码：使用 `lcmgen.c` / `tokenize.c` 解析 `.lcm` 类型定义，并按 LCM 消息 hash 匹配结构体
- 写入：通过 SDK 的 `writeHost().ensureTopic(...)` / `appendRecord(...)` 把每个标量写入 PJ4 catalog
- 典型 topic 形态：`leg_control_data/q/0`、`arm_control_data/joint_curPos/3` 等

### `data_stream_lcm` — LCM Server（在线）

- 入口：`lcm_stream_source.cpp`
- 传输：LCM 单播/组播（IPv4/IPv6）
- 额外逻辑：`robot_state_publisher`（根据关节数据发布 TF）、`lidar_subscriber`（雷达数据订阅）
- 面向实时调试：在 PlotJuggler 中直接观察机器人在线通道

---

## 快速开始

```bash
# 构建单个插件
./build.sh data_load_lcm
./build.sh data_stream_lcm

# 构建并打包
./pack.sh          # 生成 pack.tar.gz
```

## 与上游的关系

- 上游插件仓库：`https://github.com/PlotJuggler/pj-official-plugins`
- 上游 SDK：`plotjuggler_sdk`
- 本工作区的改动重点：LCM 编解码、机器人通道、X1 打包；通用插件逻辑尽量保持与上游一致，便于后续同步。

---

*本文件是 `tmp/` 工作区的总览，具体插件的使用说明请查看各插件目录下的 README / CHANGELOG。*
