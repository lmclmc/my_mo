# PlotJuggler 机器人数据插件

本仓库是基于 PlotJuggler SDK v4 的插件工程，面向机器人数据的离线加载和在线调试。当前重点维护三类数据源：

- `data_load_lcm`：加载 `.lcmlog` 文件并将 LCM 消息展开为 PlotJuggler 曲线。
- `data_stream_lcm`：通过 LCM 单播或组播实时订阅机器人数据，支持 IPv4 和 IPv6。
- `data_stream_dds`：通过 CycloneDDS 发现和订阅 DDS 话题，动态解析消息字段，并支持 IDL 导入、3D 机器人显示和 MCAP 录制。

项目使用 C++20、CMake 和 Conan 构建。SDK 版本固定在 [`SDK_VERSION`](SDK_VERSION)（当前为 `0.34.1`）。插件通过 PlotJuggler v4 插件 ABI 输出共享库和 `.pjmanifest.json` 清单。

## 功能概览

### LCM Loader

`data_load_lcm` 读取 LCM 日志文件，使用仓库内的 `lcmgen` / `tokenize` 代码解析消息类型，并将嵌套消息、数组和标量字段展开到 PlotJuggler 的数据树中。插件还包含机器人状态和 TF 相关处理逻辑。

输入文件扩展名为 `.lcmlog`。具体加载选项和 LCM 日志映射配置见 [`data_load_lcm/`](data_load_lcm/) 及其界面文件 `dialog_lcm.ui`。

### LCM Server

`data_stream_lcm` 用于实时 LCM 数据调试，支持：

- LCM 单播和组播；
- IPv4 和 IPv6；
- 实时通道订阅和字段展开；
- 机器人状态、TF 和雷达数据相关处理。

插件入口为 `lcm_stream_source.cpp`，构建目标为 `lcm_stream_source_plugin`。

### DDS Server

`data_stream_dds` 基于 CycloneDDS，主要功能包括：

- 发现 Domain、Partition、话题和端点；
- 配置 Reliability、Durability、History 等订阅 QoS；
- 根据 DDS Complete TypeObjects 动态展开结构体、数组、sequence、枚举、bitmask 和字符串；
- 导入本地 IDL，作为在线类型发现的补充；
- 导入 URDF / MuJoCo XML，使用 DDS 关节位置和四元数驱动 PlotJuggler 3D 机器人视图；
- 将选定 DDS 话题录制为包含 CDR 和 OMG IDL Schema 的 MCAP 文件；
- 保存和恢复 DDS、IDL、机器人模型及录制配置。

插件入口为 `dds_stream_source.cpp`，构建目标为 `dds_stream_source_plugin`。DDS 详细使用说明见 [`data_stream_dds/README.md`](data_stream_dds/README.md)。

## 目录结构

| 路径 | 用途 |
| --- | --- |
| `data_load_lcm/` | LCM 日志加载插件 |
| `data_stream_lcm/` | LCM 实时数据流插件 |
| `data_stream_dds/` | CycloneDDS 实时数据流插件 |
| `common/` | 插件共用的编码、点云、雷达、网格、查询、传输速率等辅助库 |
| `cmake/` | UI、manifest、Git 历史和插件导出符号相关的 CMake 模块 |
| `scripts/` | SDK 校验、构建辅助、发布和 Conan 工具脚本 |
| `pack/` | 运行时分发目录，包含插件和依赖库 |
| `build/` | CMake / Conan 构建目录，不应作为源码提交 |
| `build.sh` | Conan、CMake 和编译入口 |
| `pack.sh` | 构建三个机器人数据插件并生成分发包 |

仓库还保留了 PlotJuggler 官方插件工程中的其他插件目录；根目录全量构建会配置这些插件，机器人数据分发包则由 `pack.sh` 只构建上述三个插件。

## 环境要求

基础构建工具：

- CMake 3.22 或更高版本；
- Ninja；
- 支持 C++20 的 GCC 或 Clang；
- Conan；
- GTest；
- PlotJuggler SDK 对应的 Conan 包，版本由 `SDK_VERSION` 指定。

插件额外依赖由各自的 `CMakeLists.txt` 和 Conan 配方声明：

- LCM 插件需要 LCM、GLib、Orocos KDL、nlohmann-json、LZ4、Zstandard，以及其机器人状态处理所需的 ROS 头文件和库；
- DDS 插件需要 CycloneDDS、`idlc`、TinyXML2、nlohmann-json 和线程库；
- DDS 的 MCAP 写入器使用 `data_stream_dds/contrib/mcap/` 中的代码，不需要额外安装 MCAP 库。

如果 CycloneDDS 不在系统默认路径，请设置 `CMAKE_PREFIX_PATH` 或 `CycloneDDS_DIR`。LCM 和 DDS 的 CMake 配置会直接查找本机提供的通信库及相关工具。

## 构建

在仓库根目录执行：

```bash
# 构建全部插件
./build.sh

# 只构建一个插件
./build.sh data_load_lcm
./build.sh data_stream_lcm
./build.sh data_stream_dds

# 使用本地 PlotJuggler SDK 工作树进行开发
./build.sh --sdk-local=/path/to/plotjuggler_sdk data_stream_dds
```

默认构建类型是 `Release`，输出位于对应插件的 `build/<plugin>/Release/`。单插件构建会通过 `PJ_BUILD_PLUGIN` 限制 CMake 配置范围，适合日常开发和 CI。

### Sanitizer 构建

```bash
./build.sh --asan data_stream_lcm
./build.sh --tsan data_stream_dds
```

AddressSanitizer 和 ThreadSanitizer 构建分别写入 `build/asan/` 和 `build/tsan/`。TSan 构建会在编译后运行测试；也可以通过 `PJ_SANITIZE=asan` 或 `PJ_SANITIZE=tsan` 设置同样的模式。

`--sdk-local` 面向本地开发，不保证可复现，不应作为发布构建的 SDK 来源。

## 打包

```bash
./pack.sh
```

`pack.sh` 会依次构建：

1. `data_load_lcm`；
2. `data_stream_lcm`；
3. `data_stream_dds`。

生成的插件会复制到：

```text
pack/lcm_plugin/lcm-loader/
pack/lcm_plugin/lcm-streamer/
pack/lcm_plugin/dds-streamer/
```

最终分发文件为根目录下的 `pack.tar.gz`。每个插件目录包含共享库和对应的 `.pjmanifest.json`。`pack.sh` 使用 Git 生成构建历史并嵌入插件，因此应在 Git 工作树中执行。

## 插件清单

| 插件 | ID | 类型 | 版本 |
| --- | --- | --- | --- |
| LCM Loader | `lcm-loader` | `data_loader` | `1.3.0` |
| LCM Server | `lcm-server` | `data_stream` | `0.9.1` |
| DDS Server | `dds-server` | `data_stream` | `0.9.1` |

清单源文件分别位于各插件目录下的 `manifest.json`，构建时会生成 PlotJuggler 使用的 sidecar manifest。

## 运行与调试注意事项

- LCM Loader 只处理 `.lcmlog`；在线 LCM 数据使用 LCM Server。
- DDS 订阅需要先选择至少一个话题，再点击连接；不会默认订阅所有话题。
- DDS 在线动态解析优先使用发布端提供的 Complete TypeObjects；类型发现不完整时可导入匹配的本地 IDL。
- DDS 的 3D 机器人功能需要准备匹配的 URDF、MuJoCo XML、关节话题和必要的消息类型解析能力。完整配置流程见 [`data_stream_dds/README.md`](data_stream_dds/README.md)。
- DDS MCAP 录制保存的是 CDR 数据和类型描述；读取录制文件时，消费端需要支持相应的 `cdr` / `omgidl` 编码及消息类型。
- `build/`、`pack/` 以及日志、MCAP、压缩包等运行时产物不应混入源码提交。

## 上游关系与许可证

项目沿用 PlotJuggler 官方插件的组织方式和 SDK 接口，通用插件代码与发布脚本可继续从上游同步。仓库内各插件 manifest 当前沿用 PlotJuggler 的项目信息，具体第三方组件的许可证以对应目录中的许可证文件和源码声明为准。
