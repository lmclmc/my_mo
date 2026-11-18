# Changelog — data_stream_dds

## [Unreleased]

- DDS、LCM 实时订阅与 LCM 日志加载界面在顶部 GitLab 仓库栏右侧增加“更新记录”按钮；弹窗展示构建版本的提交日期、作者、说明及提交链接，并提供 GitLab 在线记录入口。
- 连接窗口增加“保存配置…”与“加载配置…”按钮，支持独立 JSON 文件保存和恢复 DDS、话题、QoS、IDL、3D 模型与映射、MCAP 录制设置；保存采用原子替换，加载失败保留当前设置，相对资源路径按配置文件目录解析。
- 修复容器内已固定旧 ROS 解析器时仍报 `no parse_scalars`：启动脚本安装包内解析器后，仅解除当前用户的 `ros-parser` 缓存版本绑定；从任意工作目录启动均使用脚本所在的包。
- 更新打包的 ROS 解析插件，修复启用 3D 机器人时 `/robot_description` 的 `std_msgs/msg/String` 缺少 `parse_scalars` 导致 DDS 启动失败；打包时同步构建 ROS 解析器及其 manifest，避免再次使用旧插件。
- DDS 话题与 QoS、3D 机器人和 MCAP 录制使用三个固定尺寸的紧凑页签，靠左排列并减小内边距与高度；切换选中、悬停及调整窗口时页签不伸缩。选中项使用低饱和青绿色、细描边和轻微投影，未选中项保持淡底，文字跟随亮暗主题；保持当前选中页，并调整最小窗口高度以避免内容重叠。
- 自动订阅 state_estimator，将 quat 按 [w, x, y, z] 归一化后用于世界坐标系到机器人根节点的旋转，支持整体姿态与关节位置同时展示；无效四元数保留上次姿态，界面提示 Fixed frame 选择 world。
- 首次启动传入空配置时使用默认设置，修复无已保存配置时出现 Invalid config JSON 的提示。
- Conan 配方补齐 TinyXML2 10.0.0 静态依赖，修复未安装系统 TinyXML2 的容器无法配置 DDS 插件的问题；打包脚本在构建失败时立即退出。
- 新增 URDF / MuJoCo 文件导入和关节映射预览，按 MuJoCo actuator 顺序匹配 URDF 关节名；配置可保存恢复，导入及校验在后台执行。
- 支持 leg、waist、arm、head 的 data 话题位置数组驱动实时 3D 机器人，arm 兼容 joint_curPos；生成 /robot_description 和 /tf，支持固定、旋转、移动和 mimic 关节，并保留原采样率曲线。
- 数组和 sequence 按“话题 / 数组变量 / [索引]”分组；结构体数组继续展开成员，多维数组逐维展开索引。
- 增加 MCAP 后台滚动录制，可设置目录、单文件最大 MB 数和最多文件数；达到上限后覆盖最早文件，重新连接时已有文件计入限制，停止时写完缓存、统计及文件尾。
- MCAP 改为保存 Reader 提供的 CDR 数据（含封装头），按订阅类型携带从 Complete TypeObjects 重建的 OMG IDL Schema；支持在线 XTypes 和本地 IDL 两种来源。
- 接收时间写入 logTime/publishTime；文件大小上限包含所有类型 Schema、Channel 和完整 MCAP 文件结构；录制在字段展开之前进行。
- 加宽右侧话题列表，XTypes 列以颜色标记展示 IDL 匹配状态，并增加对应 IDL 文件路径列。
- 曲线横轴统一使用上位机订阅读取样本时的本机时间，在消息解析及进入绘图队列前记录。
- 保留 CycloneDDS 分区与话题发现、所选话题订阅、动态字段解析和 IDL 导入。
- 删除旧通信实现、日志与机器人可视化代码，移除对应构建依赖。
- 将插件入口和界面文件统一为 DDS 命名，配置仅保存 DDS 设置。

## [0.9.1] - 2026-08-04
