# CycloneDDS 发现与动态解析

连接页面顶部的“更新记录”按钮可查看当前构建版本的最近提交，包括日期、作者、提交说明及 GitLab 提交链接。记录在构建时生成，离线也可查看；弹窗内的 GitLab 入口用于浏览仓库在线最新记录。

保留界面首行的作者信息，其他原有配置控件已替换为：

- **Domain ID**：范围 0–232，默认沿用 200；修改后自动重新发现。
- **订阅 QoS**：Reliability、Durability、History、History depth 和最大阻塞时间。
- **Partition**：输入订阅分区，或点击发现列表中的分区。留空使用默认分区。
- **Topic**：展示当前订阅分区中的话题名称、原生类型、XTypes / IDL 匹配状态、IDL 文件路径、分区、发布/订阅端数量，以及端点声明的 Reliability / Durability。每个话题名称一行，同名端点的类型和分区合并展示，端点数量按端点统计。连接窗口默认宽度为 1540，右侧话题区最小宽度为 880；左侧分区区宽度为 240–320，扩大窗口时优先增加话题区宽度。
- **MCAP 数据录制**：可启用连接后录制，选择或输入录制目录，设置单文件最大 MB 数和最多保留文件数，达到上限后滚动覆盖最早的文件。
- **3D 机器人**：在独立页签导入 URDF 和 MuJoCo XML，预览关节映射，使用四组 DDS 位置数组和 `state_estimator.quat` 驱动关节及整体姿态。

发现使用 `DCPSPublication` 和 `DCPSSubscription` 内置 Reader，持续收集当前域的端点信息；默认分区显示为“(默认分区)”。分区来自端点的 Partition QoS，不会扫描没有端点声明的分区。端点的分区表达式按声明原样展示。

对话框每 250 ms 非阻塞更新发现结果，“重新发现”清空旧结果并重建发现 Participant。切换域、确认、取消和销毁对话框时会释放旧发现实体。端点下线后，列表会在 DDS 收到离线通知或租约超时后更新。

在话题表中单击一行即可选择或取消，可直接点击多行进行多选，高亮行表示选中；也可使用“全选”和“清空”。至少选中一个话题后才能点击“连接”。刷新发现、端点更新和表格排序会保留选择；切换 Domain ID 或订阅 Partition 会清空选择，需要在新域或分区重新选择。分区表达式的匹配方式与接收端一致。

连接后，插件仅为选中的精确话题名称创建动态 Reader，Domain ID、Partition 和 Reader QoS 使用界面配置。启用 3D 机器人时会自动选中配置的关节 data 话题和姿态话题 `state_estimator`；这些必需话题在启用期间保持选中。其他未选话题不会触发类型解析或业务数据订阅；新上线的其他话题不会自动加入订阅。“全选”只选中点击时已发现的话题。最后一个发布端下线后释放相应订阅，已选话题重新上线后自动重新解析和订阅。已选的离线话题保留在表中并显示为等待发布端，仅由用户选择的话题可以取消。仅发现订阅端的话题也可以先选择，连接后等待发布端出现。

收到有效数据后，所选话题的实际字段出现在 PlotJuggler 左侧变量列表；将需要的数值字段拖入绘图区即可绘制曲线。如需更换订阅话题，停止数据流后重新打开连接窗口选择，再次连接。

XTypes 列根据**发布端**发现样本中的 TypeInformation 显示状态：

- **支持（Complete）**：声明了完整类型标识，可在线发现实际字段名；是否能取得全部 TypeObject 仍取决于发布端响应。
- **仅 Minimal（需 IDL）**：只有 Minimal 信息，无法恢复真实字段名。
- **未提供（需 IDL）**：发布端没有声明可用的类型信息。
- **等待发布端**：尚未发现该话题的发布端，不能从订阅端信息推断发布能力。

同名话题有多个发布端时合并展示各发布端的状态。导入 IDL 不会改变发布端的 XTypes 能力；全部类型匹配成功时，XTypes 列以绿色状态标记显示“IDL 已匹配”，仅部分类型匹配时以黄色标记显示“部分类型已匹配 IDL”，后方保留原有的发布端 XTypes 状态。紧邻的“IDL 文件路径”列直接显示所匹配 IDL 的完整路径，多个文件以 ` | ` 分隔；悬停任一单元格可查看各类型与文件的对应关系。未匹配时路径列显示“—”。匹配来源按最新导入的定义确定，重新导入及清除 IDL 后标记和路径同步更新。颜色标记与话题的选择高亮独立，匹配 IDL 不会自动选中话题。

对于仅 Minimal、未提供完整 XTypes 或在线类型解析失败的话题，点击“导入 IDL…”选择与发布端消息定义一致的 `.idl` 文件。导入会在后台处理，按 **完整 DDS 类型名**（含 module，例如 `hal::canfd::canfd_state_t`）自动匹配话题；一个 IDL 可覆盖多个话题，也可继续导入其他文件。同名类型使用最后导入的定义，再次导入同一个文件会替换其旧定义。`#include` 依赖按原文件的目录及相对路径展开，无需用户预生成 C/H 文件。缺少依赖、IDL 语法错误、工具缺失等错误会显示在导入状态中；导入失败保留此前成功导入的定义。“清除 IDL”移除全部本地定义。

本地 IDL 的 module、字段顺序、字段类型、key 以及 extensibility 注解必须与发布端一致。没有远端类型信息时，插件无法证明用户提供的 IDL 与发布端二进制完全一致。导入只补充本地消息定义，不会启用发布端的类型发现，也不会绕过 Partition 或 QoS 匹配。

在线解析通过发布端的 Complete TypeInformation / TypeObject 获取字段名，通过 `dds_create_topic_descriptor` 创建动态 Topic 和 Reader。本地 IDL 使用与插件匹配的 CycloneDDS `idlc` 和本机 C 编译器在临时目录生成类型描述及 Complete TypeMapping，创建本地 Topic 后复用同一套字段解析器，停止及释放最后一个使用者后清理临时产物。运行机器需安装匹配版本的 `idlc`、CycloneDDS 开发头文件/库及本机 C 编译器；构建时记录工具和库路径，工具路径失效时尝试 PATH 中的 `idlc` / `cc`。插件通过参数数组调用工具，文件路径不经过 shell。IDL 文件仅用于生成消息类型，用户无需修改或重新编译插件代码。CANFD 不使用固定类型映射。

PlotJuggler 左侧变量采用“原生 DDS 话题名 / 实际字段路径”，例如：

```text
/hal/canfd/command/header/time_pub
/hal/canfd/command/joints/[0]/pos
/hal/canfd/command/joints/[0]/vel
/hal/canfd/command/joints/[1]/pos
```

字段名称来自类型发现结果，上述名称仅为示例。嵌套结构用 `/` 分层，sequence 和数组的每个 `[索引]` 也独立占一层。因此话题下先显示数组变量分组，展开后再列出 `[0]`、`[1]` 等索引；结构体数组在索引下继续显示成员。多维数组逐维展开，如 `matrix/[1]/[2]`。这些分组是 PlotJuggler 中的字段层级，DDS 仍订阅原始话题。收到第一个有效样本后生成相应变量；空 sequence、缺省 optional 字段不会产生伪造值。枚举和 bitmask 保存数值，字符串保存文本。

数组曲线路径相较旧版增加了索引前的 `/`，已有布局若使用 `joints[0]/pos` 等旧路径，需要重新选择相应曲线。

当前支持 struct、基类成员、alias、布尔/整数/浮点数、枚举、bitmask、字符串、有界/无界 sequence、数组及其嵌套组合，并读取已存在的 optional / external 成员。Final、Appendable、Mutable 的内存布局和 XCDR1/XCDR2 反序列化由 CycloneDDS 处理；插件根据运行时描述中的偏移和 stride 读取数据。

在线解析要求发布端启用 XTypes 类型发现并提供 **Complete** 类型信息；Minimal 类型信息只有成员名称的哈希，无法恢复真实字段名。无法在线解析时会尝试匹配的本地 IDL。类型信息缺失且没有匹配 IDL、类型依赖解析超时、尚不支持的类型或 QoS 不兼容会在 PlotJuggler 消息栏提示。获取类型失败会自动重试，其他话题继续接收。当前未支持 union、map、bitset、宽字符/宽字符串、float128 和 SCC 递归类型；本地 IDL 也沿用这些解析限制。

接收线程在归还 DDS loan 前展开并复制数值及字符串，PlotJuggler 的写入只在 `onPoll()` 中执行。曲线横轴统一使用上位机订阅读取样本时的本机系统时间，单位为 Unix epoch 纳秒；每条样本在动态解析和进入待绘图队列前记录时间，随记录传递到 `onPoll()`，绘图时沿用该时间。消息中的 `timestamp`、`header/time_pub`、`header/time_meas` 等字段仍按原值展示。这里的接收时间是应用订阅线程读取到样本的时刻；线程调度及 DDS Reader 内的缓存等待会影响该时刻，因此不同话题的到达时间仍可能不同。

停止时唤醒并退出线程，先释放 Reader，再释放动态类型描述。队列最多缓存 2048 条记录、合计 100000 个字段；超出时丢弃最旧记录并提示，单个样本展开也限制为 100000 个字段。

## URDF / MuJoCo 与实时 3D 机器人

1. 打开“3D 机器人 · URDF / MuJoCo”页签，导入 URDF 和配套的 MuJoCo XML。文件选择后自动启用机器人；也可手动勾选或取消启用。
2. 查看映射表中的“位置索引 → XML 索引 → 关节名”。MuJoCo `<actuator>` 下各执行器的 `joint` 引用顺序作为索引来源，再按名字匹配 URDF，URDF 的声明顺序不影响结果。支持 `motor`、`position`、`velocity`、`general` 等引用关节的执行器及相对路径 `<include>`。
3. 使用当前 X1 的默认映射，或调整各组的精确 DDS 话题名、XML 起始索引和关节数。索引从 0 开始，每组位置数组也从 0 开始；关节数为 0 表示不驱动该组。
4. 在 DDS 页签为缺少 Complete XTypes 的话题导入匹配的消息 IDL（包括 `state_estimator`），然后连接。将生成的 `/robot_description` 拖入 3D 窗口，Fixed frame 选择界面提示的世界坐标系，通常为 `world`。已有视图若固定为 `pelvis`，需切换到 `world` 才能看到整体姿态。

默认映射如下，保留 MuJoCo XML 中“腿、腰、头、臂”的执行器顺序：

| 部位 | DDS 话题 | 位置字段 | XML 起始索引 | 数量 |
| --- | --- | --- | --- | --- |
| 腿 | `leg_control_data` | `q` | 0 | 12 |
| 腰 | `waist_control_data` | `q` | 12 | 3 |
| 臂 | `arm_control_data` | `q`，没有 `q` 时兼容 `joint_curPos` | 17 | 14 |
| 头 | `head_control_data` | `q` | 15 | 2 |

例如腿话题的 `q/[0]` 驱动 XML 第 0 个执行器引用的 URDF 关节，手臂话题的 `q/[0]` 驱动 XML 第 17 个执行器引用的关节。旋转关节使用弧度，移动关节使用米。初始关节位置为 0；各组独立保留最近一次有效位置，无需等待四个话题同时到达。固定关节、URDF origin/axis 和 mimic 关系参与 TF 计算，根节点姿态由 `state_estimator.quat` 驱动。模型预览最多约 60 Hz，数值曲线保留原采样率。

模型和映射在后台校验；范围重叠、文件无效、XML 关节未出现在 URDF、fixed/mimic 关节被执行器直接驱动等情况会显示原因并禁止连接。编辑映射后自动重新校验，修改外部文件内容后点击“重新读取模型并校验映射”。运行中位置数组长度不符、索引缺失或包含 NaN/Inf 时，保留该组上次有效姿态并报告原因，其他话题和曲线继续处理。URDF 支持 fixed、revolute、continuous、prismatic 及 mimic；xacro 需先展开。相对 mesh 路径转换为相对于 URDF 文件的绝对路径，邻近目录可找到的 `package://` 网格也会转换；其余 package URI 可使用 3D 窗口的包搜索路径解析。

`state_estimator.quat[0..3]` 按 X1 的 `[w, x, y, z]` 顺序读取，归一化后驱动 `world → URDF 根 link`（X1 为 `pelvis`）的旋转，根节点平移保持为零。若 URDF 已含 `world` link，将使用不冲突的 `dds_world` 等名称，具体以界面提示为准。收到首个有效四元数前使用单位旋转；零四元数、缺失/重复/越界索引、非数值或 NaN/Inf 样本会被跳过并保留上次有效姿态。整体姿态与各组关节位置独立更新，`state_estimator` 暂未上线也可继续查看关节动作。

沿用 LCM 插件的显示接口，在 PlotJuggler 数据集中生成 `/robot_description`（ROS 2 String）和 `/tf`（TFMessage），供现有 3D 场景读取。启用此功能需要安装支持 `ros2msg` 的 ROS 消息解析插件。插件自身不需要 ROS 节点、ROS 运行库或 MuJoCo 运行库。

若连接时报 `registered handler has no parse_scalars: std_msgs/msg/String`，需同时更新 ROS 解析插件：旧版 `robot_description` 只有对象解码，没有主程序实时接入所需的空标量入口。修复位于 `../pj-official-plugins/parser_ros/ros_parser.cpp`（相对于本仓库根目录）；仅重新编译 DDS 插件不能修复旧 ROS 插件。首次连接没有保存配置时使用默认设置，实际损坏的 JSON 仍会报告错误。

模型配置保存在 `dds.robot` 中：`enabled`、`urdf_path`、`mujoco_path`，以及 `groups.leg/waist/arm/head` 下的 `topic`、`start`、`count`。恢复配置后重新读取模型文件；缺少 `robot` 配置的旧布局默认关闭 3D 功能。

## MCAP 数据录制

在连接窗口勾选“连接后录制”，设置目录、单文件最大大小和最多文件数，再选择话题并连接。目录可手动输入，或点击“选择目录…”浏览；不存在的目录会自动创建。录制默认关闭，默认大小为每文件 128 MB、最多 10 个文件。1 MB 为 1024 × 1024 字节；允许设置 1–1048576 MB 和 1–10000 个文件。

录制生成标准 `.mcap` 文件。所选话题的 Channel 使用 `messageEncoding = "cdr"`，Schema 使用 `encoding = "omgidl"`，Schema 名称是原生 DDS 完整类型名。不同消息类型分别保存类型描述；相同订阅类型被多个话题使用时可共享 Schema。每个滚动文件独立携带其实际使用的 Schema、Channel 和统计摘要。MCAP Message 的 `logTime` 和 `publishTime` 均使用上位机接收时间（Unix epoch 纳秒），与实时曲线保持一致。

同时启用 3D 机器人时，MCAP 还会保存生成的 `/tf`，使用 `cdr`、`ros2msg` 和 `tf2_msgs/msg/TFMessage`，采样间隔和时间戳与实时 3D 显示一致。若也订阅了名为 `/tf` 的原生 DDS 话题，其样本在 MCAP 中保存为 `dds#/tf`。回放时可在 3D 窗口加载对应的 URDF。

录制通过 CycloneDDS `dds_takecdr` 获取序列化样本，保存包含四字节 DDS encapsulation header 的 CDR 数据，保留 CycloneDDS 提供的字节序与 XCDR1/XCDR2 表示。数值、字符串、数组、sequence 及其嵌套结构直接保留在 CDR 中；NaN/Infinity、64 位整数、空 sequence 不经过 JSON 转换，也不受绘图字段数量上限影响。这里保存的是 Reader 提供的 CDR 表示；CycloneDDS 可能进行字节序规范化或类型转换，文件不包含 RTPS 网络包。

IDL 从当前订阅使用的 Complete TypeObjects 重建，包括依赖类型、module、结构成员顺序、成员 ID、key/optional/external/must_understand、extensibility、数组维度、sequence/string 的界限、枚举值和 bitmask 位位置。在线 XTypes 及本地 IDL 两条订阅路径均使用同一套导出逻辑；本地 IDL 已展开的依赖也会进入类型图，读取录制文件不依赖原 IDL 文件路径。类型描述针对插件当前支持的类型生成，不保留源文件注释、常量及所有用户自定义注解。无法导出类型描述的话题会明确提示并继续订阅绘图，该话题暂不录制。

原生 DDS 话题只保存选中话题的有效数据样本，DDS dispose/unregister 等生命周期通知不写入 MCAP。原生样本在反序列化和字段展开之前入队，空字段样本、绘图队列淘汰及字段展开失败不会丢弃已经进入录制队列的 CDR 样本。其 Channel 元数据标明来源为 CycloneDDS、序列化为 DDS CDR、时间来自本机接收、Schema 来自 Complete TypeObjects。

加载文件需要消费端提供支持该类型及 XCDR 表示的 `cdr` / `omgidl` 解析能力。PlotJuggler 的 MCAP 加载器会根据 `omgidl` 选择相应消息解析插件（当前官方插件为 ROS Parser）；DDS 订阅插件本身不依赖 ROS。MCAP 容器格式可读取不代表任意第三方解析器都支持全部 DDS 类型、注解或 XCDR2。

编码变更只影响新录制文件，已有 JSON 编码的 `.mcap` 文件不自动转换；若继续使用原录制目录，这些旧文件仍按相同命名规则参与滚动覆盖。

文件名为 `cyclonedds_record_000001.mcap`、`cyclonedds_record_000002.mcap` 等。每次写入前检查消息、首次出现的话题/Schema，以及关闭文件时需要的 Summary、Statistics、SummaryOffset、DataEnd、Footer 和 magic 字节，确保完成后的文件也不超过设置的大小。空间不足时先完成当前文件，再切换到下一个文件，不拆开一条样本。每个滚动文件均独立携带该文件所用的 Schema 和 Channel，可单独打开。

当前使用无压缩、无 Chunk 的标准 MCAP 记录，并保存完整统计和摘要，便于精确限制文件大小。没有 Chunk/Message Index，支持该标准布局的消费端需按数据段扫描消息。文件数量达到上限后删除最早的录制文件并复用空闲编号，因此**文件名编号不代表先后时间**，应使用文件修改时间或消息的 `logTime` 判断顺序。文件数设为 1 时持续替换这一个文件。

同一目录中符合上述精确命名规则的 `.mcap` 文件属于同一个滚动录制集合。重新连接或重启时，已有文件按修改时间排序，并计入数量上限；第一条新样本写入时按当前限制清理旧文件。调小大小上限时，超过新大小上限的旧录制文件也会被清理；调小数量上限时优先清理最早的文件。其他名称的文件保留。之前生成的 `.jsonl` 文件保留，不自动转换，也不计入 MCAP 文件数量限制。建议每个独立录制任务使用独立目录。同一目录同时仅允许一个录制实例，目录中持久保留一个零字节锁文件，不计入录制文件数。

接收线程在归还序列化样本前将 CDR 复制到独立的录制队列，再对同一样本反序列化并生成实时曲线。未启用录制时保留原有的 `dds_take` 接收路径。暂停绘图或绘图队列淘汰样本不影响已交给录制线程的数据；停止订阅时先结束接收，再写完录制缓存、MCAP 统计与文件尾，刷新并同步到磁盘后关闭文件。录制启用、结束和错误会出现在 PlotJuggler 消息栏。

录制队列限制为 4096 条样本、估算内存 64 MB（类型描述按每条引用保守计入），在分配 CDR 副本前检查容量。磁盘持续过慢导致队列耗尽，或单条样本超过该预算时，停止接受新的录制数据，写完已排队的数据并提示；磁盘写入失败、空间不足、单条样本及必要 MCAP 结构大于文件大小上限等错误会停止录制并提示，DDS 订阅和绘图继续运行。发生写盘错误的当前文件可能缺少完整文件尾，可使用 MCAP 恢复工具处理。排除原因后重新连接可再次录制。录制目录无法创建、目录被占用等启动错误会让本次连接失败，避免用户误以为已开始录制。

录制设置保存在 `dds.recording` 中，字段为 `enabled`、`directory`、`max_file_size_mb` 和 `max_files`；没有录制设置的旧配置仍可正常加载，并默认关闭录制。

## 配置与构建

连接窗口底部提供“保存配置…”和“加载配置…”按钮，可独立保存或恢复 JSON 配置，包括 Domain、Partition、所选话题、QoS、IDL 路径、3D 模型与关节映射，以及 MCAP 录制设置。保存时使用临时文件写完后原子替换目标文件；加载失败保留当前设置并显示原因。导入 IDL 期间需等待完成后再保存。加载成功后会重新发现话题、导入 IDL、校验模型，检查状态后点击“连接”即可订阅，不会自动启动连接或录制。

独立配置文件使用与布局配置相同的 `dds` JSON 对象。保存时资源路径转为绝对路径；加载文件中的相对 IDL、URDF、XML 和录制目录路径时，以该 JSON 所在目录为基准。配置只保存设置和文件路径，不打包模型、网格或 IDL 内容，移动到其他机器后需同时准备资源并调整路径。空文件、无 DDS 设置的 JSON、字段类型或范围错误会被拒绝。

配置通过 PlotJuggler 原有的配置保存机制持久化，字段位于 `dds` 对象中，`selected_topics` 为所选话题名称数组，`idl_files` 为导入的 IDL 源文件路径数组；配置恢复时重新加载这些文件，因此需保留 IDL 及其包含的依赖。恢复配置后可等待所选话题上线。旧配置仍可读取，其中与 DDS 无关的字段会被忽略，再次保存时只写入 DDS 设置。缺失 DDS 配置时使用默认值：Domain 200、默认分区、Best effort、Volatile、Keep last、深度 1；缺失 `selected_topics` 时没有默认选中的话题，需要在连接窗口选择后才能启动。空选择不会回退为订阅全部话题。

插件依赖 PlotJuggler SDK、nlohmann_json、CycloneDDS、TinyXML2、系统线程库及动态加载库。MCAP 使用目录内 `contrib/mcap` 中的上游 libmcap 2.1.1 writer 头文件及 MIT 许可证，无需额外下载 MCAP 或依赖 ROS；无压缩录制无需 LZ4/Zstd。通过上级工程构建此插件，CMake 使用 `find_package(CycloneDDS REQUIRED)` 查找原生 CycloneDDS 安装，`find_package(tinyxml2 CONFIG REQUIRED)` 查找 XML 解析库。DDS 插件的 Conan 配方提供 SDK、JSON 和 TinyXML2 10.0.0；TinyXML2 默认静态链接，无需在构建容器或运行机器额外安装系统 TinyXML2。CycloneDDS 和 `idlc` 仍由本机安装提供。

更新依赖后，在仓库根目录运行 `./build.sh data_stream_dds`（或 `./pack.sh`），让 Conan 重新生成依赖和工具链，再执行 CMake。不要仅重跑旧构建目录中的 `cmake --build`，旧工具链尚未包含新依赖。第一次构建需要从配置的 Conan 源下载 TinyXML2；后续可复用缓存。`pack.sh` 会在构建失败时立即退出，避免继续复制上次编译的插件。

3D 桥接生成的 `/robot_description` 和 `/tf` 需要 ROS 解析插件。`pack.sh` 会同时构建并复制 ROS 解析器的共享库和 manifest，默认源码目录为相邻的 `../pj-official-plugins`；可通过 `PJ_ROS_PLUGIN_SOURCE_DIR=/path/to/pj-official-plugins ./pack.sh` 指定其他位置。若出现 `registered handler has no parse_scalars: std_msgs/msg/String`，表示正在使用的 ROS 解析器未包含机器人描述的实时解析修复，应同步更新包内及已安装的 `ros-parser` 插件并重启 PlotJuggler。

插件安装目录以运行用户为准：容器内以 root 启动时使用 `/root/.local/share/PlotJuggler/PlotJuggler4`。`extensions.profiles/loaded.json` 中的 `ros-parser` 绑定可让应用优先加载 `extensions.artifacts` 中的旧库，覆盖 `extensions/ros-parser` 后仍需解除该绑定。`pack/start.sh` 安装包内解析器后会自动处理此项，保留其他插件的版本绑定；必须完全退出旧进程再启动，使新库生效。

若 CycloneDDS 安装在非系统目录，将其安装前缀加入 `CMAKE_PREFIX_PATH`，或设置 `CycloneDDS_DIR` 指向其 CMake 配置目录。本机所用 CycloneDDS 位于 `/opt/ros/jazzy`，使用该路径只为查找 CycloneDDS，无需初始化 ROS 环境。

目录中的入口为 `dds_stream_source.cpp/.hpp`，界面逻辑和布局为 `dds_dialog.hpp` 与 `datastream_dds.ui`；`dds_discovery.hpp`、`dds_dynamic.cpp/.hpp`、`dds_idl.cpp/.hpp` 和 `dds_settings.hpp` 分别负责端点发现、消息动态解析、本地 IDL 加载及 DDS 配置。`dds_type_schema.cpp/.hpp` 从 Complete TypeObjects 导出录制用 IDL，`dds_cdr_record.hpp` 定义跨线程持有的 CDR 数据及共享类型描述；`dds_recorder.cpp/.hpp` 与 `dds_recording_settings.hpp` 负责后台滚动录制及其配置，`dds_mcap_writer.cpp/.hpp` 负责 MCAP 通道、类型 Schema、大小预估和文件完成。实时订阅的数据解析在插件内部完成，直接写入 PlotJuggler，无需额外消息解析插件；从 MCAP 文件加载 CDR 数据时需要对应解析插件。

本次在实体机 GCC 13.3、ROS Jazzy / CycloneDDS 0.10.5 环境编译通过，构建输出：

```text
/home/lmc/.cache/plotjuggler-dds-ui/native/build/data_stream_dds/libdds_stream_source_plugin.so
```

`dds_robot_settings.hpp` 定义模型配置，`dds_robot_model.cpp/.hpp` 负责 XML 顺序、URDF 关节运动学和显示消息生成。可用 `-DPJ_DDS_BUILD_TESTS=ON` 构建 `dds_robot_model_test`，然后在构建目录的 `data_stream_dds` 下运行 CTest；测试覆盖乱序 URDF、MJCF include、四组驱动、固定/mimic/移动关节及错误样本。该程序还可接收三个参数：真实 X1 URDF、MuJoCo XML、ROS 解析插件路径，额外验证 X1 的 31 个关节以及 CDR 与现有解析插件的兼容性。

已通过原生编译、插件导出检查、真实 X1 模型的 31 关节映射测试和现有 ROS 解析插件的 CDR 解码测试。隔离 DDS 域中的实际发布/订阅测试确认四个 data 话题可经过新插件更新模型描述及 TF；另已验证界面导入回调、映射校验和配置保存恢复。
