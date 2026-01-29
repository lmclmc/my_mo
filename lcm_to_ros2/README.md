# LCM to ROS2

这个包用于订阅 [Lightweight Communications and Marshalling (LCM)](https://lcm-proj.github.io/) 消息并重新发布到 [ROS2](https://docs.ros.org/) 消息（反之亦然）。LCM 是一个基于 UDP 多播的高带宽、低延迟通信的开源消息工具。

目标是创建一个相对通用的系统，只需要 LCM 消息文件，并自动生成重新发布器，可以：

1. 订阅 LCM 主题并将等效（相同数据）消息重新发布到 ROS2 主题，或
2. 订阅 ROS2 主题并将等效（相同数据）消息重新发布到 LCM 主题。

此包自动生成 ROS2 消息文件、CPP 文件、launch 文件和相应的 CMakeLists，用于 colcon build 创建一组重新发布节点。该系统已在 Ubuntu 22.04 和 ROS2 Humble 上测试。

用户只需要创建 \*.lcm 消息文件。

注意，此代码仅限于从 LCM 文件生成 ROS2 消息，而不是相反（部分原因是 LCM 的数据类型数量更有限，但主要是因为这是我所需要的）。LCM 原始类型列表可在[这里](https://lcm-proj.github.io/type_specification.html#type_specification_primitives)找到。我们已测试：

| LCM 原始类型 | 标量 | 固定长度数组 | 可变长度数组 |
| --- | --- | --- | --- |
| int8_t | ✅ | ✅ | ✅ |
| int16_t | ✅ | ✅ | ✅ |
| int32_t | ✅ | ✅ | ✅ |
| int64_t | ✅ | ✅ | ✅ |
| float | ✅ | ✅ | ✅ |
| double | ✅ | ✅ | ✅ |
| string | n/a | n/a | ✅ |
| boolean | ✅ | ✅ | ✅ |
| byte | ✅ | ✅ | ✅ |

LCM 原始类型可以正常工作，因为它们可以直接转换为 ROS2 类型（实际上这些类型是标准数据类型）。重要的是要注意，代码使用 reinterpret_cast 直接将传入的 LCM 消息引用为 ROS2 消息。这意味着您**不应该**直接编辑自动生成的 msg 文件，因为更改 msg 中项目的顺序（相对于 LCM）会搞乱事情。我对内存管理也不是 100% 有信心，但它似乎可以工作。

**注意：** LCM 中使用的多维数组（如 `double image[3][128][128]`）将**无法**工作。这是因为 ros 不以相同的方式处理多维数组。选项是使用多个消息类嵌套数组，或者如果有兴趣，我可以尝试使用 [multiarrays](https://docs.ros2.org/humble/api/std_msgs/msg/Float64MultiArray.html) 来修复这个问题。

## 安装：

首先，您需要安装 LCM。我不会在这里复制说明，而是链接到 LCM 构建说明，其中包含各种平台的说明。

[LCM 构建说明](https://github.com/lcm-proj/lcm/blob/master/docs/content/build-instructions.md)

然后，克隆此存储库（假设默认的 ROS2 工作空间在 `~/ros2_ws`）：

```bash
cd ~/ros2_ws/src
git clone <repository_url> lcm_to_ros2
cd lcm_to_ros2
```

## 一般流程

代码通过两个主要的 bash 脚本工作：

1. `ros2msg-gen.sh` - 此脚本将 LCM 消息文件列表作为输入，对于每个文件：
    - 使用 LCM 工具 `lcm-gen [-x]` 生成消息的 CPP 头定义，
    - 生成相应的 ROS2 消息类型，以及
    - 如果在 lcm 消息文件中提供了 lcm 指纹值作为注释行（请参阅 `lcm/example_type.lcm` 文件），程序还会生成一个派生类定义，该类将使用该消息的覆盖哈希值（通常不推荐，如果您不知道是否需要，请忽略它）。

2. `ros2repub-gen.sh` - 此脚本将配置文件作为输入，并生成 CPP 代码和 ROS2 launch 文件，用于 ROS2 节点，这些节点：
    - 监听 LCM 主题并将接收到的消息重新发布到 ROS2 主题，或
    - 监听 ROS2 主题并将接收到的消息重新发布到 LCM 主题。

## 测试示例

存储库包含一个测试示例。如果您还没有添加任何其他 LCM 消息（或者即使您有），您应该能够通过以下方式确认代码是否正常工作：

1. 通过运行（在 `lcm_to_ros2` 项目根目录）为 lcm 文件夹中的所有 lcm 文件生成 ROS2 消息：
    ```bash
    ./ros2msg-gen.sh lcm/*.lcm
    ```
    这将生成标准 LCM hpp 消息定义（在 `exlcm` 目录中）、ROS2 消息（在 `msg` 目录中）和带有覆盖哈希值的 `example_type` 消息定义（在 `exlcm_rehash` 目录中）。请随意检查（但请不要修改）ROS2 消息定义。

2. 检查配置文件 `repub_configs/example_republishers.cfg`。它包含用于生成两个重新发布器的规范（每个非 `#` 注释行一个）。第一个将订阅 LCM 主题 *example_topic*，消息类型为 `exlcm/example_type`，并将消息发布到同名的 ROS2 主题（*example_topic*），使用新生成的 ROS2 消息类型 `lcm_to_ros2/example_type`。第二个示例使用主题名称 *other_topic* 执行相反操作，但使用带有覆盖哈希值的 LCM 消息类型（使用 `exlcm_rehash` 包说明符）。通过运行生成 ROS2 重新发布器代码：
    ```bash
    ./ros2repub-gen.sh repub_configs/example_republishers.cfg
    ```
    这将在 `autosrc` 文件夹中生成 CPP 重新发布器代码，并在 `launch` 目录中生成相应的 launch 文件。

3. 使用 colcon 编译新生成的代码：
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select lcm_to_ros2
    source install/setup.bash
    ```

4. 确认它有效！
    1. 在新终端中，运行 launch 文件：`ros2 launch lcm_to_ros2 example_republishers.py`
    2. 在新终端中：
        - 运行 `ros2 topic list` 并确认主题 `/example_topic` 和 `/other_topic` 存在。
        - 然后，使用以下命令显示传入消息：`ros2 topic echo /example_topic`
    3. 最后，在第三个终端中：`ros2 run lcm_to_ros2 example_send_lcm`。这是一个小的仅 LCM（无 ROS2 依赖）程序，在 LCM 主题 *example_topic* 上发布单个消息
    4. 确认消息出现在 `ros2 topic echo` 终端中。

## 生成 ROS2 消息

将 lcm 消息添加到 `lcm` 子目录。关于每个 lcm 文件的一些关键说明：
* 包名称 - 包名称由 lcm 文件第一行的 `package` 说明符设置。lcm-gen 将在名为 PACKAGE_NAME 的文件夹中生成 cpp 头文件。
* 结构名称 - 消息类型将是 lcm 文件中指定的 `struct` 名称。生成的 ROS2 消息将反映这一点，因此即使它们具有不同的包名称，也要为每个消息类型指定唯一名称。

因此，lcm 子目录中的示例 lcm 定义 `example_type.lcm` 指定了 `exlcm` 包下的 lcm 消息类型 `example_type`。还要注意，存储库 `.gitignore` 设置为忽略 lcm 目录中的文件（示例除外），这样您的消息就不会提交到存储库。要生成 LCM hpp 文件和 ROS2 msg 文件，请使用：
```bash
./ros2msg-gen.sh lcm/MY_LCM_MESSAGE.lcm
```
或者，要为 lcm 目录中的所有 .lcm 文件生成 ROS2 消息，请使用：
```bash
./ros2msg-gen.sh -a
```

对于两个 bash 脚本，使用 `-h` 标志将显示帮助消息。

## 生成重新发布器节点代码

生成 ROS2 消息后，创建配置文件以创建重新发布器。配置文件是一个简单的逗号分隔文本文件（以 `#` 开头的行被视为注释并被忽略）。我建议在 `repub_configs` 目录中创建它们，并使用 `.cfg` 扩展名。为要生成的每个重新发布器在配置文件中添加一行。每个（非注释）行应包含四个逗号分隔值的列表（不要在字段名称中使用空格）：

> topic_name, package_name, message_type, direction

其中：
* *topic_name* 是 lcm 主题的名称（相同的名称用于 ROS2 主题，但如果需要可以重新映射）
* *package_name* 是您正在使用的 lcm 消息的包名称（默认情况下，消息的重新哈希版本在 *original_package_rehash* 中生成，因此如果您想使用消息的重新哈希版本，可以使用它，如示例中所示）
* *message_type* 是消息类型（lcm 消息定义中 `struct` 之后的关键字）
* *direction* 是字符串 `lcm2ros2` 或 `ros2lcm`。请注意，同一主题上只允许一个方向，以防止无限消息反弹。如果您需要双向消息传递，可以使用一些变通方法来过滤消息，例如时间戳，但这里没有实现。

使用 `ros2repub-gen.sh` 脚本从配置文件生成重新发布器节点代码：
```bash
./ros2repub-gen.sh repub_configs/CONFIG_NAME.cfg
```

程序将在 `autosrc` 文件夹中生成 cpp 代码和这些重新发布器节点的 `CMakeLists.txt` 文件。它还会在 `launch` 文件夹中创建与配置文件名对应的 launch 文件。生成代码后，需要使用 `colcon build` 构建它。

完成后，可以使用以下命令运行重新发布器：
```bash
ros2 launch lcm_to_ros2 CONFIG_NAME.py
```
（其中 `CONFIG_NAME` 是不带文件扩展名的配置文件名）。请注意，此 launch 文件将所有重新发布器放在 `lcm_to_ros2` 命名空间下。
每个重新发布器也可以单独启动：`ros2 run lcm_to_ros2 TOPIC_NAME_republisher`。与往常一样，如果需要，可以使用标准 ROS2 命令重新映射输出主题。

## 重新哈希工具

[LCM 使用特殊的指纹计算](https://lcm-proj.github.io/type_specification.html)来识别消息，以确保可以使用用于生成它们的相同规范正确解码它们。总的来说，这是一个很好的想法。但是，在某些情况下，您可能希望指定特定的哈希值。如果需要，`ros2msg-gen` 工具将尝试创建每个消息的用户指定哈希值版本。**不建议这样做**，但在尝试强制 LCM 消息类型匹配以进行自动解码时可能很有用。总的来说，我建议您不要使用它，但如果您需要，在创建自定义 lcm 消息后，在 lcm 文件的底部添加一行，以格式 `// HASH 0x0123456789abcdef` 指定 64 位指纹（如 example_type.lcm 消息中所示）。这将创建一个新文件夹和 hpp 文件，包名称在 lcm 文件中指定，附加 `_rehash`。您可以像往常一样继续使用此消息，如示例重新发布器之一所示，它将强制 LCM 编码和解码使用指定的指纹值。

## 与 ROS1 版本的主要区别

1. **构建系统**: 使用 `ament_cmake` 而不是 `catkin`
2. **ROS API**: 使用 `rclcpp` 而不是 `roscpp`
3. **消息系统**: 使用 `rosidl` 而不是 `message_generation`
4. **Launch 文件**: 使用 Python launch 文件而不是 XML
5. **节点创建**: 使用现代 C++ 和智能指针

## 系统要求

- Ubuntu 22.04
- ROS2 Humble (或其他 ROS2 发行版)
- LCM 库
- CMake 3.8+
- C++17 编译器

