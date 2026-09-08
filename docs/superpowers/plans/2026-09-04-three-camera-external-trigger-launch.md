# 三相机外部触发启动 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 ROS 2 Jazzy 的两台 Gemini 305g 与一台 Gemini 335Lg 提供项目自有的外部硬件触发三相机启动入口，并将其接入已有时间戳采集器。

**Architecture:** 新 launch 直接创建三组唯一命名的 ROS 2 component container，每组装载一个 `orbbec_camera::OBCameraNodeDriver`。每台设备使用单独的 YAML 保存稳定序列号绑定和已验证的被动硬件触发参数；组合 launch 包含项目 launch 和已有采集器节点。

**Tech Stack:** ROS 2 Jazzy launch Python、`launch_ros`、`rclcpp_components`、Orbbec ROS Wrapper 2.7.6、ament_cmake。

**Spec:** `docs/superpowers/specs/2026-09-04-ob-ros2-three-camera-launch-design.md`

## Global Constraints

- 目标环境是 Ubuntu 24.04、ROS 2 Jazzy、Jetson arm64；本 Windows 环境不具备 ROS 2/colcon 硬件运行条件。
- 必须通过序列号绑定：`CV3T561000B5`、`CV3T561000H0`、`CPBG1630011D`。
- 绝不写 GPIO sysfs，绝不产生 GMSL/SoC trigger，绝不启用 `trigger_out_enabled`。
- 所有相机使用 `sync_mode: hardware_triggering`、`frames_per_trigger: 1`、`enable_point_cloud: false`、`enable_sync_host_time: false`、`time_domain: global`、`enable_color: true`、`enable_depth: true`、`device_preset: Default`。
- 仅两台 Gemini 305g 使用 `enable_gmsl_trigger: false`；Gemini 335Lg YAML 不得包含该参数。
- 不固定分辨率、格式或 FPS；由设备和 Jazzy 驱动选择默认兼容 profile。
- 不修改 `src/` 下的采集器 C++ 代码，也不修改 `/opt/ros/jazzy/share/orbbec_camera` 中的驱动文件。
- 不提交 Git 变更，除非用户后来明确请求。

---

## 文件结构

| 文件 | 职责 |
| --- | --- |
| `OB ROS2/config/gmsl/camera_305_01.yaml` | 305g（`CV3T561000B5`）的稳定设备绑定与外部触发参数。 |
| `OB ROS2/config/gmsl/camera_305_02.yaml` | 305g（`CV3T561000H0`）的稳定设备绑定与外部触发参数。 |
| `OB ROS2/config/gmsl/camera_335lg_01.yaml` | 335Lg（`CPBG1630011D`）的稳定设备绑定与外部触发参数，不传 305g 专用 GMSL 触发参数。 |
| `OB ROS2/launch/three_gmsl_external_trigger.launch.py` | 创建三组独立容器，并将每份 YAML 参数传给对应的 Orbbec component。 |
| `OB ROS2/launch/gmsl_scheme_a_capture.launch.py` | 默认包含三相机 launch 并启动时间戳采集器。 |
| `OB ROS2/config/gmsl_scheme_a_collector.yaml` | 六路实际命名空间主题的采集器订阅配置。 |
| `OB ROS2/README.md` | Jazzy 构建、外部触发、相机与采集器启动、验证流程文档。 |
| `OB ROS2/package.xml` | 声明新 launch 所需的运行时依赖。 |

### Task 1: 添加每台相机的已验证参数 YAML

**Files:**
- Create: `OB ROS2/config/gmsl/camera_305_01.yaml`
- Create: `OB ROS2/config/gmsl/camera_305_02.yaml`
- Create: `OB ROS2/config/gmsl/camera_335lg_01.yaml`

**Interfaces:**
- Consumes: Orbbec component 接受的参数字典，来自已验证的官方 `gemini_330_gmsl.launch.py` 与 `gemini345_lg.launch.py`。
- Produces: 供 `three_gmsl_external_trigger.launch.py` 作为 `parameters=[yaml_path]` 加载的 YAML 文件。

- [ ] **Step 1: 新建 305g-01 参数文件**

创建 `OB ROS2/config/gmsl/camera_305_01.yaml`：

```yaml
/**:
  ros__parameters:
    serial_number: "CV3T561000B5"
    sync_mode: hardware_triggering
    frames_per_trigger: 1
    trigger_out_enabled: false
    enable_gmsl_trigger: false
    enable_point_cloud: false
    enable_sync_host_time: false
    time_domain: global
    enable_color: true
    enable_depth: true
    device_preset: Default
```

- [ ] **Step 2: 新建 305g-02 参数文件**

创建 `OB ROS2/config/gmsl/camera_305_02.yaml`，唯一设备差异为序列号：

```yaml
/**:
  ros__parameters:
    serial_number: "CV3T561000H0"
    sync_mode: hardware_triggering
    frames_per_trigger: 1
    trigger_out_enabled: false
    enable_gmsl_trigger: false
    enable_point_cloud: false
    enable_sync_host_time: false
    time_domain: global
    enable_color: true
    enable_depth: true
    device_preset: Default
```

- [ ] **Step 3: 新建 335Lg 参数文件**

创建 `OB ROS2/config/gmsl/camera_335lg_01.yaml`。不要添加 `enable_gmsl_trigger`：

```yaml
/**:
  ros__parameters:
    serial_number: "CPBG1630011D"
    sync_mode: hardware_triggering
    frames_per_trigger: 1
    trigger_out_enabled: false
    enable_point_cloud: false
    enable_sync_host_time: false
    time_domain: global
    enable_color: true
    enable_depth: true
    device_preset: Default
```

- [ ] **Step 4: 静态验证 YAML 数据与参数差异**

Run (PowerShell):

```powershell
python -c "import pathlib, yaml; p=pathlib.Path(r'OB ROS2/config/gmsl'); docs={x.name: yaml.safe_load(x.read_text(encoding='utf-8'))['/**']['ros__parameters'] for x in p.glob('*.yaml')}; assert docs['camera_305_01.yaml']['serial_number']=='CV3T561000B5'; assert docs['camera_305_02.yaml']['serial_number']=='CV3T561000H0'; assert docs['camera_335lg_01.yaml']['serial_number']=='CPBG1630011D'; assert all(v['sync_mode']=='hardware_triggering' and v['trigger_out_enabled'] is False and v['time_domain']=='global' for v in docs.values()); assert all(v['enable_gmsl_trigger'] is False for n,v in docs.items() if '335lg' not in n); assert 'enable_gmsl_trigger' not in docs['camera_335lg_01.yaml']; print('YAML parameters verified')"
```

Expected: `YAML parameters verified`.

### Task 2: 创建三相机 component 启动入口

**Files:**
- Create: `OB ROS2/launch/three_gmsl_external_trigger.launch.py`
- Modify: `OB ROS2/package.xml:9-15`

**Interfaces:**
- Consumes: Task 1 的三份 YAML 文件；`ament_index_python.packages.get_package_share_directory`；`launch_ros.descriptions.ComposableNode`。
- Produces: `generate_launch_description() -> LaunchDescription`，可通过 `ros2 launch ob_ros2_timestamp_collector three_gmsl_external_trigger.launch.py` 启动三台相机。

- [ ] **Step 1: 创建 launch 文件，定义相机描述**

创建 `OB ROS2/launch/three_gmsl_external_trigger.launch.py`，使用不可变相机描述列表确保 YAML、namespace、container 和 node 名称一一对应：

```python
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


CAMERAS = (
    ("camera_305_01", "camera_305_01_container", "camera_305_01.yaml"),
    ("camera_305_02", "camera_305_02_container", "camera_305_02.yaml"),
    ("camera_335lg_01", "camera_335lg_01_container", "camera_335lg_01.yaml"),
)


def generate_launch_description():
    package_share = get_package_share_directory("ob_ros2_timestamp_collector")
    actions = []

    for camera_name, container_name, config_name in CAMERAS:
        config_path = f"{package_share}/config/gmsl/{config_name}"
        actions.append(
            ComposableNodeContainer(
                name=container_name,
                namespace=camera_name,
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="orbbec_camera",
                        plugin="orbbec_camera::OBCameraNodeDriver",
                        name=camera_name,
                        parameters=[config_path],
                    ),
                ],
                output="screen",
            )
        )

    return LaunchDescription(actions)
```

- [ ] **Step 2: 为 launch 添加缺失的运行时依赖**

在 `OB ROS2/package.xml` 的现有 `<exec_depend>` 后添加：

```xml
  <exec_depend>rclcpp_components</exec_depend>
  <exec_depend>orbbec_camera</exec_depend>
```

不要将官方驱动添加为 build 依赖：项目仅在运行时加载它的 component。

- [ ] **Step 3: 运行 Python 语法检查**

Run (PowerShell):

```powershell
python -m py_compile "OB ROS2/launch/three_gmsl_external_trigger.launch.py"
```

Expected: exit code 0 and no output.

- [ ] **Step 4: 在 Jetson 验证三机启动**

Run (Jetson):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ob_ros2_timestamp_collector three_gmsl_external_trigger.launch.py
```

Expected: 三个独立 container 启动；日志分别确认 `CV3T561000B5`、`CV3T561000H0`、`CPBG1630011D` 已匹配，且不出现由项目参数引入的 `Invalid preset name: Standard`。

- [ ] **Step 5: 在 Jetson 验证节点与主题**

保持 Step 4 launch 运行，在另一个 Jetson 终端执行：

```bash
source /opt/ros/jazzy/setup.bash
ros2 node list
ros2 topic list | grep -E '^/camera_(305_01|305_02|335lg_01)/(color|depth)/image_raw$'
```

Expected: 三个相机命名空间下均存在 color 和 depth 原始图像主题。若实际 topic 命名不同，记录真实名称并在 Task 3 的 YAML 中替换，不改相机 launch 的 namespace。

### Task 3: 接入采集器默认配置与组合启动

**Files:**
- Modify: `OB ROS2/config/gmsl_scheme_a_collector.yaml:1-26`
- Modify: `OB ROS2/launch/gmsl_scheme_a_capture.launch.py:1-47`

**Interfaces:**
- Consumes: Task 2 的 `three_gmsl_external_trigger.launch.py`；采集器现有 `timestamp_collector_node`；实际 image topic 路径。
- Produces: `ros2 launch ob_ros2_timestamp_collector gmsl_scheme_a_capture.launch.py` 默认同时启动三台相机及采集器。

- [ ] **Step 1: 替换采集器订阅配置为六路主题**

将 `OB ROS2/config/gmsl_scheme_a_collector.yaml` 的 `subscriptions` 替换为：

```yaml
    subscriptions:
      - "camera_305_01|depth|/camera_305_01/depth/image_raw"
      - "camera_305_01|color|/camera_305_01/color/image_raw"
      - "camera_305_02|depth|/camera_305_02/depth/image_raw"
      - "camera_305_02|color|/camera_305_02/color/image_raw"
      - "camera_335lg_01|depth|/camera_335lg_01/depth/image_raw"
      - "camera_335lg_01|color|/camera_335lg_01/color/image_raw"
```

保留既有的输出、QoS 和写盘参数。首次 Jetson 三机启动如果 topic 路径不同，只替换对应路径字符串，保持 `camera_name|stream_type|topic` 格式不变。

- [ ] **Step 2: 改造组合 launch 为默认包含项目三机 launch**

用下列实现替换 `OB ROS2/launch/gmsl_scheme_a_capture.launch.py`，删除 `driver_launch_file`、`OpaqueFunction` 和用户自定义驱动 launch 的校验：

```python
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("ob_ros2_timestamp_collector")
    default_config = f"{package_share}/config/gmsl_scheme_a_collector.yaml"
    camera_launch = f"{package_share}/launch/three_gmsl_external_trigger.launch.py"

    return LaunchDescription([
        DeclareLaunchArgument(
            "collector_config_file",
            default_value=default_config,
            description="Path to the timestamp collector YAML parameter file.",
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(camera_launch)),
        Node(
            package="ob_ros2_timestamp_collector",
            executable="timestamp_collector_node",
            name="timestamp_collector_node",
            output="screen",
            parameters=[LaunchConfiguration("collector_config_file")],
        ),
    ])
```

- [ ] **Step 3: 运行 Python/YAML 静态检查**

Run (PowerShell):

```powershell
python -m py_compile "OB ROS2/launch/gmsl_scheme_a_capture.launch.py" "OB ROS2/launch/three_gmsl_external_trigger.launch.py"
python -c "import yaml; p=r'OB ROS2/config/gmsl_scheme_a_collector.yaml'; s=yaml.safe_load(open(p, encoding='utf-8'))['/**']['ros__parameters']['subscriptions']; assert len(s)==6; assert {x.split('|')[0] for x in s}=={'camera_305_01','camera_305_02','camera_335lg_01'}; assert {x.split('|')[1] for x in s}=={'color','depth'}; print('Collector subscriptions verified')"
```

Expected: `Collector subscriptions verified`.

- [ ] **Step 4: 在 Jetson 验证组合启动的 CSV 输出**

临时将 collector YAML 的 `duration_sec` 设置为 `10`、`save_images` 设置为 `false`，然后运行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ob_ros2_timestamp_collector gmsl_scheme_a_capture.launch.py
```

Expected: 所有三台相机启动，采集器在约 10 秒后退出；`timestamps.csv` 包含六种唯一的 `camera_name,stream_type` 组合。

### Task 4: 更新 Jazzy 操作与验证文档

**Files:**
- Modify: `OB ROS2/README.md:1-151`

**Interfaces:**
- Consumes: Tasks 1-3 的启动入口、YAML 与主题约定。
- Produces: 一个可从干净 Jetson workspace 跟随执行的构建和验证说明。

- [ ] **Step 1: 更新运行边界与驱动说明**

将 README 开头的“collector-only”和“必须单独启动自定义官方驱动 launch”描述改为：项目提供 `three_gmsl_external_trigger.launch.py` 负责三台 Orbbec driver component，采集器仍只订阅 image topic，不处理 GPIO、触发生成、帧匹配或同步精度计算。

列出设备映射：

```text
camera_305_01  -> Gemini 305g  -> CV3T561000B5 -> gmsl2-5
camera_305_02  -> Gemini 305g  -> CV3T561000H0 -> gmsl2-4
camera_335lg_01 -> Gemini 335Lg -> CPBG1630011D -> gmsl2-6
```

- [ ] **Step 2: 替换 Humble 构建命令为 Jazzy**

所有 `source /opt/ros/humble/setup.bash` 替换为：

```bash
source /opt/ros/jazzy/setup.bash
```

构建章节保留不含空格的工作空间包目录建议：

```bash
mkdir -p ~/ros2_ws/src
cp -a "OB ROS2" ~/ros2_ws/src/ob_ros2_timestamp_collector
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ob_ros2_timestamp_collector
source install/setup.bash
```

- [ ] **Step 3: 增加分阶段启动和主题验证命令**

添加以下顺序：先由操作者启用外部 GPIO 触发，再运行项目 launch；明确 launch 不执行 GPIO 命令。

```bash
echo 30 | sudo tee /sys/kernel/debug/gpio_trigger/framerate
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ob_ros2_timestamp_collector three_gmsl_external_trigger.launch.py
```

另一个终端：

```bash
ros2 topic list | grep -E '^/camera_(305_01|305_02|335lg_01)/(color|depth)/image_raw$'
ros2 topic hz /camera_305_01/color/image_raw
ros2 topic hz /camera_335lg_01/color/image_raw
```

注明 335Lg 在当前硬件触发设置下 color 约 15 Hz 是预期行为；也要求检查每台相机的 depth 主题持续输出。

- [ ] **Step 4: 增加组合采集与图像保存验证**

记录默认组合启动：

```bash
ros2 launch ob_ros2_timestamp_collector gmsl_scheme_a_capture.launch.py
```

文档要求先用 `duration_sec: 10` 和 `save_images: false` 确认 CSV 具有六路记录，再启用 `save_images: true`，并验证：每个非空 `image_path` 存在、文件大小等于 `data_size`，退出日志中 `dropped_image_jobs` 为零。

- [ ] **Step 5: 在 Jetson 执行最终 package 验证**

Run (Jetson):

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ob_ros2_timestamp_collector
colcon test --packages-select ob_ros2_timestamp_collector --event-handlers console_direct+
colcon test-result --verbose
python3 -m py_compile install/ob_ros2_timestamp_collector/share/ob_ros2_timestamp_collector/launch/timestamp_collector.launch.py
python3 -m py_compile install/ob_ros2_timestamp_collector/share/ob_ros2_timestamp_collector/launch/three_gmsl_external_trigger.launch.py
python3 -m py_compile install/ob_ros2_timestamp_collector/share/ob_ros2_timestamp_collector/launch/gmsl_scheme_a_capture.launch.py
```

Expected: 构建成功、现有 C++ 测试通过、三个 launch 都可编译；硬件阶段验证按 Task 2-3 完成。

- [ ] **Step 6: 检查工作树，不提交**

Run (PowerShell):

```powershell
git status --short -- "OB ROS2" "docs/superpowers"
```

Expected: 仅显示本计划所列的新建/修改文件，以及之前已有的未跟踪 `OB ROS2/`；不要执行 `git add` 或 `git commit`。
