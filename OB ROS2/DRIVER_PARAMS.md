# Orbbec 驱动参数对照表（Jazzy 实测版）

> 定位：把"启动三台 GMSL 相机 + 外部触发采集"用到的驱动参数整理成一份对照表，作为写 per-camera YAML 的直接依据。
>
> 数据来源：Jetson 上实际安装的 `orbbec_camera` 包（`/opt/ros/jazzy/share/orbbec_camera/`），非归档文档。

---

## 0. 先建立心智模型

```
相机硬件
   ↓ 由 orbbec_camera_node（驱动节点）打开并出图
/camera_XXX/depth/image_raw  等 topic
   ↓ 由 ob_ros2_timestamp_collector 订阅并记录
timestamps.csv
```

- **collector 只订阅 topic**，它不会打开相机、不会读帧。
- 下面这些参数全部是**驱动节点**的参数：它们决定「打开哪台相机、什么时候触发、往 `header.stamp` 填什么时间、发布哪些流」。
- 每个参数默认值若不动，相机很可能处于「自主运行、时间戳不可比」的状态 —— 而这恰好会毁掉本项目要的「跨相机可比时间戳」。

---

## 1. 设备绑定：打开哪一台

三台相机同时接在 Jetson 上，启动驱动节点时必须指明「你负责哪一台」，否则驱动可能抓错设备或报"检测到多台设备"。

| 参数 | 默认值 | 含义 | 本项目取值 |
| --- | --- | --- | --- |
| `serial_number` | `""` | 按设备序列号绑定（字符串，不会被类型转换） | **用这个**，精确 |
| `usb_port` | `""` | 按 USB 端口绑定（字符串） | 备用 |
| `device_num` | `1` | 按第 N 台设备序号绑定 | 备用，多台时不可靠 |

本项目三台的绑定（来自 `list_devices_node` 实测）：

| namespace | 型号 | `serial_number` | GMSL 端口 |
| --- | --- | --- | --- |
| `camera_305_01` | Gemini 305g | `CV3T561000B5` | `gmsl2-5` |
| `camera_305_02` | Gemini 305g | `CV3T561000H0` | `gmsl2-4` |
| `camera_335lg_01` | Gemini 335Lg | `CPBG1630011D` | `gmsl2-6` |

---

## 2. 触发与同步：什么时候曝光（最关键）

相机默认是**软件触发、自主运行**，会无视 Jetson GPIO 那个 30 Hz 外部触发信号。必须显式改成被动外部触发。

### 2.1 `sync_mode` — 同步模式（7 个枚举值）

权威来源：`config/common.yaml:373` 的注释：

```
# Device synchronization mode. supports:
# 1.free_run 2.standalone 3.primary 4.secondary 5.secondary_synced
# 6.software_triggering 7.hardware_triggering
sync_mode: "standalone"
```

| 值 | 含义 |
| --- | --- |
| `free_run` | 自由运行，无同步 |
| `standalone` | 单机独立运行（**默认**） |
| `primary` | 同步主设备（对外输出触发） |
| `secondary` | 同步从设备 |
| `secondary_synced` | 同步从设备（GMSL 切流场景） |
| `software_triggering` | 软件定时触发 |
| `hardware_triggering` | **被动外部硬件触发** ← 本项目要这个 |

> **本项目取值：`sync_mode: hardware_triggering`**。这是唯一让相机"等 Jetson GPIO 那个外部信号来了才曝光"的模式。

### 2.2 触发相关参数

| 参数 | 默认值 | 含义 | 本项目取值 | 备注 |
| --- | --- | --- | --- | --- |
| `software_trigger_enabled` | `true` | 软件触发开关 | `false` | ⚠️ 仅 `gemini305`（305g）有；335Lg 无此参数 |
| `frames_per_trigger` | `2` | 每个触发信号出几帧 | `1` | 默认 2 会导致一个触发出两帧，同步分析会被拉偏 |
| `software_trigger_period` | `33`(ms) | 软件触发周期 | 保持/忽略 | 仅在软件触发模式生效 |
| `enable_frame_sync` | `true` | 同一触发内 color/depth 帧对齐 | `true` | 保留，保证同机 color 与 depth 配对 |
| `trigger_out_enabled` | `true` | 相机对外输出触发信号 | `false` | 本项目是外部统一触发、非菊花链，相机不该再对外发触发 |

> ⚠️ 最容易被忽略的两个坑：
> 1. `software_trigger_enabled` 默认 `true` —— 不改，相机自己在内部触发自己，外部信号被无视。
> 2. `frames_per_trigger` 默认 `2` —— 不改，一次触发出两帧。

---

## 3. 时间戳：`header.stamp` 是什么时间

collector 记录的关键字段 `header_stamp_us` 就来自驱动的 `header.stamp`。这三个参数决定它是不是「跨相机全局可比」。

| 参数 | 默认值 | 含义 | 本项目取值 |
| --- | --- | --- | --- |
| `time_domain` | `global` | 时间域：`global`(全局基准) / `device`(相机本地钟) / `system`(主机钟) | `global` |
| `enable_sync_host_time` | `true` | 是否用主机时间去校正相机钟 | **待实测确认**（见 7） |
| `time_sync_period` | `6.0`(s) | 主机时间同步周期 | 保持默认（仅 305g 有） |

**为什么 `time_domain` 必须是 `global`**：如果设成 `device`，三台相机各用各的本地钟，它们的 `header.stamp` 互相不可加减，后面的跨相机同步分析全部失效。

> 两个型号都有 `time_domain`，取值都是 `global`/`device`/`system`，默认都是 `global` ✅（已从 launch 文件双重确认）。

---

## 4. 图像流：发布哪些 topic

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `enable_color` | `true` | 出彩色流（→ 发布 color topic） |
| `enable_depth` | `true` | 出深度流（→ 发布 depth topic） |
| `enable_left_ir` / `enable_right_ir` | `false` | 红外流（本项目不需要） |
| `color_width` / `color_height` / `color_fps` | `0` | `0` = 用相机 profile 默认值 |
| `depth_width` / `depth_height` / `depth_fps` | `0` | 同上 |
| `color_format` / `depth_format` | `MJPG`/`ANY`(305g) 或 `ANY`/`ANY`(335Lg) | 像素格式 |

本项目只要 **color + depth** 两个流，共 3 台 × 2 = 6 个 topic。width/height/fps 先保持 `0`（让相机 profile 决定），等实测确认分辨率后再决定要不要指定。

---

## 5. 305g vs 335Lg 的差异（写 YAML 时不能搞混）

| 维度 | Gemini 305g (`gemini305.launch.py`) | Gemini 335Lg (`gemini345_lg.launch.py`) |
| --- | --- | --- |
| 软件触发参数 | 有 `software_trigger_enabled` | **无**（只有 `frames_per_trigger`） |
| 双彩色流 | 有 `enable_left_color` / `enable_right_color` | 无 |
| GMSL 触发参数 | 有 `enable_gmsl_trigger` / `gmsl_trigger_fps` | 待确认 |
| `device_preset` 默认 | `Default`（双彩配置里为 `Dual Color Streams`） | `Standard` |
| `align_mode` 默认 | `SW` | `HW` |
| `depth_registration` 默认 | `false` | `true` |
| `time_sync_period` | `6.0` | 无 |

> ⚠️ 型号命名混乱：HANDOFF 写「Gemini 335Lg」，但 launch 文件叫 `gemini345_lg`，config 目录里另有 `gemini330Lg_series.yaml`。三者对不上，写 YAML 前需以 `list_devices_node` 报出的型号 + 实测 topic 为准。

---

## 6. GMSL 专属参数（已读全，结论如下）

三台相机全是 **GMSL2** 连接，官方专用 launch 在 `examples/gmsl_camera/` 下：

```
/opt/ros/jazzy/share/orbbec_camera/examples/gmsl_camera/
├── gemini_330_gmsl.launch.py            # 单台 GMSL 相机的通用启动（约 200 个参数，全默认）
├── multi_gmsl_camera.launch.py          # 2 台，sync_mode=standalone（不同步）
└── multi_gmsl_camera_synced.launch.py    # 2 台，secondary_synced + enable_gmsl_trigger（SoC 生成触发）
```

`find ... | grep -i gmsl` 返回**空**：包里不存在 GMSL 专属 YAML，所有 GMSL 参数都复用普通参数名，定义在 `config/common.yaml`：

```yaml
# common.yaml:392
gmsldevice_trigger:
  enable_gmsl_trigger: false
  gmsl_trigger_fps: 3000
```

**两个 GMSL 参数的确切含义（已从官方示例反推确认）：**

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `enable_gmsl_trigger` | `false` | **由 SoC 生成 GMSL 触发信号**。`multi_gmsl_camera_synced.launch.py` 里有一台设为 `true`、另一台跟随——那是「SoC 当触发源」的拓扑 |
| `gmsl_trigger_fps` | `3000` | SoC 生成触发时的频率 |

**本项目拓扑（外部 GPIO 30Hz 信号进来）→ 两者都不该动：`enable_gmsl_trigger` 保持 `false`，相机只被动接收外部信号。** 外部被动触发走的是通用的 `sync_mode: hardware_triggering`，不是这套 GMSL 触发生成。

---

## 7. 尚未敲定、需实测的两个点

1. **`enable_sync_host_time` 到底开还是关**：驱动默认 `true`，但本项目 README 之前写的是 `false`。两者矛盾。语义上是「是否用主机钟校正设备钟」，直接影响 `header.stamp` 是曝光时刻还是主机接收时刻。**需起一台相机实测 `header.stamp` 的数值特征后确认**，不盲信任何一方。
2. **`sync_mode: hardware_triggering` 与 GMSL 的关系（已解决 ✅）**：GMSL 相机照常走 `sync_mode`。官方 `secondary_synced` 是 GMSL 内部同步（SoC 生成触发）拓扑，与我们无关；`enable_gmsl_trigger` 是「SoC 生成触发」开关，必须保持 `false`。外部 GPIO 被动触发 → `sync_mode: hardware_triggering` + `enable_gmsl_trigger: false`。

---

## 8. 汇总：每个参数的「本项目取值」清单

| 参数 | 本项目取值 | 状态 |
| --- | --- | --- |
| `serial_number` | 按三台各自的串号 | ✅ 确定 |
| `camera_name` | `camera_305_01` / `camera_305_02` / `camera_335lg_01` | ✅ 确定 |
| `sync_mode` | `hardware_triggering` | ✅ 确定 |
| `software_trigger_enabled` | `false`（仅 305g） | ✅ 确定 |
| `frames_per_trigger` | `1` | ✅ 确定 |
| `trigger_out_enabled` | `false` | ✅ 确定 |
| `enable_frame_sync` | `true` | ✅ 确定 |
| `time_domain` | `global` | ✅ 确定 |
| `enable_sync_host_time` | 待实测 | ⚠️ §7-1 |
| `enable_color` / `enable_depth` | `true` / `true` | ✅ 确定 |
| `enable_left_ir` / `enable_right_ir` 等 | `false` | ✅ 确定 |
| `enable_gmsl_trigger` | `false` | ✅ 确定（SoC 生成触发，外部触发拓扑下关） |
| `gmsl_trigger_fps` | 不启用 | ✅ 确定（跟随 enable_gmsl_trigger 关闭） |
| 其余 filter/exposure/点云 | 保持默认关闭或不动 | ✅ 默认即可 |