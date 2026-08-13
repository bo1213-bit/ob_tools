# 项目交接文档（TimestampRecorder 多相机时间戳精度验证）

> 本文件是给接手本项目的开发者/AI 的交接说明。总结了已完成的改动、关键机制、踩过的坑和遗留问题。

## 1. 项目目标

验证 **3 台 Orbbec 相机**（两台 Gemini305g + 一台 Gemini335Lg，全部仅支持 `SECONDARY` 同步模式）之间的**时间戳同步精度**，并生成直方图。

三台相机**只能设为 SECONDARY**（无 PRIMARY 主机发硬件触发信号），因此没有硬件级帧同步，帧在时间轴上天然错开（常差一个帧周期 33ms）。

## 2. 项目目录（注意有两个相似目录）

| 目录 | 作用 | 是否被改动 |
|---|---|---|
| `Multi-Device-Synchronization-Example-2-main/` | **实际编译运行的项目**（Orbbec 官方示例 + 我们的 TimestampRecorder） | ✅ 改这里 |
| `MultiDeviceSync-Headless/` | 另一个独立的无头示例副本 | ❌ 没动它，是独立项目 |

**接手的人要改的是 `Multi-Device-Synchronization-Example-2-main/`，不是 Headless。** 两者 `common/` 下文件逻辑几乎相同但互不相干，别改错目录。

## 3. 新增/修改的文件清单（当前状态）

### ✅ 新增
| 文件 | 作用 |
|---|---|
| `Multi-Device-Synchronization-Example-2-main/MultiDeviceSync/TimestampRecorder.cpp` | **新写的时间戳采集器**：读 JSON 配置同步 → 开流 → FramePairingManager 配对 → 把配对成功的每帧 `hwTimestampUs/globalTimestampUs/sysTimestampUs` 写入 CSV。用法 `./TimestampRecorder --csv <path> [--duration <秒>]`，不传 duration 则一直跑（Ctrl+C 退出）。复用官方 loadConfigFile/configMultiDeviceSync/startDeviceStreams 逻辑 |
| `src/visualize_timestamps.py` | 直方图脚本：读 TimestampRecorder 的 CSV，按 groupId 算跨设备 hw 差，输出与 `src/visualize_sync.py` 同风格双面板直方图（Depth/Color，50us bin，Mean/Median/Std 统计框）。用法 `python visualize_timestamps.py <csv> -o <输出目录>` |

### ✅ 修改
| 文件 | 改动 | 状态 |
|---|---|---|
| `common/PipelineHolder.cpp` | **方案 B（核心）**：`startStream()` 不再用 `getProfile(OB_PROFILE_DEFAULT)`，改为**遍历 profile 列表，优先选 1280×800 @ 30fps**（找不到 30fps 退任意帧率 1280×800，再找不到打印所有可用 profile 并回退第一个）。并打印实际启用的分辨率/帧率/格式。**影响所有用 PipelineHolder 的程序**（MultiDeviceSync 预览等也会 1280×800） | ✅ 已生效 |
| `CMakeLists.txt` | 新增 `TimestampRecorder` target（复用 common/ 的 PipelineHolder/FramePairingManager/cJSON/utils，与 MultiDeviceSyncRecord 同款链接） | ✅ 已生效 |
| `FramePairingManager.cpp` | **临时改动过又已回退到原样**（曾在配对失败分支加"弹帧防滞留"逻辑，后回退）。当前是官方原版 | ⚠️ 已回退 |

### ⚠️ 已改但最终未采用的方案（别再用它）
| 文件 | 改动 | 结论 |
|---|---|---|
| `config/OrbbecSDKConfig.xml` | 改过 `<Pipeline>` 段的 `UseDefaultStreamProfile=false` + 1280×800，以及 `<Device>` 段 Gemini305g/335Lg 的 Width/Height | **无效**。实测证明 `getProfile(OB_PROFILE_DEFAULT)` 不读 XML 的 Pipeline 段，分辨率由相机固件默认 profile 决定。**最终靠方案 B（改代码）解决，XML 改动已无意义** |

## 4. 关键机制（数据流向）

```
生产（SDK采集线程）                    消费（主线程）
相机 → 回调 processFrame()            → FramePairingManager::getFramePairs()
  → 加锁 push 进 obFrames 队列          → 等6条队列非空
  （每设备每流一个队列，容量16满丢最老）    → 读各队首按时间戳排序
                                        → 以最早队首为参考，逐个比 diff
                                        → 全在 halfGap(17ms)内 → getFrame()弹出6帧成组
                                        → 任一超窗 → 整组丢弃，帧滞留队列
```

- **配对单位**：6 条流水线（3 台 × depth+color）的队首帧，必须全部落在 ±17ms（halfGap = 500/fps @30fps）窗口内才弹出一组。
- **滞留帧**：配对失败时帧不弹出、留在队首，下一轮又读到 → 若相机时钟错开一整帧会反复失败。
- **`pair.first` = depth，`pair.second` = color**（emplace_back(depth, color)）。
- `refTsp` 打印的是毫秒（`getTimeStampUs()/1000`），配对成功打印的是微秒，别混看。

## 5. 踩过的坑 / 已解决

1. **分辨率配不出来**：改 XML 无效（见上），最终方案 B 遍历 profile 列表挑 1280×800@30fps 解决。
2. **Color 帧率 60fps**：相机 1280×800 下 Color 有 60fps 和 30fps 两档，遍历时优先挑 30fps 使 Color/Depth 一致。
3. **开局 0 groups**：相机刚开机时 `enableDeviceClockSync(60000)` 还没校准完，硬件时钟差几百 ms，配对全失败（CSV 空）。**等一会儿（时钟稳定）再采集就正常**，本次实测 274 组全配对成功。
4. **PAIR-DROP 正常性**：三台 SECONDARY 无硬件同步，偶发差一整帧（33ms）导致 PAIR-DROP 是**预期行为**，不是 bug。

## 6. 运行方法（Linux：192.168.137.6, 用户 user）

```bash
# 项目在 Linux 上：
cd ~/Multi-Device-Synchronization-Example/Multi-Device-Synchronization-Example-2-main/build
make TimestampRecorder -j$(nproc)
cd bin
./TimestampRecorder --csv ./ts.csv --duration 30   # 采集30秒
```

改代码后需要把文件 scp 到 Linux 对应路径再重新编译。**注意：若删过 `build/bin` 目录，需 `mkdir -p bin` 再 make；配置文件靠 `cmake ..` 的 file(COPY) 拷贝，重配置后才有。**

生成直方图（Windows 上）：
```bash
cd D:/Data/robotPackage/ob_tools/src
python visualize_timestamps.py "C:/path/to/timestamps.csv" -o "C:/path/to/output"
```

## 7. 遗留问题 / 待接手者考虑

1. **同步精度分析的正确数据源**：当前 TimestampRecorder 保留 FramePairingManager 配对，CSV 只含"配对成功的帧"（时钟稳定时约 90%+）。若要**每台相机完整帧序列**做离线帧间隔/漂移分析，需改成"每台独立 `tryGetFrame()` 记录全部帧"（类似 `src/data_collector.cpp`）。
2. **用 global 还是 hw 时间戳**：`timeStampUs()` 是各相机本地硬件时钟（可能差几百 ms）；`globalTimeStampUs()` 是 `enableDeviceClockSync` 校准后的跨设备时钟。配对目前用 hw（`timeStampUs()`），若要更高配对率可考虑改用 global。
3. **开机时钟校准**：`enableDeviceClockSync(60000)` 每 60s 校准一次，采集前建议等时钟稳定。
4. `OrbbecSDKConfig.xml` 的改动残留（Pipeline 段 UseDefaultStreamProfile=false 等）已无作用，可清理或保留，不影响（代码不读它）。

## 8. 源码位置速查

| 内容 | 文件:行号 |
|---|---|
| TimestampRecorder 主循环（配对+写CSV） | `MultiDeviceSync/TimestampRecorder.cpp` 约 L309+ |
| 分辨率选择（方案B） | `common/PipelineHolder.cpp` L12-77 |
| 配对逻辑 | `common/FramePairingManager.cpp` `getFramePairs()` L48-136 |
| 队列生产 | `common/PipelineHolder.cpp` `processFrame()` L79-93 |
| 队列消费（front/get） | `common/PipelineHolder.cpp` L73-107 |
| 直方图脚本 | `src/visualize_timestamps.py` |
