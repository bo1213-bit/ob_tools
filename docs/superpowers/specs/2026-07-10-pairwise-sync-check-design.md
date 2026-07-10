# Pairwise System Timestamp Sync Check — Design

## Date

2026-07-10

## Purpose

将 `timestamp_sync_check.cpp` 从"2 台相机 master/slave 硬件时间戳对比"改为"N 台相机 pairwise 软件端到端时间戳精度验证"，验证阈值 < 1ms。

## 时间戳定义

| 时间戳 | 来源 | 用途 |
|--------|------|------|
| **硬件时间戳 (hw)** | `frame->getMetadataValue(OB_FRAME_METADATA_TYPE_TIMESTAMP)` | 帧配对 — 硬件同步保证同一帧值一致 |
| **软件时间戳 (sys)** | `std::chrono::system_clock::now()` 在 SDK 回调**第一行**记录 | 对比验证 — 两相机 sys 差值 = 软件端到端精度 |

**关键约束**：
- 硬件时间戳只用于配对，不做对比
- 软件时间戳在回调入口立刻记录，之后再取 frame metadata，避免业务逻辑引入额外延迟
- 不使用 `OB_FRAME_METADATA_TYPE_SENSOR_TIMESTAMP`，因为文档明确说不同设备型号单位可能不同

## 架构

### 类 `TimestampSyncChecker`

```cpp
class TimestampSyncChecker {
public:
    void enumerateDevices();          // 枚举 N 台设备
    void configureSyncMode();         // 第一台 PRIMARY，其余 SECONDARY
    void resetTimestampAndSyncClock();
    void collectFrames(int durationSec);
    void matchAndComputeDiffs();      // 全矩阵 pairwise 配对 + 对比
    bool printStatistics();

private:
    // 配置常量
    static constexpr int kDefaultFPS    = 30;
    static constexpr int kDefaultWidth  = 640;
    static constexpr int kDefaultHeight = 400;

    // 运行时参数（可从命令行覆盖）
    int fps_    = kDefaultFPS;
    int width_  = kDefaultWidth;
    int height_ = kDefaultHeight;

    // N 台设备
    std::shared_ptr<ob::Context>                     context_;
    std::vector<std::shared_ptr<ob::Device>>         devices_;
    std::vector<std::shared_ptr<ob::Pipeline>>       pipelines_;
    std::vector<std::queue<FrameStamp>>              queues_;
    std::vector<std::mutex>                          mutexes_;
    std::vector<std::vector<FrameStamp>>             allFrames_;

    // 配对结果：pairDiffs_[{i,j}] = 以 i 为基准，配对到 j 的 sys 差值列表
    std::map<std::pair<int, int>, std::vector<int64_t>> pairDiffs_;
    int missedCount_ = 0;
};
```

### 配对逻辑 (`matchAndComputeDiffs`)

```
for each pair (i, j) where i < j:
    for each frame f_i in allFrames_[i]:
        在 allFrames_[j] 中找 hw 时间戳最近的帧 f_j
        if |f_i.hw - f_j.hw| < hwThreshold:
            diff = f_i.sys - f_j.sys
            pairDiffs_[{i,j}].push_back(diff)
        else:
            missedCount_++
```

**配对阈值**：默认 500us，因为硬件同步帧的 hw 时间戳应该非常接近。

### 数据流

```
enumerateDevices() → configureSyncMode() → resetTimestampAndSyncClock()
    → collectFrames(duration) → matchAndComputeDiffs() → printStatistics()
```

### 不变条件

- 方法调用顺序固定，调用方负责
- 类不继承、不拷贝、不移动
- 错误通过异常传播，main() 统一 catch

## 统计输出

### 每对相机统计

每对 (i,j) 输出：
- pair count、missed count
- min / max / abs max / mean / stddev（单位 us）
- PASS/FAIL 判定（abs max < sync-threshold）

### 全局汇总

- 最差 pair（abs max 最大）
- 全局 PASS/FAIL 判定

### 直方图

- 默认只画最差 pair 的直方图
- `--histogram-all` 参数可展开所有 pair
- 10 bin 可配置

### CSV 导出

- 可选，通过 `--csv=path` 启用
- 格式：`pair_i,pair_j,diff_us`
- 不传 `--csv` 则不导出

## 命令行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--duration` | `30` | 采集时长（秒） |
| `--hw-threshold` | `500` | 硬件时间戳配对阈值（us） |
| `--sync-threshold` | `1000` | 软件同步精度判定阈值（us），即 <1ms |
| `--fps` | `30` | 采集帧率 |
| `--width` | `640` | 深度图宽度 |
| `--height` | `400` | 深度图高度 |
| `--csv` | `""` | CSV 文件路径，不传则不导出 |
| `--histogram-bins` | `10` | 直方图 bin 数 |
| `--histogram-all` | `false` | 是否对所有 pair 画直方图 |

**解析方式**：手动解析 `argc/argv`，零外部依赖。

## 使用说明

另行编写 `README_timestamp_sync_check.md`，包含编译步骤、使用示例、输出解读。

## 构建

不变，使用现有 `CMakeLists.txt`。`fps_quality_test` 目标保持不变。

## 不涉及

- 不修改 CMakeLists.txt
- 不引入新的第三方库
- 保持 C++11 标准