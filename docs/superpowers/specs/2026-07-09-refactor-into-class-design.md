# Refactor timestamp_sync_check.cpp into a Class

## Date

2026-07-09

## Purpose

将 `timestamp_sync_check.cpp` 中 `main()` 的 6 个步骤拆分为 `TimestampSyncChecker` 类的成员方法，提高代码可读性和可维护性。

## Design

### Class: `TimestampSyncChecker`

#### 成员变量

| 变量 | 类型 | 说明 |
|------|------|------|
| `context` | `std::shared_ptr<ob::Context>` | SDK 上下文 |
| `masterDev` | `std::shared_ptr<ob::Device>` | 主设备（设备 0） |
| `slaveDev` | `std::shared_ptr<ob::Device>` | 从设备（设备 1） |
| `masterPipe` | `std::shared_ptr<ob::Pipeline>` | 主设备 pipeline |
| `slavePipe` | `std::shared_ptr<ob::Pipeline>` | 从设备 pipeline |
| `masterQueue` | `std::queue<FrameStamp>` | 主设备帧队列（回调写入） |
| `slaveQueue` | `std::queue<FrameStamp>` | 从设备帧队列（回调写入） |
| `masterMutex` | `std::mutex` | 保护 masterQueue |
| `slaveMutex` | `std::mutex` | 保护 slaveQueue |
| `masterFrames` | `std::vector<FrameStamp>` | 从队列搬出的主设备帧（Step 5 消费） |
| `slaveFrames` | `std::vector<FrameStamp>` | 从队列搬出的从设备帧（Step 5 消费） |
| `diffs` | `std::vector<int64_t>` | 配对后的硬件时间戳差（Step 5 产出，Step 6 消费） |

#### 方法

| 方法 | 可见性 | 职责 |
|------|--------|------|
| `void enumerateDevices()` | public | 枚举设备、打印信息、检查 >=2 台、赋值 masterDev/slaveDev |
| `void configureSyncMode()` | public | 设 PRIMARY/SECONDARY 同步模式、回读验证 |
| `void resetTimestampAndSyncClock()` | public | 发送时间戳复位信号 + 开启设备时钟同步 |
| `void collectFrames(int durationSec)` | public | 启动 pipeline、回调采集、sleep、停止、取队列数据到 vector |
| `void matchFrames()` | public | 按系统时间戳配对、计算硬件时间戳差存到 diffs |
| `bool printStatistics()` | public | 排序、min/max/mean/stddev、直方图、判定 <1ms，返回 PASS/FAIL |

#### main() 调用链

```cpp
try {
    TimestampSyncChecker checker;
    checker.enumerateDevices();
    checker.configureSyncMode();
    checker.resetTimestampAndSyncClock();
    checker.collectFrames(30);
    checker.matchFrames();
    return checker.printStatistics() ? EXIT_SUCCESS : EXIT_FAILURE;
} catch (ob::Error &e) {
    std::cerr << "Orbbec SDK error: " << e.what() << std::endl;
    return EXIT_FAILURE;
}
```

### 错误处理

继续使用异常，和 Orbbec SDK 的 `ob::Error` 风格一致。各方法内部遇到错误直接 throw，main() 统一 catch。

### 信号处理

`g_running`（`std::atomic<bool>`）和 `signalHandler` 保持在类外，`collectFrames()` 内部读取 `g_running` 判断是否提前退出。

### 不变条件

- 方法调用顺序固定：`enumerateDevices → configureSyncMode → resetTimestampAndSyncClock → collectFrames → matchFrames → printStatistics`
- 不在类内部做顺序校验，调用方负责正确调用
- 类不继承、不拷贝、不移动

### 构建

不变，仍然用现有的 `CMakeLists.txt`。`fps_quality_test` 目标保持不变（待写）。

### 不涉及

- 不改变任何业务逻辑，只做结构重组
- 不修改 CMakeLists.txt
- 不新增文件