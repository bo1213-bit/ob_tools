# MultiDeviceSyncRecord Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新建 `MultiDeviceSyncRecord.cpp`，从多设备同步流中录制彩色图像和时间戳到磁盘，用于验证时间戳同步精度。

**Architecture:** 复用 `common/` 中的 `PipelineHolder`、`FramePairingManager` 模块，新增一个独立可执行文件。从 `MultiDeviceSync.cpp` 复制必要的设备配置和开流脚手架，将显示循环替换为录制循环。通过 `--record <seconds>` 命令行参数控制录制时长。

**Tech Stack:** C++11, Orbbec SDK, OpenCV, CMake

## Global Constraints

- 不修改原有 `MultiDeviceSync.cpp` 及 `common/` 目录下任何文件
- 只新建文件，在 CMakeLists.txt 中新增一个 target
- 只保存彩色帧，只保存 `deviceTimestampUs`
- 录制时长由命令行参数 `--record <seconds>` 指定
- 输出目录名包含时间戳：`./sync_capture_YYYYMMDD_HHMMSS/`

---

## File Structure

| 文件 | 操作 | 职责 |
|------|------|------|
| `MultiDeviceSync/MultiDeviceSyncRecord.cpp` | 新建 | 主程序：解析命令行、配置设备、开流、录制 |
| `CMakeLists.txt` | 修改 | 新增 `MultiDeviceSyncRecord` target |
| `config/MultiDeviceSyncConfig.json` | 不修改 | 设备同步配置文件，由用户自行编辑 |

---

### Task 1: 创建 MultiDeviceSyncRecord.cpp — 基础框架

**Files:**
- Create: `MultiDeviceSync/MultiDeviceSyncRecord.cpp`

**Interfaces:**
- Consumes: `common/PipelineHolder.hpp`, `common/FramePairingManager.hpp`, `common/utils/utils_opencv.hpp`, `common/utils/utils.hpp`, `common/utils/cJSON.h`, `<libobsensor/ObSensor.hpp>`, `<opencv2/opencv.hpp>`
- Produces: `int main(int argc, char *argv[])`, `int recordMultiDeviceSync(int recordSeconds)`, config/stream helper functions

- [ ] **Step 1: 写入文件骨架**

从 `MultiDeviceSync.cpp` 复制所有必需的 `#include`、宏定义、全局类型定义、全局变量。省略 `CVWindow` 相关的 include 和代码，改为添加 `<iomanip>` 和 `<direct.h>`（Windows mkdir）或 `<sys/stat.h>`（Linux）。

```cpp
// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#include <libobsensor/ObSensor.hpp>
#include "PipelineHolder.hpp"
#include "FramePairingManager.hpp"
#include "utils.hpp"
#include "utils_opencv.hpp"
#include "utils/cJSON.h"

#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <fstream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cstring>

#ifdef _WIN32
#include <direct.h>
#define mkdir(path) _mkdir(path)
#else
#include <sys/stat.h>
#define mkdir(path) mkdir(path, 0755)
#endif

#define MAX_DEVICE_COUNT 9
#define CONFIG_FILE "./MultiDeviceSyncConfig.json"

typedef struct DeviceConfigInfo_t {
    std::string             deviceSN;
    OBMultiDeviceSyncConfig syncConfig;
} DeviceConfigInfo;

// ---- 全局状态 ----
static std::vector<std::shared_ptr<ob::Device>>       streamDevList;
static std::vector<std::shared_ptr<ob::Device>>       configDevList;
static std::vector<std::shared_ptr<DeviceConfigInfo>> deviceConfigList;
static std::vector<std::shared_ptr<PipelineHolder>>   pipelineHolderList;

static ob::Context context;

// ---- 录制状态 ----
static std::string  g_saveDir;
static std::ofstream g_csvFile;
static int          g_frameSeq   = 0;
static bool         g_recording  = false;

// ---- 前向声明 ----
bool loadConfigFile();
int  configMultiDeviceSync();
int  recordMultiDeviceSync(int recordSeconds);
std::shared_ptr<PipelineHolder> createPipelineHolder(
    std::shared_ptr<ob::Device> device, OBSensorType sensorType, int deviceIndex);
std::string readFileContent(const char *filePath);
OBMultiDeviceSyncMode stringToOBSyncMode(const std::string &modeString);
int strcmp_nocase(const char *str0, const char *str1);
```

- [ ] **Step 2: 确认文件创建成功**

```powershell
Test-Path "D:\Data\robotPackage\ob_tools\Multi-Device-Synchronization-Example-2-main\MultiDeviceSync\MultiDeviceSyncRecord.cpp"
```

---

### Task 2: 复制并实现辅助函数

**Files:**
- Modify: `MultiDeviceSync/MultiDeviceSyncRecord.cpp`

**Interfaces:**
- Produces: `loadConfigFile()`, `configMultiDeviceSync()`, `createPipelineHolder()`, `readFileContent()`, `stringToOBSyncMode()`, `strcmp_nocase()`, `OBSyncModeToString()`

- [ ] **Step 1: 复制辅助函数**

从 `MultiDeviceSync.cpp` 原样复制以下函数到新文件：
- `readFileContent()` (第316-326行)
- `loadConfigFile()` (第328-403行)
- `stringToOBSyncMode()` (第405-423行)
- `OBSyncModeToString()` (第425-443行)
- `strcmp_nocase()` (第445-451行)
- `configMultiDeviceSync()` (第111-166行)
- `createPipelineHolder()` (第310-314行)

这些函数完全不改逻辑和签名，一字不动地复制。

- [ ] **Step 2: 编译验证**

```powershell
# 先不加 CMakeLists，仅检查语法（使用 g++ 快速语法检查）
# 实际通过 CMake 编译时再完整验证
```

---

### Task 3: 实现 frameToMat — 彩色帧转 cv::Mat

**Files:**
- Modify: `MultiDeviceSync/MultiDeviceSyncRecord.cpp`

**Interfaces:**
- Produces: `cv::Mat frameToMat(std::shared_ptr<ob::Frame> frame)` — 将彩色帧解码为 BGR Mat
- Consumes: `ob::VideoFrame::getFormat()`, `getData()`, `getWidth()`, `getHeight()`, `getDataSize()`

- [ ] **Step 1: 写入 frameToMat 函数**

从 `utils_opencv.cpp` 中的 `CVWindow::visualize()` 提取彩色帧格式转换逻辑（OB_FRAME_COLOR/COLOR_LEFT/COLOR_RIGHT case），创建独立函数：

```cpp
cv::Mat frameToMat(std::shared_ptr<ob::Frame> frame) {
    if (!frame) return cv::Mat();

    auto frameType = frame->getType();
    if (frameType != OB_FRAME_COLOR && frameType != OB_FRAME_COLOR_LEFT &&
        frameType != OB_FRAME_COLOR_RIGHT) {
        return cv::Mat();
    }

    auto videoFrame = frame->as<const ob::VideoFrame>();
    cv::Mat rstMat;

    switch (videoFrame->getFormat()) {
    case OB_FORMAT_MJPG: {
        cv::Mat rawMat(1, videoFrame->getDataSize(), CV_8UC1, videoFrame->getData());
        rstMat = cv::imdecode(rawMat, 1);
    } break;
    case OB_FORMAT_NV21: {
        cv::Mat rawMat(videoFrame->getHeight() * 3 / 2, videoFrame->getWidth(),
                       CV_8UC1, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_NV21);
    } break;
    case OB_FORMAT_YUYV:
    case OB_FORMAT_YUY2: {
        cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(),
                       CV_8UC2, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_YUY2);
    } break;
    case OB_FORMAT_BGR: {
        cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(),
                       CV_8UC3, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_BGR2RGB);
    } break;
    case OB_FORMAT_RGB: {
        cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(),
                       CV_8UC3, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_RGB2BGR);
    } break;
    case OB_FORMAT_RGBA: {
        cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(),
                       CV_8UC4, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_RGBA2BGR);
    } break;
    case OB_FORMAT_BGRA: {
        cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(),
                       CV_8UC4, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_BGRA2RGB);
    } break;
    case OB_FORMAT_UYVY: {
        cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(),
                       CV_8UC2, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_UYVY);
    } break;
    case OB_FORMAT_I420: {
        cv::Mat rawMat(videoFrame->getHeight() * 3 / 2, videoFrame->getWidth(),
                       CV_8UC1, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_I420);
    } break;
    case OB_FORMAT_Y8: {
        cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(),
                       CV_8UC1, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_GRAY2BGR);
    } break;
    case OB_FORMAT_Y16: {
        cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(),
                       CV_16UC1, videoFrame->getData());
        cv::Mat gray8;
        rawMat.convertTo(gray8, CV_8UC1, 255.0 / 65535.0);
        cv::cvtColor(gray8, rstMat, cv::COLOR_GRAY2BGR);
    } break;
    default:
        break;
    }
    return rstMat;
}
```

---

### Task 4: 实现录制逻辑 — initSaveDir + saveFrame

**Files:**
- Modify: `MultiDeviceSync/MultiDeviceSyncRecord.cpp`

**Interfaces:**
- Produces: `void initSaveDir()` — 创建带时间戳的目录并打开 CSV 写表头
- Produces: `void saveFrame(std::shared_ptr<ob::Frame> frame, int deviceIndex, const std::string& deviceSN)` — 保存单帧图像并追加 CSV 行

- [ ] **Step 1: 写入 initSaveDir 和 saveFrame**

```cpp
void initSaveDir() {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm;
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << "./sync_capture_" << std::put_time(&tm, "%Y%m%d_%H%M%S");
    g_saveDir = oss.str();
    mkdir(g_saveDir.c_str());

    g_csvFile.open(g_saveDir + "/timestamps.csv");
    if (g_csvFile.is_open()) {
        g_csvFile << "groupId,deviceIndex,deviceSN,deviceTimestampUs,fileName\n";
        g_csvFile.flush();
    }
    g_frameSeq = 0;

    std::cout << "Recording started. Saving to: " << g_saveDir << std::endl;
}

void saveFrame(std::shared_ptr<ob::Frame> frame, int deviceIndex, int groupId,
               const std::string& deviceSN) {
    if (!frame || !g_csvFile.is_open()) return;

    cv::Mat mat = frameToMat(frame);
    if (mat.empty()) return;

    g_frameSeq++;
    uint64_t ts = frame->timeStampUs();

    std::ostringstream fname;
    fname << "Device" << deviceIndex << "_frame_"
          << std::setfill('0') << std::setw(6) << g_frameSeq
          << "_" << ts << ".png";

    std::string filePath = g_saveDir + "/" + fname.str();
    cv::imwrite(filePath, mat);

    g_csvFile << groupId << ","
              << deviceIndex << ","
              << deviceSN << ","
              << ts << ","
              << fname.str() << "\n";
    g_csvFile.flush();
}
```

---

### Task 5: 实现 startDeviceStreams + 主录制函数

**Files:**
- Modify: `MultiDeviceSync/MultiDeviceSyncRecord.cpp`

**Interfaces:**
- Produces: `void startDeviceStreams(const std::vector<std::shared_ptr<ob::Device>>& devices, int startIndex)`
- Produces: `int recordMultiDeviceSync(int recordSeconds)` — 核心录制循环

- [ ] **Step 1: 复制 startDeviceStreams**

从 `MultiDeviceSync.cpp` 第168-179行复制 `startDeviceStreams`，原样不动。这里同时启动 depth 和 color 传感器，录制阶段只取 color frame。

- [ ] **Step 2: 写入 recordMultiDeviceSync**

这个函数替代原来的 `testMultiDeviceSync` + 显示循环：

```cpp
int recordMultiDeviceSync(int recordSeconds) {
    try {
        streamDevList.clear();
        auto devList  = context.queryDeviceList();
        int  devCount = devList->deviceCount();
        for (int i = 0; i < devCount; i++) {
            streamDevList.push_back(devList->getDevice(i));
        }

        if (streamDevList.empty()) {
            std::cerr << "Device list is empty." << std::endl;
            return -1;
        }

        // 分组 primary/secondary
        std::vector<std::shared_ptr<ob::Device>> primary_devices;
        std::vector<std::shared_ptr<ob::Device>> secondary_devices;
        for (auto dev : streamDevList) {
            auto config = dev->getMultiDeviceSyncConfig();
            if (config.syncMode == OB_MULTI_DEVICE_SYNC_MODE_PRIMARY) {
                primary_devices.push_back(dev);
            } else {
                secondary_devices.push_back(dev);
            }
        }

        // 先启动 secondary，再启动 primary
        std::cout << "Secondary devices start..." << std::endl;
        startDeviceStreams(secondary_devices, 0);

        if (!primary_devices.empty()) {
            std::cout << "Primary device start..." << std::endl;
            startDeviceStreams(primary_devices,
                static_cast<int>(secondary_devices.size()));
        }

        // 开启时钟同步
        context.enableDeviceClockSync(60000);

        // FramePairingManager 做帧配对
        auto framePairingManager = std::make_shared<FramePairingManager>();
        framePairingManager->setPipelineHolderList(pipelineHolderList);

        // 初始化保存目录
        initSaveDir();
        g_recording = true;

        auto startTime  = std::chrono::steady_clock::now();
        auto recordEnd  = startTime + std::chrono::seconds(recordSeconds);

        // 为每个 pipelineHolder 建立 SN 映射
        std::map<int, std::string> deviceSNMap;
        for (auto& holder : pipelineHolderList) {
            int idx = holder->getDeviceIndex();
            // 只在 color sensor 的 holder 中记录 SN
            if (holder->getSensorType() == OB_SENSOR_COLOR) {
                deviceSNMap[idx] = holder->getSerialNumber();
            }
        }

        std::cout << "Recording " << recordSeconds << " seconds..." << std::endl;

        while (g_recording) {
            auto now = std::chrono::steady_clock::now();
            if (now >= recordEnd) {
                g_recording = false;
                break;
            }

            auto framePairs = framePairingManager->getFramePairs();
            if (framePairs.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            int groupId = 0;
            for (const auto& pair : framePairs) {
                groupId++;
                // pair.first = depth, pair.second = color
                auto colorFrame = pair.second;
                if (!colorFrame) continue;

                // 从 framePairingManager 的 colorPipelineHolderList_ 获取 deviceIndex
                // 简化：利用 groupId-1 作为 deviceIndex
                int    deviceIdx = groupId - 1;
                auto   it        = deviceSNMap.find(deviceIdx);
                std::string sn   = (it != deviceSNMap.end()) ? it->second : "unknown";

                saveFrame(colorFrame, deviceIdx, groupId, sn);
            }

            // 打印进度
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                now - startTime).count();
            std::cout << "\rElapsed: " << elapsed << "s / " << recordSeconds
                      << "s, frames saved: " << g_frameSeq << std::flush;
        }

        std::cout << "\nRecording complete. " << g_frameSeq
                  << " frames saved to " << g_saveDir << std::endl;

        // 关闭 CSV
        if (g_csvFile.is_open()) {
            g_csvFile.close();
        }

        // 清理
        framePairingManager->release();
        for (auto& holder : pipelineHolderList) {
            holder->stopStream();
        }
        pipelineHolderList.clear();
        streamDevList.clear();
        configDevList.clear();
        deviceConfigList.clear();

        return 0;
    } catch (ob::Error& e) {
        std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs()
                  << "\nmessage:" << e.getMessage() << std::endl;
        return -1;
    }
}
```

---

### Task 6: 实现 main 入口 — 命令行参数解析

**Files:**
- Modify: `MultiDeviceSync/MultiDeviceSyncRecord.cpp`

**Interfaces:**
- Produces: `int main(int argc, char *argv[])`

- [ ] **Step 1: 写入 main 函数**

```cpp
int main(int argc, char *argv[]) try {
    int recordSeconds = 0;

    // 解析命令行参数
    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--record") == 0 && i + 1 < argc) {
            recordSeconds = std::atoi(argv[++i]);
        }
    }

    if (recordSeconds <= 0) {
        std::cout << "Usage: MultiDeviceSyncRecord --record <seconds>\n"
                  << "  e.g. MultiDeviceSyncRecord --record 300\n"
                  << "  Records color frames and timestamps for the specified duration.\n";
        return 1;
    }

    std::cout << "MultiDeviceSyncRecord - Recording " << recordSeconds
              << " seconds\n";

    // 加载配置
    if (!loadConfigFile()) {
        std::cout << "load config failed" << std::endl;
        return -1;
    }
    if (deviceConfigList.empty()) {
        std::cout << "DeviceConfigList is empty. Check config file: "
                  << CONFIG_FILE << std::endl;
        return -1;
    }

    // 配置设备同步模式
    int ret = configMultiDeviceSync();
    if (ret != 0) {
        std::cerr << "Config MultiDeviceSync failed." << std::endl;
        return ret;
    }
    std::cout << "Config MultiDeviceSync Success." << std::endl;

    // 开始录制
    return recordMultiDeviceSync(recordSeconds);

} catch (ob::Error& e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs()
              << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType()
              << std::endl;
    return -1;
}
```

---

### Task 7: 修改 CMakeLists.txt 添加新 target

**Files:**
- Modify: `CMakeLists.txt` (在 `MultiDeviceSyncGmslTrigger` target 之后插入)

- [ ] **Step 1: 添加 MultiDeviceSyncRecord target**

在 `CMakeLists.txt` 的第100行（`endif()` 结束 GMSL target 之后）插入：

```cmake
# ============================================================================
# Target: MultiDeviceSyncRecord (record sync frames with timestamps)
# ============================================================================
add_executable(MultiDeviceSyncRecord
    MultiDeviceSync/MultiDeviceSyncRecord.cpp
    common/PipelineHolder.cpp
    common/FramePairingManager.cpp
    common/utils/cJSON.c
    common/utils/utils.cpp
    common/utils/utils_c.c
    common/utils/utils_opencv.cpp
)

target_include_directories(MultiDeviceSyncRecord PUBLIC
    "${OB_SDK_DIR}/include/"
    "${COMMON_DIR}"
    "${COMMON_DIR}/utils"
)

target_include_directories(MultiDeviceSyncRecord PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(MultiDeviceSyncRecord PUBLIC ${OpenCV_LIBS})
target_link_libraries(MultiDeviceSyncRecord PUBLIC Threads::Threads ${CMAKE_DL_LIBS})

if(WIN32)
    target_link_libraries(MultiDeviceSyncRecord PUBLIC "${OB_SDK_DIR}/lib/OrbbecSDK.lib")
    add_custom_command(TARGET MultiDeviceSyncRecord POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${OB_SDK_DIR}/bin" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
    )
else()
    file(GLOB LIB_ORBBECSDK_FILES_REC "${OB_SDK_DIR}/lib/*.so")
    target_link_libraries(MultiDeviceSyncRecord PUBLIC ${LIB_ORBBECSDK_FILES_REC})
    add_custom_command(TARGET MultiDeviceSyncRecord POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${OB_SDK_DIR}/lib" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
    )
endif()
```

- [ ] **Step 2: 验证 CMake 语法**

```powershell
cd D:\Data\robotPackage\ob_tools\Multi-Device-Synchronization-Example-2-main\build
cmake ..
```

Expected: 无错误，MultiDeviceSyncRecord target 出现在生成列表中

---

### Task 8: 编译与运行验证

- [ ] **Step 1: 完整编译**

```powershell
cd D:\Data\robotPackage\ob_tools\Multi-Device-Synchronization-Example-2-main\build
cmake --build . --config Release --target MultiDeviceSyncRecord
```

- [ ] **Step 2: 运行测试（需连接设备）**

```powershell
cd D:\Data\robotPackage\ob_tools\Multi-Device-Synchronization-Example-2-main\build\bin
.\MultiDeviceSyncRecord --record 10
```

Expected:
- 打印 "Recording 10 seconds..."
- 10 秒后自动停止
- `./sync_capture_YYYYMMDD_HHMMSS/` 目录存在
- 目录内有 `timestamps.csv` 和若干 `Device0_frame_*.png` / `Device1_frame_*.png` 文件
- CSV 每一行有 5 个逗号分隔的字段

- [ ] **Step 3: 验证 CSV 内容**

```powershell
Get-Content .\sync_capture_*\timestamps.csv | Select-Object -First 5
```

Expected: 表头行 + 数据行，时间戳值递增

- [ ] **Step 4: 验证无参数时显示用法**

```powershell
.\MultiDeviceSyncRecord.exe
```

Expected: 打印 usage 并返回 1
