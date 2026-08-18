// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

/**
 * @file TimestampRecorder.cpp
 * @brief Record paired-frame timestamps from a synced multi-device setup into a CSV.
 *
 * Reuses the official MultiDeviceSync pipeline (config file + FramePairingManager),
 * but instead of rendering to a window it extracts the timestamps of each
 * successfully-paired (depth, color) frame group and writes them to a CSV.
 *
 * @usage TimestampRecorder --csv <path> [--duration <seconds>]
 */

#include <libobsensor/ObSensor.hpp>
#include "PipelineHolder.hpp"
#include "FramePairingManager.hpp"
#include "utils.hpp"
#include "utils_opencv.hpp"
#include "utils/cJSON.h"

#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <algorithm>
#include <fstream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <chrono>
#include <sstream>
#include <cstring>
#include <csignal>

#define MAX_DEVICE_COUNT 9
#define CONFIG_FILE "./MultiDeviceSyncConfig.json"

typedef struct DeviceConfigInfo_t {
    std::string             deviceSN;
    OBMultiDeviceSyncConfig syncConfig;
} DeviceConfigInfo;

// ---- shared with config loading (mirrors MultiDeviceSync.cpp) ----
static std::vector<std::shared_ptr<ob::Device>>       streamDevList;
static std::vector<std::shared_ptr<ob::Device>>       configDevList;
static std::vector<std::shared_ptr<DeviceConfigInfo>> deviceConfigList;
static std::vector<std::shared_ptr<PipelineHolder>>   pipelineHolderList;

static ob::Context context;

// ---- CSV output ----
static std::ofstream g_csvFile;

// ---- graceful shutdown ----
static volatile sig_atomic_t g_stopRequested = 0;
static void signalHandler(int /*signum*/) {
    g_stopRequested = 1;
}

// ---- forward declarations ----
bool                    loadConfigFile();
int                     configMultiDeviceSync();
int                     recordTimestampCsv(const std::string &csvPath, int recordSeconds);
std::shared_ptr<PipelineHolder> createPipelineHolder(std::shared_ptr<ob::Device> device, OBSensorType sensorType, int deviceIndex);
std::string             readFileContent(const char *filePath);
OBMultiDeviceSyncMode   stringToOBSyncMode(const std::string &modeString);
int                     strcmp_nocase(const char *str0, const char *str1);

// ============================================================================
//  main
// ============================================================================
int main(int argc, char *argv[]) try {
    std::signal(SIGINT, signalHandler);   // Ctrl+C 优雅退出

    int         recordSeconds = 0;
    std::string csvPath;

    for(int i = 1; i < argc; i++) {
        if(std::strcmp(argv[i], "--csv") == 0 && i + 1 < argc) {
            csvPath = argv[++i];
        }
        else if(std::strcmp(argv[i], "--duration") == 0 && i + 1 < argc) {
            recordSeconds = std::atoi(argv[++i]);
        }
    }

    if(csvPath.empty()) {
        std::cout << "Usage: TimestampRecorder --csv <path> [--duration <seconds>]\n"
                  << "  e.g. TimestampRecorder --csv ./timestamps.csv --duration 300\n"
                  << "  Records paired-frame timestamps (depth+color, all devices) to CSV.\n";
        return 1;
    }

    std::cout << "TimestampRecorder - csv=" << csvPath
              << "  duration=" << recordSeconds << "s" << std::endl;

    if(!loadConfigFile()) {
        std::cout << "load config failed" << std::endl;
        return -1;
    }
    if(deviceConfigList.empty()) {
        std::cout << "DeviceConfigList is empty. Check config file: " << CONFIG_FILE << std::endl;
        return -1;
    }

    int ret = configMultiDeviceSync();
    if(ret != 0) {
        std::cerr << "Config MultiDeviceSync failed." << std::endl;
        return ret;
    }
    std::cout << "Config MultiDeviceSync Success." << std::endl;

    return recordTimestampCsv(csvPath, recordSeconds);
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what()
              << "\ntype:" << e.getExceptionType() << std::endl;
    return -1;
}

// ============================================================================
//  config loading (copied from MultiDeviceSync.cpp)
// ============================================================================

std::string readFileContent(const char *filePath) {
    std::ostringstream oss;
    std::ifstream      file(filePath, std::fstream::in);
    if(!file.is_open()) {
        std::cerr << "Failed to open file: " << filePath << std::endl;
        return "";
    }
    oss << file.rdbuf();
    file.close();
    return oss.str();
}

bool loadConfigFile() {
    int                               deviceCount   = 0;
    std::shared_ptr<DeviceConfigInfo> devConfigInfo = nullptr;
    cJSON                            *deviceElem    = nullptr;

    auto content = readFileContent(CONFIG_FILE);
    if(content.empty()) {
        std::cerr << "load config file failed." << std::endl;
        return false;
    }

    cJSON *rootElem = cJSON_Parse(content.c_str());
    if(rootElem == nullptr) {
        const char *errMsg = cJSON_GetErrorPtr();
        std::cout << std::string(errMsg) << std::endl;
        cJSON_Delete(rootElem);
        return true;
    }

    cJSON *devicesElem = cJSON_GetObjectItem(rootElem, "devices");
    cJSON_ArrayForEach(deviceElem, devicesElem) {
        devConfigInfo = std::make_shared<DeviceConfigInfo>();
        memset(&devConfigInfo->syncConfig, 0, sizeof(devConfigInfo->syncConfig));
        devConfigInfo->syncConfig.syncMode = OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN;

        cJSON *snElem = cJSON_GetObjectItem(deviceElem, "sn");
        if(cJSON_IsString(snElem) && snElem->valuestring != nullptr) {
            devConfigInfo->deviceSN = std::string(snElem->valuestring);
        }
        cJSON *deviceConfigElem = cJSON_GetObjectItem(deviceElem, "syncConfig");
        if(cJSON_IsObject(deviceConfigElem)) {
            cJSON *numberElem = nullptr;
            cJSON *strElem    = nullptr;
            cJSON *bElem      = nullptr;
            strElem           = cJSON_GetObjectItemCaseSensitive(deviceConfigElem, "syncMode");
            if(cJSON_IsString(strElem) && strElem->valuestring != nullptr) {
                devConfigInfo->syncConfig.syncMode = stringToOBSyncMode(strElem->valuestring);
                std::cout << "config[" << (deviceCount++) << "]: SN=" << std::string(devConfigInfo->deviceSN)
                          << ", mode=" << strElem->valuestring << std::endl;
            }
            numberElem = cJSON_GetObjectItemCaseSensitive(deviceConfigElem, "depthDelayUs");
            if(cJSON_IsNumber(numberElem)) {
                devConfigInfo->syncConfig.depthDelayUs = numberElem->valueint;
            }
            numberElem = cJSON_GetObjectItemCaseSensitive(deviceConfigElem, "colorDelayUs");
            if(cJSON_IsNumber(numberElem)) {
                devConfigInfo->syncConfig.colorDelayUs = numberElem->valueint;
            }
            numberElem = cJSON_GetObjectItemCaseSensitive(deviceConfigElem, "trigger2ImageDelayUs");
            if(cJSON_IsNumber(numberElem)) {
                devConfigInfo->syncConfig.trigger2ImageDelayUs = numberElem->valueint;
            }
            numberElem = cJSON_GetObjectItemCaseSensitive(deviceConfigElem, "triggerOutDelayUs");
            if(cJSON_IsNumber(numberElem)) {
                devConfigInfo->syncConfig.triggerOutDelayUs = numberElem->valueint;
            }
            bElem = cJSON_GetObjectItemCaseSensitive(deviceConfigElem, "triggerOutEnable");
            if(cJSON_IsBool(bElem)) {
                devConfigInfo->syncConfig.triggerOutEnable = (bool)bElem->valueint;
            }
            bElem = cJSON_GetObjectItemCaseSensitive(deviceConfigElem, "framesPerTrigger");
            if(cJSON_IsNumber(bElem)) {
                devConfigInfo->syncConfig.framesPerTrigger = bElem->valueint;
            }
        }

        if(OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN != devConfigInfo->syncConfig.syncMode) {
            deviceConfigList.push_back(devConfigInfo);
        }
        else {
            std::cerr << "Invalid sync mode of deviceSN: " << devConfigInfo->deviceSN << std::endl;
        }
        devConfigInfo = nullptr;
    }
    cJSON_Delete(rootElem);
    return true;
}

OBMultiDeviceSyncMode stringToOBSyncMode(const std::string &modeString) {
    static const std::unordered_map<std::string, OBMultiDeviceSyncMode> syncModeMap = {
        { "OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN",            OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN            },
        { "OB_MULTI_DEVICE_SYNC_MODE_STANDALONE",          OB_MULTI_DEVICE_SYNC_MODE_STANDALONE          },
        { "OB_MULTI_DEVICE_SYNC_MODE_PRIMARY",             OB_MULTI_DEVICE_SYNC_MODE_PRIMARY             },
        { "OB_MULTI_DEVICE_SYNC_MODE_SECONDARY",           OB_MULTI_DEVICE_SYNC_MODE_SECONDARY           },
        { "OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED",    OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED    },
        { "OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING", OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING },
        { "OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING", OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING }
    };
    auto it = syncModeMap.find(modeString);
    if(it != syncModeMap.end()) {
        return it->second;
    }
    std::stringstream ss;
    ss << "Unrecognized sync mode: " << modeString;
    throw std::invalid_argument(ss.str());
}

int strcmp_nocase(const char *str0, const char *str1) {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
    return _strcmpi(str0, str1);
#else
    return strcasecmp(str0, str1);
#endif
}

int configMultiDeviceSync() {
    try {
        auto devList  = context.queryDeviceList();
        int  devCount = devList->deviceCount();
        for(int i = 0; i < devCount; i++) {
            configDevList.push_back(devList->getDevice(i));
        }

        if(configDevList.empty()) {
            std::cerr << "Device list is empty. please check device connection state" << std::endl;
            return -1;
        }

        for(auto config: deviceConfigList) {
            auto findItr = std::find_if(configDevList.begin(), configDevList.end(),
                                        [config](std::shared_ptr<ob::Device> device) {
                                            auto serialNumber = device->getDeviceInfo()->serialNumber();
                                            return strcmp_nocase(serialNumber, config->deviceSN.c_str()) == 0;
                                        });
            if(findItr != configDevList.end()) {
                auto device    = (*findItr);
                auto curConfig = device->getMultiDeviceSyncConfig();
                curConfig.syncMode             = config->syncConfig.syncMode;
                curConfig.depthDelayUs         = config->syncConfig.depthDelayUs;
                curConfig.colorDelayUs         = config->syncConfig.colorDelayUs;
                curConfig.trigger2ImageDelayUs = config->syncConfig.trigger2ImageDelayUs;
                curConfig.triggerOutEnable     = config->syncConfig.triggerOutEnable;
                curConfig.triggerOutDelayUs    = config->syncConfig.triggerOutDelayUs;
                curConfig.framesPerTrigger     = config->syncConfig.framesPerTrigger;
                std::cout << "-Config Device syncMode:" << curConfig.syncMode << std::endl;
                device->setMultiDeviceSyncConfig(curConfig);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return 0;
    }
    catch(ob::Error &e) {
        std::cerr << "configMultiDeviceSync failed! \n";
        std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage()
                  << "\nstatus:" << e.getStatus() << "\ntype:" << e.getExceptionType() << std::endl;
        return -1;
    }
}

std::shared_ptr<PipelineHolder> createPipelineHolder(std::shared_ptr<ob::Device> device, OBSensorType sensorType,
                                                     int deviceIndex) {
    auto pipeline = std::make_shared<ob::Pipeline>(device);
    auto holder   = std::make_shared<PipelineHolder>(pipeline, sensorType,
                                                     device->getDeviceInfo()->serialNumber(), deviceIndex);
    return holder;
}

// ============================================================================
//  stream setup (copied from MultiDeviceSync.cpp)
// ============================================================================

void startDeviceStreams(const std::vector<std::shared_ptr<ob::Device>> &devices, int startIndex) {
    // depth + color 一起开：这些 Orbbec 设备在 SECONDARY_SYNCED 模式下，只开 color 流不出帧，
    // 必须 depth 和 color 同时启用 color 才会有数据。depth 帧只占流水线，不保存。
    std::vector<OBSensorType> sensorTypes = { OB_SENSOR_DEPTH, OB_SENSOR_COLOR };
    for(auto &dev: devices) {
        for(auto sensorType: sensorTypes) {
            auto holder = createPipelineHolder(dev, sensorType, startIndex);
            pipelineHolderList.push_back(holder);
            holder->startStream();
        }
        startIndex++;
    }
}

// ============================================================================
//  main recording loop — write paired-frame timestamps to CSV
// ============================================================================

int recordTimestampCsv(const std::string &csvPath, int recordSeconds) {
    try {
        streamDevList.clear();

        auto devList  = context.queryDeviceList();
        int  devCount = devList->deviceCount();
        for(int i = 0; i < devCount; i++) {
            streamDevList.push_back(devList->getDevice(i));
        }

        if(streamDevList.empty()) {
            std::cerr << "Device list is empty. please check device connection state" << std::endl;
            return -1;
        }

        // separate primary / secondary
        std::vector<std::shared_ptr<ob::Device>> primary_devices;
        std::vector<std::shared_ptr<ob::Device>> secondary_devices;
        for(auto dev: streamDevList) {
            auto config = dev->getMultiDeviceSyncConfig();
            if(config.syncMode == OB_MULTI_DEVICE_SYNC_MODE_PRIMARY) {
                primary_devices.push_back(dev);
            }
            else {
                secondary_devices.push_back(dev);
            }
        }

        std::cout << "Secondary devices start..." << std::endl;
        startDeviceStreams(secondary_devices, 0);

        if(!primary_devices.empty()) {
            std::cout << "Primary device start..." << std::endl;
            startDeviceStreams(primary_devices, static_cast<int>(secondary_devices.size()));
        }

        // sync each device's timer with host once at startup (per FAE recommendation)
        // use per-device timerSyncWithHost() instead of context.enableDeviceClockSync(60000)
        for(auto &dev: streamDevList) {
            dev->timerSyncWithHost();
        }
        std::cout << "Per-device timer sync completed (" << streamDevList.size() << " devices)" << std::endl;

        // open CSV
        g_csvFile.open(csvPath);
        if(!g_csvFile.is_open()) {
            std::cerr << "Failed to open csv file: " << csvPath << std::endl;
            return -1;
        }
        g_csvFile << "groupId,deviceIndex,streamType,hwTimestampUs,globalTimestampUs,sysTimestampUs\n";
        g_csvFile.flush();
        std::cout << "CSV opened: " << csvPath << std::endl;

        // pairing manager — consumes all pipeline queues and pairs by timestamp
        auto framePairingManager = std::make_shared<FramePairingManager>();
        framePairingManager->setPipelineHolderList(pipelineHolderList);

        auto startTime = std::chrono::steady_clock::now();
        auto recordEnd = startTime + std::chrono::seconds(recordSeconds > 0 ? recordSeconds : 0);

        // recordSeconds <= 0 means "run until Ctrl+C" (never auto-stop)
        bool  endless  = (recordSeconds <= 0);
        int   groupId  = 0;
        int   lastSec  = -1;
        int   savedRow = 0;

        std::cout << "Recording timestamps" << (endless ? " (until Ctrl+C)" : "") << "..." << std::endl;

        while(!g_stopRequested) {
            auto now = std::chrono::steady_clock::now();
            if(!endless && now >= recordEnd) {
                break;
            }

            // 拿配对成功的一组帧（每台相机 depth+color 各一帧）
            auto framePairs = framePairingManager->getFramePairs();
            if(framePairs.empty()) {
                // 本次没有相机配对成功（队列未齐 / 超时 / 配对失败），不记
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            groupId++;
            // deviceIndex 语义：getFramePairs() 按 i=0,1,2... 逐台弹帧
            // (FramePairingManager.cpp:137 的 for(i) 循环，i 即 deviceIndex，
            //  缺 key 时 map[i] 返回 nullptr 但不跳号)，
            // 所以第 idx 对 = 相机 idx；idx 超出实际设备时 pair 为 nullptr，下面挡掉。
            for(size_t idx = 0; idx < framePairs.size(); idx++) {
                const auto &pair = framePairs[idx];
                if(!pair.first && !pair.second) continue;   // 该设备未配对成功（map 缺 key）
                // pair.first  = depth 帧, pair.second = color 帧 (FramePairingManager.cpp:152)
                for(int which = 0; which < 2; which++) {
                    auto frame = (which == 0) ? pair.first : pair.second;
                    if(!frame) continue;

                    g_csvFile << groupId << ","
                              << idx << ","
                              << (which == 0 ? "DEPTH" : "COLOR") << ","
                              << frame->timeStampUs() << ","
                              << frame->globalTimeStampUs() << ","
                              << frame->systemTimeStampUs() << "\n";
                    savedRow++;
                }
            }
            g_csvFile.flush();

            // 进度：每秒打印一行
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - startTime).count();
            if(static_cast<int>(elapsed) != lastSec) {
                lastSec = static_cast<int>(elapsed);
                std::cout << "Elapsed: " << elapsed << "s, groups: " << groupId
                          << ", rows: " << savedRow << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        std::cout << "\nRecording complete. " << groupId << " groups, "
                  << savedRow << " rows written to " << csvPath << std::endl;
        framePairingManager->printSummary();

        // write summary to CSV
        g_csvFile << "# ---- summary ----\n";
        g_csvFile << "# groups " << groupId << "\n";
        g_csvFile << "# rows " << savedRow << "\n";
        g_csvFile << "# deviceIndex,streamType,framesReceived,framesConsumed,queueRemaining,dropped\n";
        for(const auto &holder: pipelineHolderList) {
            uint64_t recv   = holder->getFramesReceived();
            uint64_t cons   = holder->getFramesConsumed();
            uint64_t queued = static_cast<uint64_t>(holder->getFrameQueueSize());
            uint64_t dropped = recv > cons + queued ? (recv - cons - queued) : 0;
            g_csvFile << "# " << holder->getDeviceIndex() << ","
                      << (holder->getSensorType() == OB_SENSOR_DEPTH ? "DEPTH" : "COLOR") << ","
                      << recv << "," << cons << "," << queued << "," << dropped << "\n";
        }
        g_csvFile.flush();

        // cleanup
        g_csvFile.close();
        for(auto &holder: pipelineHolderList) {
            holder->stopStream();
        }
        pipelineHolderList.clear();
        streamDevList.clear();
        configDevList.clear();
        deviceConfigList.clear();

        return 0;
    }
    catch(ob::Error &e) {
        std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage()
                  << "\nstatus:" << e.getStatus() << "\ntype:" << e.getExceptionType() << std::endl;
        return -1;
    }
}
