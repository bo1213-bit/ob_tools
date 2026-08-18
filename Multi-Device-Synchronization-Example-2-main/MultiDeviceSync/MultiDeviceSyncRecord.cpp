// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

/**
 * @file MultiDeviceSyncRecord.cpp
 * @brief Record color frames and timestamps from synced multi-device setup for timestamp precision verification.
 * @usage MultiDeviceSyncRecord --record <seconds>
 */

#include <libobsensor/ObSensor.hpp>
#include "PipelineHolder.hpp"
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
#define MKDIR(path) _mkdir(path)
#else
#include <sys/stat.h>
#define MKDIR(path) mkdir(path, 0755)
#endif

#define MAX_DEVICE_COUNT 9
#define CONFIG_FILE "./MultiDeviceSyncConfig.json"

typedef struct DeviceConfigInfo_t {
    std::string             deviceSN;
    OBMultiDeviceSyncConfig syncConfig;
} DeviceConfigInfo;

// ---- shared with config loading ----
static std::vector<std::shared_ptr<ob::Device>>       streamDevList;
static std::vector<std::shared_ptr<ob::Device>>       configDevList;
static std::vector<std::shared_ptr<DeviceConfigInfo>> deviceConfigList;
static std::vector<std::shared_ptr<PipelineHolder>>   pipelineHolderList;

static ob::Context context;

// ---- recording state ----
static std::string   g_saveDir;
static std::ofstream g_csvFile;
static int           g_frameSeq  = 0;
static bool          g_recording = false;

// ---- forward declarations ----
bool                   loadConfigFile();
int                    configMultiDeviceSync();
int                    recordMultiDeviceSync(int recordSeconds);
std::shared_ptr<PipelineHolder> createPipelineHolder(std::shared_ptr<ob::Device> device, OBSensorType sensorType, int deviceIndex);
std::string            readFileContent(const char *filePath);
OBMultiDeviceSyncMode  stringToOBSyncMode(const std::string &modeString);
int                    strcmp_nocase(const char *str0, const char *str1);
cv::Mat                frameToMat(std::shared_ptr<ob::Frame> frame);
void                   initSaveDir();
void                   saveFrame(std::shared_ptr<ob::Frame> frame, int deviceIndex, int groupId, const std::string &deviceSN);

// ============================================================================
//  main
// ============================================================================
int main(int argc, char *argv[]) try {
    int recordSeconds = 0;

    for(int i = 1; i < argc; i++) {
        if(std::strcmp(argv[i], "--record") == 0 && i + 1 < argc) {
            recordSeconds = std::atoi(argv[++i]);
        }
    }

    if(recordSeconds <= 0) {
        std::cout << "Usage: MultiDeviceSyncRecord --record <seconds>\n"
                  << "  e.g. MultiDeviceSyncRecord --record 300\n"
                  << "  Records color frames and timestamps for the specified duration.\n";
        return 1;
    }

    std::cout << "MultiDeviceSyncRecord - Recording " << recordSeconds << " seconds\n";

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

    return recordMultiDeviceSync(recordSeconds);

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
        { "OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN",           OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN           },
        { "OB_MULTI_DEVICE_SYNC_MODE_STANDALONE",         OB_MULTI_DEVICE_SYNC_MODE_STANDALONE         },
        { "OB_MULTI_DEVICE_SYNC_MODE_PRIMARY",            OB_MULTI_DEVICE_SYNC_MODE_PRIMARY            },
        { "OB_MULTI_DEVICE_SYNC_MODE_SECONDARY",          OB_MULTI_DEVICE_SYNC_MODE_SECONDARY          },
        { "OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED",   OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED   },
        { "OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING",OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING },
        { "OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING",OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING }
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
//  frame → cv::Mat conversion (color only)
// ============================================================================

cv::Mat frameToMat(std::shared_ptr<ob::Frame> frame) {
    if(!frame)
        return cv::Mat();

    auto frameType = frame->getType();
    if(frameType != OB_FRAME_COLOR && frameType != OB_FRAME_COLOR_LEFT && frameType != OB_FRAME_COLOR_RIGHT) {
        return cv::Mat();
    }

    auto    videoFrame = frame->as<const ob::VideoFrame>();
    cv::Mat rstMat;

    switch(videoFrame->getFormat()) {
    case OB_FORMAT_MJPG: {
        cv::Mat rawMat(1, videoFrame->getDataSize(), CV_8UC1, videoFrame->getData());
        rstMat = cv::imdecode(rawMat, 1);
    } break;
    case OB_FORMAT_NV21: {
        cv::Mat rawMat(videoFrame->getHeight() * 3 / 2, videoFrame->getWidth(), CV_8UC1, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_NV21);
    } break;
    case OB_FORMAT_YUYV:
    case OB_FORMAT_YUY2: {
        cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(), CV_8UC2, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_YUY2);
    } break;
    case OB_FORMAT_BGR: {
        cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(), CV_8UC3, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_BGR2RGB);
    } break;
    case OB_FORMAT_RGB: {
        cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(), CV_8UC3, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_RGB2BGR);
    } break;
    case OB_FORMAT_RGBA: {
        cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(), CV_8UC4, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_RGBA2BGR);
    } break;
    case OB_FORMAT_BGRA: {
        cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(), CV_8UC4, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_BGRA2RGB);
    } break;
    case OB_FORMAT_UYVY: {
        cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(), CV_8UC2, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_UYVY);
    } break;
    case OB_FORMAT_I420: {
        cv::Mat rawMat(videoFrame->getHeight() * 3 / 2, videoFrame->getWidth(), CV_8UC1, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_I420);
    } break;
    case OB_FORMAT_Y8: {
        cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(), CV_8UC1, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_GRAY2BGR);
    } break;
    case OB_FORMAT_Y16: {
        cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(), CV_16UC1, videoFrame->getData());
        cv::Mat gray8;
        rawMat.convertTo(gray8, CV_8UC1, 255.0 / 65535.0);
        cv::cvtColor(gray8, rstMat, cv::COLOR_GRAY2BGR);
    } break;
    default:
        break;
    }
    return rstMat;
}

// ============================================================================
//  recording helpers
// ============================================================================

void initSaveDir() {
    auto        now = std::chrono::system_clock::now();
    std::time_t t   = std::chrono::system_clock::to_time_t(now);
    std::tm     tm;
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << "./sync_capture_" << std::put_time(&tm, "%Y%m%d_%H%M%S");
    g_saveDir = oss.str();
    MKDIR(g_saveDir.c_str());

    g_csvFile.open(g_saveDir + "/timestamps.csv");
    if(g_csvFile.is_open()) {
        g_csvFile << "groupId,deviceIndex,deviceSN,deviceTimestampUs,fileName\n";
        g_csvFile.flush();
    }
    g_frameSeq = 0;

    std::cout << "Recording started. Saving to: " << g_saveDir << std::endl;
}

void saveFrame(std::shared_ptr<ob::Frame> frame, int deviceIndex, int groupId, const std::string &deviceSN) {
    if(!frame || !g_csvFile.is_open())
        return;

    cv::Mat mat = frameToMat(frame);
    if(mat.empty())
        return;

    g_frameSeq++;
    uint64_t ts = frame->timeStampUs();

    // 存 JPEG（质量90）：编码和写入都比 PNG 快得多，避免磁盘成为丢帧瓶颈。
    // process_sync.py 用 PIL 读取，JPEG 完全兼容。
    std::ostringstream fname;
    fname << "Device" << deviceIndex << "_frame_" << std::setfill('0') << std::setw(6) << g_frameSeq << "_" << ts
          << ".jpg";

    std::string filePath = g_saveDir + "/" + fname.str();
    std::vector<int> jpegParams = { cv::IMWRITE_JPEG_QUALITY, 90 };
    cv::imwrite(filePath, mat, jpegParams);

    g_csvFile << groupId << "," << deviceIndex << "," << deviceSN << "," << ts << "," << fname.str() << "\n";
    g_csvFile.flush();
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
//  main recording loop
// ============================================================================

int recordMultiDeviceSync(int recordSeconds) {
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

        // enable device clock sync every 60s
        context.enableDeviceClockSync(60000);

        // build device-index → SN map (color-sensor holders only)
        std::map<int, std::string> deviceSNMap;
        for(auto &holder: pipelineHolderList) {
            if(holder->getSensorType() == OB_SENSOR_COLOR) {
                deviceSNMap[holder->getDeviceIndex()] = holder->getSerialNumber();
            }
        }

        // init save directory & csv
        initSaveDir();
        g_recording = true;

        auto startTime = std::chrono::steady_clock::now();
        auto recordEnd = startTime + std::chrono::seconds(recordSeconds);

        std::cout << "Recording " << recordSeconds << " seconds... (saving every color frame, no pairing)" << std::endl;

        int groupId    = 0;
        int lastSecond = -1;
        while(g_recording) {
            auto now = std::chrono::steady_clock::now();
            if(now >= recordEnd) {
                g_recording = false;
                break;
            }

            // 排空每台设备的 color 帧队列，全部保存（depth 帧跳过，不做配对、不丢弃）
            // 注意：每轮每台最多取 16 帧，避免"存图慢于到达"时 while 排空死循环
            for(auto &holder: pipelineHolderList) {
                if(holder->getSensorType() != OB_SENSOR_COLOR) {
                    continue;
                }
                std::string sn = deviceSNMap[holder->getDeviceIndex()];
                for(int k = 0; k < 16; k++) {
                    auto frame = holder->tryGetFrame();
                    if(!frame) {
                        break;
                    }
                    groupId++;
                    saveFrame(frame, holder->getDeviceIndex(), groupId, sn);
                }
            }

            // 进度：每秒打印一行（带换行，便于通过管道/tee查看）
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - startTime).count();
            if(static_cast<int>(elapsed) != lastSecond) {
                lastSecond = static_cast<int>(elapsed);
                int colorRecv = 0;
                for(auto &h: pipelineHolderList) {
                    if(h->getSensorType() == OB_SENSOR_COLOR) {
                        colorRecv += static_cast<int>(h->getFramesReceived());
                    }
                }
                std::cout << "Elapsed: " << elapsed << "s / " << recordSeconds
                          << "s, frames saved: " << g_frameSeq
                          << "  colorRecv(3设备合计): " << colorRecv << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        std::cout << "\nRecording complete. " << g_frameSeq << " frames saved to " << g_saveDir << std::endl;

        // cleanup
        if(g_csvFile.is_open()) {
            g_csvFile.close();
        }
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
