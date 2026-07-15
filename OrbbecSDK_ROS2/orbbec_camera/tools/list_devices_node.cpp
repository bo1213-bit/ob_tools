/*******************************************************************************
 * Copyright (c) 2023 Orbbec 3D Technology, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/
#include <rclcpp/rclcpp.hpp>

#include <iomanip>
#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <orbbec_camera/ob_camera_node_driver.h>
#include <orbbec_camera/utils.h>

namespace {
struct CliArgs {
  bool help = false;
  std::string sdk_log_level = "off";
};

void printUsage() {
  std::cout << "Usage:\n"
            << "  ros2 run orbbec_camera list_devices_node -- [options]\n\n"
            << "Options:\n"
            << "  --enable_sdk_log       Enable SDK file log at debug level under ~/.ros/Log.\n"
            << "  --sdk_log_level LEVEL  SDK file log level: debug/info/warn/error/fatal/off "
               "(default: off).\n\n"
            << "Examples:\n"
            << "  ros2 run orbbec_camera list_devices_node -- --enable_sdk_log --sdk_log_level "
               "debug\n";
}

bool parseArgs(int argc, char **argv, CliArgs &args, std::string &error) {
  for (int i = 1; i < argc; ++i) {
    const std::string current = argv[i];
    if (current == "-h" || current == "--help") {
      args.help = true;
      return true;
    }
    if (current == "--enable_sdk_log") {
      args.sdk_log_level = "debug";
      continue;
    }
    if (current.rfind("--sdk_log_level=", 0) == 0) {
      args.sdk_log_level = current.substr(std::strlen("--sdk_log_level="));
      continue;
    }
    if (current == "--sdk_log_level") {
      if (++i >= argc) {
        error = "--sdk_log_level requires a value";
        return false;
      }
      args.sdk_log_level = argv[i];
      continue;
    }
    error = "Unknown argument: " + current;
    return false;
  }

  const auto log_severity = orbbec_camera::obLogSeverityFromString(args.sdk_log_level);
  if (log_severity == OBLogSeverity::OB_LOG_SEVERITY_OFF && args.sdk_log_level != "off" &&
      args.sdk_log_level != "none") {
    error = "--sdk_log_level expects one of: debug, info, warn, error, fatal, off";
    return false;
  }

  return true;
}

std::string ipSourceTypeToString(int ip_source_type) {
  switch (ip_source_type) {
    case 0:
      return "NONE";
    case 1:
      return "LLA";
    case 2:
      return "DHCP";
    case 3:
      return "PERSISTENT";
    default:
      return std::string("UNKNOWN(") + std::to_string(ip_source_type) + ")";
  }
}

void printPresetInfo(const std::shared_ptr<ob::Device>& device) {
  auto logger = rclcpp::get_logger("list_device_node");
  try {
    auto preset_list = device->getAvailablePresetList();
    RCLCPP_INFO_STREAM(logger, "Preset count: " << preset_list->getCount());
    for (uint32_t i = 0; i < preset_list->getCount(); ++i) {
      RCLCPP_INFO_STREAM(logger, "  - " << preset_list->getName(i));
    }

    std::string key = "PresetVer";
    if (device->isExtensionInfoExist(key)) {
      std::string value = device->getExtensionInfo(key);
      RCLCPP_INFO_STREAM(logger, "Preset version: " << value);
    } else {
      RCLCPP_INFO_STREAM(logger, "Preset version: not available");
    }
  } catch (ob::Error& e) {
    RCLCPP_WARN_STREAM(logger,
                       "Failed to get preset info: " << orbbec_camera::formatObErrorWithStatus(e));
  } catch (const std::exception& e) {
    RCLCPP_WARN_STREAM(logger, "Failed to get preset info: " << e.what());
  } catch (...) {
    RCLCPP_WARN_STREAM(logger, "Failed to get preset info");
  }
}
}  // namespace

int main(int argc, char **argv) {
  CliArgs args;
  std::string parse_error;
  if (!parseArgs(argc, argv, args, parse_error)) {
    std::cerr << "Argument error: " << parse_error << std::endl;
    printUsage();
    return 1;
  }
  if (args.help) {
    printUsage();
    return 0;
  }

  try {
    const auto sdk_log_path =
        orbbec_camera::configureObSdkLoggerForTool("list_devices_node", args.sdk_log_level);
    if (!sdk_log_path.empty()) {
      RCLCPP_INFO(rclcpp::get_logger("list_device_node"), "SDK file log enabled: %s",
                  sdk_log_path.c_str());
    }
    auto context = std::make_unique<ob::Context>();
    auto list = context->queryDeviceList();
    for (size_t i = 0; i < list->deviceCount(); i++) {
      auto device_ = list->getDevice(i);
      auto device_info_ = device_->getDeviceInfo();
      if (std::string(list->getConnectionType(i)) != "Ethernet") {
        std::string serial = list->serialNumber(i);
        std::string uid = list->uid(i);
        auto usb_port = orbbec_camera::parseUsbPort(uid);
        auto connection_type = list->getConnectionType(i);
        auto firmware_version = device_info_->getFirmwareVersion();
        std::stringstream pid_hex;
        pid_hex << std::hex << std::setw(4) << std::setfill('0') << list->getPid(i);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), "name: " << list->getName(i));
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), "pid: 0x" << pid_hex.str());
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), "serial: " << serial);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"),
                           "connection: " << connection_type);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"),
                           "firmware version: " << firmware_version);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), "usb port: " << usb_port);
        printPresetInfo(device_);
        std::cout << std::endl;
      } else {
        std::string serial = list->serialNumber(i);
        auto connection_type = list->getConnectionType(i);
        auto ip_address = list->getIpAddress(i);
        std::stringstream pid_hex;
        auto firmware_version = device_info_->getFirmwareVersion();
        pid_hex << std::hex << std::setw(4) << std::setfill('0') << list->getPid(i);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), "name: " << list->getName(i));
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), "pid: 0x" << pid_hex.str());
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), "serial: " << serial);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"),
                           "connection: " << connection_type);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"),
                           "firmware version: " << firmware_version);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), "ip address: " << ip_address);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"),
                           "MAC address: " << list->getUid(i));
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"),
                           "subnet mask: " << list->getSubnetMask(i));
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"),
                           "gateway: " << list->getGateway(i));
        RCLCPP_INFO_STREAM(
            rclcpp::get_logger("list_device_node"),
            "local net interface: " << list->getLocalNetInterfaceName(static_cast<uint32_t>(i)));
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"),
                           "ip source type: " << ipSourceTypeToString(
                               static_cast<int>(list->getIpSourceType(static_cast<uint32_t>(i)))));
        printPresetInfo(device_);
        std::cout << std::endl;
      }
    }
  } catch (ob::Error& e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("list_device_node"),
                        orbbec_camera::formatObErrorWithStatus(e));
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("list_device_node"), e.what());
  } catch (...) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("list_device_node"), "unknown error");
  }
  return 0;
}
