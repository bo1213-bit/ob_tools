// main.cpp
// 入口: 编排 DataCollector → SyncAnalyzer → 输出

#include "data_collector.h"
#include "sync_analyzer.h"

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>

static DataCollector* g_collector = nullptr;

static void signalHandler(int /*signum*/) {
    std::cout << "\n[INFO] Received signal, shutting down..." << std::endl;
    if (g_collector) {
        g_collector->stop();
    }
}

static void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " [OPTIONS]\n"
              << "Options:\n"
              << "  --duration=N        Collection duration in seconds (default: 30)\n"
              << "  --fps=N             Stream frame rate (default: 30)\n"
              << "  --width=N           Stream width (default: 848)\n"
              << "  --height=N          Stream height (default: 480)\n"
              << "  --hw-threshold=N    HW timestamp pairing threshold in us (default: 500)\n"
              << "  --no-depth          Disable depth stream\n"
              << "  --no-color          Disable color stream\n"
              << "  --csv=PATH          CSV export path (required)\n"
              << "  --help              Show this help message\n"
              << std::endl;
}

static bool parseArg(const std::string& arg, const std::string& prefix, int& out) {
    if (arg.compare(0, prefix.size(), prefix) == 0) {
        out = std::atoi(arg.substr(prefix.size()).c_str());
        return true;
    }
    return false;
}

static bool parseArg(const std::string& arg, const std::string& prefix, std::string& out) {
    if (arg.compare(0, prefix.size(), prefix) == 0) {
        out = arg.substr(prefix.size());
        return true;
    }
    return false;
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);

    try {
        DataCollector::Config dcCfg;
        SyncAnalyzer::Config  saCfg;
        std::string csvPath;

        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];
            if (arg == "--help" || arg == "-h") {
                printUsage(argv[0]);
                return EXIT_SUCCESS;
            }
            if (arg == "--no-depth") { dcCfg.useDepth = false; continue; }
            if (arg == "--no-color") { dcCfg.useColor = false; continue; }
            if (parseArg(arg, "--duration=",      dcCfg.durationSec))   continue;
            if (parseArg(arg, "--fps=",           dcCfg.fps))           continue;
            if (parseArg(arg, "--width=",         dcCfg.width))         continue;
            if (parseArg(arg, "--height=",        dcCfg.height))        continue;
            if (parseArg(arg, "--hw-threshold=",  saCfg.hwThresholdUs)) continue;
            if (parseArg(arg, "--csv=",           csvPath))             continue;
            std::cerr << "Unknown option: " << arg << "\n";
            printUsage(argv[0]);
            return EXIT_FAILURE;
        }

        if (csvPath.empty()) {
            std::cerr << "Error: --csv=PATH is required\n";
            printUsage(argv[0]);
            return EXIT_FAILURE;
        }

        std::cout << "Configuration:" << std::endl;
        std::cout << "  resolution=" << dcCfg.width << "x" << dcCfg.height
                  << "  fps=" << dcCfg.fps
                  << "  duration=" << dcCfg.durationSec << "s"
                  << "  depth=" << (dcCfg.useDepth ? "on" : "off")
                  << "  color=" << (dcCfg.useColor ? "on" : "off") << std::endl;
        std::cout << "  hw-threshold=" << saCfg.hwThresholdUs << "us"
                  << "  csv=" << csvPath << std::endl;
        std::cout << std::endl;

        DataCollector collector;
        g_collector = &collector;
        collector.run(dcCfg);
        g_collector = nullptr;

        SyncAnalyzer analyzer;
        analyzer.run(collector.getFrames(), collector.getDevices(), saCfg);

        analyzer.printReport();
        analyzer.exportCSV(csvPath);

        return EXIT_SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}