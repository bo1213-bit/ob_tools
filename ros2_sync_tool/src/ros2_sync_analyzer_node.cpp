// ros2_sync_analyzer_node.cpp
// ROS2 analysis node: subscribes to multi-camera image topics, collects
// frame timestamps, then runs SyncAnalyzerCore for 3-way comparison.

#include "sync_analyzer_core.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

class SyncAnalyzerNode : public rclcpp::Node {
public:
    SyncAnalyzerNode() : Node("sync_analyzer_node") {
        // Declare parameters
        this->declare_parameter("camera_names", std::vector<std::string>{});
        this->declare_parameter("stream_types", std::vector<std::string>{"depth", "color"});
        this->declare_parameter("duration_sec", 30);
        this->declare_parameter("hw_threshold_us", 500);
        this->declare_parameter("csv_path", std::string(""));

        // Read parameters
        camera_names_  = this->get_parameter("camera_names").as_string_array();
        stream_types_  = this->get_parameter("stream_types").as_string_array();
        duration_sec_  = this->get_parameter("duration_sec").as_int();
        csv_path_      = this->get_parameter("csv_path").as_string();

        if (camera_names_.empty()) {
            RCLCPP_ERROR(this->get_logger(),
                         "camera_names parameter is empty! No topics to subscribe.");
            rclcpp::shutdown();
            return;
        }

        // Build camera name -> index map
        for (size_t i = 0; i < camera_names_.size(); i++) {
            camera_index_map_[camera_names_[i]] = static_cast<int>(i);
        }

        // Build stream type -> index map
        for (size_t i = 0; i < stream_types_.size(); i++) {
            stream_index_map_[stream_types_[i]] = static_cast<int>(i);
        }

        // Allocate storage: [camera][stream][frame]
        int camCount   = static_cast<int>(camera_names_.size());
        int streamCount = static_cast<int>(stream_types_.size());
        allFrames_.resize(camCount);
        for (int i = 0; i < camCount; i++) {
            allFrames_[i].resize(streamCount);
        }

        // Create subscriptions
        subscriptions_.reserve(camCount * streamCount);
        for (int ci = 0; ci < camCount; ci++) {
            for (int si = 0; si < streamCount; si++) {
                std::string topic = "/" + camera_names_[ci] + "/"
                                    + stream_types_[si] + "/image_raw";

                auto sub = this->create_subscription<sensor_msgs::msg::Image>(
                    topic, 10,
                    [this, ci, si](sensor_msgs::msg::Image::ConstSharedPtr msg) {
                        this->imageCallback(msg, ci, si);
                    });
                subscriptions_.push_back(sub);

                RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", topic.c_str());
            }
        }

        // Timer to stop collection after duration_sec
        RCLCPP_INFO(this->get_logger(),
                    "Collecting frames for %d seconds...", duration_sec_);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(duration_sec_),
            [this]() { this->onCollectionDone(); });
    }

private:
    void imageCallback(sensor_msgs::msg::Image::ConstSharedPtr msg,
                       int cameraIndex, int streamIndex) {
        FrameStamp fs;

        // Extract timestamp from header.stamp
        fs.hwTimestampUs  = static_cast<int64_t>(msg->header.stamp.sec) * 1000000LL
                          + static_cast<int64_t>(msg->header.stamp.nanosec) / 1000LL;
        fs.sysTimestampUs = fs.hwTimestampUs;  // Same source for now
        fs.frameNumber    = 0;                  // Will be index in the vector
        fs.deviceIndex    = cameraIndex;

        // Determine stream type from stream_types_[streamIndex]
        if (stream_types_[streamIndex] == "depth") {
            fs.streamType = StreamType::DEPTH;
        } else if (stream_types_[streamIndex] == "color") {
            fs.streamType = StreamType::COLOR;
        } else {
            fs.streamType = StreamType::DEPTH;  // default
        }

        allFrames_[cameraIndex][streamIndex].push_back(fs);
    }

    void onCollectionDone() {
        RCLCPP_INFO(this->get_logger(), "Collection finished. Running analysis...");

        // Cancel timer
        timer_->cancel();

        // Reset subscriptions to stop receiving more data
        for (auto& sub : subscriptions_) {
            sub.reset();
        }
        subscriptions_.clear();

        // Print frame counts
        for (size_t ci = 0; ci < allFrames_.size(); ci++) {
            for (size_t si = 0; si < allFrames_[ci].size(); si++) {
                RCLCPP_INFO(this->get_logger(),
                            "Camera %s / %s: %zu frames",
                            camera_names_[ci].c_str(),
                            stream_types_[si].c_str(),
                            allFrames_[ci][si].size());
            }
        }

        // Run analysis
        SyncAnalyzerCore core;
        SyncAnalyzerCore::Config cfg;
        cfg.hwThresholdUs = this->get_parameter("hw_threshold_us").as_int();
        core.run(allFrames_, cfg);

        // Output
        core.printReport();

        if (!csv_path_.empty()) {
            core.exportCSV(csv_path_);
        }

        RCLCPP_INFO(this->get_logger(), "Done. Shutting down.");
        rclcpp::shutdown();
    }

    // --- Parameters ---
    std::vector<std::string> camera_names_;
    std::vector<std::string> stream_types_;
    int                      duration_sec_;
    std::string              csv_path_;

    // --- Camera/stream name -> index maps ---
    std::map<std::string, int> camera_index_map_;
    std::map<std::string, int> stream_index_map_;

    // --- Subscriptions ---
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subscriptions_;

    // --- Collected data: [cameraIndex][streamIndex][frameIndex] ---
    std::vector<std::vector<std::vector<FrameStamp>>> allFrames_;

    // --- Timer ---
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SyncAnalyzerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}