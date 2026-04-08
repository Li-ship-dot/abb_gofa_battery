/**
 * @file cscan_visualizer.cpp
 * @brief C-scan real-time visualization node
 *
 * Subscribes to:
 *   /ultrasonic_data (Float32MultiArray) - A-scan waveform data from cscan_udp_bridge
 *   /cscan_grid_trigger (Int32) - Grid index trigger from cscan_trajectory_generator
 *   /cscan_status (CscanStatus) - Scan status from cscan_trajectory_generator
 *   /joint_states (JointState) - Current robot joint states
 *
 * Publishes:
 *   /cscan_live_image (Image) - Live C-scan image with progress overlay
 *   /ascan_display (Image) - Current A-scan waveform display
 *   /cscan_status (CscanStatus) - Forwarded scan status
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// Generated message header
#include <abb_vision/msg/cscan_status.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std::placeholders;

class CscanVisualizer : public rclcpp::Node
{
public:
    CscanVisualizer()
        : Node("cscan_visualizer"),
          cscan_buffer_(),
          nx_(0),
          ny_(0),
          mutex_(),
          initialized_(false),
          current_grid_index_(0),
          total_points_(0),
          progress_(0.0f)
    {
        // Declare parameters
        this->declare_parameter("battery_length_m", 0.07);
        this->declare_parameter("battery_width_m", 0.025);
        this->declare_parameter("scan_resolution_m", 0.001);
        this->declare_parameter("image_width", 800);
        this->declare_parameter("image_height", 600);

        this->get_parameter("battery_length_m", battery_length_m_);
        this->get_parameter("battery_width_m", battery_width_m_);
        this->get_parameter("scan_resolution_m", scan_resolution_m_);
        this->get_parameter("image_width", image_width_);
        this->get_parameter("image_height", image_height_);

        RCLCPP_INFO(this->get_logger(), "C-scan Visualizer initialized");
        RCLCPP_INFO(this->get_logger(), "  Battery dimensions: %.3f x %.3f m", battery_length_m_, battery_width_m_);
        RCLCPP_INFO(this->get_logger(), "  Scan resolution: %.3f m", scan_resolution_m_);
        RCLCPP_INFO(this->get_logger(), "  Display image: %d x %d", image_width_, image_height_);

        // Initialize C-scan buffer
        initializeBuffer();

        // Subscriptions
        ultrasonic_data_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/ultrasonic_data", 10,
            std::bind(&CscanVisualizer::onUltrasonicData, this, _1));

        grid_trigger_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/cscan_grid_trigger", 10,
            std::bind(&CscanVisualizer::onGridTrigger, this, _1));

        status_sub_ = this->create_subscription<abb_vision::msg::CscanStatus>(
            "/cscan_status", 10,
            std::bind(&CscanVisualizer::onStatus, this, _1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&CscanVisualizer::onJointState, this, _1));

        // Publishers
        cscan_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/cscan_live_image", 10);

        ascan_display_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/ascan_display", 10);

        status_pub_ = this->create_publisher<abb_vision::msg::CscanStatus>(
            "/cscan_status", 10);

        // Set initialized flag BEFORE the sleep to avoid DDS messages
        // processed during sleep triggering guards that see initialized_=false
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        initialized_ = true;

        RCLCPP_INFO(this->get_logger(), "Subscribers: /ultrasonic_data, /cscan_grid_trigger, /cscan_status, /joint_states");
        RCLCPP_INFO(this->get_logger(), "Publishers: /cscan_live_image, /ascan_display, /cscan_status (forwarded)");
    }

private:
    /**
     * @brief Initialize C-scan buffer based on battery dimensions and resolution
     */
    void initializeBuffer()
    {
        nx_ = static_cast<int>(std::ceil(battery_length_m_ / scan_resolution_m_));
        ny_ = static_cast<int>(std::ceil(battery_width_m_ / scan_resolution_m_));

        // Ensure at least 1 point in each direction
        nx_ = std::max(nx_, 1);
        ny_ = std::max(ny_, 1);

        cscan_buffer_ = cv::Mat::zeros(ny_, nx_, CV_16UC1);
        current_grid_index_ = 0;

        RCLCPP_INFO(this->get_logger(), "C-scan buffer initialized: %d x %d = %d points",
                    nx_, ny_, nx_ * ny_);
    }

    /**
     * @brief Convert grid index to pixel coordinates with snake-like path
     *
     * Snake path pattern:
     *   row = idx / nx
     *   if row is even: x = idx % nx (left to right)
     *   if row is odd: x = nx - 1 - (idx % nx) (right to left)
     *   y = row
     */
    void gridIndexToPixel(int idx, int& x, int& y)
    {
        int row = idx / nx_;
        int col_in_row = idx % nx_;

        if (row % 2 == 0) {
            x = col_in_row;
        } else {
            x = nx_ - 1 - col_in_row;
        }
        y = row;
    }

    /**
     * @brief Process ultrasonic A-scan data and update C-scan image
     */
    void onUltrasonicData(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // Guard: discard messages before initialization completes
        {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            if (!initialized_) {
                return;
            }
        }

        if (msg->data.empty()) {
            return;
        }

        // Calculate peak amplitude (envelope detection)
        float peak = *std::max_element(msg->data.begin(), msg->data.end());

        // Get current grid index with thread safety
        int idx;
        {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            idx = current_grid_index_;
        }

        // Convert grid index to pixel coordinates
        int x, y;
        gridIndexToPixel(idx, x, y);

        // Write to C-scan buffer and publish image (lock held throughout)
        // Bounds check and buffer write both under mutex to avoid TOCTOU race
        {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            if (idx >= nx_ * ny_) {
                return;
            }
            float clamped_peak = std::min(1.0f, std::max(0.0f, peak));
            ushort pixel_value = static_cast<ushort>(clamped_peak * 65535.0f);
            cscan_buffer_.at<ushort>(y, x) = pixel_value;
            publishCscanImage();
        }

        // Publish A-scan waveform
        publishAscanWaveform(msg);
    }

    /**
     * @brief Handle grid trigger callback - updates current grid index
     */
    void onGridTrigger(const std_msgs::msg::Int32::SharedPtr msg)
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (!initialized_) {
            return;
        }
        current_grid_index_ = msg->data;
    }

    /**
     * @brief Handle status callback
     */
    void onStatus(const abb_vision::msg::CscanStatus::SharedPtr msg)
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (!initialized_) {
            return;
        }
        current_grid_index_ = msg->current_point;
        total_points_ = msg->total_points;
        progress_ = msg->progress;

        // Map status byte to string
        switch (msg->status) {
            case abb_vision::msg::CscanStatus::STATUS_IDLE:
                status_message_ = "IDLE";
                break;
            case abb_vision::msg::CscanStatus::STATUS_PATH_PLANNING:
                status_message_ = "PATH_PLANNING";
                break;
            case abb_vision::msg::CscanStatus::STATUS_READY:
                status_message_ = "READY";
                break;
            case abb_vision::msg::CscanStatus::STATUS_SCANNING:
                status_message_ = "SCANNING";
                break;
            case abb_vision::msg::CscanStatus::STATUS_DWELL:
                status_message_ = "DWELL";
                break;
            case abb_vision::msg::CscanStatus::STATUS_COMPLETE:
                status_message_ = "COMPLETE";
                break;
            case abb_vision::msg::CscanStatus::STATUS_ERROR:
                status_message_ = "ERROR";
                break;
            default:
                status_message_ = "UNKNOWN";
                break;
        }

        // Forward status message
        status_pub_->publish(*msg);
    }

    /**
     * @brief Handle joint state callback (for robot position display)
     */
    void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.empty()) {
            return;
        }
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (!initialized_) {
            return;
        }
        joint_state_ = *msg;
    }

    /**
     * @brief Publish C-scan image with color map and overlays
     */
    void publishCscanImage()
    {
        // Guard against empty buffer (e.g., parameters not yet loaded)
        // Also check that buffer has valid data pointer (crashes if total()>0 but data==NULL)
        if (cscan_buffer_.total() == 0 || cscan_buffer_.data == NULL) {
            return;
        }

        // Convert 16UC1 to 8UC1 for display
        cv::Mat display;
        if (!cscan_buffer_.isContinuous()) {
            return;
        }
        cscan_buffer_.convertTo(display, CV_8UC1, 1.0 / 256.0);

        // Apply jet colormap (blue -> red)
        cv::Mat color;
        if (display.empty() || display.data == NULL) {
            return;
        }
        cv::applyColorMap(display, color, cv::COLORMAP_JET);

        // Get current state for overlay (mutex should be held by caller onUltrasonicData)
        int idx, total;
        float progress;
        std::string status_str;
        int current_x, current_y;
        {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            idx = current_grid_index_;
            total = total_points_;
            progress = progress_;
            status_str = status_message_;
        }
        gridIndexToPixel(idx, current_x, current_y);

        // Add white grid lines (every 10 pixels)
        for (int i = 0; i < color.rows; i += 10) {
            color.row(i) = cv::Scalar(255, 255, 255);
        }
        for (int j = 0; j < color.cols; j += 10) {
            color.col(j) = cv::Scalar(255, 255, 255);
        }

        // Mark current scan position with green dot
        cv::circle(color, cv::Point(current_x, current_y), 5, cv::Scalar(0, 255, 0), -1);

        // Draw progress text
        std::string progress_text = cv::format("Grid: %d / %d (%.1f%%)", idx, total, progress);
        cv::putText(color, progress_text,
                    cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(255, 255, 255), 1);

        // Draw status text with appropriate color
        cv::Scalar status_color;
        if (status_str == "COMPLETE") {
            status_color = cv::Scalar(0, 255, 0);  // Green
        } else if (status_str == "ERROR") {
            status_color = cv::Scalar(0, 0, 255);   // Red
        } else {
            status_color = cv::Scalar(0, 255, 255); // Yellow
        }
        cv::putText(color, status_str,
                    cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    status_color, 1);

        // Add joint state info if available (requires at least 3 joints)
        {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            if (joint_state_.position.size() >= 3) {
                std::string joint_text = cv::format("J1:%.1f J2:%.1f J3:%.1f",
                                                     joint_state_.position[0] * 180.0 / M_PI,
                                                     joint_state_.position[1] * 180.0 / M_PI,
                                                     joint_state_.position[2] * 180.0 / M_PI);
                cv::putText(color, joint_text,
                            cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                            cv::Scalar(200, 200, 200), 1);
            }
        }

        // Resize to desired display size
        cv::Mat resized;
        cv::resize(color, resized, cv::Size(image_width_, image_height_), 0, 0, cv::INTER_LINEAR);

        // Convert to ROS image message
        sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "rgb8", resized).toImageMsg();
        img_msg->header.stamp = this->get_clock()->now();
        img_msg->header.frame_id = "cscan_image";

        cscan_image_pub_->publish(*img_msg);
    }

    /**
     * @brief Publish A-scan waveform display
     */
    void publishAscanWaveform(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        int width = 800;
        int height = 200;
        cv::Mat waveform(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

        // Find max value for normalization (guard against empty data)
        // Find max value in waveform for normalization (std::max_element never returns end() on non-empty range)
        float max_val = 1.0f;
        if (!msg->data.empty()) {
            max_val = *std::max_element(msg->data.begin(), msg->data.end());
        }
        if (max_val < 1e-6) {
            max_val = 1.0f;
        }

        // Draw waveform as red line
        for (size_t i = 1; i < msg->data.size(); ++i) {
            float v0 = msg->data[i - 1] / max_val;
            float v1 = msg->data[i] / max_val;

            int x0 = static_cast<int>((i - 1) * width / msg->data.size());
            int x1 = static_cast<int>(i * width / msg->data.size());
            int y0 = static_cast<int>((1.0f - v0) * (height - 1));
            int y1 = static_cast<int>((1.0f - v1) * (height - 1));

            cv::line(waveform, cv::Point(x0, y0), cv::Point(x1, y1),
                     cv::Scalar(0, 0, 255), 1);
        }

        // Add axis lines
        cv::line(waveform, cv::Point(0, height / 2), cv::Point(width, height / 2),
                 cv::Scalar(100, 100, 100), 1);
        cv::line(waveform, cv::Point(0, 0), cv::Point(0, height),
                 cv::Scalar(100, 100, 100), 1);

        // Add title
        cv::putText(waveform, "A-scan Waveform",
                    cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 0, 0), 1);

        // Convert to ROS image message
        sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", waveform).toImageMsg();
        img_msg->header.stamp = this->get_clock()->now();
        img_msg->header.frame_id = "ascan_waveform";

        ascan_display_pub_->publish(*img_msg);
    }

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ultrasonic_data_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr grid_trigger_sub_;
    rclcpp::Subscription<abb_vision::msg::CscanStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cscan_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ascan_display_pub_;
    rclcpp::Publisher<abb_vision::msg::CscanStatus>::SharedPtr status_pub_;

    // C-scan buffer (ny rows x nx columns, CV_16UC1)
    cv::Mat cscan_buffer_;
    int nx_;
    int ny_;

    // Thread-safe state (recursive to allow nested locking)
    std::recursive_mutex mutex_;
    bool initialized_;
    int current_grid_index_;
    int total_points_;
    float progress_;
    std::string status_message_;

    // Current joint state for display
    sensor_msgs::msg::JointState joint_state_;

    // Parameters
    double battery_length_m_;
    double battery_width_m_;
    double scan_resolution_m_;
    int image_width_;
    int image_height_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CscanVisualizer>();

    rclcpp::Rate rate(30);  // 30 Hz
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}