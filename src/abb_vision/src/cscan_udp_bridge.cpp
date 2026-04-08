/**
 * @file cscan_udp_bridge.cpp
 * @brief NI ↔ ROS2 UDP bridge node for C-scan ultrasonic acquisition
 *
 * Data flow:
 * ROS2 /cscan_grid_trigger (Int32) ──→ UDP ──→ NI hardware trigger
 *                                                        │
 * NI UDP ──→ UDP ──→ /ultrasonic_data (Float32MultiArray)
 *                  ──→ /ultrasonic_envelope (Float32)
 *                  ──→ /cscan_ni_status (Bool)
 *
 * NI data format (binary):
 *   Header (12 bytes, big-endian):
 *     uint32_t sample_count    // A-scan sampling points
 *     uint32_t timestamp_us    // microsecond timestamp
 *     uint32_t grid_index      // current grid point index
 *   Payload:
 *     float32[sample_count]    // normalized amplitude (-1.0 ~ 1.0)
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include <boost/asio.hpp>

#include <memory>
#include <string>
#include <vector>
#include <cstring>
#include <cstdint>
#include <algorithm>
#include <functional>
#include <mutex>
#include <random>

using boost::asio::ip::udp;

constexpr size_t HEADER_SIZE = 12;  // 3 x uint32_t
constexpr size_t MAX_SAMPLE_COUNT = 8192;
constexpr size_t MAX_PACKET_SIZE = HEADER_SIZE + MAX_SAMPLE_COUNT * sizeof(float);
constexpr uint32_t DEFAULT_TIMEOUT_MS = 1000;
constexpr size_t MOCK_SAMPLE_COUNT = 1024;
constexpr double MOCK_PUBLISH_INTERVAL_MS = 100.0;

namespace {
/// NI acquisition packet header (packed, big-endian)
struct NIAcquisitionHeader {
    uint32_t sample_count;
    uint32_t timestamp_us;
    uint32_t grid_index;
} __attribute__((packed));

/**
 * @brief Extract envelope peak (max absolute value) from A-scan samples
 */
float getEnvelopePeak(const float* samples, size_t count) {
    if (count == 0) return 0.0f;
    const float* end = samples + count;
    return *std::max_element(samples, end,
        [](float a, float b) { return std::abs(a) < std::abs(b); });
}
}  // anonymous namespace

class CscanUdpBridge : public rclcpp::Node
{
public:
    CscanUdpBridge()
        : Node("cscan_udp_bridge"),
          recv_socket_(io_context_),
          send_socket_(io_context_),
          recv_buffer_(MAX_PACKET_SIZE),
          ni_connected_(false),
          initialized_ok_(true),
          last_receive_time_(this->get_clock()->now())
    {
        // Declare parameters
        this->declare_parameter<std::string>("ni_ip", "");
        this->declare_parameter<bool>("use_mock_ni", false);
        this->declare_parameter<int>("ni_receive_port", 5000);
        this->declare_parameter<int>("ni_send_port", 5001);
        this->declare_parameter<std::string>("local_ip", "0.0.0.0");

        this->get_parameter("ni_ip", ni_ip_);
        this->get_parameter("use_mock_ni", use_mock_ni_);
        this->get_parameter("ni_receive_port", ni_receive_port_);
        this->get_parameter("ni_send_port", ni_send_port_);
        this->get_parameter("local_ip", local_ip_);

        RCLCPP_INFO(this->get_logger(), "C-scan UDP Bridge initializing...");

        // Create publishers FIRST (required before ni_ip check publishes ni_status)
        ultrasonic_data_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/ultrasonic_data", 10);
        ultrasonic_envelope_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/ultrasonic_envelope", 10);
        ni_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/cscan_ni_status", 10);
        packets_lost_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/cscan_packets_lost", 10);

        // Subscription
        grid_trigger_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/cscan_grid_trigger", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                this->onGridTrigger(msg);
            });

        // Check NI IP configuration
        initialized_ok_ = true;
        if (ni_ip_.empty()) {
            if (use_mock_ni_) {
                RCLCPP_INFO(this->get_logger(), "  NI IP: <empty> (using mock mode)");
            } else {
                RCLCPP_ERROR(this->get_logger(), "NI IP not configured and mock mode disabled. Node entering degraded mode - UDP bridge will not function.");
                RCLCPP_ERROR(this->get_logger(), "To fix: either set ni_ip parameter or use use_mock_ni:=true");
                ni_connected_ = false;
                ni_status_msg_.data = false;
                ni_status_pub_->publish(ni_status_msg_);
                initialized_ok_ = false;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "  NI IP: %s", ni_ip_.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "  NI receive port: %d", ni_receive_port_);
        RCLCPP_INFO(this->get_logger(), "  NI send port: %d", ni_send_port_);
        RCLCPP_INFO(this->get_logger(), "  Local listen address: %s", local_ip_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Mock NI mode: %s", use_mock_ni_ ? "enabled" : "disabled");

        if (use_mock_ni_) {
            // Mock mode: respond to grid trigger with mock data
            RCLCPP_INFO(this->get_logger(), "  Mock NI mode: responding to grid trigger");
            // Set mock connected status
            ni_connected_ = true;
            ni_status_msg_.data = true;
            ni_status_pub_->publish(ni_status_msg_);
        } else if (initialized_ok_) {
            // Initialize send endpoint (real hardware)
            try {
                ni_endpoint_ = udp::endpoint(
                    boost::asio::ip::address::from_string(ni_ip_),
                    static_cast<unsigned short>(ni_send_port_));
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Invalid NI IP address: %s", e.what());
                throw;
            }

            // Setup receive socket
            setupReceiveSocket();

            // Setup send socket
            send_socket_.open(udp::v4());

            // Start receive loop
            startReceive();

            // Start io_context thread
            io_thread_ = std::thread([this]() {
                io_context_.run();
            });
        }

        // Initialize NI status for non-mock mode
        if (!use_mock_ni_) {
            ni_status_msg_.data = false;
            ni_status_pub_->publish(ni_status_msg_);
        }

        // Timeout detection timer
        timeout_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(DEFAULT_TIMEOUT_MS),
            [this]() { this->checkTimeout(); });

        RCLCPP_INFO(this->get_logger(), "C-scan UDP Bridge started");
        RCLCPP_INFO(this->get_logger(), "  Subscribers: /cscan_grid_trigger");
        RCLCPP_INFO(this->get_logger(), "  Publishers: /ultrasonic_data, /ultrasonic_envelope, /cscan_ni_status");
    }

    ~CscanUdpBridge()
    {
        // Close sockets first, then stop io_context, then join thread
        boost::system::error_code ec;
        recv_socket_.close(ec);
        send_socket_.close(ec);
        io_context_.stop();
        if (io_thread_.joinable()) {
            io_thread_.join();
        }
    }

private:
    void setupReceiveSocket()
    {
        try {
            recv_socket_.open(udp::v4());
            recv_socket_.bind(udp::endpoint(
                boost::asio::ip::address::from_string(local_ip_),
                static_cast<unsigned short>(ni_receive_port_)));
            recv_socket_.set_option(boost::asio::socket_base::reuse_address(true));
            RCLCPP_INFO(this->get_logger(), "Receive socket bound to %s:%d",
                        local_ip_.c_str(), ni_receive_port_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to setup receive socket: %s", e.what());
            throw;
        }
    }

    void startReceive()
    {
        recv_socket_.async_receive_from(
            boost::asio::buffer(recv_buffer_, MAX_PACKET_SIZE),
            remote_endpoint_,
            [this](const boost::system::error_code& error, size_t bytes) {
                this->onReceive(error, bytes);
            });
    }

    /**
     * @brief Handle incoming UDP packet from NI
     */
    void onReceive(const boost::system::error_code& error, size_t bytes)
    {
        // Restart receive immediately for next packet
        startReceive();

        if (error) {
            RCLCPP_WARN(this->get_logger(), "UDP receive error: %s", error.message().c_str());
            return;
        }

        if (bytes < HEADER_SIZE) {
            RCLCPP_WARN(this->get_logger(), "Packet too small: %zu bytes (expected >= %zu)",
                        bytes, HEADER_SIZE);
            return;
        }

        if (bytes > MAX_PACKET_SIZE) {
            RCLCPP_WARN(this->get_logger(), "Packet too large: %zu bytes (max: %zu). Discarding to prevent buffer overflow.",
                        bytes, MAX_PACKET_SIZE);
            return;
        }

        // Parse header (big-endian network byte order)
        NIAcquisitionHeader header;
        std::memcpy(&header, recv_buffer_.data(), sizeof(header));
        header.sample_count = ntohl(header.sample_count);
        header.timestamp_us = ntohl(header.timestamp_us);
        header.grid_index = ntohl(header.grid_index);

        // Sequence number check for packet loss detection
        if (last_grid_index_ != 0 && header.grid_index != 0) {
            uint32_t expected = last_grid_index_ + 1;
            if (header.grid_index != expected) {
                uint32_t lost = (header.grid_index > expected) ? (header.grid_index - expected) : 0;
                packets_lost_count_ += lost;
                std_msgs::msg::Int32 lost_msg;
                lost_msg.data = packets_lost_count_;
                packets_lost_pub_->publish(lost_msg);
                RCLCPP_WARN(this->get_logger(), "Packet loss detected: expected=%u, got=%u, total_lost=%u",
                             expected, header.grid_index, packets_lost_count_);
            }
        }
        last_grid_index_ = header.grid_index;

        // Validate sample count
        if (header.sample_count == 0 || header.sample_count > MAX_SAMPLE_COUNT) {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid sample_count: %u (max: %zu)", header.sample_count, MAX_SAMPLE_COUNT);
            return;
        }

        size_t expected_size = HEADER_SIZE + header.sample_count * sizeof(float);
        if (bytes < expected_size) {
            RCLCPP_WARN(this->get_logger(),
                        "Incomplete packet: %zu bytes (expected %zu for %u samples)",
                        bytes, expected_size, header.sample_count);
            return;
        }

        // Extract amplitude samples
        const float* samples = reinterpret_cast<const float*>(recv_buffer_.data() + HEADER_SIZE);

        // Calculate envelope peak
        float envelope_peak = getEnvelopePeak(samples, header.sample_count);

        // Update connection status and publish under lock (avoids data race with checkTimeout)
        bool newly_connected = false;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_receive_time_ = this->get_clock()->now();
            if (!ni_connected_) {
                ni_connected_ = true;
                newly_connected = true;
                ni_status_msg_.data = true;
                ni_status_pub_->publish(ni_status_msg_);
            }
        }
        if (newly_connected) {
            RCLCPP_INFO(this->get_logger(), "NI connection established");
        }

        // Publish ultrasonic data (A-scan waveform)
        std_msgs::msg::Float32MultiArray ultrasonic_data;
        ultrasonic_data.layout.dim.resize(1);
        ultrasonic_data.layout.dim[0].label = "samples";
        ultrasonic_data.layout.dim[0].size = header.sample_count;
        ultrasonic_data.layout.dim[0].stride = header.sample_count;
        ultrasonic_data.data.resize(header.sample_count);
        std::memcpy(ultrasonic_data.data.data(), samples, header.sample_count * sizeof(float));
        ultrasonic_data_pub_->publish(ultrasonic_data);

        // Publish envelope (peak amplitude)
        std_msgs::msg::Float32 envelope_msg;
        envelope_msg.data = envelope_peak;
        ultrasonic_envelope_pub_->publish(envelope_msg);

        RCLCPP_DEBUG(this->get_logger(),
                     "Received A-scan: grid=%u, samples=%u, envelope=%.4f",
                     header.grid_index, header.sample_count, envelope_peak);
    }

    /**
     * @brief Send UDP trigger packet to NI on /cscan_grid_trigger
     */
    void onGridTrigger(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (!initialized_ok_) {
            RCLCPP_WARN_ONCE(this->get_logger(), "onGridTrigger called but node is in degraded mode (not initialized)");
            return;
        }

        if (use_mock_ni_) {
            // Mock mode: respond to trigger with mock frame
            publishMockFrame(msg->data);
            return;
        }

        uint32_t packet = htonl(static_cast<uint32_t>(msg->data));

        boost::system::error_code error;
        send_socket_.send_to(
            boost::asio::buffer(&packet, sizeof(packet)),
            ni_endpoint_,
            0,
            error);

        if (error) {
            RCLCPP_WARN(this->get_logger(), "Failed to send trigger: %s", error.message().c_str());
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Sent trigger: grid_index=%d", msg->data);
        }
    }

    /**
     * @brief Check for NI connection timeout
     */
    void checkTimeout()
    {
        if (!initialized_ok_) return;

        bool connected;
        rclcpp::Time last_time;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            connected = ni_connected_;
            last_time = last_receive_time_;
        }

        if (!connected) return;

        auto now = this->get_clock()->now();
        double elapsed = (now - last_time).seconds();

        if (elapsed > 1.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 5000,
                                 "No data from NI for %.1f seconds - resetting connection", elapsed);
            std::lock_guard<std::mutex> lock(state_mutex_);
            ni_connected_ = false;
            ni_status_msg_.data = false;
            ni_status_pub_->publish(ni_status_msg_);
        }
    }

    /**
     * @brief Publish mock ultrasonic data for testing without NI hardware
     * @param grid_index Grid point index from trigger
     */
    void publishMockFrame(int grid_index)
    {
        if (!use_mock_ni_) return;

        static std::mt19937 rng{std::random_device{}()};
        static std::uniform_real_distribution<float> amplitude_dist{-0.5f, 0.5f};
        static std::uniform_real_distribution<float> envelope_dist{0.1f, 0.8f};

        rclcpp::Time last_trigger;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_trigger = last_trigger_time_;
        }
        auto now = this->get_clock()->now();
        double elapsed_ms = (now - last_trigger).seconds() * 1000.0;

        // If trigger came too fast (< 100ms), generate noise to simulate real acquisition delay
        bool fast_trigger = (elapsed_ms < 100.0 && last_trigger.seconds() > 0);

        // Generate mock A-scan waveform
        std_msgs::msg::Float32MultiArray mock_data;
        mock_data.layout.dim.resize(1);
        mock_data.layout.dim[0].label = "samples";
        mock_data.layout.dim[0].size = MOCK_SAMPLE_COUNT;
        mock_data.layout.dim[0].stride = MOCK_SAMPLE_COUNT;
        mock_data.data.resize(MOCK_SAMPLE_COUNT);
        for (size_t i = 0; i < MOCK_SAMPLE_COUNT; ++i) {
            if (fast_trigger) {
                // High-frequency noise for fast triggers
                mock_data.data[i] = amplitude_dist(rng) * 2.0f;
            } else {
                mock_data.data[i] = amplitude_dist(rng);
            }
        }
        ultrasonic_data_pub_->publish(mock_data);

        // Generate mock envelope
        std_msgs::msg::Float32 mock_envelope;
        mock_envelope.data = fast_trigger ? 0.9f : envelope_dist(rng);
        ultrasonic_envelope_pub_->publish(mock_envelope);

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_trigger_time_ = now;
        }
        RCLCPP_DEBUG(this->get_logger(), "Published mock frame: grid_index=%d, fast_trigger=%s",
                     grid_index, fast_trigger ? "yes" : "no");
    }

    // ROS2 interfaces
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr grid_trigger_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ultrasonic_data_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ultrasonic_envelope_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ni_status_pub_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;

    // boost::asio UDP
    boost::asio::io_context io_context_;
    udp::socket recv_socket_;
    udp::socket send_socket_;
    udp::endpoint remote_endpoint_;
    udp::endpoint ni_endpoint_;
    std::vector<uint8_t> recv_buffer_;
    std::thread io_thread_;

    // Parameters
    std::string ni_ip_;
    bool use_mock_ni_;
    int ni_receive_port_;
    int ni_send_port_;
    std::string local_ip_;

    // State
    bool ni_connected_;
    bool initialized_ok_;
    rclcpp::Time last_receive_time_;
    rclcpp::Time last_trigger_time_;  // Last trigger timestamp for mock mode fast-trigger detection
    std_msgs::msg::Bool ni_status_msg_;
    std::mutex state_mutex_;

    // Sequence tracking for packet loss detection
    uint32_t last_grid_index_ = 0;
    uint32_t packets_lost_count_ = 0;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr packets_lost_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<CscanUdpBridge>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "Exception in CscanUdpBridge: " << e.what() << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}
