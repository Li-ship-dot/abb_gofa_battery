/**
 * @file cscan_trajectory_generator.cpp
 * @brief C-scan raster scan trajectory generator node
 *
 * Data flow:
 * /battery_poses (base_link, battery center + orientation)
 *        +
 * /joint_states (current joint states for IK seed)
 *        ↓
 * generateRasterPath() → grid points in battery local frame
 *        ↓
 * For each grid point:
 *   transformToBaseLink() → base_link coordinates
 *        ↓
 *   MoveIt2 /compute_ik → joint_target
 *        ↓
  * publish /cscan_grid_trigger (Int32) → NI system trigger
 * publish /cscan_status → progress monitoring
 *
 * State machine:
 * IDLE → PATH_PLANNING → READY → SCANNING → DWELL → SCANNING → ... → COMPLETE
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>

// Generated message header
#include <abb_vision/msg/cscan_status.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>

using namespace std::placeholders;

enum class ScanState {
    IDLE,
    PATH_PLANNING,
    READY,
    SCANNING,
    DWELL,
    COMPLETE,
    ERROR
};

struct GridPoint {
    double x_local;  // In battery local frame
    double y_local;
    double z_local;  // approach_height (constant)
    sensor_msgs::msg::JointState joint_target;  // Joint angles for this grid point
};

class CscanTrajectoryGenerator : public rclcpp::Node
{
public:
    CscanTrajectoryGenerator()
        : Node("cscan_trajectory_generator"),
          state_(ScanState::IDLE),
          ik_failure_count_(0),
          pending_ik_request_(false)
    {
        // Declare parameters
        this->declare_parameter("battery_length_m", 0.07);
        this->declare_parameter("battery_width_m", 0.025);
        this->declare_parameter("scan_resolution_m", 0.001);
        this->declare_parameter("scan_speed_m_s", 0.01);
        this->declare_parameter("approach_height_m", 0.05);
        this->declare_parameter("dwell_time_sec", 0.05);
        this->declare_parameter("ik_group_name", "manipulator");
        this->declare_parameter("ik_timeout_sec", 2.0);
        this->declare_parameter("position_threshold_m", 0.002);
        this->declare_parameter("max_ik_failures", 3);
        this->declare_parameter<std::vector<std::string>>("joint_names", {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});

        this->get_parameter("battery_length_m", battery_length_m_);
        this->get_parameter("battery_width_m", battery_width_m_);
        this->get_parameter("scan_resolution_m", scan_resolution_m_);
        this->get_parameter("scan_speed_m_s", scan_speed_m_s_);
        this->get_parameter("approach_height_m", approach_height_m_);
        this->get_parameter("dwell_time_sec", dwell_time_sec_);
        this->get_parameter("ik_group_name", ik_group_name_);
        this->get_parameter("ik_timeout_sec", ik_service_timeout_sec_);
        this->get_parameter("position_threshold_m", position_threshold_m_);
        this->get_parameter("max_ik_failures", max_ik_failures_);
        this->get_parameter("joint_names", joint_names_);

        RCLCPP_INFO(this->get_logger(), "C-scan Trajectory Generator initialized");
        RCLCPP_INFO(this->get_logger(), "  Battery dimensions: %.3f x %.3f m", battery_length_m_, battery_width_m_);
        RCLCPP_INFO(this->get_logger(), "  Scan resolution: %.3f m", scan_resolution_m_);
        RCLCPP_INFO(this->get_logger(), "  Scan speed: %.3f m/s", scan_speed_m_s_);
        RCLCPP_INFO(this->get_logger(), "  Approach height: %.3f m", approach_height_m_);
        RCLCPP_INFO(this->get_logger(), "  Dwell time: %.3f s", dwell_time_sec_);
        RCLCPP_INFO(this->get_logger(), "  IK service timeout: %.1f s, max failures: %zu",
                    ik_service_timeout_sec_, max_ik_failures_);

        // Subscriptions
        battery_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/battery_poses", 10,
            std::bind(&CscanTrajectoryGenerator::batteryPoseCallback, this, _1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&CscanTrajectoryGenerator::jointStateCallback, this, _1));

        // Publishers
        grid_trigger_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/cscan_grid_trigger", 10);

        status_pub_ = this->create_publisher<abb_vision::msg::CscanStatus>(
            "/cscan_status", 10);

        // Services
        start_scan_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/cscan/start_scan",
            std::bind(&CscanTrajectoryGenerator::startScanCallback, this, _1, _2));

        stop_scan_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/cscan/stop_scan",
            std::bind(&CscanTrajectoryGenerator::stopScanCallback, this, _1, _2));

        // IK service client
        ik_client_ = this->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");

        RCLCPP_INFO(this->get_logger(), "Waiting for /compute_ik service...");
        ik_client_->wait_for_service(std::chrono::seconds(5));

        RCLCPP_INFO(this->get_logger(), "Subscribers: /battery_poses, /joint_states");
        RCLCPP_INFO(this->get_logger(), "Publishers: /cscan_grid_trigger, /cscan_status");
        RCLCPP_INFO(this->get_logger(), "Services: /cscan/start_scan, /cscan/stop_scan");

        // Create timer for periodic callback (20 Hz - every 50ms)
        // This replaces the manual timerCallback() polling in main()
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            [this]() { this->timerCallback(); });

        // Initialize status message
        status_msg_.status = abb_vision::msg::CscanStatus::STATUS_IDLE;
        status_msg_.current_point = 0;
        status_msg_.total_points = 0;
        status_msg_.progress = 0.0;
        status_msg_.message = "Waiting for battery pose";
    }

private:
    /**
     * @brief Generate raster (zigzag) scan path in battery local frame
     */
    void generateRasterPath()
    {
        grid_points_.clear();

        // Calculate number of grid points
        int nx = static_cast<int>(std::ceil(battery_length_m_ / scan_resolution_m_));
        int ny = static_cast<int>(std::ceil(battery_width_m_ / scan_resolution_m_));

        // Ensure at least 1 point in each direction
        nx = std::max(nx, 1);
        ny = std::max(ny, 1);

        RCLCPP_INFO(this->get_logger(), "Generating raster path: %d x %d = %d points",
                    nx, ny, nx * ny);

        // Generate zigzag path
        // x ranges from -length/2 to +length/2
        // y ranges from -width/2 to +width/2
        double x_start = -battery_length_m_ / 2.0;
        double y_start = -battery_width_m_ / 2.0;
        double y_step = battery_width_m_ / static_cast<double>(std::max(ny - 1, 1));
        double x_step = battery_length_m_ / static_cast<double>(std::max(nx - 1, 1));

        for (int row = 0; row < ny; ++row) {
            double y = y_start + row * y_step;

            if (row % 2 == 0) {
                // Forward direction: x from start to end
                for (int col = 0; col < nx; ++col) {
                    double x = x_start + col * x_step;
                    GridPoint point;
                    point.x_local = x;
                    point.y_local = y;
                    point.z_local = approach_height_m_;
                    grid_points_.push_back(point);
                }
            } else {
                // Reverse direction: x from end to start
                for (int col = nx - 1; col >= 0; --col) {
                    double x = x_start + col * x_step;
                    GridPoint point;
                    point.x_local = x;
                    point.y_local = y;
                    point.z_local = approach_height_m_;
                    grid_points_.push_back(point);
                }
            }
        }

        total_points_ = grid_points_.size();
        current_point_index_ = 0;

        RCLCPP_INFO(this->get_logger(), "Generated %zu grid points", grid_points_.size());
    }

    /**
     * @brief Transform a point from battery local frame to base_link frame
     */
    geometry_msgs::msg::Pose transformLocalToBaseLink(const GridPoint& local_point)
    {
        geometry_msgs::msg::Pose base_link_pose;
        geometry_msgs::msg::Pose battery_pose_copy;

        // Copy battery pose under lock to avoid data race with batteryPoseCallback
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            battery_pose_copy = battery_pose_;
        }

        // T9.2 Fix: Validate quaternion before using to prevent UB from invalid values
        tf2::Quaternion q(
            battery_pose_copy.orientation.x,
            battery_pose_copy.orientation.y,
            battery_pose_copy.orientation.z,
            battery_pose_copy.orientation.w);
        // Check quaternion is valid (norm should be 1.0, not 0 or NaN)
        double q_norm = std::sqrt(q.x()*q.x() + q.y()*q.y() + q.z()*q.z() + q.w()*q.w());
        if (std::abs(q_norm - 1.0) > 0.01) {
            RCLCPP_ERROR(this->get_logger(), "Invalid quaternion (norm=%.4f) in battery pose, rejecting transform", q_norm);
            return geometry_msgs::msg::Pose();  // Return empty pose to signal error
        }
        tf2::Matrix3x3 rot(q);

        // Transform local coordinates to base_link
        tf2::Vector3 local_pos(local_point.x_local, local_point.y_local, local_point.z_local);
        tf2::Vector3 base_link_pos = rot * local_pos;

        base_link_pose.position.x = battery_pose_copy.position.x + base_link_pos.x();
        base_link_pose.position.y = battery_pose_copy.position.y + base_link_pos.y();
        base_link_pose.position.z = battery_pose_copy.position.z + base_link_pos.z();

        // TCP orientation:垂直向下 (roll=0, pitch=PI, yaw=battery_yaw)
        double battery_roll, battery_pitch, battery_yaw;
        rot.getRPY(battery_roll, battery_pitch, battery_yaw);

        tf2::Quaternion tcp_q;
        tcp_q.setRPY(0.0, std::acos(-1.0), battery_yaw);
        base_link_pose.orientation.x = tcp_q.x();
        base_link_pose.orientation.y = tcp_q.y();
        base_link_pose.orientation.z = tcp_q.z();
        base_link_pose.orientation.w = tcp_q.w();

        return base_link_pose;
    }

    /**
     * @brief Compute IK for a given pose
     */
    void computeIKForPoint(const geometry_msgs::msg::Pose& target_pose)
    {
        if (!ik_client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "/compute_ik service not ready");
            return;
        }

        auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
        request->ik_request.group_name = ik_group_name_;
        request->ik_request.pose_stamped.header.frame_id = "base_link";
        request->ik_request.pose_stamped.header.stamp = this->get_clock()->now();
        request->ik_request.pose_stamped.pose = target_pose;

        // Set seed state
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (has_joint_state_ && current_joint_positions_.size() >= 6) {
                request->ik_request.robot_state.joint_state.header.stamp = this->get_clock()->now();
                request->ik_request.robot_state.joint_state.name = current_joint_names_;
                request->ik_request.robot_state.joint_state.position = current_joint_positions_;
            }
        }

        // Mark that we have a pending IK request and record sent time for timeout tracking
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            pending_ik_request_ = true;
            ik_request_sent_time_ = this->get_clock()->now();
        }

        auto result_callback =
            [this, target_pose](rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture future) {
                this->handleIKResponse(future, target_pose);
            };

        ik_client_->async_send_request(request, result_callback);
    }

    /**
     * @brief Handle IK response
     */
    void handleIKResponse(rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture future,
                         const geometry_msgs::msg::Pose& target_pose)
    {
        (void)target_pose;  // unused but kept for signature consistency

        // Clear pending IK flag
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            pending_ik_request_ = false;
        }

        try {
            auto response = future.get();

            size_t point_idx;
            ScanState current_state;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                point_idx = current_point_index_;
                current_state = state_;
            }

            if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                RCLCPP_WARN(this->get_logger(), "IK failed for point %zu, error code: %d",
                            point_idx, response->error_code.val);
                // Set state to ERROR on IK failure
                bool should_update_status = false;
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    ++ik_failure_count_;
                    // Off-by-one fix: use >= so "max_ik_failures" means exactly
                    // "stop after max_ik_failures consecutive failures"
                    if (ik_failure_count_ >= max_ik_failures_) {
                        state_ = ScanState::ERROR;
                        status_msg_.status = abb_vision::msg::CscanStatus::STATUS_ERROR;
                        status_msg_.message = "IK failed after " + std::to_string(ik_failure_count_) + " consecutive failures - scan stopped";
                        RCLCPP_ERROR(this->get_logger(), "IK failure count %zu > max %zu, entering ERROR state",
                                     ik_failure_count_, max_ik_failures_);
                        should_update_status = true;
                    } else {
                        state_ = ScanState::ERROR;
                    }
                }
                // Call outside lock to avoid deadlock (updateStatus acquires state_mutex_)
                if (should_update_status) {
                    updateStatus();
                    return;
                }
                // Continue to next point despite IK failure
                moveToNextPoint();
                return;
            }

            // Reset IK failure count on success
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                ik_failure_count_ = 0;
                // If coming from ERROR state, restore to SCANNING
                if (state_ == ScanState::ERROR) {
                    state_ = ScanState::SCANNING;
                    status_msg_.status = abb_vision::msg::CscanStatus::STATUS_SCANNING;
                    status_msg_.message = "IK recovered, resuming scan";
                    RCLCPP_INFO(this->get_logger(), "IK recovered, restoring SCANNING state");
                }
            }

            const auto& solution = response->solution.joint_state;

            if (solution.position.size() < 6) {
                RCLCPP_ERROR(this->get_logger(), "IK solution has insufficient joints");
                moveToNextPoint();
                return;
            }

            // Store joint target for this grid point
            sensor_msgs::msg::JointState joint_target;
            joint_target.header.stamp = this->get_clock()->now();
            joint_target.name = joint_names_;
            joint_target.position = {
                solution.position[0], solution.position[1], solution.position[2],
                solution.position[3], solution.position[4], solution.position[5]
            };

            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                grid_points_[point_idx].joint_target = joint_target;
            }

            // Publish grid trigger (only in SCANNING state, not during path planning)
            if (current_state == ScanState::SCANNING) {
                std_msgs::msg::Int32 trigger;
                trigger.data = static_cast<int32_t>(point_idx);
                grid_trigger_pub_->publish(trigger);
                RCLCPP_INFO(this->get_logger(), "Published grid trigger: point %zu", point_idx);
            }

            // Update status
            updateStatus();

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "IK service call failed: %s", e.what());
            // Clear pending flag on exception
            bool should_update_status = false;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                pending_ik_request_ = false;
                ++ik_failure_count_;
                if (ik_failure_count_ > max_ik_failures_) {
                    state_ = ScanState::ERROR;
                    status_msg_.status = abb_vision::msg::CscanStatus::STATUS_ERROR;
                    status_msg_.message = "IK service exception - scan stopped";
                    RCLCPP_ERROR(this->get_logger(), "IK failure count %zu > max %zu, entering ERROR state",
                                 ik_failure_count_, max_ik_failures_);
                    should_update_status = true;
                } else {
                    state_ = ScanState::ERROR;
                }
            }
            // Call outside lock to avoid deadlock (updateStatus acquires state_mutex_)
            if (should_update_status) {
                updateStatus();
                return;
            }
            moveToNextPoint();
        }
    }

    /**
     * @brief Move to next grid point
     */
    void moveToNextPoint()
    {
        size_t next_idx;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            next_idx = current_point_index_ + 1;

            if (next_idx >= total_points_) {
                state_ = ScanState::COMPLETE;
                status_msg_.status = abb_vision::msg::CscanStatus::STATUS_COMPLETE;
                status_msg_.message = "Scan complete";
                status_msg_.progress = 100.0;
                ik_failure_count_ = 0;  // Reset on completion
                RCLCPP_INFO(this->get_logger(), "Scan complete!");
                return;
            }

            current_point_index_ = next_idx;
            state_ = ScanState::DWELL;
            dwell_start_time_ = this->get_clock()->now();
            status_msg_.status = abb_vision::msg::CscanStatus::STATUS_DWELL;
            status_msg_.message = "Dwelling at point " + std::to_string(next_idx);
        }
    }

    /**
     * @brief Update status message
     */
    void updateStatus()
    {
        size_t idx;
        size_t total;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            idx = current_point_index_;
            total = total_points_;
        }
        status_msg_.current_point = idx;
        status_msg_.total_points = total;
        status_msg_.progress = total > 0 ? static_cast<float>((idx * 100.0) / total) : 0.0f;
        status_pub_->publish(status_msg_);
    }

    /**
     * @brief Process next point in scan (called after dwell timer)
     */
    void processNextPoint()
    {
        size_t idx;
        size_t total;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            idx = current_point_index_;
            total = total_points_;
        }

        if (idx >= total) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_ = ScanState::COMPLETE;
            status_msg_.status = abb_vision::msg::CscanStatus::STATUS_COMPLETE;
            status_msg_.progress = 100.0;
            status_msg_.message = "Scan complete";
            return;
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_ = ScanState::SCANNING;
            status_msg_.status = abb_vision::msg::CscanStatus::STATUS_SCANNING;
        }

        // Compute IK for next point
        GridPoint& point = grid_points_[idx];
        geometry_msgs::msg::Pose base_link_pose = transformLocalToBaseLink(point);

        RCLCPP_INFO(this->get_logger(), "Moving to point %zu: [%.3f, %.3f, %.3f] in base_link",
                    idx,
                    base_link_pose.position.x,
                    base_link_pose.position.y,
                    base_link_pose.position.z);

        computeIKForPoint(base_link_pose);
    }

    /**
     * @brief Battery pose callback
     */
    void batteryPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (msg->poses.empty()) {
            return;
        }

        // Check pose freshness using header stamp (stale if older than 5 seconds)
        auto now = this->get_clock()->now();
        double age = (now - msg->header.stamp).seconds();
        if (age > 5.0) {
            RCLCPP_WARN(this->get_logger(), "Battery pose stale (%.1f s old) - ignoring", age);
            std::lock_guard<std::mutex> lock(state_mutex_);
            has_battery_pose_ = false;
            return;
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            battery_pose_ = msg->poses[0];
            has_battery_pose_ = true;
        }

        RCLCPP_INFO(this->get_logger(), "Received battery pose: [%.3f, %.3f, %.3f]",
                    msg->poses[0].position.x, msg->poses[0].position.y, msg->poses[0].position.z);

        ScanState current_state;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_state = state_;
        }

        if (current_state == ScanState::IDLE) {
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                state_ = ScanState::PATH_PLANNING;
                status_msg_.status = abb_vision::msg::CscanStatus::STATUS_PATH_PLANNING;
                status_msg_.message = "Planning scan path";
            }
            updateStatus();

            generateRasterPath();

            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                state_ = ScanState::READY;
                status_msg_.status = abb_vision::msg::CscanStatus::STATUS_READY;
                status_msg_.message = "Ready to scan";
                RCLCPP_INFO(this->get_logger(), "Path planning complete. Ready to start scan.");
            }
            updateStatus();
        }
    }

    /**
     * @brief Joint state callback
     */
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.empty()) {
            return;
        }
        std::lock_guard<std::mutex> lock(state_mutex_);
        joint_state_ = *msg;
        has_joint_state_ = true;
        current_joint_names_ = msg->name;
        current_joint_positions_ = msg->position;
    }

    /**
     * @brief Start scan service callback
     */
    void startScanCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        (void)req;

        if (state_ != ScanState::READY) {
            res->success = false;
            res->message = "Cannot start scan. Current state: " + std::to_string(static_cast<int>(state_));
            RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Starting scan...");
        current_point_index_ = 0;
        ik_failure_count_ = 0;
        state_ = ScanState::SCANNING;
        status_msg_.status = abb_vision::msg::CscanStatus::STATUS_SCANNING;
        status_msg_.message = "Scanning started";
        updateStatus();

        // Process first point
        processNextPoint();

        res->success = true;
        res->message = "Scan started";
    }

    /**
     * @brief Stop scan service callback
     */
    void stopScanCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        (void)req;

        RCLCPP_INFO(this->get_logger(), "Stopping scan...");
        state_ = ScanState::IDLE;
        status_msg_.status = abb_vision::msg::CscanStatus::STATUS_IDLE;
        status_msg_.message = "Scan stopped by user";
        current_point_index_ = 0;
        ik_failure_count_ = 0;
        pending_ik_request_ = false;
        updateStatus();

        res->success = true;
        res->message = "Scan stopped";
    }

    /**
     * @brief Check for pending IK request timeout
     */
    void checkIKTimeout()
    {
        bool pending = false;
        rclcpp::Time sent_time;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            pending = pending_ik_request_;
            if (!pending) {
                return;
            }
            sent_time = ik_request_sent_time_;
        }

        auto now = this->get_clock()->now();
        double elapsed = (now - sent_time).seconds();

        if (elapsed >= ik_service_timeout_sec_) {
            RCLCPP_ERROR(this->get_logger(), "IK service timeout after %.1f seconds", elapsed);

            std::lock_guard<std::mutex> lock(state_mutex_);
            pending_ik_request_ = false;
            ++ik_failure_count_;
            if (ik_failure_count_ >= max_ik_failures_) {
                state_ = ScanState::ERROR;
                status_msg_.status = abb_vision::msg::CscanStatus::STATUS_ERROR;
                status_msg_.message = "IK timeout after " + std::to_string(ik_failure_count_) + " failures - scan stopped";
                RCLCPP_ERROR(this->get_logger(), "IK failure count %zu > max %zu, entering ERROR state",
                             ik_failure_count_, max_ik_failures_);
                updateStatus();
                return;
            }
            state_ = ScanState::ERROR;
            // Continue to next point
            moveToNextPoint();
        }
    }

public:
    /**
     * @brief Timer callback for dwell state (public for main loop access)
     */
    void timerCallback()
    {
        ScanState current_state;
        size_t current_idx;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_state = state_;
            current_idx = current_point_index_;
        }

        if (current_state == ScanState::DWELL) {
            rclcpp::Time dwell_start;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                dwell_start = dwell_start_time_;
            }
            auto now = this->get_clock()->now();
            double elapsed = (now - dwell_start).seconds();

            if (elapsed >= dwell_time_sec_) {
                RCLCPP_INFO(this->get_logger(), "Dwell complete for point %zu", current_idx);
                processNextPoint();
            }
        }

        // Check for IK service timeout
        checkIKTimeout();
    }

    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr battery_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr grid_trigger_pub_;
    rclcpp::Publisher<abb_vision::msg::CscanStatus>::SharedPtr status_pub_;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_scan_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_scan_srv_;

    // IK service client
    rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;

    // Timer for dwell state
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double battery_length_m_;
    double battery_width_m_;
    double scan_resolution_m_;
    double scan_speed_m_s_;
    double approach_height_m_;
    double dwell_time_sec_;
    std::string ik_group_name_;
    double position_threshold_m_;
    double ik_service_timeout_sec_;
    size_t max_ik_failures_;
    std::vector<std::string> joint_names_;

    // State
    ScanState state_;
    size_t current_point_index_;
    size_t total_points_;
    rclcpp::Time dwell_start_time_;

    // IK failure tracking
    size_t ik_failure_count_;
    bool pending_ik_request_;
    rclcpp::Time ik_request_sent_time_;

    // Cached data
    geometry_msgs::msg::Pose battery_pose_;
    sensor_msgs::msg::JointState joint_state_;
    std::vector<GridPoint> grid_points_;
    abb_vision::msg::CscanStatus status_msg_;

    // Flags
    bool has_battery_pose_ = false;
    bool has_joint_state_ = false;

    // Current joint state for IK seed
    std::vector<std::string> current_joint_names_;
    std::vector<double> current_joint_positions_;

    // Mutex for thread-safe access to shared state
    std::mutex state_mutex_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CscanTrajectoryGenerator>();

    // Timer is now created in constructor (20 Hz wall timer)
    // Just spin - the timer's callback will drive DWELL state and IK timeout checks
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
