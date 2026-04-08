/**
 * @file battery_alignment_controller.cpp
 * @brief PBVS视觉伺服节点 - 电池对齐控制器
 *
 * 数据流：
 * /battery_poses (base_link下，电池中心+姿态)
 *       +
 * /egm/robot_pose (base_link下，当前TCP pose)
 *       ↓
 * computePoseError() → desired_pose = battery_pose + approach_offset
 *       ↓
 * MoveIt2 /compute_ik (GetPositionIK) → joint_target
 *       ↓
 * publish /target_joint_states (JointState) → quintic_trajectory_planner
 *
 * 状态机：
 * IDLE → COMPUTING_IK → MOVING → COMPLETE/ERROR
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <memory>
#include <string>
#include <cmath>
#include <mutex>

using namespace std::placeholders;

enum class ControllerState {
    IDLE,
    COMPUTING_IK,
    MOVING,
    COMPLETE,
    ERROR
};

class BatteryAlignmentController : public rclcpp::Node
{
public:
    BatteryAlignmentController()
        : Node("battery_alignment_controller"),
          state_(ControllerState::IDLE),
          ik_retry_count_(0),
          alignment_start_time_(rclcpp::Time(0))
    {
        // 声明参数
        this->declare_parameter("approach_height_m", 0.05);
        this->declare_parameter("alignment_timeout_sec", 2.0);
        this->declare_parameter("max_retry", 3);
        this->declare_parameter("alignment_threshold_m", 0.005);
        this->declare_parameter("alignment_threshold_rad", 0.1);  // ~5.7 degrees
        this->declare_parameter("ik_group_name", "manipulator");
        this->declare_parameter<std::vector<std::string>>("joint_names", {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});

        this->get_parameter("approach_height_m", approach_height_m_);
        this->get_parameter("alignment_timeout_sec", alignment_timeout_sec_);
        this->get_parameter("max_retry", max_retry_);
        this->get_parameter("alignment_threshold_m", alignment_threshold_m_);
        this->get_parameter("alignment_threshold_rad", alignment_threshold_rad_);
        this->get_parameter("ik_group_name", ik_group_name_);
        this->get_parameter("joint_names", joint_names_);

        RCLCPP_INFO(this->get_logger(), "Battery Alignment Controller initialized");
        RCLCPP_INFO(this->get_logger(), "  approach_height: %.3f m", approach_height_m_);
        RCLCPP_INFO(this->get_logger(), "  alignment_threshold: %.3f m", alignment_threshold_m_);
        RCLCPP_INFO(this->get_logger(), "  IK group: %s", ik_group_name_.c_str());

        // 订阅
        battery_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/battery_poses", 10,
            std::bind(&BatteryAlignmentController::batteryPoseCallback, this, _1));

        robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/egm/robot_pose", 10,
            std::bind(&BatteryAlignmentController::robotPoseCallback, this, _1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&BatteryAlignmentController::jointStateCallback, this, _1));

        // 发布
        target_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/target_joint_states", 10);

        alignment_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/alignment_status", 10);

        desired_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(
            "/desired_pose", 10);

        // IK service client (async)
        ik_client_ = this->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");

        RCLCPP_INFO(this->get_logger(), "Waiting for /compute_ik service...");
        if (!ik_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "/compute_ik service timeout after 5s - entering ERROR state");
            state_ = ControllerState::ERROR;
            std_msgs::msg::Bool status;
            status.data = false;
            alignment_status_pub_->publish(status);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Subscribers: /battery_poses, /egm/robot_pose, /joint_states");
        RCLCPP_INFO(this->get_logger(), "Publishers: /target_joint_states, /alignment_status, /desired_pose");

        // 启动超时检查定时器（每0.5秒检查一次）
        alignment_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&BatteryAlignmentController::checkAlignmentTimeout, this));
    }

private:
    /**
     * @brief 计算期望的TCP位姿
     *
     * 电池 pose: position = 电池中心(x,y,z), orientation = 电池朝向四元数
     * 目标: TCP 移动到电池正上方，保持末端垂直向下（用于超声 C-scan）
     *
     * approach_offset: TCP 相对于电池的上方偏移
     * 假设电池表面距离探头 50mm
     */
    geometry_msgs::msg::Pose computeDesiredTCPPose(const geometry_msgs::msg::Pose& battery_pose)
    {
        geometry_msgs::msg::Pose desired;

        // 电池中心位置 + 上方偏移
        desired.position.x = battery_pose.position.x;
        desired.position.y = battery_pose.position.y;
        desired.position.z = battery_pose.position.z + approach_height_m_;

        // 计算电池朝向的yaw角
        tf2::Quaternion battery_q(
            battery_pose.orientation.x,
            battery_pose.orientation.y,
            battery_pose.orientation.z,
            battery_pose.orientation.w);

        tf2::Matrix3x3 battery_rot(battery_q);
        double battery_roll, battery_pitch, battery_yaw;
        battery_rot.getRPY(battery_roll, battery_pitch, battery_yaw);

        // 末端垂直向下：roll = 0, pitch = PI (180°), yaw = 电池朝向
        // TCP在电池正上方，工具朝下
        double roll = 0.0;
        double pitch = std::acos(-1.0);  // 末端朝下 (180°)
        double yaw = battery_yaw;  // 工具朝向与电池对齐

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        desired.orientation.x = q.x();
        desired.orientation.y = q.y();
        desired.orientation.z = q.z();
        desired.orientation.w = q.w();

        return desired;
    }

    /**
     * @brief 计算两个pose之间的位置误差
     */
    double computePositionError(const geometry_msgs::msg::Pose& pose1,
                                const geometry_msgs::msg::Pose& pose2)
    {
        double dx = pose1.position.x - pose2.position.x;
        double dy = pose1.position.y - pose2.position.y;
        double dz = pose1.position.z - pose2.position.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    /**
     * @brief 计算两个姿态之间的角度误差（弧度）
     * 用四元数的点积蕴涵求夹角
     */
    double computeOrientationError(const geometry_msgs::msg::Pose& pose1,
                                   const geometry_msgs::msg::Pose& pose2)
    {
        tf2::Quaternion q1(pose1.orientation.x, pose1.orientation.y,
                          pose1.orientation.z, pose1.orientation.w);
        tf2::Quaternion q2(pose2.orientation.x, pose2.orientation.y,
                          pose2.orientation.z, pose2.orientation.w);

        // 四元数点积
        double dot = q1.x()*q2.x() + q1.y()*q2.y() + q1.z()*q2.z() + q1.w()*q2.w();

        // 夹角 = 2 * acos(|dot|)，dot可能为负所以取绝对值
        double angle = 2.0 * std::acos(std::abs(dot));
        return angle;
    }

    /**
     * @brief 检查对齐是否超时
     */
    void checkAlignmentTimeout()
    {
        ControllerState current_state;
        rclcpp::Time start_time;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_state = state_;
            if (current_state != ControllerState::COMPUTING_IK && current_state != ControllerState::MOVING) {
                return;
            }
            start_time = alignment_start_time_;
        }

        // Guard against uninitialized alignment_start_time_
        if (start_time.seconds() == 0.0) {
            return;
        }

        rclcpp::Duration elapsed = this->now() - start_time;
        if (elapsed > rclcpp::Duration::from_seconds(alignment_timeout_sec_)) {
            RCLCPP_ERROR(this->get_logger(), "Alignment timeout after %.2f s (limit: %.2f s)",
                        elapsed.seconds(), alignment_timeout_sec_);
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                pending_ik_request_ = false;  // 防止死锁：超时后必须清标志，否则后续 computeIK() 永远跳过
                state_ = ControllerState::ERROR;
            }
            std_msgs::msg::Bool status;
            status.data = false;
            alignment_status_pub_->publish(status);
        }
    }

    /**
     * @brief 电池位姿回调
     */
    void batteryPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (msg->poses.empty()) {
            return;
        }

        ControllerState current_state;
        bool robot_ready = false;
        bool joint_ready = false;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            // 取第一个电池的位姿（后续可扩展为选择逻辑）
            battery_pose_ = msg->poses[0];
            has_battery_pose_ = true;
            current_state = state_;
            robot_ready = has_robot_pose_;
            joint_ready = has_joint_state_;
        }

        RCLCPP_INFO(this->get_logger(), "Received battery pose: [%.3f, %.3f, %.3f]",
                    battery_pose_.position.x, battery_pose_.position.y, battery_pose_.position.z);

        // 如果处于IDLE状态，尝试计算IK
        if (current_state == ControllerState::IDLE) {
            if (robot_ready && joint_ready) {
                computeIK();
            }
        } else if (current_state == ControllerState::MOVING) {
            // 移动中收到新电池位姿，重新计算IK
            RCLCPP_INFO(this->get_logger(), "New battery pose received, recomputing IK");
            computeIK();
        } else if (current_state == ControllerState::ERROR) {
            // ERROR状态可通过新电池位姿恢复
            RCLCPP_INFO(this->get_logger(), "ERROR state recovered by new battery pose, returning to IDLE");
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                state_ = ControllerState::IDLE;
                ik_retry_count_ = 0;
            }
            if (robot_ready && joint_ready) {
                computeIK();
            }
        }
    }

    /**
     * @brief 机器人TCP位姿回调
     */
    void robotPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        ControllerState current_state;
        bool desired_valid = false;
        geometry_msgs::msg::Pose current_robot_pose;
        geometry_msgs::msg::Pose current_desired_pose;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            robot_pose_ = *msg;
            has_robot_pose_ = true;
            current_state = state_;
            desired_valid = has_desired_pose_;
            current_robot_pose = robot_pose_;
            current_desired_pose = desired_pose_;
        }

        // 检查是否已对齐（位置误差 AND 姿态误差同时满足才判定COMPLETE）
        if (current_state == ControllerState::MOVING && desired_valid) {
            double pos_error = computePositionError(current_robot_pose, current_desired_pose);
            double ori_error = computeOrientationError(current_robot_pose, current_desired_pose);

            bool pos_ok = pos_error < alignment_threshold_m_;
            bool ori_ok = ori_error < alignment_threshold_rad_;

            if (pos_ok && ori_ok) {
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    state_ = ControllerState::COMPLETE;
                }
                RCLCPP_INFO(this->get_logger(), "Battery alignment complete! Pos error: %.4f m, Ori error: %.4f rad",
                           pos_error, ori_error);

                std_msgs::msg::Bool status;
                status.data = true;
                alignment_status_pub_->publish(status);
            } else if (pos_ok && !ori_ok) {
                // 位置满足但姿态不满足，继续MOVING
                RCLCPP_DEBUG(this->get_logger(), "Position OK (%.4f m) but orientation not OK (%.4f rad > %.4f rad)",
                            pos_error, ori_error, alignment_threshold_rad_);
            }
        }
    }

    /**
     * @brief 关节状态回调（用于IK seed）
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
     * @brief 调用MoveIt2 IK服务（异步）
     */
    void computeIK()
    {
        if (!ik_client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "/compute_ik service not ready");
            return;
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (pending_ik_request_) {
                RCLCPP_DEBUG(this->get_logger(), "IK request already pending, skipping duplicate call");
                return;
            }
            pending_ik_request_ = true;
            state_ = ControllerState::COMPUTING_IK;
            ik_retry_count_ = 0;
            alignment_start_time_ = this->now();  // 记录对齐开始时间
        }

        callIKService();
    }

    /**
     * @brief 实际发起IK调用
     */
    void callIKService()
    {
        auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();

        // 设置IK请求
        request->ik_request.group_name = ik_group_name_;
        request->ik_request.pose_stamped.header.frame_id = "base_link";
        request->ik_request.pose_stamped.header.stamp = this->get_clock()->now();

        // 计算desired pose
        desired_pose_ = computeDesiredTCPPose(battery_pose_);
        request->ik_request.pose_stamped.pose = desired_pose_;

        // 发布desired pose用于调试
        desired_pose_pub_->publish(desired_pose_);

        // 设置seed state（当前关节位置）
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (has_joint_state_ && current_joint_positions_.size() >= 6) {
                request->ik_request.robot_state.joint_state.header.stamp = this->get_clock()->now();
                request->ik_request.robot_state.joint_state.name = current_joint_names_;
                request->ik_request.robot_state.joint_state.position = current_joint_positions_;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Calling /compute_ik for desired pose: [%.3f, %.3f, %.3f]",
                    desired_pose_.position.x, desired_pose_.position.y, desired_pose_.position.z);

        // 异步调用IK
        auto result_callback =
            [this](rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture future) {
                this->handleIKResponse(future);
            };

        ik_client_->async_send_request(request, result_callback);
    }

    /**
     * @brief 处理IK响应
     */
    void handleIKResponse(rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture future)
    {
        try {
            auto response = future.get();

            if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                RCLCPP_WARN(this->get_logger(), "IK failed with error code: %d",
                            response->error_code.val);
                int current_retry = 0;
                bool should_retry = false;
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    pending_ik_request_ = false;
                    // Only count retries if we're actually in a computing state
                    if (state_ != ControllerState::COMPUTING_IK) {
                        return;
                    }
                    ik_retry_count_++;
                    current_retry = ik_retry_count_;
                    should_retry = (current_retry < max_retry_);
                }

                if (should_retry) {
                    RCLCPP_INFO(this->get_logger(), "Retrying IK (attempt %d/%d)",
                                current_retry + 1, max_retry_);
                    callIKService();
                } else {
                    RCLCPP_ERROR(this->get_logger(), "IK failed after %d attempts", max_retry_);
                    {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        state_ = ControllerState::ERROR;
                    }
                    // T9.3 Fix: Publish error status for consistency with timeout handler
                    std_msgs::msg::Bool status;
                    status.data = false;
                    alignment_status_pub_->publish(status);
                }
                return;
            }

            // IK成功，发布关节目标
            const auto& solution = response->solution.joint_state;

            if (solution.position.size() < 6) {
                RCLCPP_ERROR(this->get_logger(), "IK solution has insufficient joints: %zu",
                            solution.position.size());
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    pending_ik_request_ = false;
                    state_ = ControllerState::ERROR;
                }
                return;
            }

            sensor_msgs::msg::JointState target;
            target.header.stamp = this->get_clock()->now();
            target.name = joint_names_;
            target.position = {
                solution.position[0], solution.position[1], solution.position[2],
                solution.position[3], solution.position[4], solution.position[5]
            };

            target_joint_pub_->publish(target);

            RCLCPP_INFO(this->get_logger(), "Published joint target: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                        target.position[0], target.position[1], target.position[2],
                        target.position[3], target.position[4], target.position[5]);

            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                pending_ik_request_ = false;
                state_ = ControllerState::MOVING;
                has_desired_pose_ = true;
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "IK service call failed: %s", e.what());
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                pending_ik_request_ = false;
                state_ = ControllerState::ERROR;
            }
        }
    }

    // 订阅
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr battery_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr robot_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    // 发布
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr target_joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alignment_status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr desired_pose_pub_;

    // IK service client
    rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;

    // Timeout timer
    rclcpp::TimerBase::SharedPtr alignment_timer_;
    rclcpp::Time alignment_start_time_;

    // 参数
    double approach_height_m_;
    double alignment_timeout_sec_;
    int max_retry_;
    double alignment_threshold_m_;
    double alignment_threshold_rad_;
    std::string ik_group_name_;
    std::vector<std::string> joint_names_;

    // 状态
    ControllerState state_;
    int ik_retry_count_;
    std::mutex state_mutex_;

    // 缓存的pose
    geometry_msgs::msg::Pose battery_pose_;
    geometry_msgs::msg::Pose robot_pose_;
    geometry_msgs::msg::Pose desired_pose_;
    sensor_msgs::msg::JointState joint_state_;

    // 标志位
    bool has_battery_pose_ = false;
    bool has_robot_pose_ = false;
    bool has_joint_state_ = false;
    bool has_desired_pose_ = false;
    bool pending_ik_request_ = false;  // 防止并发IK请求（修复数据争夺）

    // 当前关节状态
    std::vector<std::string> current_joint_names_;
    std::vector<double> current_joint_positions_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BatteryAlignmentController>());
    rclcpp::shutdown();
    return 0;
}
