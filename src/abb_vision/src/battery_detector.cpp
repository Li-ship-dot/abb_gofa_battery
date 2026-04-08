/**
 * @file battery_detector.cpp
 * @brief 锂电池视觉检测节点
 *
 * 算法说明：
 * 1. 预处理：颜色空间转换 (BGR -> HSV)，高斯模糊去噪
 * 2. 边缘检测：使用 Canny 算子检测边缘
 * 3. 轮廓提取：使用 OpenCV findContours 提取连通区域
 * 4. 形状筛选：通过面积、周长、矩形度筛选候选区域
 * 5. 矩形校正：对检测到的矩形进行旋正，计算边界线
 * 6. 输出：电池中心、4个角点、边缘线方程
 *
 * 适用场景：机台固定相机 (Eye-to-Hand)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>
#include <cmath>
#include <filesystem>
#include <string>
#include <sstream>

using namespace std;

class BatteryDetector : public rclcpp::Node
{
public:
    BatteryDetector() : Node("battery_detector")
    {
        // 声明参数
        this->declare_parameter("debug_mode", true);
        this->declare_parameter("min_area", 5000.0);
        this->declare_parameter("max_area", 100000.0);
        this->declare_parameter("aspect_ratio_tol", 0.3);
        this->declare_parameter("battery_length_mm", 70.0);
        this->declare_parameter("battery_width_mm", 25.0);
        this->declare_parameter("camera_info_file", "config/d415_camera_info.yaml");
        this->declare_parameter<int>("hsv_h_min", 0);
        this->declare_parameter<int>("hsv_h_max", 40);
        this->declare_parameter<int>("hsv_s_min", 100);
        this->declare_parameter<int>("hsv_s_max", 255);
        this->declare_parameter<int>("hsv_v_min", 100);
        this->declare_parameter<int>("hsv_v_max", 255);

        // HSV 自适应机制参数
        this->declare_parameter<int>("adaptive_failure_threshold", 30);   // 连续 N 帧失败后开始宽化
        this->declare_parameter<int>("adaptive_recovery_threshold", 60); // 连续 M 帧成功后开始收紧
        this->declare_parameter<int>("adaptive_widening_step", 2);      // 每次宽化 H 范围调整量
        this->declare_parameter<int>("adaptive_h_min_limit", 0);        // H 最小值扩展边界
        this->declare_parameter<int>("adaptive_h_max_limit", 50);        // H 最大值扩展边界

        this->get_parameter("debug_mode", debug_mode_);
        this->get_parameter("min_area", min_area_);
        this->get_parameter("max_area", max_area_);
        this->get_parameter("aspect_ratio_tol", aspect_ratio_tol_);
        battery_length_mm_ = this->get_parameter("battery_length_mm").as_double();
        battery_width_mm_ = this->get_parameter("battery_width_mm").as_double();

        string camera_info_file;
        this->get_parameter("camera_info_file", camera_info_file);
        this->get_parameter("hsv_h_min", hsv_h_min_);
        this->get_parameter("hsv_h_max", hsv_h_max_);
        this->get_parameter("hsv_s_min", hsv_s_min_);
        this->get_parameter("hsv_s_max", hsv_s_max_);
        this->get_parameter("hsv_v_min", hsv_v_min_);
        this->get_parameter("hsv_v_max", hsv_v_max_);

        // HSV 自适应参数获取
        this->get_parameter("adaptive_failure_threshold", adaptive_failure_threshold_);
        this->get_parameter("adaptive_recovery_threshold", adaptive_recovery_threshold_);
        this->get_parameter("adaptive_widening_step", adaptive_widening_step_);
        this->get_parameter("adaptive_h_min_limit", adaptive_h_min_limit_);
        this->get_parameter("adaptive_h_max_limit", adaptive_h_max_limit_);

        // 存储原始 HSV 值（用于恢复）
        original_hsv_h_min_ = hsv_h_min_;
        original_hsv_h_max_ = hsv_h_max_;
        current_hsv_h_min_ = hsv_h_min_;
        current_hsv_h_max_ = hsv_h_max_;

        loadCameraInfo(camera_info_file);

        expected_aspect_ratio_ = battery_length_mm_ / battery_width_mm_;

        // ========== T13: 设置参数回调支持动态修改 ==========
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&BatteryDetector::onParameterChanged, this, std::placeholders::_1));

        // 订阅图像话题
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",
            10,
            std::bind(&BatteryDetector::imageCallback, this, std::placeholders::_1));

        // 发布检测结果
        detection_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/battery_detections", 10);
        bbox_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/battery_bboxes", 10);

        // ========== T13: 添加诊断 topic ==========
        adaptive_status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/battery_detector/hsv_adaptive_status", 10);

        if (debug_mode_) {
            image_pub_ = image_transport::create_publisher(this, "/battery_detector/debug_image");
        }

        RCLCPP_INFO(this->get_logger(), "Battery Detector initialized (T13: HSV Adaptive enabled)");
        RCLCPP_INFO(this->get_logger(), "Original HSV range: H=[%d,%d], S=[%d,%d], V=[%d,%d]",
                    original_hsv_h_min_, original_hsv_h_max_, hsv_s_min_, hsv_s_max_, hsv_v_min_, hsv_v_max_);
        RCLCPP_INFO(this->get_logger(), "Adaptive limits: H_min_limit=%d, H_max_limit=%d",
                    adaptive_h_min_limit_, adaptive_h_max_limit_);
    }

private:
    // 检测结果结构体 - 必须在使用之前声明
    struct BatteryDetection {
        cv::Point2f center;
        float angle;
        float width;
        float height;
        float area;
        std::vector<cv::Point2f> corners;
        cv::Point2f scan_start_corner;
        cv::Point2f scan_direction;
    };

    // ========== T13: 参数变更回调 ==========
    rcl_interfaces::msg::SetParametersResult onParameterChanged(
        const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : parameters) {
            if (param.get_name() == "hsv_h_min") {
                hsv_h_min_ = param.as_int();
                RCLCPP_INFO(this->get_logger(), "HSV H min updated to: %d", hsv_h_min_);
            } else if (param.get_name() == "hsv_h_max") {
                hsv_h_max_ = param.as_int();
                RCLCPP_INFO(this->get_logger(), "HSV H max updated to: %d", hsv_h_max_);
            } else if (param.get_name() == "hsv_s_min") {
                hsv_s_min_ = param.as_int();
            } else if (param.get_name() == "hsv_s_max") {
                hsv_s_max_ = param.as_int();
            } else if (param.get_name() == "hsv_v_min") {
                hsv_v_min_ = param.as_int();
            } else if (param.get_name() == "hsv_v_max") {
                hsv_v_max_ = param.as_int();
            } else if (param.get_name() == "adaptive_failure_threshold") {
                adaptive_failure_threshold_ = param.as_int();
            } else if (param.get_name() == "adaptive_recovery_threshold") {
                adaptive_recovery_threshold_ = param.as_int();
            } else if (param.get_name() == "adaptive_widening_step") {
                adaptive_widening_step_ = param.as_int();
            }
        }

        return result;
    }

    /**
     * @brief 图像回调函数
     */
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
            std::vector<BatteryDetection> detections = detectBattery(frame);
            bool detection_success = !detections.empty();

            // ========== T13: HSV 自适应逻辑 ==========
            updateAdaptiveHSV(detection_success);

            publishDetections(detections, msg->header);

            if (debug_mode_) {
                visualizeResults(frame, detections);
            }

            // ========== T13: 发布诊断状态 ==========
            publishAdaptiveStatus(detection_success, detections.size());

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
        }
    }

    // ========== T13: HSV 自适应核心逻辑 ==========
    void updateAdaptiveHSV(bool detection_success)
    {
        if (detection_success) {
            consecutive_failures_ = 0;
            consecutive_successes_++;

            // 恢复策略：连续 M 帧成功后，逐步收紧回原始值
            if (consecutive_successes_ >= adaptive_recovery_threshold_) {
                bool changed = false;

                // 逐步收紧 H 范围
                if (current_hsv_h_min_ < original_hsv_h_min_) {
                    current_hsv_h_min_ = std::min(current_hsv_h_min_ + adaptive_widening_step_, original_hsv_h_min_);
                    hsv_h_min_ = current_hsv_h_min_;
                    changed = true;
                }
                if (current_hsv_h_max_ > original_hsv_h_max_) {
                    current_hsv_h_max_ = std::max(current_hsv_h_max_ - adaptive_widening_step_, original_hsv_h_max_);
                    hsv_h_max_ = current_hsv_h_max_;
                    changed = true;
                }

                if (changed) {
                    RCLCPP_INFO(this->get_logger(),
                                "[HSV Adaptive] Recovering: H=[%d,%d] (original: [%d,%d])",
                                current_hsv_h_min_, current_hsv_h_max_,
                                original_hsv_h_min_, original_hsv_h_max_);
                }
            }
        } else {
            consecutive_successes_ = 0;
            consecutive_failures_++;

            // 宽化策略：连续 N 帧失败后，自动宽化 H 范围
            if (consecutive_failures_ >= adaptive_failure_threshold_) {
                bool changed = false;

                // 宽化 H 范围，直到达到扩展边界
                if (current_hsv_h_min_ > adaptive_h_min_limit_) {
                    current_hsv_h_min_ = std::max(current_hsv_h_min_ - adaptive_widening_step_, adaptive_h_min_limit_);
                    hsv_h_min_ = current_hsv_h_min_;
                    changed = true;
                }
                if (current_hsv_h_max_ < adaptive_h_max_limit_) {
                    current_hsv_h_max_ = std::min(current_hsv_h_max_ + adaptive_widening_step_, adaptive_h_max_limit_);
                    hsv_h_max_ = current_hsv_h_max_;
                    changed = true;
                }

                if (changed) {
                    RCLCPP_WARN(this->get_logger(),
                                "[HSV Adaptive] Widening: H=[%d,%d] (limit: [%d,%d]), failures=%d",
                                current_hsv_h_min_, current_hsv_h_max_,
                                adaptive_h_min_limit_, adaptive_h_max_limit_,
                                consecutive_failures_);
                }
            }
        }
    }

    // ========== T13: 发布诊断状态 ==========
    void publishAdaptiveStatus(bool detection_success, size_t detection_count)
    {
        std_msgs::msg::String status_msg;

        std::ostringstream oss;
        oss << "adaptive_mode:"
            << (consecutive_failures_ >= adaptive_failure_threshold_ ? "WIDENING" : "NORMAL")
            << ",current_h:[" << current_hsv_h_min_ << "," << current_hsv_h_max_ << "]"
            << ",original_h:[" << original_hsv_h_min_ << "," << original_hsv_h_max_ << "]"
            << ",limits:[" << adaptive_h_min_limit_ << "," << adaptive_h_max_limit_ << "]"
            << ",at_limit:" << ((current_hsv_h_min_ <= adaptive_h_min_limit_ && current_hsv_h_max_ >= adaptive_h_max_limit_) ? "YES" : "NO")
            << ",consecutive_failures:" << consecutive_failures_
            << ",consecutive_successes:" << consecutive_successes_
            << ",last_detection:" << (detection_success ? "SUCCESS(" + std::to_string(detection_count) + ")" : "FAIL");

        status_msg.data = oss.str();
        adaptive_status_pub_->publish(status_msg);
    }

    /**
     * @brief 电池检测主函数
     */
    std::vector<BatteryDetection> detectBattery(const cv::Mat& image)
    {
        std::vector<BatteryDetection> detections;

        // 1. 预处理
        cv::Mat blurred, hsv;
        cv::GaussianBlur(image, blurred, cv::Size(5, 5), 1.5);
        cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

        // 2. 颜色分割 - 橙色范围
        cv::Mat mask;
        cv::inRange(hsv, cv::Scalar(hsv_h_min_, hsv_s_min_, hsv_v_min_), cv::Scalar(hsv_h_max_, hsv_s_max_, hsv_v_max_), mask);

        // 形态学操作
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

        // 3. 边缘检测
        cv::Mat edges;
        cv::Canny(mask, edges, 50, 150, 3);

        // 4. 轮廓提取
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 5. 形状筛选
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < min_area_ || area > max_area_) continue;

            double perimeter = cv::arcLength(contour, true);
            if (perimeter < 100) continue;

            cv::RotatedRect rect = cv::minAreaRect(contour);
            float rect_aspect = std::max(rect.size.width, rect.size.height) /
                               std::min(rect.size.width, rect.size.height);

            if (std::abs(rect_aspect - expected_aspect_ratio_) > aspect_ratio_tol_) continue;

            double rect_area = rect.size.width * rect.size.height;
            double solidity = area / rect_area;
            if (solidity < 0.5) continue;

            // 6. 提取电池信息
            BatteryDetection det;
            det.center = rect.center;
            det.angle = rect.angle;
            det.width = rect.size.width;
            det.height = rect.size.height;
            det.area = area;

            cv::Point2f vertices[4];
            rect.points(vertices);
            for (int i = 0; i < 4; i++) {
                det.corners.push_back(vertices[i]);
            }

            // 找到最左边的角作为扫描起点
            int left_idx = 0;
            for (int i = 1; i < 4; i++) {
                if (vertices[i].x < vertices[left_idx].x) left_idx = i;
            }
            det.scan_start_corner = vertices[left_idx];

            cv::Point2f dir = (vertices[(left_idx + 1) % 4] - vertices[left_idx]) +
                             (vertices[(left_idx + 3) % 4] - vertices[left_idx]);
            double norm = std::sqrt(dir.x * dir.x + dir.y * dir.y);
            if (norm < 1e-6) {
                RCLCPP_WARN(this->get_logger(), "Degenerate contour at scan start corner (zero edge vectors), skipping detection");
                continue;
            }
            det.scan_direction = cv::Point2f(dir.x / norm, dir.y / norm);

            detections.push_back(det);
        }

        return detections;
    }

    /**
     * @brief 发布检测结果
     */
    void publishDetections(const std::vector<BatteryDetection>& detections,
                          const std_msgs::msg::Header& header)
    {
        geometry_msgs::msg::PoseArray centers_msg;
        centers_msg.header = header;
        geometry_msgs::msg::PoseArray bboxes_msg;
        bboxes_msg.header = header;

        for (const auto& det : detections) {
            geometry_msgs::msg::Pose center_pose;
            center_pose.position.x = det.center.x;
            center_pose.position.y = det.center.y;
            center_pose.position.z = det.angle;
            centers_msg.poses.push_back(center_pose);

            for (const auto& corner : det.corners) {
                geometry_msgs::msg::Pose bbox_pose;
                bbox_pose.position.x = corner.x;
                bbox_pose.position.y = corner.y;
                bbox_pose.position.z = det.angle;
                bboxes_msg.poses.push_back(bbox_pose);
            }
        }

        size_t total_corners = bboxes_msg.poses.size();
        size_t R = total_corners % 4;
        RCLCPP_INFO(this->get_logger(),
                    "Detected %zu batteries, publishing %zu=%zu*4 corner poses (+ %zu leftover)",
                    detections.size(), total_corners, detections.size(), R);

        detection_pub_->publish(centers_msg);
        bbox_pub_->publish(bboxes_msg);
    }

    /**
     * @brief 可视化结果
     */
    void visualizeResults(const cv::Mat& image, const std::vector<BatteryDetection>& detections)
    {
        cv::Mat vis_image = image.clone();

        for (const auto& det : detections) {
            cv::Point2f vertices[4];
            cv::RotatedRect(det.center, cv::Size(det.width, det.height), det.angle).points(vertices);

            for (int i = 0; i < 4; i++) {
                cv::line(vis_image, vertices[i], vertices[(i+1)%4], cv::Scalar(0, 255, 0), 2);
            }

            cv::circle(vis_image, det.center, 5, cv::Scalar(0, 0, 255), -1);
            cv::circle(vis_image, det.scan_start_corner, 8, cv::Scalar(255, 0, 0), -1);

            cv::Point end_point(det.scan_start_corner.x + det.scan_direction.x * 50,
                               det.scan_start_corner.y + det.scan_direction.y * 50);
            cv::arrowedLine(vis_image, det.scan_start_corner, end_point, cv::Scalar(255, 255, 0), 2);

            std::string info = cv::format("Area: %.0f, Angle: %.1f", det.area, det.angle);
            cv::putText(vis_image, info, cv::Point(det.center.x + 30, det.center.y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
        }

        sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", vis_image).toImageMsg();
        image_pub_.publish(img_msg);
    }

    void loadCameraInfo(const string& filename)
    {
        vector<string> paths = {
            filename,
            filesystem::path(filename).is_absolute() ? filename : "",
        };

        string resolved_path;
        for (const auto& p : paths) {
            if (p.empty()) continue;
            if (filesystem::exists(p)) {
                resolved_path = p;
                break;
            }
        }

        if (resolved_path.empty()) {
            resolved_path = filename;
        }

        cv::FileStorage fs;
        try {
            fs.open(resolved_path, cv::FileStorage::READ);
        } catch (const cv::Exception& e) {
            RCLCPP_WARN(this->get_logger(),
                        "Could not parse camera info file '%s' (%s), using defaults",
                        resolved_path.c_str(), e.what());
            camera_matrix_ = (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);
            dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
            return;
        }
        if (!fs.isOpened()) {
            RCLCPP_WARN(this->get_logger(), "Could not open camera info file '%s', using defaults",
                        resolved_path.c_str());
            camera_matrix_ = (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);
            dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
            return;
        }

        cv::FileNode cm = fs["camera_matrix"];
        if (!cm.empty() && !cm.isNone()) {
            cm["data"] >> camera_matrix_;
        } else {
            RCLCPP_WARN(this->get_logger(), "camera_matrix not found in YAML, using D415 factory default");
            camera_matrix_ = (cv::Mat_<double>(3, 3) << 385.5, 0.0, 319.5, 0.0, 385.5, 239.5, 0.0, 0.0, 1.0);
        }

        cv::FileNode dc = fs["distortion_coefficients"];
        if (!dc.empty() && !dc.isNone()) {
            dc["data"] >> dist_coeffs_;
        } else {
            RCLCPP_WARN(this->get_logger(), "distortion_coefficients not found in YAML, using zero distortion");
            dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
        }

        fs.release();

        RCLCPP_INFO(this->get_logger(), "Loaded camera intrinsics from %s", resolved_path.c_str());
    }

    // 订阅/发布
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr detection_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr bbox_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr adaptive_status_pub_;  // T13: 诊断 topic
    image_transport::Publisher image_pub_;

    // 参数回调 handle
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // 参数
    bool debug_mode_;
    double min_area_;
    double max_area_;
    double aspect_ratio_tol_;
    double expected_aspect_ratio_;
    double battery_length_mm_;
    double battery_width_mm_;
    int hsv_h_min_;
    int hsv_h_max_;
    int hsv_s_min_;
    int hsv_s_max_;
    int hsv_v_min_;
    int hsv_v_max_;

    // ========== T13: HSV 自适应机制 ==========
    // 原始 HSV 值（恢复目标）
    int original_hsv_h_min_;
    int original_hsv_h_max_;

    // 当前（运行时调整后的）HSV 值
    int current_hsv_h_min_;
    int current_hsv_h_max_;

    // 自适应参数
    int adaptive_failure_threshold_;   // 连续 N 帧失败后开始宽化
    int adaptive_recovery_threshold_;  // 连续 M 帧成功后开始收紧
    int adaptive_widening_step_;      // 每次宽化调整量
    int adaptive_h_min_limit_;        // H 最小值扩展边界
    int adaptive_h_max_limit_;        // H 最大值扩展边界

    // 连续失败/成功计数器
    int consecutive_failures_ = 0;
    int consecutive_successes_ = 0;

    // 相机内参 (从 YAML 加载或默认)
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BatteryDetector>());
    rclcpp::shutdown();
    return 0;
}
